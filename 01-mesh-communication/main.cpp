/**
 * ============================================================================
 * ICE STUPA AUTONOMOUS IRRIGATION CONTROLLER
 * Ice Potential Index (IPI) Engine — ESP32 Edge Firmware
 * ============================================================================
 *
 * Project     : Maximizing Artificial Glacier Operational Season
 * Author      : Open-Source Contribution (NIT Goa)
 * Target MCU  : ESP32 (tested on ESP32-WROOM-32 / DevKit v1)
 * Arduino Core: ESP32 Arduino Core v2.x (Espressif)
 * Libraries   : Adafruit BME280 v2.2.x, RTClib v2.1.x, Wire (built-in)
 * License     : MIT
 *
 * ── SYSTEM OVERVIEW ──────────────────────────────────────────────────────────
 * This firmware operates an autonomous edge-computed environmental station
 * that continuously evaluates microclimate conditions at a high-altitude
 * Ice Stupa site. It calculates a composite "Ice Potential Index" (IPI) from
 * wet-bulb temperature, barometric pressure, relative humidity, and diurnal
 * cycle data. When IPI exceeds a tunable threshold, a solenoid valve is opened
 * to spray mist water, which freezes on contact with the ice cone structure.
 *
 * ── HARDWARE SUMMARY ─────────────────────────────────────────────────────────
 *  • ESP32 DevKit v1 (3.3 V I/O, 240 MHz dual-core)
 *  • BME280 — Temperature / Humidity / Pressure (I2C addr 0x76)
 *  • DS3231  — Real-Time Clock with TCXO (I2C addr 0x68)
 *  • 5 V Active Buzzer — IPI alert feedback (GPIO 32)
 *  • MOSFET/Relay Module — Solenoid valve control (GPIO 33)
 *  • Status LED — System health indicator (GPIO 2, onboard)
 *  • Optional: OLED SSD1306 128×64 — live IPI readout (I2C addr 0x3C)
 *
 * ── I2C BUS ──────────────────────────────────────────────────────────────────
 *  GPIO 21 → SDA  (4.7 kΩ pull-up to 3.3 V on the bus)
 *  GPIO 22 → SCL  (4.7 kΩ pull-up to 3.3 V on the bus)
 *  All devices share the same SDA/SCL lines; each has a unique 7-bit address.
 *
 * ── IPI ALGORITHM OVERVIEW ───────────────────────────────────────────────────
 *  1. Wet-Bulb Temperature (Tw) — Stull (2011) approximation from T and RH.
 *     Tw is the primary freezing indicator; a Tw ≤ 0 °C means any water
 *     sprayed will lose heat to the air and freeze before hitting the ground.
 *
 *  2. Altitude-Corrected Pressure Bonus — thinner air at high altitude allows
 *     faster evaporative cooling, slightly enhancing freeze potential.
 *
 *  3. Humidity Penalty — very high RH (>90%) suppresses evaporative cooling;
 *     near-saturated air provides almost no latent heat sink.
 *
 *  4. Nocturnal Bonus — radiative cooling at night lowers effective surface
 *     temperature well below the measured air temperature. A diurnal multiplier
 *     (from the RTC hour) boosts IPI during the high-probability 22:00–06:00
 *     window.
 *
 *  5. Thermal Hysteresis Guard — prevents valve flutter when IPI hovers near
 *     the threshold. Valve opens at IPI ≥ OPEN_THRESHOLD and closes only when
 *     IPI falls below CLOSE_THRESHOLD (a lower hysteresis band).
 *
 *  Final IPI is normalised to [0, 100]. Typical interpretation:
 *    0–30  : Poor freeze potential  — valve OFF
 *   31–55  : Marginal               — valve OFF (but logged)
 *   56–75  : Good                   — valve ON
 *   76–100 : Excellent              — valve ON
 *
 * ── RAPID MICROCLIMATE TRANSITION HANDLING ──────────────────────────────────
 *  • A rolling 5-sample circular buffer stores recent IPI values.
 *  • The control decision is made on the MOVING AVERAGE, not the raw sample,
 *    suppressing transient spikes from solar radiation, wind gusts, or sensor
 *    noise.
 *  • A minimum ON-time (MIN_VALVE_ON_SECONDS) prevents micro-cycling.
 *  • A forced OFF-time after any sustained warming event (THERMAL_LOCKOUT_S)
 *    avoids partial-melt layering on the ice cone.
 *
 * ============================================================================
 */

// ─────────────────────────────────────────────────────────────────────────────
//  INCLUDES
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>   // Install via Arduino Library Manager: "Adafruit BME280"
#include <RTClib.h>            // Install via Arduino Library Manager: "RTClib by Adafruit"
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
//  COMPILE-TIME CONFIGURATION  (tune these for your deployment site)
// ─────────────────────────────────────────────────────────────────────────────

// ── Site parameters ──────────────────────────────────────────────────────────
// Altitude of the Ice Stupa site above sea level (metres).
// Used to compute sea-level pressure reference for the pressure correction term.
#define SITE_ALTITUDE_M          4200.0f

// Standard sea-level pressure (hPa). ICAO standard atmosphere reference.
#define SEA_LEVEL_PRESSURE_HPA   1013.25f

// ── IPI control thresholds ────────────────────────────────────────────────────
// Valve opens when smoothed IPI ≥ OPEN_THRESHOLD.
// Valve closes when smoothed IPI < CLOSE_THRESHOLD.
// The dead-band between them provides thermal hysteresis.
#define IPI_OPEN_THRESHOLD       56.0f   // [0-100] open valve above this IPI
#define IPI_CLOSE_THRESHOLD      48.0f   // [0-100] close valve below this IPI

// ── Absolute safety override ──────────────────────────────────────────────────
// If measured air temperature (dry-bulb) is at or above this value in °C,
// the valve is forced OFF regardless of IPI. Prevents daytime melt events.
#define AIR_TEMP_HARD_CUTOFF_C   1.5f

// ── Valve timing guards ───────────────────────────────────────────────────────
// Minimum continuous valve ON duration (seconds). Prevents micro-cycling.
#define MIN_VALVE_ON_SECONDS     120UL

// Thermal lockout period (seconds). After a hard-cutoff event (warm daytime),
// the valve is locked out for this duration even if IPI recovers quickly.
// This allows the ice surface to stabilise before new water is applied.
#define THERMAL_LOCKOUT_S        600UL

// ── Sampling ─────────────────────────────────────────────────────────────────
// Main loop sample interval (milliseconds). 30 s default.
#define SAMPLE_INTERVAL_MS       30000UL

// Depth of the IPI moving-average buffer (number of samples).
// Window duration = SAMPLE_INTERVAL_MS × IPI_BUFFER_DEPTH
//   → 5 × 30 s = 2.5 minutes of temporal smoothing
#define IPI_BUFFER_DEPTH         5

// ── Nocturnal window ─────────────────────────────────────────────────────────
// Hours (24-h clock) defining the high-probability freeze window.
#define NOCTURNAL_HOUR_START     22   // 10 PM
#define NOCTURNAL_HOUR_END       6    //  6 AM

// Bonus added to raw IPI during the nocturnal window (radiative cooling effect).
#define NOCTURNAL_IPI_BONUS      12.0f

// ── GPIO assignments ─────────────────────────────────────────────────────────
#define PIN_VALVE_RELAY          33   // LOW = valve OFF, HIGH = valve ON
#define PIN_BUZZER               32   // Active buzzer, momentary pulse on IPI events
#define PIN_STATUS_LED           2    // Onboard LED: heartbeat blink

// ─────────────────────────────────────────────────────────────────────────────
//  DATA STRUCTURES
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Full environmental payload captured on every sample tick.
 *
 * All derived quantities (wet-bulb, IPI components) are stored alongside the
 * raw sensor readings to facilitate USB-serial logging and debugging.
 */
struct EnvironmentalPayload {
    // ── Raw sensor data ───────────────────────────────────────────────────
    float     temperature_C;        ///< Dry-bulb air temperature (°C)
    float     humidity_pct;         ///< Relative humidity (%)
    float     pressure_hPa;         ///< Barometric pressure (hPa, station-level)
    uint32_t  timestamp_epoch;      ///< Unix timestamp from DS3231 RTC

    // ── Derived quantities ────────────────────────────────────────────────
    float     wetBulb_C;            ///< Wet-bulb temperature (°C) [Stull 2011]
    float     dewPoint_C;           ///< Dew point (°C) [Magnus formula]
    float     pressureBonus;        ///< Altitude pressure correction term
    float     humidityPenalty;      ///< High-RH evap suppression penalty
    float     nocturnalBonus;       ///< Diurnal radiative cooling bonus
    float     rawIPI;               ///< IPI before smoothing [0-100]
    float     smoothedIPI;          ///< Moving-average IPI [0-100]

    // ── Control state ─────────────────────────────────────────────────────
    bool      valveOpen;            ///< Current valve state
    bool      hardCutoffActive;     ///< True if warm-air override is in effect
    bool      thermalLockout;       ///< True if lockout timer is running
};

/**
 * @brief Circular buffer for IPI smoothing.
 *
 * Stores the last IPI_BUFFER_DEPTH raw IPI values.
 * Moving average is computed by summing all slots and dividing by depth.
 */
struct IPIBuffer {
    float    values[IPI_BUFFER_DEPTH];  ///< Ring storage
    uint8_t  head;                      ///< Index of next write position
    uint8_t  count;                     ///< Number of valid entries so far
};

// ─────────────────────────────────────────────────────────────────────────────
//  GLOBAL INSTANCES
// ─────────────────────────────────────────────────────────────────────────────
Adafruit_BME280 bme;          // BME280 sensor object (default I2C addr 0x76)
RTC_DS3231      rtc;          // DS3231 RTC object

// Persistent state across iterations
static IPIBuffer            ipiBuffer         = {};
static EnvironmentalPayload lastPayload       = {};
static bool                 valveCurrentState = false;
static unsigned long        valveOnSince_ms   = 0;      // millis() when valve opened
static unsigned long        lockoutEnd_ms     = 0;      // millis() when lockout expires
static unsigned long        lastSampleTime_ms = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  FUNCTION DECLARATIONS (forward declarations for readability)
// ─────────────────────────────────────────────────────────────────────────────
bool    initSensors();
void    readEnvironment(EnvironmentalPayload &p);
float   computeWetBulb(float T, float RH);
float   computeDewPoint(float T, float RH);
float   computeRawIPI(EnvironmentalPayload &p, uint8_t hour);
float   updateIPIBuffer(IPIBuffer &buf, float newValue);
void    evaluateValveControl(EnvironmentalPayload &p);
void    setValve(bool open);
void    buzzerBeep(uint8_t count, uint16_t durationMs);
void    logPayload(const EnvironmentalPayload &p);
void    fatalHalt(const char *msg);

// ─────────────────────────────────────────────────────────────────────────────
//  ARDUINO SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    // ── Serial console ────────────────────────────────────────────────────
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { delay(10); }
    Serial.println(F("\n\n==================================="));
    Serial.println(F("  Ice Stupa IPI Controller v1.0"));
    Serial.println(F("===================================\n"));

    // ── GPIO configuration ────────────────────────────────────────────────
    pinMode(PIN_STATUS_LED,  OUTPUT);
    pinMode(PIN_VALVE_RELAY, OUTPUT);
    pinMode(PIN_BUZZER,      OUTPUT);

    // Ensure valve starts CLOSED on power-up / reset (failsafe)
    digitalWrite(PIN_VALVE_RELAY, LOW);
    digitalWrite(PIN_BUZZER,      LOW);
    digitalWrite(PIN_STATUS_LED,  LOW);

    // ── I2C bus ───────────────────────────────────────────────────────────
    // Default I2C pins on ESP32: SDA = GPIO 21, SCL = GPIO 22.
    // External 4.7 kΩ pull-ups to 3.3 V are required on the PCB/breadboard.
    Wire.begin(21, 22);
    Wire.setClock(100000UL);   // 100 kHz standard mode; DS3231 max is 400 kHz,
                                // but 100 kHz is more reliable on long wire runs.

    // ── Sensor initialisation ─────────────────────────────────────────────
    if (!initSensors()) {
        fatalHalt("Sensor initialisation failed. Check I2C wiring and addresses.");
    }

    // ── IPI buffer — prefill with neutral value ───────────────────────────
    for (uint8_t i = 0; i < IPI_BUFFER_DEPTH; i++) {
        ipiBuffer.values[i] = 0.0f;
    }
    ipiBuffer.head  = 0;
    ipiBuffer.count = 0;

    // ── Startup confirmation ──────────────────────────────────────────────
    buzzerBeep(2, 80);   // Two short beeps = system ready
    Serial.println(F("[BOOT] System initialised successfully.\n"));

    // Print CSV header for easy serial-monitor / SD-card logging
    Serial.println(F("timestamp,temp_C,rh_pct,pres_hPa,wetBulb_C,dewPt_C,"
                     "presBonus,rhPenalty,noctBonus,rawIPI,smIPI,valve,lockout"));
}

// ─────────────────────────────────────────────────────────────────────────────
//  ARDUINO MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();

    // ── Heartbeat LED (1 Hz blink regardless of valve state) ─────────────
    digitalWrite(PIN_STATUS_LED, (now / 500) % 2 == 0);

    // ── Sample gate ───────────────────────────────────────────────────────
    if (now - lastSampleTime_ms < SAMPLE_INTERVAL_MS) {
        return;   // Wait for next sample interval
    }
    lastSampleTime_ms = now;

    // ── Sample sensors ────────────────────────────────────────────────────
    readEnvironment(lastPayload);

    // ── Get current hour from RTC ─────────────────────────────────────────
    DateTime dt = rtc.now();
    uint8_t  currentHour = dt.hour();
    lastPayload.timestamp_epoch = dt.unixtime();

    // ── Compute IPI ──────────────────────────────────────────────────────
    lastPayload.rawIPI     = computeRawIPI(lastPayload, currentHour);
    lastPayload.smoothedIPI = updateIPIBuffer(ipiBuffer, lastPayload.rawIPI);

    // ── Evaluate and actuate valve ────────────────────────────────────────
    evaluateValveControl(lastPayload);

    // ── Serial log ────────────────────────────────────────────────────────
    logPayload(lastPayload);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SENSOR INITIALISATION
// ─────────────────────────────────────────────────────────────────────────────
/**
 * @brief Initialise BME280 and DS3231 over the shared I2C bus.
 *
 * BME280 is configured in forced mode: it takes one measurement per trigger
 * and then returns to sleep, minimising self-heating inside the enclosure.
 * The DS3231's 32 kHz output is disabled to reduce power consumption.
 *
 * @return true if both devices acknowledge on the bus, false otherwise.
 */
bool initSensors() {
    // ── BME280 ────────────────────────────────────────────────────────────
    // Default I2C address when SDO pin is tied LOW = 0x76.
    // Change to 0x77 if SDO is tied HIGH.
    if (!bme.begin(0x76)) {
        Serial.println(F("[ERROR] BME280 not found at 0x76. "
                         "Check SDO pin and pull-ups."));
        return false;
    }

    // Forced mode: one measurement, then deep sleep.
    // Oversampling ×1 for temperature, humidity, pressure is sufficient —
    // we do not need higher resolution for freeze prediction.
    bme.setSampling(
        Adafruit_BME280::MODE_FORCED,       // take one sample per bme.takeForcedMeasurement()
        Adafruit_BME280::SAMPLING_X1,       // temperature oversampling
        Adafruit_BME280::SAMPLING_X1,       // pressure oversampling
        Adafruit_BME280::SAMPLING_X1,       // humidity oversampling
        Adafruit_BME280::FILTER_X2,         // IIR filter coefficient: light smoothing
        Adafruit_BME280::STANDBY_MS_0_5     // n/a in FORCED mode
    );
    Serial.println(F("[INIT] BME280 OK (forced mode, 0x76)"));

    // ── DS3231 RTC ────────────────────────────────────────────────────────
    if (!rtc.begin()) {
        Serial.println(F("[ERROR] DS3231 not found at 0x68. "
                         "Check SDA/SCL and 4.7k pull-ups."));
        return false;
    }

    // If the RTC lost power and the clock is invalid, set to compile-time.
    // NOTE: Re-flash or use serial commands to set accurate time on-site.
    if (rtc.lostPower()) {
        Serial.println(F("[WARN] RTC lost power — setting to compile time. "
                         "Re-synchronise on-site!"));
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    // Disable the 32 kHz output pin to save ~0.5 mA.
    rtc.disable32K();

    Serial.print(F("[INIT] DS3231 OK. Current time: "));
    DateTime now = rtc.now();
    Serial.printf("%04d-%02d-%02d %02d:%02d:%02d\n",
                  now.year(), now.month(), now.day(),
                  now.hour(), now.minute(), now.second());

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  SENSOR READING
// ─────────────────────────────────────────────────────────────────────────────
/**
 * @brief Trigger a BME280 forced-mode measurement and populate the payload.
 *
 * Forced mode ensures the sensor sleeps between reads, reducing the
 * self-heating error that would otherwise bias temperature upward in a
 * sealed enclosure (typical bias is +0.5 to +2 °C in normal mode).
 *
 * @param[out] p  Reference to payload struct to be filled.
 */
void readEnvironment(EnvironmentalPayload &p) {
    // Wake BME280 and wait for measurement to complete (~10 ms for ×1 OS)
    bme.takeForcedMeasurement();

    p.temperature_C = bme.readTemperature();    // °C
    p.humidity_pct  = bme.readHumidity();       // %
    p.pressure_hPa  = bme.readPressure() / 100.0f; // Pa → hPa

    // Basic sanity clamp — catches I2C glitch values
    p.temperature_C = constrain(p.temperature_C, -50.0f,  60.0f);
    p.humidity_pct  = constrain(p.humidity_pct,    0.0f, 100.0f);
    p.pressure_hPa  = constrain(p.pressure_hPa,  300.0f,1100.0f);

    // Compute derived thermodynamic quantities
    p.wetBulb_C  = computeWetBulb(p.temperature_C, p.humidity_pct);
    p.dewPoint_C = computeDewPoint(p.temperature_C, p.humidity_pct);
}

// ─────────────────────────────────────────────────────────────────────────────
//  WET-BULB TEMPERATURE  (Stull 2011 empirical approximation)
// ─────────────────────────────────────────────────────────────────────────────
/**
 * @brief Compute wet-bulb temperature using the Stull (2011) polynomial.
 *
 * Reference:
 *   Stull, R. (2011). "Wet-Bulb Temperature from Relative Humidity and Air
 *   Temperature." Journal of Applied Meteorology and Climatology, 50(11),
 *   2267-2269.  DOI: 10.1175/JAMC-D-11-0143.1
 *
 * This approximation is valid for:
 *   −20 °C ≤ T ≤ 50 °C  and  5 % ≤ RH ≤ 99 %
 *
 * Maximum error ≈ 0.65 °C within the valid range — more than adequate for
 * binary freeze/no-freeze prediction (threshold is 0 °C, a wide margin).
 *
 * Physical interpretation for this application:
 *   Tw ≤ 0 °C → sprayed water droplets will undergo evaporative AND
 *   sensible cooling, guaranteeing freezing before ground contact.
 *   Tw > 0 °C → the air can absorb latent heat without cooling water to 0 °C;
 *   water will not freeze in mid-air and may melt surface ice.
 *
 * @param T   Dry-bulb temperature (°C)
 * @param RH  Relative humidity (%)
 * @return    Wet-bulb temperature (°C)
 */
float computeWetBulb(float T, float RH) {
    // Stull (2011) Eq. 1 — polynomial in T and atan(RH) terms
    float Tw = T * atanf(0.151977f * sqrtf(RH + 8.313659f))
             + atanf(T + RH)
             - atanf(RH - 1.676331f)
             + 0.00391838f * powf(RH, 1.5f) * atanf(0.023101f * RH)
             - 4.686035f;
    return Tw;
}

// ─────────────────────────────────────────────────────────────────────────────
//  DEW POINT  (Magnus formula)
// ─────────────────────────────────────────────────────────────────────────────
/**
 * @brief Compute dew-point temperature using the Magnus formula.
 *
 * Dew point is logged as an auxiliary diagnostic: when Tdp approaches T,
 * the air is near saturation and evaporative cooling is minimal.
 * When Tdp is much lower than T, vigorous evaporative cooling is possible.
 *
 * Constants from Alduchov & Eskridge (1996) over ice:
 *   a = 17.625, b = 243.04 °C
 *
 * @param T   Dry-bulb temperature (°C)
 * @param RH  Relative humidity (%)
 * @return    Dew-point temperature (°C)
 */
float computeDewPoint(float T, float RH) {
    const float a = 17.625f;
    const float b = 243.04f;
    float alpha = logf(RH / 100.0f) + (a * T) / (b + T);
    return (b * alpha) / (a - alpha);
}

// ─────────────────────────────────────────────────────────────────────────────
//  ICE POTENTIAL INDEX (IPI) COMPUTATION
// ─────────────────────────────────────────────────────────────────────────────
/**
 * @brief Compute the composite Ice Potential Index for the current sample.
 *
 * ── ALGORITHM ──────────────────────────────────────────────────────────────
 *
 *  COMPONENT 1: Wet-Bulb Score (primary driver, 0–60 points)
 *  ──────────────────────────────────────────────────────────
 *  Maps Tw linearly from -15 °C → 60 pts to +5 °C → 0 pts.
 *  Tw ≤ -15 °C is saturated at 60; Tw ≥ +5 °C is 0.
 *
 *  Rationale: The wet-bulb threshold for guaranteed droplet freezing is 0 °C.
 *  We use +5 °C as the zero boundary (not 0 °C) because water jetting several
 *  metres requires a safety margin — the droplets must freeze in-flight.
 *
 *  COMPONENT 2: Altitude Pressure Bonus (0–10 points)
 *  ────────────────────────────────────────────────────
 *  High-altitude sites have low atmospheric pressure, which lowers the
 *  boiling/evaporation point of water and accelerates evaporative cooling.
 *  The correction bonus is proportional to how far the measured station
 *  pressure is below the standard sea-level pressure.
 *
 *  bonus = clamp((SEA_LEVEL_HPA - p_station) / 50.0, 0, 10)
 *  At 4200 m, p_station ≈ 610 hPa → bonus ≈ 8 pts.
 *
 *  COMPONENT 3: Humidity Penalty (−20 to 0 points)
 *  ──────────────────────────────────────────────────
 *  Very high RH (>90%) suppresses evaporative cooling — the air is nearly
 *  saturated and cannot accept more water vapour.
 *  penalty = clamp((RH - 90) / 10.0 * 20.0, 0, 20)
 *  At RH = 100 %: penalty = −20 pts. At RH = 90 %: penalty = 0.
 *
 *  COMPONENT 4: Nocturnal Bonus (0 or NOCTURNAL_IPI_BONUS points)
 *  ────────────────────────────────────────────────────────────────
 *  Between NOCTURNAL_HOUR_START and NOCTURNAL_HOUR_END, long-wave radiative
 *  cooling from the snow/ice surface can lower effective temperature several
 *  degrees below the measured screen-level value.
 *  A fixed bonus of NOCTURNAL_IPI_BONUS (default 12 pts) is applied.
 *
 *  FINAL IPI
 *  ──────────
 *  rawIPI = clamp(wetBulbScore + pressureBonus - humidityPenalty + nocturnalBonus, 0, 100)
 *
 * @param p     Environmental payload (must already have Tw, RH, pressure)
 * @param hour  Current hour (0–23) from RTC
 * @return      Raw IPI score [0–100]
 */
float computeRawIPI(EnvironmentalPayload &p, uint8_t hour) {

    // ── Component 1: Wet-Bulb Score ───────────────────────────────────────
    // Linear mapping: Tw ≤ -15 °C → 60 pts; Tw ≥ +5 °C → 0 pts
    const float TW_MIN = -15.0f;   // Maximum freeze potential anchor
    const float TW_MAX =   5.0f;   // Zero freeze potential cutoff
    float wetBulbScore = 0.0f;
    if (p.wetBulb_C <= TW_MIN) {
        wetBulbScore = 60.0f;
    } else if (p.wetBulb_C >= TW_MAX) {
        wetBulbScore = 0.0f;
    } else {
        // Proportional mapping within the [TW_MIN, TW_MAX] range
        wetBulbScore = (TW_MAX - p.wetBulb_C) / (TW_MAX - TW_MIN) * 60.0f;
    }

    // ── Component 2: Altitude / Pressure Bonus ────────────────────────────
    // Low station pressure accelerates evaporative cooling.
    // Factor of 50 hPa per point keeps the bonus in [0, 10] for realistic
    // high-altitude site pressures (600–800 hPa).
    float pressureBonus = (SEA_LEVEL_PRESSURE_HPA - p.pressure_hPa) / 50.0f;
    pressureBonus = constrain(pressureBonus, 0.0f, 10.0f);
    p.pressureBonus = pressureBonus;   // Store in payload for logging

    // ── Component 3: Humidity Penalty ────────────────────────────────────
    // Ramps from 0 at RH = 90 % to full 20 pt penalty at RH = 100 %.
    float humidityPenalty = 0.0f;
    if (p.humidity_pct > 90.0f) {
        humidityPenalty = ((p.humidity_pct - 90.0f) / 10.0f) * 20.0f;
    }
    humidityPenalty = constrain(humidityPenalty, 0.0f, 20.0f);
    p.humidityPenalty = humidityPenalty;  // Store in payload for logging

    // ── Component 4: Nocturnal Bonus ──────────────────────────────────────
    float nocturnalBonus = 0.0f;
    bool isNocturnal = (hour >= NOCTURNAL_HOUR_START) || (hour < NOCTURNAL_HOUR_END);
    if (isNocturnal) {
        nocturnalBonus = NOCTURNAL_IPI_BONUS;
    }
    p.nocturnalBonus = nocturnalBonus;    // Store in payload for logging

    // ── Final composite IPI ───────────────────────────────────────────────
    float ipi = wetBulbScore + pressureBonus - humidityPenalty + nocturnalBonus;
    ipi = constrain(ipi, 0.0f, 100.0f);

    return ipi;
}

// ─────────────────────────────────────────────────────────────────────────────
//  IPI MOVING AVERAGE BUFFER
// ─────────────────────────────────────────────────────────────────────────────
/**
 * @brief Insert a new IPI sample into the circular buffer and return the mean.
 *
 * The moving average provides temporal smoothing over the last N samples.
 * This suppresses transient IPI spikes caused by:
 *   • Solar radiation heating the BME280 enclosure momentarily
 *   • Passing cloud shadows
 *   • Wind gusts changing the local humidity transiently
 *
 * @param buf       Reference to the IPI circular buffer struct.
 * @param newValue  Latest raw IPI sample to insert.
 * @return          Arithmetic mean of all valid entries in the buffer.
 */
float updateIPIBuffer(IPIBuffer &buf, float newValue) {
    // Write to the current head position
    buf.values[buf.head] = newValue;

    // Advance head pointer (wraps around)
    buf.head = (buf.head + 1) % IPI_BUFFER_DEPTH;

    // Increment count up to maximum depth
    if (buf.count < IPI_BUFFER_DEPTH) {
        buf.count++;
    }

    // Compute arithmetic mean over all valid entries
    float sum = 0.0f;
    for (uint8_t i = 0; i < buf.count; i++) {
        sum += buf.values[i];
    }
    return sum / (float)buf.count;
}

// ─────────────────────────────────────────────────────────────────────────────
//  VALVE CONTROL LOGIC
// ─────────────────────────────────────────────────────────────────────────────
/**
 * @brief Evaluate environmental conditions and actuate valve accordingly.
 *
 * Decision hierarchy (highest precedence first):
 *
 *  1. HARD CUTOFF: If dry-bulb air temperature ≥ AIR_TEMP_HARD_CUTOFF_C,
 *     force valve OFF and start the thermal lockout timer. This is the absolute
 *     safety override — no IPI computation matters if the air is clearly warm.
 *
 *  2. THERMAL LOCKOUT: If a previous hard-cutoff event is still within its
 *     lockout window, keep valve OFF. Prevents applying new water layers to a
 *     partially-melted surface.
 *
 *  3. MIN ON-TIME GUARD: If the valve was recently opened and has not yet been
 *     ON for MIN_VALVE_ON_SECONDS, keep it ON. Prevents micro-cycling that
 *     stresses the solenoid and creates ice drips rather than layers.
 *
 *  4. IPI HYSTERESIS CONTROL (normal operation):
 *       smoothedIPI ≥ IPI_OPEN_THRESHOLD  → open valve
 *       smoothedIPI < IPI_CLOSE_THRESHOLD → close valve
 *       otherwise                         → maintain current state
 *
 * @param[in,out] p  Current environmental payload. Control flags are updated.
 */
void evaluateValveControl(EnvironmentalPayload &p) {
    unsigned long now_ms = millis();

    // ── Check 1: Hard Cutoff (warm air) ──────────────────────────────────
    if (p.temperature_C >= AIR_TEMP_HARD_CUTOFF_C) {
        if (valveCurrentState) {
            Serial.printf("[CTRL] Hard cutoff! T=%.1f°C ≥ %.1f°C — closing valve.\n",
                          p.temperature_C, AIR_TEMP_HARD_CUTOFF_C);
            buzzerBeep(3, 150);   // 3 beeps = warning close event
        }
        setValve(false);
        // Reset lockout timer from this moment
        lockoutEnd_ms = now_ms + (THERMAL_LOCKOUT_S * 1000UL);
        p.hardCutoffActive = true;
        p.thermalLockout   = true;
        p.valveOpen        = false;
        return;
    }
    p.hardCutoffActive = false;

    // ── Check 2: Thermal Lockout Window ──────────────────────────────────
    if (now_ms < lockoutEnd_ms) {
        uint32_t remaining_s = (lockoutEnd_ms - now_ms) / 1000UL;
        Serial.printf("[CTRL] Thermal lockout active — %lu s remaining.\n", remaining_s);
        setValve(false);
        p.thermalLockout = true;
        p.valveOpen      = false;
        return;
    }
    p.thermalLockout = false;

    // ── Check 3: Minimum ON-time Guard ───────────────────────────────────
    if (valveCurrentState) {
        uint32_t onDuration_s = (now_ms - valveOnSince_ms) / 1000UL;
        if (onDuration_s < MIN_VALVE_ON_SECONDS) {
            // Valve is ON and has not yet hit minimum on-time; keep it open
            // regardless of current IPI (transient drop tolerance).
            p.valveOpen = true;
            return;
        }
    }

    // ── Check 4: IPI Hysteresis Control ──────────────────────────────────
    if (!valveCurrentState && p.smoothedIPI >= IPI_OPEN_THRESHOLD) {
        // IPI crossed open threshold → open valve
        Serial.printf("[CTRL] IPI=%.1f ≥ %.1f — opening valve.\n",
                      p.smoothedIPI, IPI_OPEN_THRESHOLD);
        buzzerBeep(1, 200);   // 1 beep = valve opened
        setValve(true);

    } else if (valveCurrentState && p.smoothedIPI < IPI_CLOSE_THRESHOLD) {
        // IPI fell below close threshold → close valve
        Serial.printf("[CTRL] IPI=%.1f < %.1f — closing valve.\n",
                      p.smoothedIPI, IPI_CLOSE_THRESHOLD);
        setValve(false);

    }
    // else: maintain current state (hysteresis dead-band)

    p.valveOpen = valveCurrentState;
}

// ─────────────────────────────────────────────────────────────────────────────
//  VALVE ACTUATION
// ─────────────────────────────────────────────────────────────────────────────
/**
 * @brief Set the physical valve state via the relay/MOSFET gate.
 *
 * The relay module used in this design is ACTIVE HIGH:
 *   • HIGH on PIN_VALVE_RELAY → relay coil energised → valve OPEN (water flows)
 *   • LOW  on PIN_VALVE_RELAY → relay coil de-energised → valve CLOSED
 *
 * If your relay module is ACTIVE LOW, invert the logic here.
 *
 * @param open  true = open valve, false = close valve.
 */
void setValve(bool open) {
    if (open == valveCurrentState) return;   // No change needed

    valveCurrentState = open;
    digitalWrite(PIN_VALVE_RELAY, open ? HIGH : LOW);

    if (open) {
        valveOnSince_ms = millis();   // Record open time for min-ON guard
        Serial.println(F("[VALVE] >>> OPEN <<<"));
    } else {
        Serial.println(F("[VALVE] --- CLOSED ---"));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  BUZZER
// ─────────────────────────────────────────────────────────────────────────────
/**
 * @brief Generate a series of short beeps on the active buzzer.
 *
 * @param count       Number of beep pulses.
 * @param durationMs  Duration of each beep in milliseconds.
 */
void buzzerBeep(uint8_t count, uint16_t durationMs) {
    for (uint8_t i = 0; i < count; i++) {
        digitalWrite(PIN_BUZZER, HIGH);
        delay(durationMs);
        digitalWrite(PIN_BUZZER, LOW);
        if (i < count - 1) delay(durationMs);   // Gap between beeps
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  SERIAL LOGGING
// ─────────────────────────────────────────────────────────────────────────────
/**
 * @brief Output a CSV-formatted log line to the Serial console.
 *
 * Output is structured so it can be directly piped into a spreadsheet or
 * data logger for post-trip analysis and IPI calibration.
 *
 * Column order matches the header printed in setup().
 *
 * @param p  Populated environmental payload.
 */
void logPayload(const EnvironmentalPayload &p) {
    // Print human-readable summary
    Serial.println(F("─────────────────────────────────────"));
    Serial.printf("[DATA] T=%.2f°C  RH=%.1f%%  P=%.1fhPa\n",
                  p.temperature_C, p.humidity_pct, p.pressure_hPa);
    Serial.printf("[IPI]  Tw=%.2f°C  Tdp=%.2f°C\n",
                  p.wetBulb_C, p.dewPoint_C);
    Serial.printf("[IPI]  WB=%.1f  PB=%.1f  HP=%.1f  NB=%.1f\n",
                  /* note: recalculate wet-bulb score inline for display only */
                  constrain((5.0f - p.wetBulb_C) / 20.0f * 60.0f, 0.0f, 60.0f),
                  p.pressureBonus, p.humidityPenalty, p.nocturnalBonus);
    Serial.printf("[IPI]  Raw=%.1f  Smoothed=%.1f\n",
                  p.rawIPI, p.smoothedIPI);
    Serial.printf("[CTRL] Valve=%s  Cutoff=%s  Lockout=%s\n",
                  p.valveOpen        ? "OPEN"   : "CLOSED",
                  p.hardCutoffActive ? "YES"    : "no",
                  p.thermalLockout   ? "YES"    : "no");

    // Machine-parseable CSV line
    Serial.printf("%lu,%.2f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%d,%d\n",
                  (unsigned long)p.timestamp_epoch,
                  p.temperature_C, p.humidity_pct, p.pressure_hPa,
                  p.wetBulb_C, p.dewPoint_C,
                  p.pressureBonus, p.humidityPenalty, p.nocturnalBonus,
                  p.rawIPI, p.smoothedIPI,
                  (int)p.valveOpen,
                  (int)p.thermalLockout);
}

// ─────────────────────────────────────────────────────────────────────────────
//  FATAL ERROR HANDLER
// ─────────────────────────────────────────────────────────────────────────────
/**
 * @brief Print an error message and halt in an infinite error-blink loop.
 *
 * The valve is explicitly closed before halting to ensure failsafe.
 * The status LED blinks rapidly (10 Hz) to signal a fault to the operator.
 * The system can only be recovered by power cycling after fixing the fault.
 *
 * @param msg  Null-terminated C-string describing the fault.
 */
void fatalHalt(const char *msg) {
    // Ensure valve is de-energised (failsafe)
    digitalWrite(PIN_VALVE_RELAY, LOW);

    Serial.print(F("[FATAL] "));
    Serial.println(msg);
    Serial.println(F("System halted. Power-cycle to restart."));

    // Rapid blink loop — never returns
    while (true) {
        digitalWrite(PIN_STATUS_LED, HIGH); delay(50);
        digitalWrite(PIN_STATUS_LED, LOW);  delay(50);
    }
}
