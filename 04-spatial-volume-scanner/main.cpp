/**
 * =============================================================================
 * Ice Stupa Volume Scanner -- ESP32 Firmware
 * =============================================================================
 * Author  : Open-Source Portfolio Project
 * Target  : ESP32 (any variant: DevKit V1, WROOM-32, S3, etc.)
 * Sensor  : Benewake TF-Luna (UART, 115200 baud, binary 9-byte frame)
 *           Compatible fallback: TFmini-S (same binary protocol)
 * Servos  : 2x Standard SG90 / MG90S (5V, 50Hz PWM, 500us-2500us pulse)
 * IDE     : Arduino IDE 2.x + esp32 board package by Espressif (>=3.0.0)
 *
 * HOW THE SCAN WORKS
 * ------------------
 *   - Pan servo (horizontal, YAW)   : GPIO 18, sweeps PHI   0 to 180 deg
 *   - Tilt servo (vertical, PITCH)  : GPIO 19, sweeps THETA 10 to 90 deg
 *   - THETA (elevation) is the angle from the vertical (Z) axis.
 *       THETA=0   => sensor points straight up
 *       THETA=90  => sensor points horizontally
 *   - PHI (azimuth) is the horizontal rotation angle.
 *   - For each (PHI, THETA) position, one TF-Luna measurement is taken.
 *   - Data is emitted as ASCII over USB Serial (115200 baud) to the PC.
 *
 * OUTPUT FORMAT (one line per valid measurement)
 * ----------------------------------------------
 *   PHI:045.00,THETA:030.00,DIST:152,STRENGTH:500,QUAL:1
 *   Fields:
 *     PHI      : azimuth angle in degrees (float, 3 decimal places)
 *     THETA    : elevation angle in degrees (float, 3 decimal places)
 *     DIST     : distance in centimetres (integer, 0-800 cm for TF-Luna)
 *     STRENGTH : signal return strength (integer, >100 = reliable)
 *     QUAL     : 1 if measurement is valid, 0 if checksum failed or dist=0
 *   A special line "SCAN_COMPLETE" is sent after every full pan-tilt sweep.
 *
 * POWER ISOLATION
 * ---------------
 *   Servos are powered from a DEDICATED 5V rail (USB power bank or LM7805).
 *   Only the PWM signal wires connect to the ESP32 GPIO pins.
 *   GND of servo supply is tied to ESP32 GND (common ground required).
 *   The TF-Luna is powered from the ESP32 3.3V pin (it accepts 3.3-5V).
 *   A 100uF electrolytic + 100nF ceramic capacitor are placed across the
 *   servo 5V rail to suppress PWM switching noise.
 *
 * SCAN PARAMETERS (adjust to match your setup)
 * =============================================
 *   PHI_START   0    deg  -- leftmost pan position
 *   PHI_END   180    deg  -- rightmost pan position
 *   PHI_STEP    5    deg  -- angular resolution in azimuth
 *   THETA_START 10   deg  -- highest tilt (nearly vertical, avoids 0-deg blind spot)
 *   THETA_END   90   deg  -- lowest tilt (horizontal, points at base of object)
 *   THETA_STEP   5   deg  -- angular resolution in elevation
 *   SETTLE_MS   150  ms   -- time for servo to reach position and vibration to settle
 *   MEASURE_N    3         -- measurements averaged per position (noise reduction)
 * =============================================================================
 */

#include <Arduino.h>

// ---------------------------------------------------------------------------
// PIN DEFINITIONS
// ---------------------------------------------------------------------------
// Pan servo  (horizontal / azimuth / PHI)
static const int PIN_SERVO_PAN  = 18;

// Tilt servo (vertical / elevation / THETA)
static const int PIN_SERVO_TILT = 19;

// TF-Luna hardware UART
// TX of TF-Luna goes to RX2 of ESP32 (GPIO 16)
// RX of TF-Luna goes to TX2 of ESP32 (GPIO 17)  -- optional, for config cmds
static const int PIN_LIDAR_RX   = 16;  // ESP32 RX2 <-- TF-Luna TX
static const int PIN_LIDAR_TX   = 17;  // ESP32 TX2 --> TF-Luna RX (optional)

// ---------------------------------------------------------------------------
// LEDC (PWM) CONFIGURATION
// ESP32 LEDC generates hardware PWM independent of CPU load.
// Two separate LEDC timers are used so both servos run simultaneously.
// ---------------------------------------------------------------------------
static const int LEDC_FREQ_HZ   = 50;         // Standard servo frequency
static const int LEDC_RES_BITS  = 16;         // 16-bit resolution
static const uint32_t LEDC_MAX_DUTY = (1U << LEDC_RES_BITS) - 1;  // 65535
static const uint32_t LEDC_PERIOD_US = 1000000UL / LEDC_FREQ_HZ;  // 20000 us

// LEDC channels (0-15 available on ESP32; use 0 and 1)
static const int LEDC_CH_PAN    = 0;
static const int LEDC_CH_TILT   = 1;

// Servo mechanical limits (microseconds)
// Standard SG90 / MG90S: 500us = 0 deg, 1500us = 90 deg, 2500us = 180 deg
static const int SERVO_MIN_US   = 500;
static const int SERVO_MAX_US   = 2500;
static const int SERVO_MIN_DEG  = 0;
static const int SERVO_MAX_DEG  = 180;

// ---------------------------------------------------------------------------
// SCAN PARAMETERS -- edit these to change resolution and angular coverage
// ---------------------------------------------------------------------------
static const float PHI_START    =   0.0f;   // degrees, azimuth start
static const float PHI_END      = 180.0f;   // degrees, azimuth end
static const float PHI_STEP     =   5.0f;   // degrees, azimuth step

static const float THETA_START  =  10.0f;   // degrees, elevation start (near-vertical)
static const float THETA_END    =  90.0f;   // degrees, elevation end   (horizontal)
static const float THETA_STEP   =   5.0f;   // degrees, elevation step

// Timing
static const int SETTLE_MS      = 150;       // ms to wait after servo moves
static const int MEASURE_N      = 3;         // readings to average per position

// TF-Luna minimum acceptable signal strength
static const uint16_t MIN_STRENGTH = 100;

// TF-Luna maximum valid distance (cm) -- rated range is 0.2 to 8 m
static const uint16_t MAX_DIST_CM  = 800;

// ---------------------------------------------------------------------------
// TF-Luna binary frame definition
// Frame format: [0x59][0x59][DistL][DistH][StrL][StrH][TempL][TempH][CkSum]
// ---------------------------------------------------------------------------
static const uint8_t  TFLUNA_HEADER = 0x59;
static const int      TFLUNA_FRAME_SIZE = 9;

// ---------------------------------------------------------------------------
// HELPER STRUCTURES
// ---------------------------------------------------------------------------
struct LidarReading {
    uint16_t dist_cm;       // distance in centimetres
    uint16_t strength;      // return signal strength
    bool     valid;         // true if frame checksum passed and values in range
};

// ---------------------------------------------------------------------------
// FUNCTION PROTOTYPES
// ---------------------------------------------------------------------------
void     initServoPWM(int gpio, int channel);
void     setServoAngle(int channel, float angle_deg);
uint32_t angleToDuty(float angle_deg);
LidarReading readTFLuna();
LidarReading averageMeasurements(int n);
void     emitDataLine(float phi, float theta, const LidarReading& reading);
void     moveToCenter();

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {
    // USB serial to PC -- Python reads this port
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    // TF-Luna hardware UART on Serial2 (pins 16/17)
    // TF-Luna default baud rate is 115200
    Serial2.begin(115200, SERIAL_8N1, PIN_LIDAR_RX, PIN_LIDAR_TX);
    delay(100);

    // Initialise LEDC PWM for both servos
    initServoPWM(PIN_SERVO_PAN,  LEDC_CH_PAN);
    initServoPWM(PIN_SERVO_TILT, LEDC_CH_TILT);

    // Move both servos to a safe starting position
    moveToCenter();
    delay(1000);

    Serial.println("# Ice Stupa Volume Scanner -- ESP32 Firmware");
    Serial.println("# Format: PHI:ddd.dd,THETA:ddd.dd,DIST:ddd,STRENGTH:ddd,QUAL:d");
    Serial.println("# Starting scan...");
    delay(500);
}

// ---------------------------------------------------------------------------
// MAIN SCAN LOOP
// ---------------------------------------------------------------------------
void loop() {
    // Outer loop: elevation (THETA) -- scan from near-vertical down to horizontal
    for (float theta = THETA_START; theta <= THETA_END + 0.01f; theta += THETA_STEP) {

        // Set tilt servo to current elevation angle
        setServoAngle(LEDC_CH_TILT, theta);
        delay(SETTLE_MS);

        // Inner loop: azimuth (PHI) -- alternate sweep direction for efficiency
        // Forward sweep on even rows, backward on odd rows (boustrophedon pattern)
        int row = (int)((theta - THETA_START) / THETA_STEP + 0.5f);
        bool forward = (row % 2 == 0);

        float phi_begin = forward ? PHI_START : PHI_END;
        float phi_stop  = forward ? PHI_END   : PHI_START;
        float phi_inc   = forward ? PHI_STEP  : -PHI_STEP;

        for (float phi = phi_begin;
             forward ? (phi <= phi_stop + 0.01f) : (phi >= phi_stop - 0.01f);
             phi += phi_inc) {

            // Move pan servo to current azimuth angle
            setServoAngle(LEDC_CH_PAN, phi);
            delay(SETTLE_MS);

            // Take averaged LiDAR measurement
            LidarReading result = averageMeasurements(MEASURE_N);

            // Stream result to PC over USB serial
            emitDataLine(phi, theta, result);
        }
    }

    // Signal end of one complete scan
    Serial.println("SCAN_COMPLETE");

    // Return both servos to a safe rest position between scans
    moveToCenter();

    // Wait 2 seconds then repeat (continuous mode for live monitoring)
    delay(2000);
}

// ---------------------------------------------------------------------------
// SERVO PWM INITIALISATION
// ---------------------------------------------------------------------------

/**
 * Attach a GPIO pin to an LEDC channel and configure the 50Hz timer.
 * Called once per servo during setup().
 */
void initServoPWM(int gpio, int channel) {
    // ledcSetup: channel, frequency_Hz, resolution_bits
    ledcSetup(channel, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(gpio, channel);
    // Start at 90 degrees (neutral / centre)
    ledcWrite(channel, angleToDuty(90.0f));
}

/**
 * Convert a servo angle (degrees) to a 16-bit LEDC duty cycle value.
 *
 * Mathematics:
 *   pulse_us  = SERVO_MIN_US + (angle / 180) * (SERVO_MAX_US - SERVO_MIN_US)
 *   duty      = (pulse_us / PERIOD_US) * MAX_DUTY
 *
 *   For SG90 at 50Hz (20000us period):
 *     0   deg => 500us  => duty = (500/20000)*65535  = 1638
 *     90  deg => 1500us => duty = (1500/20000)*65535 = 4915
 *     180 deg => 2500us => duty = (2500/20000)*65535 = 8191
 */
uint32_t angleToDuty(float angle_deg) {
    // Clamp to valid servo range
    if (angle_deg < SERVO_MIN_DEG) angle_deg = SERVO_MIN_DEG;
    if (angle_deg > SERVO_MAX_DEG) angle_deg = SERVO_MAX_DEG;

    float pulse_us = SERVO_MIN_US
                   + (angle_deg - SERVO_MIN_DEG)
                   * (float)(SERVO_MAX_US - SERVO_MIN_US)
                   / (float)(SERVO_MAX_DEG - SERVO_MIN_DEG);

    uint32_t duty = (uint32_t)((pulse_us / (float)LEDC_PERIOD_US) * (float)LEDC_MAX_DUTY);
    return duty;
}

/**
 * Set a servo to a target angle by writing the computed duty to its LEDC channel.
 */
void setServoAngle(int channel, float angle_deg) {
    ledcWrite(channel, angleToDuty(angle_deg));
}

/**
 * Move both servos to a safe centre/rest position.
 * Pan = 90 deg (centre), Tilt = 45 deg (angled gently downward).
 */
void moveToCenter() {
    setServoAngle(LEDC_CH_PAN,  90.0f);
    setServoAngle(LEDC_CH_TILT, 45.0f);
}

// ---------------------------------------------------------------------------
// TF-LUNA LIDAR READING
// ---------------------------------------------------------------------------

/**
 * Read one 9-byte binary frame from the TF-Luna over Serial2.
 *
 * Binary frame structure (9 bytes):
 *   Byte 0 : 0x59  (header byte 1)
 *   Byte 1 : 0x59  (header byte 2)
 *   Byte 2 : Dist_L  (distance low byte, LSB)
 *   Byte 3 : Dist_H  (distance high byte, MSB)
 *   Byte 4 : Strength_L
 *   Byte 5 : Strength_H
 *   Byte 6 : Temp_L
 *   Byte 7 : Temp_H
 *   Byte 8 : Checksum = (sum of bytes 0-7) & 0xFF
 *
 * Distance = Dist_L | (Dist_H << 8)  -- in centimetres
 * Strength = Str_L  | (Str_H  << 8)  -- signal amplitude
 *
 * A strength value below MIN_STRENGTH indicates an unreliable measurement
 * (common on semi-transparent or highly absorptive surfaces like ice).
 */
LidarReading readTFLuna() {
    LidarReading result = {0, 0, false};
    uint8_t frame[TFLUNA_FRAME_SIZE];

    // Timeout: wait up to 50ms for a complete frame
    unsigned long t0 = millis();
    while (millis() - t0 < 50) {
        if (Serial2.available() > 0) {
            // Look for the first header byte
            if (Serial2.read() == TFLUNA_HEADER) {
                // Wait for second header byte
                unsigned long t1 = millis();
                while (Serial2.available() < 1 && millis() - t1 < 10) {}
                if (Serial2.available() < 1) break;
                if (Serial2.read() != TFLUNA_HEADER) continue;

                // Read remaining 7 bytes
                frame[0] = TFLUNA_HEADER;
                frame[1] = TFLUNA_HEADER;
                unsigned long t2 = millis();
                while (Serial2.available() < 7 && millis() - t2 < 20) {}
                if (Serial2.available() < 7) break;
                for (int i = 2; i < TFLUNA_FRAME_SIZE; i++) {
                    frame[i] = (uint8_t)Serial2.read();
                }

                // Validate checksum: sum of bytes 0-7, take lower 8 bits
                uint8_t checksum = 0;
                for (int i = 0; i < TFLUNA_FRAME_SIZE - 1; i++) {
                    checksum += frame[i];
                }
                // Note: uint8_t addition naturally wraps at 256, giving the low byte
                if (checksum != frame[8]) {
                    // Checksum mismatch -- discard frame
                    continue;
                }

                // Extract distance and strength (little-endian 16-bit)
                uint16_t dist     = (uint16_t)frame[2] | ((uint16_t)frame[3] << 8);
                uint16_t strength = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);

                // Quality checks
                if (dist == 0 || dist > MAX_DIST_CM) {
                    result.valid = false;
                    return result;
                }
                if (strength < MIN_STRENGTH) {
                    // Low-strength reading: distance is unreliable, still emit with QUAL:0
                    result.dist_cm  = dist;
                    result.strength = strength;
                    result.valid    = false;
                    return result;
                }

                result.dist_cm  = dist;
                result.strength = strength;
                result.valid    = true;
                return result;
            }
        }
    }
    // Timeout reached -- return invalid reading
    result.valid = false;
    return result;
}

/**
 * Take N readings and return the median-averaged result.
 * Averaging reduces random noise on specular or rough surfaces.
 * Readings marked invalid are skipped; if none are valid, returns invalid.
 */
LidarReading averageMeasurements(int n) {
    // Collect up to n valid readings
    uint32_t dist_sum     = 0;
    uint32_t strength_sum = 0;
    int      valid_count  = 0;

    for (int i = 0; i < n; i++) {
        LidarReading r = readTFLuna();
        if (r.valid) {
            dist_sum     += r.dist_cm;
            strength_sum += r.strength;
            valid_count++;
        }
        delay(10);  // TF-Luna outputs at 100Hz; 10ms guarantees a fresh frame
    }

    LidarReading out = {0, 0, false};
    if (valid_count > 0) {
        out.dist_cm  = (uint16_t)(dist_sum / valid_count);
        out.strength = (uint16_t)(strength_sum / valid_count);
        out.valid    = true;
    }
    return out;
}

// ---------------------------------------------------------------------------
// SERIAL OUTPUT
// ---------------------------------------------------------------------------

/**
 * Emit one data line to USB Serial in ASCII format.
 * Python's visualizer.py reads and parses this exact format.
 *
 * Valid measurement:
 *   PHI:045.00,THETA:030.00,DIST:152,STRENGTH:500,QUAL:1
 *
 * Invalid measurement (still emitted so Python can log the attempt):
 *   PHI:045.00,THETA:030.00,DIST:0,STRENGTH:0,QUAL:0
 */
void emitDataLine(float phi, float theta, const LidarReading& reading) {
    char buf[80];
    snprintf(buf, sizeof(buf),
             "PHI:%06.2f,THETA:%06.2f,DIST:%u,STRENGTH:%u,QUAL:%d",
             phi,
             theta,
             (unsigned)reading.dist_cm,
             (unsigned)reading.strength,
             reading.valid ? 1 : 0);
    Serial.println(buf);
}
