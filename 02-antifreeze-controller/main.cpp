/**
 * ╔══════════════════════════════════════════════════════════════╗
 * ║   ICE STUPA FREEZE PREVENTION SYSTEM v1.1                   ║
 * ║   Platform  : ESP32 (Arduino Framework)                     ║
 * ║   Author    : [Your Name]                                    ║
 * ║   License   : MIT                                           ║
 * ║   Target    : Autonomous pipe freeze detection & mitigation  ║
 * ╚══════════════════════════════════════════════════════════════╝
 *
 *  CHANGELOG v1.0 → v1.1
 *  ─────────────────────────────────────────────────────────────
 *  FIX-01 [CRITICAL]   flushAttemptCount was unconditionally reset inside
 *                      handle_MONITORING(), making CRITICAL_ALERT unreachable.
 *                      The MONITORING → FLUSHING → MONITORING cycle would reset
 *                      the counter every pass, so flushAttemptCount never
 *                      accumulated to MAX_FLUSH_ATTEMPTS (3). Counter now resets
 *                      only on confirmed safe recovery (entry into IDLE).
 *
 *  FIX-02 [CRITICAL]   Entry action guards using (timeInState < 100ms) were
 *                      executing on every loop() iteration for the full 100ms
 *                      window. Since loop() runs in < 1ms between DS18B20 reads,
 *                      this caused:
 *                        - flushAttemptCount++ firing hundreds of times,
 *                          instantly maxing the counter on the first flush.
 *                        - setValve(!valveOpen) toggling the valve hundreds of
 *                          times in VALVE_EXERCISE, stressing the motor/MOSFET.
 *                      Replaced with a stateEntryPending boolean flag set by
 *                      transitionTo() and cleared exactly once per state entry.
 *
 *  FIX-03 [FUNCTIONAL] handle_SENSOR_FAULT() called tempSensor.requestTemperatures()
 *                      on every loop() iteration without an interval guard. This
 *                      is a 750ms synchronous blocking call, creating a continuous
 *                      750ms blocking loop while in SENSOR_FAULT — breaking LED
 *                      blink timing, serial print intervals, and the scheduler.
 *                      Added FAULT_RECOVERY_INTERVAL_MS (5s) rate-limiting guard.
 *
 *  FIX-04 [MINOR]      updateFlowRate() called millis() twice for elapsed_ms
 *                      computation and lastFlowCalcTime assignment, introducing
 *                      a systematic timing skew. Fixed with a single 'now' read.
 *
 *  FIX-05 [MINOR]      FLUSHING progress log used (timeInState % 5000 < 100),
 *                      which fires multiple times per window when loop() runs
 *                      faster than 100ms. Replaced with a static lastFlushPrint
 *                      tracker following the established millis() pattern.
 *
 *  FIX-06 [MINOR]      handle_VALVE_EXERCISE() printed its log message on every
 *                      loop() iteration (outside any guard). Moved inside the
 *                      entry action block so it prints exactly once per exercise.
 *
 *  FIX-07 [MINOR]      VALVE_EXERCISE and SENSOR_FAULT states were absent from
 *                      checkValveExerciseSchedule()'s exclusion list. After 6h
 *                      in SENSOR_FAULT, the scheduler would fire unnecessarily.
 *                      Both states added to the guard.
 */

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ─────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────
#define PIN_TEMP_SENSOR      4
#define PIN_FLOW_SENSOR      18
#define PIN_VALVE            26
#define PIN_LED_STATUS       2
#define PIN_LED_ALERT        27
#define PIN_BUZZER           25

// ─────────────────────────────────────────────
//  SYSTEM CONSTANTS & THRESHOLDS
// ─────────────────────────────────────────────

// Temperature thresholds (°C)
constexpr float    TEMP_SAFE_HIGH            = 4.0f;    // Return to IDLE above this
constexpr float    TEMP_WARN_LOW             = 2.0f;    // Enter MONITORING below this
constexpr float    TEMP_FREEZE_RISK          = 0.5f;    // Enter FLUSHING below this
constexpr float    TEMP_SENSOR_MIN           = -40.0f;  // DS18B20 valid range floor
constexpr float    TEMP_SENSOR_MAX           = 85.0f;   // DS18B20 valid range ceiling

// Flow thresholds
constexpr float    FLOW_STALL_FACTOR         = 0.40f;   // 40% drop from baseline triggers fault
constexpr uint32_t FLOW_PULSE_TIMEOUT_MS     = 3000;    // Max silence between pulses → near-zero

// Timing constants (ms)
constexpr uint32_t TEMP_SAMPLE_INTERVAL_MS   = 2000;    // Period between temperature reads
constexpr uint32_t FLOW_CALC_INTERVAL_MS     = 5000;    // Flow calculation window
constexpr uint32_t FLUSH_DURATION_MS         = 30000;   // Max time per flush attempt
constexpr uint32_t MIN_FLUSH_DWELL_MS        = 5000;    // Min dwell before evaluating flush result
constexpr uint32_t VALVE_EXERCISE_INTERVAL_MS = 21600000UL; // 6 hours between valve exercise
constexpr uint32_t SAFE_CONFIRM_DURATION_MS  = 60000;   // Must hold safe temp for 60s → IDLE
constexpr uint32_t FAULT_RECOVERY_INTERVAL_MS = 5000;   // FIX-03: sensor re-poll rate in FAULT

// Fault tolerances
constexpr uint8_t  MAX_CRC_FAILURES          = 3;       // CRC failures before SENSOR_FAULT
constexpr uint8_t  MAX_FLUSH_ATTEMPTS        = 3;       // Flush attempts before CRITICAL_ALERT
constexpr uint8_t  TEMP_MEDIAN_SAMPLES       = 3;       // Median filter depth

// ─────────────────────────────────────────────
//  STATE MACHINE DEFINITION
// ─────────────────────────────────────────────
enum class SystemState : uint8_t {
    BOOT            = 0,
    SELF_TEST       = 1,
    IDLE            = 2,
    MONITORING      = 3,
    FLUSHING        = 4,
    VALVE_EXERCISE  = 5,
    SENSOR_FAULT    = 6,
    CRITICAL_ALERT  = 7
};

const char* STATE_NAMES[] = {
    "BOOT", "SELF_TEST", "IDLE", "MONITORING",
    "FLUSHING", "VALVE_EXERCISE", "SENSOR_FAULT", "CRITICAL_ALERT"
};

// ─────────────────────────────────────────────
//  HARDWARE OBJECTS
// ─────────────────────────────────────────────
OneWire           oneWire(PIN_TEMP_SENSOR);
DallasTemperature tempSensor(&oneWire);

// ─────────────────────────────────────────────
//  GLOBAL STATE VARIABLES
// ─────────────────────────────────────────────
SystemState currentState   = SystemState::BOOT;
SystemState previousState  = SystemState::BOOT;

// FIX-02: Entry flag — set by transitionTo(), cleared once per handler entry block.
// Guarantees entry actions execute exactly once regardless of loop() speed.
bool stateEntryPending     = true;

// Sensor data
volatile uint32_t flowPulseCount   = 0;     // Incremented by ISR — must be volatile
volatile uint32_t lastPulseTime_ms = 0;     // Timestamp of last pulse — must be volatile
float    currentTemp_C             = 25.0f;
float    flowRate_Lpm              = 0.0f;
float    baselineFlowRate_Lpm      = 0.0f;
bool     baselineEstablished       = false;

// Timing trackers (millis-based, non-blocking)
uint32_t lastTempSampleTime        = 0;
uint32_t lastFlowCalcTime          = 0;
uint32_t stateEntryTime            = 0;
uint32_t lastValveExerciseTime     = 0;
uint32_t safeConditionStartTime    = 0;
bool     safeConditionActive       = false;

// Fault counters
uint8_t  crcFailCount              = 0;
// FIX-01: flushAttemptCount persists across MONITORING↔FLUSHING cycles.
//         It is only reset when a confirmed safe recovery reaches IDLE.
uint8_t  flushAttemptCount         = 0;

// Valve state
bool     valveOpen                 = false;

// Median filter ring buffer
float    tempBuffer[TEMP_MEDIAN_SAMPLES] = {25.0f, 25.0f, 25.0f};
uint8_t  tempBufferIdx             = 0;

// ─────────────────────────────────────────────
//  ISR — FLOW SENSOR PULSE
//  Attached to GPIO18 (hardware interrupt)
//  IRAM_ATTR: ensures ISR is placed in IRAM,
//  safe to call even when flash cache is busy.
//  YF-S201 calibration: ~450 pulses per litre
// ─────────────────────────────────────────────
void IRAM_ATTR flowPulseISR() {
    flowPulseCount++;
    lastPulseTime_ms = millis(); // millis() is ISR-safe on ESP32 (hardware timer)
}

// ─────────────────────────────────────────────
//  UTILITY FUNCTIONS
// ─────────────────────────────────────────────

void setValve(bool open) {
    valveOpen = open;
    digitalWrite(PIN_VALVE, open ? HIGH : LOW);
    Serial.printf("[VALVE] → %s\n", open ? "OPEN" : "CLOSED");
}

// FIX-02: transitionTo() now sets stateEntryPending = true.
// Each handler clears this flag in its entry block, guaranteeing
// entry actions run exactly once per state entry.
void transitionTo(SystemState newState) {
    if (newState == currentState) return;

    Serial.printf("[FSM] %s → %s\n",
        STATE_NAMES[(uint8_t)currentState],
        STATE_NAMES[(uint8_t)newState]);

    previousState     = currentState;
    currentState      = newState;
    stateEntryTime    = millis();
    stateEntryPending = true;  // FIX-02
}

// Sorting-network median for N=3 (branchless-friendly, MCU-efficient)
float medianOf3(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b; // a ≤ b ≤ c after all three conditional swaps
}

// ─────────────────────────────────────────────
//  SENSOR READING — TEMPERATURE (Non-blocking interval)
//
//  Note: tempSensor.requestTemperatures() is synchronous and blocks
//  for ~750ms at 12-bit resolution. This is an acknowledged design
//  simplification for v1.x. A fully non-blocking implementation would
//  call setWaitForConversion(false), requestTemperatures(), return, then
//  read the result 750ms later via a separate millis() timer.
// ─────────────────────────────────────────────
bool updateTemperature() {
    if (millis() - lastTempSampleTime < TEMP_SAMPLE_INTERVAL_MS) {
        return false; // Not time yet — return immediately, never blocks
    }
    lastTempSampleTime = millis();

    tempSensor.requestTemperatures(); // ~750ms synchronous conversion

    float raw = tempSensor.getTempCByIndex(0);

    // Validate: reject disconnected-device sentinel and out-of-range values.
    // DEVICE_DISCONNECTED_C == -127.0f, which also fails the range check,
    // but the explicit comparison documents intent clearly.
    if (raw == DEVICE_DISCONNECTED_C || raw < TEMP_SENSOR_MIN || raw > TEMP_SENSOR_MAX) {
        crcFailCount++;
        Serial.printf("[TEMP] Invalid read #%d (raw=%.2f) — rejecting sample.\n",
            crcFailCount, raw);
        return false;
    }

    crcFailCount = 0; // Any valid read resets the fault counter

    // Push into circular median filter buffer
    tempBuffer[tempBufferIdx] = raw;
    tempBufferIdx = (tempBufferIdx + 1) % TEMP_MEDIAN_SAMPLES;

    // Apply median filter to reject transient spike outliers
    currentTemp_C = medianOf3(tempBuffer[0], tempBuffer[1], tempBuffer[2]);

    Serial.printf("[TEMP] %.2f°C (raw: %.2f)\n", currentTemp_C, raw);
    return true;
}

// ─────────────────────────────────────────────
//  SENSOR READING — FLOW RATE (Non-blocking interval)
// ─────────────────────────────────────────────
void updateFlowRate() {
    if (millis() - lastFlowCalcTime < FLOW_CALC_INTERVAL_MS) return;

    // FIX-04: Single millis() snapshot prevents timing skew between
    // elapsed_ms computation and lastFlowCalcTime assignment.
    uint32_t now        = millis();
    uint32_t elapsed_ms = now - lastFlowCalcTime;
    lastFlowCalcTime    = now;

    // Atomic read of ISR-modified counter — briefly disable interrupts
    noInterrupts();
    uint32_t pulses = flowPulseCount;
    flowPulseCount  = 0;
    interrupts();

    // YF-S201 calibration formula:
    //   Sensor outputs F Hz where F ≈ 7.5 × Q (Q in L/min)
    //   → Q = (pulses / elapsed_s) / 7.5
    float pulsesPerSec = (float)pulses / (elapsed_ms / 1000.0f);
    flowRate_Lpm       = pulsesPerSec / 7.5f;

    // YF-S201 turbine stalls below ~0.3 L/min and reports zero even with
    // a slow trickle. Use pulse timeout to distinguish stall from true zero.
    if ((millis() - lastPulseTime_ms > FLOW_PULSE_TIMEOUT_MS) && (pulses == 0)) {
        flowRate_Lpm = 0.0f;
    }

    Serial.printf("[FLOW] %.3f L/min (%lu pulses in %lu ms)\n",
        flowRate_Lpm, pulses, elapsed_ms);

    // Establish baseline only once, under known-good warm conditions
    if (!baselineEstablished && currentTemp_C > TEMP_SAFE_HIGH && flowRate_Lpm > 0.1f) {
        baselineFlowRate_Lpm = flowRate_Lpm;
        baselineEstablished  = true;
        Serial.printf("[FLOW] Baseline established: %.3f L/min\n", baselineFlowRate_Lpm);
    }
}

// ─────────────────────────────────────────────
//  STATE HANDLERS
// ─────────────────────────────────────────────

// ── SELF_TEST ──────────────────────────────────────────────────
// Uses delay() intentionally — runs exactly once at boot.
// The stateEntryPending guard ensures it cannot re-execute if
// the state is somehow re-entered.
void handle_SELF_TEST() {
    if (!stateEntryPending) return; // Guard: run body only once
    stateEntryPending = false;

    Serial.println("[SELF_TEST] Starting sensor verification...");

    // ── DS18B20 Probe Test ──
    tempSensor.requestTemperatures();
    float t = tempSensor.getTempCByIndex(0);
    bool tempOK = (t != DEVICE_DISCONNECTED_C &&
                   t > TEMP_SENSOR_MIN &&
                   t < TEMP_SENSOR_MAX);
    Serial.printf("[SELF_TEST] DS18B20: %s (%.2f°C)\n", tempOK ? "PASS" : "FAIL", t);

    // ── Valve Actuation Test ──
    Serial.println("[SELF_TEST] Valve check — cycling OPEN → CLOSE → OPEN");
    setValve(true);
    delay(3000); // Allow full valve travel (~3–5s)
    setValve(false);
    delay(3000);
    setValve(true); // Leave OPEN — fail-safe default

    // ── Output Indicator Test (LED alternation) ──
    for (int i = 0; i < 5; i++) {
        digitalWrite(PIN_LED_STATUS, HIGH);
        digitalWrite(PIN_LED_ALERT,  LOW);
        delay(200);
        digitalWrite(PIN_LED_STATUS, LOW);
        digitalWrite(PIN_LED_ALERT,  HIGH);
        delay(200);
    }
    digitalWrite(PIN_LED_ALERT, LOW);

    // ── Self-test Result ──
    if (!tempOK) {
        Serial.println("[SELF_TEST] CRITICAL — DS18B20 not detected. → SENSOR_FAULT");
        transitionTo(SystemState::SENSOR_FAULT);
        return; // Early return prevents fall-through to IDLE transition
    }

    Serial.println("[SELF_TEST] All checks passed. → IDLE");
    transitionTo(SystemState::IDLE);
}

// ── IDLE ───────────────────────────────────────────────────────
void handle_IDLE() {
    // FIX-01: Entry action — reset flush counter only on confirmed recovery.
    // This is the ONLY place flushAttemptCount is reset.
    if (stateEntryPending) {
        stateEntryPending = false;
        flushAttemptCount = 0;
        Serial.println("[IDLE] Entered IDLE — flush attempt counter reset.");
    }

    static uint32_t lastIdlePrint = 0;
    if (millis() - lastIdlePrint > 10000) {
        lastIdlePrint = millis();
        Serial.printf("[IDLE] Temp: %.2f°C | Flow: %.3f L/min\n",
            currentTemp_C, flowRate_Lpm);
    }

    if (currentTemp_C < TEMP_WARN_LOW) {
        Serial.printf("[IDLE] Temp %.2f°C below warning threshold %.1f°C. → MONITORING\n",
            currentTemp_C, TEMP_WARN_LOW);
        transitionTo(SystemState::MONITORING);
    }
}

// ── MONITORING ────────────────────────────────────────────────
void handle_MONITORING() {
    // Entry action: reset safe-condition timer to prevent stale state carry-over
    if (stateEntryPending) {
        stateEntryPending   = false;
        safeConditionActive = false;
        Serial.println("[MONITORING] Entered heightened monitoring. Watching temperature and flow.");
    }

    static uint32_t lastMonPrint = 0;
    if (millis() - lastMonPrint > 3000) {
        lastMonPrint = millis();
        Serial.printf("[MONITORING] Temp: %.2f°C | Flow: %.3f L/min | Baseline: %.3f L/min\n",
            currentTemp_C, flowRate_Lpm, baselineFlowRate_Lpm);
        digitalWrite(PIN_LED_STATUS, !digitalRead(PIN_LED_STATUS)); // Blink: ~once per 3s
    }

    // ── Fault escalation ──
    if (crcFailCount >= MAX_CRC_FAILURES) {
        transitionTo(SystemState::SENSOR_FAULT);
        return;
    }

    // ── Freeze detection (dual-modality trigger) ──
    bool tempCritical = (currentTemp_C < TEMP_FREEZE_RISK);
    bool flowDropped  = baselineEstablished &&
                        (flowRate_Lpm < baselineFlowRate_Lpm * (1.0f - FLOW_STALL_FACTOR));

    if (tempCritical || flowDropped) {
        Serial.printf("[MONITORING] *** FREEZE TRIGGER *** tempCritical=%d flowDropped=%d | "
                      "Flush attempt %d will be attempt #%d of %d\n",
            tempCritical, flowDropped,
            flushAttemptCount + 1, flushAttemptCount + 1, MAX_FLUSH_ATTEMPTS);

        // FIX-01: Do NOT reset flushAttemptCount here.
        // The counter must persist across MONITORING↔FLUSHING cycles
        // within the same freeze event to allow CRITICAL_ALERT escalation.
        transitionTo(SystemState::FLUSHING);
        return;
    }

    // ── Safe recovery detection (hysteresis + confirmation timer) ──
    if (currentTemp_C > TEMP_SAFE_HIGH) {
        if (!safeConditionActive) {
            safeConditionActive    = true;
            safeConditionStartTime = millis();
            Serial.println("[MONITORING] Safe temperature detected. Confirmation timer started (60s).");
        } else if (millis() - safeConditionStartTime > SAFE_CONFIRM_DURATION_MS) {
            Serial.println("[MONITORING] Safe conditions confirmed for 60s. → IDLE");
            safeConditionActive = false;
            transitionTo(SystemState::IDLE);
        }
    } else {
        // Temperature dropped again — reset confirmation timer
        if (safeConditionActive) {
            safeConditionActive = false;
            Serial.println("[MONITORING] Temperature dropped below safe threshold. Confirmation timer reset.");
        }
    }
}

// ── FLUSHING ──────────────────────────────────────────────────
void handle_FLUSHING() {
    // FIX-02: Entry action executes exactly once per FLUSHING entry.
    // flushAttemptCount is now correctly incremented once per attempt.
    if (stateEntryPending) {
        stateEntryPending = false;
        flushAttemptCount++;
        Serial.printf("[FLUSHING] *** Flush attempt %d / %d — Opening valve. ***\n",
            flushAttemptCount, MAX_FLUSH_ATTEMPTS);
        setValve(true);
        digitalWrite(PIN_LED_ALERT, HIGH);
        tone(PIN_BUZZER, 1000, 500); // 500ms alert beep at 1kHz
    }

    uint32_t timeInState = millis() - stateEntryTime;

    // FIX-05: Static tracker replaces unreliable modulo-based logging.
    // Guarantees exactly one print per 5-second window regardless of loop speed.
    static uint32_t lastFlushPrint = 0;
    if (millis() - lastFlushPrint >= 5000) {
        lastFlushPrint = millis();
        Serial.printf("[FLUSHING] t+%lus | Temp: %.2f°C | Flow: %.3f L/min\n",
            timeInState / 1000, currentTemp_C, flowRate_Lpm);
    }

    // Minimum dwell — do not evaluate success/failure prematurely
    if (timeInState < MIN_FLUSH_DWELL_MS) return;

    // ── Success condition ──
    // Temperature rising above freeze risk threshold AND flow recovering
    bool tempRising    = (currentTemp_C > TEMP_FREEZE_RISK + 0.5f);
    bool flowRecovered = !baselineEstablished ||
                         (flowRate_Lpm > baselineFlowRate_Lpm * 0.6f);

    if (tempRising && flowRecovered) {
        Serial.println("[FLUSHING] Success — temperature rising and flow recovering. → MONITORING");
        digitalWrite(PIN_LED_ALERT, LOW);
        transitionTo(SystemState::MONITORING);
        return;
    }

    // ── Flush timeout: escalate or retry ──
    if (timeInState > FLUSH_DURATION_MS) {
        if (flushAttemptCount >= MAX_FLUSH_ATTEMPTS) {
            // FIX-01: This branch is now reachable.
            // flushAttemptCount correctly accumulated across cycles.
            Serial.printf("[FLUSHING] MAX attempts (%d) reached without recovery. → CRITICAL_ALERT\n",
                MAX_FLUSH_ATTEMPTS);
            transitionTo(SystemState::CRITICAL_ALERT);
        } else {
            Serial.printf("[FLUSHING] Cycle timed out without recovery (%d/%d attempts). → MONITORING\n",
                flushAttemptCount, MAX_FLUSH_ATTEMPTS);
            transitionTo(SystemState::MONITORING);
        }
    }
}

// ── VALVE_EXERCISE ────────────────────────────────────────────
void handle_VALVE_EXERCISE() {
    // FIX-02 + FIX-06: Entry action fires once. Serial print moved inside guard.
    // The valve toggles exactly once per exercise cycle.
    if (stateEntryPending) {
        stateEntryPending = false;
        Serial.println("[EXERCISE] Scheduled valve maintenance cycle — toggling actuator.");
        setValve(!valveOpen); // Toggle: prevents actuator grease from seizing
        Serial.printf("[EXERCISE] Valve toggled to: %s\n", valveOpen ? "OPEN" : "CLOSED");
    }

    // After 5s dwell, restore to OPEN (fail-safe) and return
    if (millis() - stateEntryTime > 5000) {
        setValve(true);
        lastValveExerciseTime = millis();
        Serial.println("[EXERCISE] Complete. Valve restored to OPEN. Returning to previous state.");
        transitionTo(previousState);
    }
}

// ── SENSOR_FAULT ──────────────────────────────────────────────
void handle_SENSOR_FAULT() {
    // Entry action: log fault entry and guarantee valve is open
    if (stateEntryPending) {
        stateEntryPending = false;
        Serial.printf("[SENSOR_FAULT] DS18B20 CRC failure threshold reached (%d errors). "
                      "Valve held OPEN as failsafe.\n", crcFailCount);
        setValve(true); // Failsafe: flowing water cannot freeze
    }

    // Redundant valve safety re-check (in case of spurious closure)
    if (!valveOpen) setValve(true);

    // Rapid LED blink: ~4Hz (250ms half-period) as visual fault indicator
    digitalWrite(PIN_LED_ALERT, (millis() / 250) % 2);

    // Periodic fault log and audible alert
    static uint32_t lastFaultPrint = 0;
    if (millis() - lastFaultPrint >= 5000) {
        lastFaultPrint = millis();
        Serial.printf("[SENSOR_FAULT] Awaiting sensor recovery. CRC errors: %d. "
                      "Valve: OPEN.\n", crcFailCount);
        tone(PIN_BUZZER, 500, 200); // Low-frequency alert beep
    }

    // FIX-03: Rate-limited recovery poll.
    // Was calling requestTemperatures() every loop() iteration — a 750ms
    // blocking call — making this an effective 750ms infinite blocking loop.
    // Now polls at FAULT_RECOVERY_INTERVAL_MS (5s).
    static uint32_t lastFaultRecoveryCheck = 0;
    if (millis() - lastFaultRecoveryCheck >= FAULT_RECOVERY_INTERVAL_MS) {
        lastFaultRecoveryCheck = millis();

        tempSensor.requestTemperatures();
        float t = tempSensor.getTempCByIndex(0);

        if (t != DEVICE_DISCONNECTED_C && t > TEMP_SENSOR_MIN && t < TEMP_SENSOR_MAX) {
            crcFailCount = 0;
            Serial.printf("[SENSOR_FAULT] Sensor recovered (%.2f°C). → IDLE\n", t);
            transitionTo(SystemState::IDLE);
        }
    }
}

// ── CRITICAL_ALERT ────────────────────────────────────────────
void handle_CRITICAL_ALERT() {
    // Entry action: log failure snapshot and open valve
    if (stateEntryPending) {
        stateEntryPending = false;
        Serial.println("[CRITICAL_ALERT] ╔══════════════════════════════════╗");
        Serial.println("[CRITICAL_ALERT] ║  SYSTEM FAILURE — ALL FLUSH      ║");
        Serial.println("[CRITICAL_ALERT] ║  ATTEMPTS EXHAUSTED              ║");
        Serial.println("[CRITICAL_ALERT] ║  Manual intervention required.   ║");
        Serial.println("[CRITICAL_ALERT] ╚══════════════════════════════════╝");
        Serial.printf("[CRITICAL_ALERT] Snapshot — Temp: %.2f°C | Flow: %.3f L/min | "
                      "Attempts: %d\n", currentTemp_C, flowRate_Lpm, flushAttemptCount);
        if (!valveOpen) setValve(true); // Never leave valve closed in failure mode
        // Future: trigger MQTT alert, GSM SMS, or LoRa uplink here
    }

    // Periodic status log
    static uint32_t lastAlertPrint = 0;
    if (millis() - lastAlertPrint >= 2000) {
        lastAlertPrint = millis();
        Serial.printf("[CRITICAL_ALERT] AWAITING MANUAL RESET — Temp: %.2f°C | "
                      "Flow: %.3f L/min\n", currentTemp_C, flowRate_Lpm);
    }

    // Redundant valve safety re-check
    if (!valveOpen) setValve(true);

    // SOS-style LED pattern (· · · — — — within a 3s cycle)
    // Encodes 'S' (3 short) + 'O' (3 long) as a recognisable distress signal
    uint32_t t  = millis() % 3000;
    bool ledOn  = (t < 200)                        // S ·
               || (t > 300  && t < 500)            // S ·
               || (t > 600  && t < 800)            // S ·
               || (t > 1000 && t < 1400)           // O —
               || (t > 1500 && t < 1900)           // O —
               || (t > 2000 && t < 2400);          // O —
    digitalWrite(PIN_LED_ALERT, ledOn);
}

// ─────────────────────────────────────────────
//  SCHEDULED TASKS (Non-blocking)
// ─────────────────────────────────────────────
void checkValveExerciseSchedule() {
    // FIX-07: VALVE_EXERCISE and SENSOR_FAULT added to exclusion list.
    // Prevents scheduler from firing while already exercising, or while
    // the sensor is actively faulted (would cause meaningless state churn).
    if (currentState == SystemState::FLUSHING       ||
        currentState == SystemState::SELF_TEST      ||
        currentState == SystemState::VALVE_EXERCISE ||  // FIX-07
        currentState == SystemState::SENSOR_FAULT   ||  // FIX-07
        currentState == SystemState::CRITICAL_ALERT) return;

    if (millis() - lastValveExerciseTime > VALVE_EXERCISE_INTERVAL_MS) {
        Serial.println("[SCHEDULER] Valve exercise interval reached. → VALVE_EXERCISE");
        transitionTo(SystemState::VALVE_EXERCISE);
    }
}

// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500); // Ensure USB-Serial is ready before first print
    Serial.println("\n╔══════════════════════════════════════╗");
    Serial.println("║  ICE STUPA FREEZE PREVENTION v1.1   ║");
    Serial.println("╚══════════════════════════════════════╝");

    // Configure GPIO directions
    pinMode(PIN_VALVE,       OUTPUT);
    pinMode(PIN_LED_STATUS,  OUTPUT);
    pinMode(PIN_LED_ALERT,   OUTPUT);
    pinMode(PIN_BUZZER,      OUTPUT);
    pinMode(PIN_FLOW_SENSOR, INPUT_PULLUP); // Open-drain YF-S201 pulled to 3.3V internally

    // Safe power-on default: valve OPEN.
    // A flowing pipe cannot freeze. Always default to safety.
    setValve(true);

    // Attach hardware interrupt to flow sensor
    // FALLING edge: hall-effect sensor pulls signal LOW on each impeller blade pass
    attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR), flowPulseISR, FALLING);

    // Initialise 1-Wire temperature sensor
    tempSensor.begin();
    uint8_t deviceCount = tempSensor.getDeviceCount();
    Serial.printf("[INIT] DS18B20 devices detected on 1-Wire bus: %d\n", deviceCount);

    // 12-bit resolution: 0.0625°C precision, 750ms conversion time
    tempSensor.setResolution(12);

    // Begin state machine — SELF_TEST runs on first loop() iteration
    transitionTo(SystemState::SELF_TEST);
}

// ─────────────────────────────────────────────
//  MAIN LOOP — Non-blocking cooperative scheduler
//
//  Execution order per iteration:
//    1. Sensor updates (always run — non-blocking via millis() guards)
//    2. Scheduled maintenance check
//    3. Current state handler dispatch
//
//  Note: updateTemperature() blocks for ~750ms at the 2s interval
//  boundary. All other paths return in < 1ms.
// ─────────────────────────────────────────────
void loop() {
    // ── Always-run sensor updates ──
    updateTemperature();    // Non-blocking except at 2s interval (750ms DS18B20 read)
    updateFlowRate();       // Non-blocking (ISR-based counting, fast math)

    // ── Scheduled maintenance ──
    checkValveExerciseSchedule();

    // ── State machine dispatch ──
    switch (currentState) {
        case SystemState::SELF_TEST:      handle_SELF_TEST();       break;
        case SystemState::IDLE:           handle_IDLE();            break;
        case SystemState::MONITORING:     handle_MONITORING();      break;
        case SystemState::FLUSHING:       handle_FLUSHING();        break;
        case SystemState::VALVE_EXERCISE: handle_VALVE_EXERCISE();  break;
        case SystemState::SENSOR_FAULT:   handle_SENSOR_FAULT();    break;
        case SystemState::CRITICAL_ALERT: handle_CRITICAL_ALERT();  break;
        default: break;
    }
}
