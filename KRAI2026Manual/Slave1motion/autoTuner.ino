// ============================================================
// AUTO-TUNER for PID (3-Phase: Deadband -> Kf -> PI)
// Adapted for Slave1motion with exact physical modelling
// ============================================================

#include "autoTuner.h"
#include <Preferences.h>

// ============================================================
// Internal constants
// ============================================================

namespace AutoTunerNS {
    constexpr uint32_t kPidTickMs = 40;
    constexpr uint32_t kQuickLogThresholdMs = 500;
    constexpr float kTargetRpm = AUTOTUNE_TARGET_RPM;
    constexpr float kSteadyStateStartFraction = 0.60f;
    constexpr float kBurstThreshold = 1.30f;
    constexpr float kBestScoreInitial = 999999.0f;
    constexpr float kBestScoreSavedCutoff = 999998.0f;
    constexpr float kSteadyStateMinSamples = 10.0f;

    // Measurement phases
    constexpr uint32_t DEADBAND_STEP_MS = 20;
    constexpr float KF_PWM_FRACTION = 0.5f;
    constexpr uint32_t KF_RUN_MS = 2000;
    constexpr float KF_MIN_RPM = 10.0f;

    // Emulasi beban tanjakan — potong PWM command (bukan feedback RPM).
    // ponytail: LOAD_INJECT_PWM = cap; fraksi dari PWM saat ini biar gak nolkan motor di awal load.
    constexpr int    LOAD_INJECT_PWM       = 400;
    constexpr float  LOAD_INJECT_FRACTION  = 0.45f;
    constexpr uint32_t LOAD_RAMP_MS        = 1500;  // naik gradual setelah 60%, integral sempat react

    constexpr float HIGH_OVERSHOOT_PCT = 10.0f;
    constexpr float MEDIUM_OVERSHOOT_PCT = 5.0f;
    constexpr float LOW_OVERSHOOT_PCT = 3.0f;
    constexpr uint32_t SLOW_RISE_MS = 3000UL;
    constexpr uint32_t MEDIUM_RISE_MS = 1500UL;
    constexpr float AVG_ERR_EXCELLENT = 1.5f;
    constexpr float AVG_ERR_GOOD = 3.0f;
    constexpr float EXCELLENT_SCORE = 20.0f;
    constexpr float GOOD_SCORE = 35.0f;

    constexpr float W_ERROR = 10.0f;
    constexpr float W_OVERSHOOT = 5.0f;
    constexpr float W_RISE_TIME = 0.05f;
    constexpr float W_STABILITY = 5.0f;
    constexpr float W_BURST = 0.6f;

    constexpr float KP_COARSE = 5.0f, KP_FINE = 1.0f, KP_ULTRA = 0.3f;
    constexpr float KI_COARSE = 1.2f, KI_FINE = 0.8f, KI_ULTRA = 0.4f;
    constexpr int STAGE_COARSE_CYCLES = 6;
    constexpr int STAGE_FINE_CYCLES = 5;
    constexpr int STAGNATION_LIMIT = 3;
}

enum class Precision : uint8_t { COARSE = 0, FINE, ULTRA_FINE };

struct CycleMetrics {
    float total_err;
    int samples;
    float peak_rpm;
    bool rise_10_done, rise_90_done;
    uint32_t rise_10_t, rise_time_ms;
    bool burst_detected;
    float burst_rpm;
    uint32_t burst_t_ms;
    float ss_mean, ss_M2;
    int ss_samples;

    void reset() {
        total_err = 0.0f; samples = 0; peak_rpm = 0.0f;
        rise_10_done = false; rise_90_done = false;
        rise_10_t = 0; rise_time_ms = 0;
        burst_detected = false; burst_rpm = 0.0f; burst_t_ms = 0;
        ss_mean = 0.0f; ss_M2 = 0.0f; ss_samples = 0;
    }

    float avg_err() const { return (samples > 0) ? total_err / samples : 0.0f; }
    float overshoot_pct() const {
        float over = peak_rpm - AutoTunerNS::kTargetRpm;
        return (over > 0.0f) ? (over / AutoTunerNS::kTargetRpm * 100.0f) : 0.0f;
    }
    float ss_variance() const { return (ss_samples > 1) ? ss_M2 / ss_samples : 0.0f; }
    float ss_error() const { return fabsf(ss_mean - AutoTunerNS::kTargetRpm); }
    bool hasLoadWindow() const { return ss_samples >= (int)AutoTunerNS::kSteadyStateMinSamples; }
};

static CycleMetrics metrics;

// Error untuk adjust: ss_error (fase beban 60%+) dominan — avg_err sendirian misleading.
static float effectiveErr() {
    if (metrics.hasLoadWindow()) {
        return fmaxf(metrics.avg_err(), metrics.ss_error());
    }
    return metrics.avg_err();
}

static int computeLoadCut(int pwm, uint32_t elapsedMs) {
    const uint32_t loadStartMs = (uint32_t)(AUTOTUNE_RUN_MS * AutoTunerNS::kSteadyStateStartFraction);
    if (elapsedMs < loadStartMs || pwm == 0) return 0;

    const uint32_t sinceLoadMs = elapsedMs - loadStartMs;
    float ramp = 1.0f;
    if (sinceLoadMs < AutoTunerNS::LOAD_RAMP_MS) {
        ramp = (float)sinceLoadMs / (float)AutoTunerNS::LOAD_RAMP_MS;
    }

    const int pwmAbs = abs(pwm);
    int maxCut = (int)min((float)AutoTunerNS::LOAD_INJECT_PWM,
                          (float)pwmAbs * AutoTunerNS::LOAD_INJECT_FRACTION);
    maxCut = (int)((float)maxCut * ramp);
    return maxCut;
}

static int applyLoadInject(int pwm, uint32_t elapsedMs) {
    const int loadCut = computeLoadCut(pwm, elapsedMs);
    if (loadCut <= 0) return pwm;

    int applied = pwm;
    if (pwm > 0) applied = pwm - loadCut;
    else if (pwm < 0) applied = pwm + loadCut;

    return (int)constrain((float)applied, (float)PWM_MIN, (float)PWM_MAX);
}

enum class ATState : uint8_t {
    IDLE = 0,
    WAIT_RELEASE,
    MOTOR_INIT,
    DEADBAND_START,
    DEADBAND_WAIT,
    KF_START,
    KF_WAIT,
    CYCLE_START,
    CYCLE_RUN,
    CYCLE_FINISH,
    CYCLE_COOLDOWN,
    MOTOR_SHOW,
    DONE,
};

static ATState state = ATState::IDLE;
static int tMotorIdx = 0;
static int targetMotor = -1;
static bool singleMode = false;
static bool aborted = false;
static bool oledDrawn = false;

static float curKp, curKi, baseKf, baseDeadband;
static float bestKp, bestKi;
static float bestScore, currentScore;

static uint32_t stateStartMs = 0;
static uint32_t lastPidTickMs = 0;
static uint32_t lastOledMs = 0;
static int cycleCount = 0;

static int deadbandPwm = 0;
static uint32_t lastDeadbandStepMs = 0;

static Precision precision = Precision::COARSE;
static float kpStep, kiStep;
static int stageCount = 0;
static int noImprove = 0;

// ============================================================
// Precision stage management
// ============================================================

static void precisionInit() {
    precision = Precision::COARSE;
    kpStep = AutoTunerNS::KP_COARSE;
    kiStep = AutoTunerNS::KI_COARSE;
    stageCount = 0;
    noImprove = 0;
}

static void precisionAdvance() {
    curKp = bestKp;
    curKi = bestKi;
    if (precision == Precision::COARSE) {
        precision = Precision::FINE;
        kpStep = AutoTunerNS::KP_FINE;
        kiStep = AutoTunerNS::KI_FINE;
    } else if (precision == Precision::FINE) {
        precision = Precision::ULTRA_FINE;
        kpStep = AutoTunerNS::KP_ULTRA;
        kiStep = AutoTunerNS::KI_ULTRA;
    }
    stageCount = 0;
    noImprove = 0;
}

static bool precisionShouldAdvance() {
    stageCount++;
    if (precision == Precision::ULTRA_FINE) return false;
    if (currentScore < AutoTunerNS::EXCELLENT_SCORE) return true;
    if (precision == Precision::COARSE && (stageCount >= AutoTunerNS::STAGE_COARSE_CYCLES || noImprove >= AutoTunerNS::STAGNATION_LIMIT)) return true;
    if (precision == Precision::FINE && (stageCount >= AutoTunerNS::STAGE_FINE_CYCLES || noImprove >= AutoTunerNS::STAGNATION_LIMIT)) return true;
    return false;
}

static const char* precisionName() {
    if (precision == Precision::COARSE) return "COARSE";
    if (precision == Precision::FINE) return "FINE";
    return "ULTRA";
}

// ============================================================
// Scoring function
// ============================================================

static float calculateScore() {
    float avgErr = metrics.avg_err();
    float over = metrics.overshoot_pct();
    uint32_t rise = metrics.rise_time_ms;
    float score = 0.0f;

    score += avgErr * avgErr * AutoTunerNS::W_ERROR;

    if (metrics.ss_samples >= (int)AutoTunerNS::kSteadyStateMinSamples) {
        float ssErr = metrics.ss_error();
        score += ssErr * ssErr * 18.0f;
    }
    if (metrics.ss_samples >= (int)AutoTunerNS::kSteadyStateMinSamples) {
        float var = metrics.ss_variance();
        score += var * 4.0f;
        if (var < 1.0f) score -= 10.0f;
    }

    if (over > AutoTunerNS::HIGH_OVERSHOOT_PCT)
        score += over * over * AutoTunerNS::W_OVERSHOOT * 2.0f;
    else if (over > AutoTunerNS::MEDIUM_OVERSHOOT_PCT)
        score += over * over * AutoTunerNS::W_OVERSHOOT;
    else
        score += over * AutoTunerNS::W_OVERSHOOT * 0.5f;

    if (rise > AutoTunerNS::SLOW_RISE_MS)
        score += (float)(rise - AutoTunerNS::SLOW_RISE_MS) * AutoTunerNS::W_RISE_TIME * 1.5f;
    else if (rise > AutoTunerNS::MEDIUM_RISE_MS)
        score += (float)(rise - AutoTunerNS::MEDIUM_RISE_MS) * AutoTunerNS::W_RISE_TIME;

    if (metrics.samples > 50) {
        if (avgErr < AutoTunerNS::AVG_ERR_EXCELLENT) score -= 8.0f * AutoTunerNS::W_STABILITY;
        else if (avgErr < AutoTunerNS::AVG_ERR_GOOD) score -= 4.0f * AutoTunerNS::W_STABILITY;
    }

    if (rise < AutoTunerNS::MEDIUM_RISE_MS && over < AutoTunerNS::MEDIUM_OVERSHOOT_PCT && avgErr < 2.5f)
        score -= 15.0f;
    else if (rise < AutoTunerNS::SLOW_RISE_MS && over < AutoTunerNS::HIGH_OVERSHOOT_PCT && avgErr < 4.0f)
        score -= 8.0f;

    if (curKp < KP_MIN || curKp > KP_MAX) score += 20.0f;
    if (curKi < KI_MIN || curKi > KI_MAX) score += 15.0f;

    if (metrics.burst_detected) {
        float burstOver = ((metrics.burst_rpm - AutoTunerNS::kTargetRpm) / AutoTunerNS::kTargetRpm) * 100.0f;
        if (burstOver > 30.0f) score += burstOver * AutoTunerNS::W_BURST;
    }

    return fmaxf(0.0f, score);
}

// ============================================================
// Parameter adjustment (Only Kp and Ki, Kf is fixed open-loop)
// ============================================================

static void adjustForLoadPhase() {
    if (!metrics.hasLoadWindow()) return;

    const float ssErr = metrics.ss_error();
    if (ssErr <= 2.0f) return;

    // Beban aktif tapi RPM jauh dari target → dorong Ki (utama) + Kp (sekunder)
    float kiBoost = kiStep * (0.8f + ssErr * 0.15f);
    if (precision == Precision::ULTRA_FINE) kiBoost *= 1.5f;
    curKi += kiBoost;

    if (ssErr > 8.0f) {
        curKp += kpStep * 0.4f;
    }

    Serial.printf("  [LOAD] ss_err=%.1f -> Ki+=%.3f (Ki=%.3f)\n", ssErr, kiBoost, curKi);
}

static void adjustCoarse() {
    float over = metrics.overshoot_pct();
    float err = effectiveErr();
    uint32_t rt = metrics.rise_time_ms;

    float stepScale = 1.0f;
    if (currentScore < AutoTunerNS::EXCELLENT_SCORE) stepScale = 0.2f;
    else if (currentScore < AutoTunerNS::GOOD_SCORE) stepScale = 0.5f;

    if (over > AutoTunerNS::HIGH_OVERSHOOT_PCT) {
        curKp *= (1.0f - 0.25f * stepScale);
        curKi *= (1.0f - 0.10f * stepScale);
    } else if (rt > AutoTunerNS::SLOW_RISE_MS) {
        curKp *= (1.0f + 0.40f * stepScale);
        curKi *= (1.0f + 0.20f * stepScale);
    } else if (err > 5.0f) {
        curKi *= (1.0f + 0.50f * stepScale);
    } else if (over > AutoTunerNS::MEDIUM_OVERSHOOT_PCT && rt < AutoTunerNS::MEDIUM_RISE_MS) {
        curKp *= (1.0f - 0.05f * stepScale);
        curKi *= (1.0f - 0.05f * stepScale);
    } else if (metrics.burst_detected) {
        curKp *= (1.0f - 0.30f * stepScale);
        curKi *= (1.0f - 0.20f * stepScale);
    } else {
        if (noImprove == 0) curKi += kiStep * 0.5f * stepScale;
        else curKi *= (1.0f - 0.10f * stepScale);
    }
}

static void adjustFine() {
    float over = metrics.overshoot_pct();
    float err = effectiveErr();
    uint32_t rt = metrics.rise_time_ms;

    float stepScale = 1.0f;
    if (currentScore < AutoTunerNS::EXCELLENT_SCORE) stepScale = 0.2f;
    else if (currentScore < AutoTunerNS::GOOD_SCORE) stepScale = 0.5f;

    if (over > AutoTunerNS::MEDIUM_OVERSHOOT_PCT) {
        curKp -= kpStep * stepScale;
    } else if (over < AutoTunerNS::LOW_OVERSHOOT_PCT && err < 3.0f && rt < AutoTunerNS::MEDIUM_RISE_MS) {
        curKi += kiStep * 0.8f * stepScale;
    } else if (err > 3.0f) {
        curKi += kiStep * 1.2f * stepScale;
    } else if (rt > AutoTunerNS::MEDIUM_RISE_MS) {
        curKp += kpStep * 0.8f * stepScale;
    }
}

static void adjustUltraFine() {
    float over = metrics.overshoot_pct();
    float err = effectiveErr();
    uint32_t rt = metrics.rise_time_ms;

    float stepScale = 1.0f;
    if (currentScore < AutoTunerNS::EXCELLENT_SCORE) stepScale = 0.2f;
    else if (currentScore < AutoTunerNS::GOOD_SCORE) stepScale = 0.5f;

    if (over > 5.0f) {
        curKp -= kpStep * 0.6f * stepScale;
    } else if (err > 2.0f) {
        curKi += kiStep * 0.8f * stepScale;
    } else if (rt > AutoTunerNS::MEDIUM_RISE_MS + 500) {
        curKp += kpStep * 0.5f * stepScale;
    } else {
        switch (cycleCount % 3) {
            case 0: curKp += kpStep * 0.15f * stepScale; break;
            case 1: curKp -= kpStep * 0.15f * stepScale; break;
            case 2: curKi += kiStep * 0.2f * stepScale; break;
        }
    }
}

static void adjustParameters() {
    if (precisionShouldAdvance()) precisionAdvance();

    switch (precision) {
        case Precision::COARSE:     adjustCoarse();   break;
        case Precision::FINE:       adjustFine();     break;
        case Precision::ULTRA_FINE: adjustUltraFine(); break;
    }

    adjustForLoadPhase();

    curKp = constrain(curKp, KP_MIN, KP_MAX);
    curKi = constrain(curKi, KI_MIN, KI_MAX);
}

// ============================================================
// Public API
// ============================================================

void startAutoTune(int motorIdx) {
    if (motorIdx < 0 || motorIdx >= (int)MOTOR_COUNT) return;
    targetMotor = motorIdx;
    singleMode = true;
    state = ATState::WAIT_RELEASE;
    Serial.printf("\n[AUTOTUNE] Start Motor %d\n", motorIdx);
}

void startAutoTuneAll() {
    targetMotor = 0;
    singleMode = false;
    state = ATState::WAIT_RELEASE;
    Serial.println("\n[AUTOTUNE] Start All Motors");
}

void autoTunerAbort() {
    aborted = true;
    state = ATState::DONE;
}

bool isAutoTunerRunning() {
    return state != ATState::IDLE;
}

// ============================================================
// State machine
// ============================================================

void autoTunerTick(bool bootPressed) {
    if (state == ATState::IDLE) return;
    uint32_t now = millis();

    switch (state) {

    case ATState::WAIT_RELEASE:
        if (!bootPressed) {
            aborted = false;
            oledDrawn = false;
            motorStopAll();
            tMotorIdx = targetMotor;
            state = ATState::MOTOR_INIT;
        }
        break;

    case ATState::MOTOR_INIT: {
        if (bootPressed) { motorStopAll(); aborted = true; state = ATState::DONE; break; }

        float kpid, kid, kfd, deadp;
        pidLoadFromNVS(tMotorIdx, kpid, kid, kfd, deadp);

        bool isFirstTime = (fabsf(kpid - 0.1f) < 0.0001f);
        if (isFirstTime) {
            curKp = 1.0f;
            curKi = 0.5f;
            Serial.printf("AutoTuner M%d: First-time.\n", tMotorIdx);
        } else {
            curKp = kpid;
            curKi = kid;
            Serial.printf("AutoTuner M%d: Loaded Kp=%.2f Ki=%.3f\n", tMotorIdx, curKp, curKi);
        }

        bestKp = curKp;
        bestKi = curKi;
        bestScore = AutoTunerNS::kBestScoreInitial;
        cycleCount = 0;
        baseKf = 0.0f;
        baseDeadband = 0.0f;

        precisionInit();
        pidResetOne(tMotorIdx);
        
        char buf1[20]; snprintf(buf1, sizeof(buf1), "M%d DEADBAND", tMotorIdx);
        oledShowStatus(buf1, "Finding Friction...");
        Serial.printf("[AUTOTUNE] Step 1: Finding Deadband (Friction) M%d\n", tMotorIdx);
        
        deadbandPwm = 0;
        lastDeadbandStepMs = now;
        stateStartMs = now;
        state = ATState::DEADBAND_START;
        break;
    }

    // ---------------------------------------------------------
    // Phase 1: DEADBAND (Friction Offset)
    // ---------------------------------------------------------
    case ATState::DEADBAND_START:
        pwmMotor(tMotorIdx, 0);
        state = ATState::DEADBAND_WAIT;
        break;

    case ATState::DEADBAND_WAIT: {
        if (bootPressed) { motorStopAll(); aborted = true; state = ATState::DONE; break; }

        if (now - lastDeadbandStepMs >= AutoTunerNS::DEADBAND_STEP_MS) {
            lastDeadbandStepMs = now;
            deadbandPwm += 1;
            pwmMotor(tMotorIdx, deadbandPwm);

            float rpm = fabsf(getEncoderVelocityRpm(tMotorIdx));
            
            // Debug print setiap 500ms
            static uint32_t lastDbDebugMs = 0;
            if (now - lastDbDebugMs >= 500) {
                lastDbDebugMs = now;
                Serial.printf("[AUTOTUNE-DEBUG] M%d Deadband Wait: PWM=%d, RPM=%.2f\n", tMotorIdx, deadbandPwm, rpm);
            }

            // If motor starts spinning
            if (rpm >= 2.0f) {
                baseDeadband = (float)deadbandPwm;
                pwmMotor(tMotorIdx, 0);
                Serial.printf("[AUTOTUNE] Deadband found: PWM %d (RPM=%.1f)\n", deadbandPwm, rpm);
                
                char buf1[20], buf2[20];
                snprintf(buf1, sizeof(buf1), "M%d KF OPEN", tMotorIdx);
                snprintf(buf2, sizeof(buf2), "DB: %.0f", baseDeadband);
                oledShowStatus(buf1, buf2);
                
                delay(500); // let motor stop
                stateStartMs = millis();
                state = ATState::KF_START;
            } else if (deadbandPwm > 300) {
                // Failsafe
                baseDeadband = 0.0f;
                pwmMotor(tMotorIdx, 0);
                Serial.println("[AUTOTUNE] ERROR: Deadband not found up to PWM 300!");
                delay(500);
                stateStartMs = millis();
                state = ATState::KF_START;
            }
        }
        break;
    }

    // ---------------------------------------------------------
    // Phase 2: KF (Open-loop linear slope)
    // ---------------------------------------------------------
    case ATState::KF_START: {
        if (bootPressed) { motorStopAll(); aborted = true; state = ATState::DONE; break; }

        int pwmHalf = (int)(PWM_MAX * AutoTunerNS::KF_PWM_FRACTION);
        pwmMotor(tMotorIdx, pwmHalf);
        stateStartMs = now;
        state = ATState::KF_WAIT;
        break;
    }

    case ATState::KF_WAIT: {
        if (bootPressed) { motorStopAll(); aborted = true; state = ATState::DONE; break; }

        if (now - stateStartMs > AutoTunerNS::KF_RUN_MS) {
            float rawRpm = getEncoderVelocityRpm(tMotorIdx);
            float absRpm = fabsf(rawRpm);
            pwmMotor(tMotorIdx, 0);

            if (absRpm < AutoTunerNS::KF_MIN_RPM) {
                Serial.println("[AUTOTUNE] ERROR: RPM < 10 — encoder slip?");
                oledShowStatus("ERROR TUNE", "Encoder slip?");
                stateStartMs = now;
                state = ATState::DONE;
                break;
            }

            int pwmHalf = (int)(PWM_MAX * AutoTunerNS::KF_PWM_FRACTION);
            // Kf = (PWM - Deadband) / RPM
            baseKf = ((float)pwmHalf - baseDeadband) / absRpm;
            baseKf = fmaxf(0.0f, baseKf); // Safety
            
            Serial.printf("[AUTOTUNE] Kf = %.4f (from %d PWM, %.0f DB, %.0f RPM)\n", 
                          baseKf, pwmHalf, baseDeadband, absRpm);

            delay(1000); // let motor stop completely
            stateStartMs = millis();
            state = ATState::CYCLE_START;
        }
        break;
    }

    // ---------------------------------------------------------
    // Phase 3: PID TUNING (Kp, Ki)
    // ---------------------------------------------------------
    case ATState::CYCLE_START:
        if (bootPressed) { motorStopAll(); aborted = true; state = ATState::DONE; break; }

        pidSetGains(tMotorIdx, curKp, curKi, baseKf, baseDeadband);
        pidResetOne(tMotorIdx);
        metrics.reset();
        stateStartMs = now;
        lastPidTickMs = now;
        state = ATState::CYCLE_RUN;
        break;

    case ATState::CYCLE_RUN: {
        if (bootPressed) { motorStopAll(); aborted = true; state = ATState::DONE; break; }

        uint32_t elapsed = now - stateStartMs;

        if (now - lastPidTickMs >= AutoTunerNS::kPidTickMs) {
            lastPidTickMs = now;
            // Dihapus: convertEncoderToRPM(); karena loop() utama sudah memanggilnya
            
            float rpm = getEncoderVelocityRpm(tMotorIdx);
            int pwm = pidCompute(tMotorIdx, AutoTunerNS::kTargetRpm, AutoTunerNS::kPidTickMs / 1000.0f);

            // Emulasi beban: ramp 60%→100%, potong PWM proporsional (cap LOAD_INJECT_PWM)
            const int applied = applyLoadInject(pwm, elapsed);
            pwmMotor(tMotorIdx, applied);

            // Metrics
            if (rpm > metrics.peak_rpm) metrics.peak_rpm = rpm;
            metrics.total_err += fabsf(rpm - AutoTunerNS::kTargetRpm);
            metrics.samples++;

            if (!metrics.rise_10_done && rpm >= AutoTunerNS::kTargetRpm * 0.10f) {
                metrics.rise_10_t = now;
                metrics.rise_10_done = true;
            }
            if (metrics.rise_10_done && !metrics.rise_90_done && rpm >= AutoTunerNS::kTargetRpm * 0.90f) {
                metrics.rise_time_ms = now - metrics.rise_10_t;
                metrics.rise_90_done = true;
            }

            if (!metrics.burst_detected && elapsed < 500 && rpm > AutoTunerNS::kTargetRpm * AutoTunerNS::kBurstThreshold) {
                metrics.burst_detected = true;
                metrics.burst_rpm = rpm;
                metrics.burst_t_ms = elapsed;
            }

            if (elapsed >= (uint32_t)(AUTOTUNE_RUN_MS * AutoTunerNS::kSteadyStateStartFraction)) {
                metrics.ss_samples++;
                float delta = rpm - metrics.ss_mean;
                metrics.ss_mean += delta / metrics.ss_samples;
                float delta2 = rpm - metrics.ss_mean;
                metrics.ss_M2 += delta * delta2;
            }

            if (now - lastOledMs >= AutoTunerNS::kQuickLogThresholdMs) {
                lastOledMs = now;
                char buf1[20], buf2[20];
                const int loadCut = computeLoadCut(pwm, elapsed);
                snprintf(buf1, sizeof(buf1), "M%d %s %d%%", tMotorIdx, precisionName(), (int)(elapsed * 100 / AUTOTUNE_RUN_MS));
                if (loadCut > 0) {
                    snprintf(buf2, sizeof(buf2), "T:%.0f C:%.0f -%d", AutoTunerNS::kTargetRpm, rpm, loadCut);
                } else {
                    snprintf(buf2, sizeof(buf2), "T:%.0f C:%.0f", AutoTunerNS::kTargetRpm, rpm);
                }
                oledShowStatus(buf1, buf2);
            }
        }

        if (elapsed >= AUTOTUNE_RUN_MS) {
            motorStopAll();
            pidResetOne(tMotorIdx);
            state = ATState::CYCLE_FINISH;
        }
        break;
    }

    case ATState::CYCLE_FINISH: {
        float score = calculateScore();
        currentScore = score;

        if (score < bestScore) {
            bestScore = score;
            bestKp = curKp;
            bestKi = curKi;
            noImprove = 0;
            Serial.printf("  + Score: %.1f (new best) Kp=%.2f Ki=%.3f\n", score, curKp, curKi);
        } else {
            noImprove++;
            curKp = bestKp;
            curKi = bestKi;
            Serial.printf("  - Score: %.1f -> rollback Kp=%.2f Ki=%.3f\n", score, bestKp, bestKi);
        }

        adjustParameters();
        cycleCount++;

        if (cycleCount >= AUTOTUNE_MAX_CYCLES) {
            pidSetGains(tMotorIdx, bestKp, bestKi, baseKf, baseDeadband);
            pidSaveToNVS(tMotorIdx, bestKp, bestKi, baseKf, baseDeadband);

            char buf1[20], buf2[20];
            snprintf(buf1, sizeof(buf1), "M%d DONE", tMotorIdx);
            snprintf(buf2, sizeof(buf2), "Kp:%.1f Ki:%.2f", bestKp, bestKi);
            oledShowStatus(buf1, buf2);
            Serial.printf("Motor %d DONE - Kp: %.2f, Ki: %.3f, Kf: %.4f, DB: %.0f\n", 
                          tMotorIdx, bestKp, bestKi, baseKf, baseDeadband);

            stateStartMs = now;
            state = ATState::MOTOR_SHOW;
        } else {
            stateStartMs = now;
            state = ATState::CYCLE_COOLDOWN;
        }
        break;
    }

    case ATState::CYCLE_COOLDOWN:
        if (bootPressed) { motorStopAll(); aborted = true; state = ATState::DONE; break; }
        if (now - stateStartMs >= AUTOTUNE_COOLDOWN_MS)
            state = ATState::CYCLE_START;
        break;

    case ATState::MOTOR_SHOW:
        if (bootPressed || (now - stateStartMs >= AUTOTUNE_SHOW_MS)) {
            if (singleMode) {
                state = ATState::DONE;
            } else {
                tMotorIdx++;
                if (tMotorIdx >= (int)MOTOR_COUNT) {
                    state = ATState::DONE;
                } else {
                    state = ATState::MOTOR_INIT;
                }
            }
        }
        break;

    case ATState::DONE:
        motorStopAll();
        if (!oledDrawn) {
            if (aborted) {
                oledShowStatus("ABORTED", "Cancelled");
                Serial.println("AUTOTUNE: Aborted");
            } else {
                oledShowStatus("ALL DONE", "Check Serial");
                Serial.println("=== AUTOTUNE COMPLETE ===");
                pidReloadFromNVS();
            }
            oledDrawn = true;
        }
        singleMode = false;
        targetMotor = -1;
        state = ATState::IDLE;
        break;
    }
}