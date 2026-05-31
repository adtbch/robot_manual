// ============================================================
// AUTO-TUNER for PID - Dynamic architecture
// Adapts to motor count automatically
// ============================================================

#include "robot_config.h"
#include <Preferences.h>

// ============================================================
// Internal constants
// ============================================================

namespace AutoTunerNS {
    constexpr uint32_t kPidTickMs = 40;
    constexpr uint32_t kBootPressResetMs = 600;
    constexpr uint32_t kQuickLogThresholdMs = 200;
    constexpr float kTargetRpm = AUTOTUNE_TARGET_RPM;
    constexpr float kSteadyStateStartFraction = 0.60f;
    constexpr float kMinActiveTargetFraction = 0.05f;
    constexpr float kMinKpForMotionFraction = 0.10f;
    constexpr float kBurstThreshold = 1.30f;
    constexpr float kBestScoreInitial = 999999.0f;
    constexpr float kBestScoreSavedCutoff = 999998.0f;
    constexpr float kSteadyStateMinSamples = 10.0f;

    // Metrics thresholds (dalam RPM)
    constexpr float HIGH_OVERSHOOT_PCT = 15.0f;
    constexpr float MEDIUM_OVERSHOOT_PCT = 8.0f;
    constexpr float LOW_OVERSHOOT_PCT = 3.0f;
    constexpr uint32_t SLOW_RISE_MS = 3000UL;
    constexpr uint32_t MEDIUM_RISE_MS = 1500UL;
    constexpr float AVG_ERR_EXCELLENT = 1.5f;
    constexpr float AVG_ERR_GOOD = 3.0f;
    constexpr float EXCELLENT_SCORE = 20.0f;
    constexpr float GOOD_SCORE = 35.0f;

    // Scoring weights
    constexpr float W_ERROR = 12.0f;
    constexpr float W_OVERSHOOT = 3.0f;
    constexpr float W_RISE_TIME = 0.003f;
    constexpr float W_STABILITY = 5.0f;
    constexpr float W_BURST = 0.6f;

    // Precision stages adjustment
    constexpr float KP_COARSE = 5.0f, KP_FINE = 1.5f, KP_ULTRA = 0.5f;
    constexpr float KI_COARSE = 2.0f, KI_FINE = 0.6f, KI_ULTRA = 0.2f;
    constexpr float KD_COARSE = 0.10f, KD_FINE = 0.04f, KD_ULTRA = 0.01f;
    constexpr int STAGE_COARSE_CYCLES = 5;
    constexpr int STAGE_FINE_CYCLES = 4;
    constexpr int STAGNATION_LIMIT = 3;
}

// ============================================================
// Precision enum
// ============================================================

enum class Precision : uint8_t { COARSE = 0, FINE, ULTRA_FINE };

// ============================================================
// Per-cycle metrics
// ============================================================

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
        total_err = 0.0f;
        samples = 0;
        peak_rpm = 0.0f;
        rise_10_done = false;
        rise_90_done = false;
        rise_10_t = 0;
        rise_time_ms = 0;
        burst_detected = false;
        burst_rpm = 0.0f;
        burst_t_ms = 0;
        ss_mean = 0.0f;
        ss_M2 = 0.0f;
        ss_samples = 0;
    }

    float avg_err() const {
        return (samples > 0) ? total_err / samples : 0.0f;
    }

    float overshoot_pct() const {
        float over = peak_rpm - AutoTunerNS::kTargetRpm;
        return (over > 0.0f) ? (over / AutoTunerNS::kTargetRpm * 100.0f) : 0.0f;
    }

    float ss_variance() const {
        return (ss_samples > 1) ? ss_M2 / ss_samples : 0.0f;
    }

    float ss_error() const {
        return fabsf(ss_mean - AutoTunerNS::kTargetRpm);
    }
};

// ============================================================
// Auto-tuner state machine
// ============================================================

enum class ATState : uint8_t {
    AT_IDLE = 0,
    AT_WAIT_RELEASE,
    AT_MOTOR_INIT,
    AT_CYCLE_START,
    AT_CYCLE_RUN,
    AT_CYCLE_FINISH,
    AT_CYCLE_COOLDOWN,
    AT_MOTOR_SHOW,
    AT_DONE,
};

// ============================================================
// Global state variables
// ============================================================

static ATState gAtState = ATState::AT_IDLE;
static int gMotorIdx = 0;
static int gCycleCount = 0;
static uint32_t gStartMs = 0;
static uint32_t gLastPidTickMs = 0;
static uint32_t gLastOledMs = 0;
static bool gAborted = false;
static bool gScreenDrawn = false;

static float gCurrentKp, gCurrentKi, gCurrentKd;
static float gBestKp, gBestKi, gBestKd;
static float gBestScore;
static CycleMetrics gMetrics;

static Precision gPrecision = Precision::COARSE;
static float gKpStep, gKiStep, gKdStep;
static int gStageCount = 0;
static int gNoImprove = 0;

// Single-motor mode support (started via API)
static int gTargetMotor = -1;
static bool gSingleMode = false;

// Results per motor (dynamic vectors)
static std::vector<float> gResultsKp, gResultsKi, gResultsKd;
static std::vector<float> gResultsScore;
static std::vector<bool> gResultsOk;
static std::vector<int> gResultsCycles;

// ============================================================
// Precision stage management
// ============================================================

static void precisionInit() {
    gPrecision = Precision::COARSE;
    gKpStep = AutoTunerNS::KP_COARSE;
    gKiStep = AutoTunerNS::KI_COARSE;
    gKdStep = AutoTunerNS::KD_COARSE;
    gStageCount = 0;
    gNoImprove = 0;
}

static void precisionAdvance() {
    if (gPrecision == Precision::COARSE) {
        gPrecision = Precision::FINE;
        gKpStep = AutoTunerNS::KP_FINE;
        gKiStep = AutoTunerNS::KI_FINE;
        gKdStep = AutoTunerNS::KD_FINE;
    } else if (gPrecision == Precision::FINE) {
        gPrecision = Precision::ULTRA_FINE;
        gKpStep = AutoTunerNS::KP_ULTRA;
        gKiStep = AutoTunerNS::KI_ULTRA;
        gKdStep = AutoTunerNS::KD_ULTRA;
    }
    gStageCount = 0;
    gNoImprove = 0;
}

static bool precisionShouldAdvance() {
    gStageCount++;
    if (gPrecision == Precision::ULTRA_FINE) return false;
    if (gPrecision == Precision::COARSE && (gStageCount >= AutoTunerNS::STAGE_COARSE_CYCLES || gNoImprove >= AutoTunerNS::STAGNATION_LIMIT)) return true;
    if (gPrecision == Precision::FINE && (gStageCount >= AutoTunerNS::STAGE_FINE_CYCLES || gNoImprove >= AutoTunerNS::STAGNATION_LIMIT)) return true;
    return false;
}

static const char* precisionName() {
    if (gPrecision == Precision::COARSE) return "COARSE";
    if (gPrecision == Precision::FINE) return "FINE";
    return "ULTRA";
}

// ============================================================
// Scoring function
// ============================================================

static float calculateScore() {
    float avgErr = gMetrics.avg_err();
    float over = gMetrics.overshoot_pct();
    uint32_t rise = gMetrics.rise_time_ms;
    float score = 0.0f;

    score += avgErr * avgErr * AutoTunerNS::W_ERROR;

    if (gMetrics.ss_samples >= (int)AutoTunerNS::kSteadyStateMinSamples) {
        float ssErr = gMetrics.ss_error();
        score += ssErr * ssErr * 18.0f;
    }

    if (gMetrics.ss_samples >= (int)AutoTunerNS::kSteadyStateMinSamples) {
        float var = gMetrics.ss_variance();
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

    if (gMetrics.samples > 50) {
        if (avgErr < AutoTunerNS::AVG_ERR_EXCELLENT) score -= 8.0f * AutoTunerNS::W_STABILITY;
        else if (avgErr < AutoTunerNS::AVG_ERR_GOOD) score -= 4.0f * AutoTunerNS::W_STABILITY;
    }

    if (rise < AutoTunerNS::MEDIUM_RISE_MS && over < AutoTunerNS::MEDIUM_OVERSHOOT_PCT && avgErr < 2.5f)
        score -= 15.0f;
    else if (rise < AutoTunerNS::SLOW_RISE_MS && over < AutoTunerNS::HIGH_OVERSHOOT_PCT && avgErr < 4.0f)
        score -= 8.0f;

    if (gCurrentKp < kpMin || gCurrentKp > 500.0f) score += 20.0f;
    if (gCurrentKi < kiMin || gCurrentKi > 100.0f) score += 15.0f;
    if (gCurrentKd < kdMin || gCurrentKd > 10.0f) score += 12.0f;

    if (gMetrics.burst_detected) {
        float burstOver = ((gMetrics.burst_rpm - AutoTunerNS::kTargetRpm) / AutoTunerNS::kTargetRpm) * 100.0f;
        if (burstOver > 30.0f) score += burstOver * AutoTunerNS::W_BURST;
    }

    return fmaxf(0.0f, score);
}

// ============================================================
// Parameter adjustment stages
// ============================================================

static void adjustCoarse() {
    float over = gMetrics.overshoot_pct();
    float err = gMetrics.avg_err();
    uint32_t rt = gMetrics.rise_time_ms;

    if (over > AutoTunerNS::HIGH_OVERSHOOT_PCT) {
        gCurrentKp *= 0.75f;
        gCurrentKd *= 1.50f;
    } else if (rt > AutoTunerNS::SLOW_RISE_MS) {
        gCurrentKp *= 1.40f;
        gCurrentKi *= 1.20f;
    } else if (err > 5.0f) {
        gCurrentKi *= 1.50f;
    } else if (over > AutoTunerNS::MEDIUM_OVERSHOOT_PCT && rt < AutoTunerNS::MEDIUM_RISE_MS) {
        gCurrentKd *= 1.30f;
        gCurrentKp *= 0.95f;
    } else if (gMetrics.burst_detected) {
        gCurrentKp *= 0.70f;
        gCurrentKd *= 1.80f;
        gCurrentKi *= 0.80f;
    } else {
        if (gNoImprove == 0) gCurrentKp += gKpStep * 1.2f;
        else gCurrentKi += gKiStep;
    }
}

static void adjustFine() {
    float over = gMetrics.overshoot_pct();
    float err = gMetrics.avg_err();
    uint32_t rt = gMetrics.rise_time_ms;

    if (over > AutoTunerNS::MEDIUM_OVERSHOOT_PCT) {
        gCurrentKp -= gKpStep;
        gCurrentKd += gKdStep * 1.5f;
    } else if (over < AutoTunerNS::LOW_OVERSHOOT_PCT && err < 3.0f && rt < AutoTunerNS::MEDIUM_RISE_MS) {
        gCurrentKi += gKiStep * 0.8f;
    } else if (err > 3.0f) {
        gCurrentKi += gKiStep * 1.2f;
    } else if (rt > AutoTunerNS::MEDIUM_RISE_MS) {
        gCurrentKp += gKpStep * 0.8f;
    } else {
        gCurrentKd += gKdStep * 0.5f;
    }
}

static void adjustUltraFine() {
    float over = gMetrics.overshoot_pct();
    float err = gMetrics.avg_err();
    uint32_t rt = gMetrics.rise_time_ms;

    if (over > 5.0f) {
        gCurrentKp -= gKpStep * 0.6f;
    } else if (err > 2.0f) {
        gCurrentKi += gKiStep * 0.8f;
    } else if (rt > AutoTunerNS::MEDIUM_RISE_MS + 500) {
        gCurrentKp += gKpStep * 0.5f;
    } else {
        switch (gCycleCount % 3) {
            case 0: gCurrentKp += gKpStep * 0.3f; break;
            case 1: gCurrentKp -= gKpStep * 0.3f; break;
            case 2: gCurrentKi += gKiStep * 0.4f; break;
        }
    }
}

static void adjustParameters() {
    if (precisionShouldAdvance()) precisionAdvance();

    switch (gPrecision) {
        case Precision::COARSE: adjustCoarse(); break;
        case Precision::FINE: adjustFine(); break;
        case Precision::ULTRA_FINE: adjustUltraFine(); break;
    }

    gCurrentKp = constrain(gCurrentKp, 0.1f, 500.0f);
    gCurrentKi = constrain(gCurrentKi, 0.0f, 100.0f);
    gCurrentKd = constrain(gCurrentKd, 0.0f, 10.0f);
}

// ============================================================
// OLED Stub functions (user fills in with actual display code)
// ============================================================

void oledAutotunerStart(const char *label) {
    Serial.print("OLED: ");
    Serial.println(label);
}

void oledAutotunerRunning(const char *label, float target, float current, uint32_t elapsed, uint32_t total) {
    if (millis() - gLastOledMs < AutoTunerNS::kQuickLogThresholdMs) return;
    gLastOledMs = millis();
    
    Serial.printf("[%s] Target: %.1f RPM, Current: %.1f RPM, Elapsed: %lu/%lu ms\n",
                  label, target, current, elapsed, total);
}

void oledAutotunerCycleResult(int motorIdx, int totalMotors, float kp, float ki, float kd, bool ok, int cycles) {
    Serial.printf("Motor %d/%d - Kp: %.2f, Ki: %.2f, Kd: %.4f [%s] Cycles: %d\n",
                  motorIdx + 1, totalMotors, kp, ki, kd, ok ? "OK" : "FAIL", cycles);
}

void oledAutotunerDone(const std::vector<float> &kps, const std::vector<float> &kis, 
                       const std::vector<float> &kds, const std::vector<bool> &oks) {
    Serial.println("=== AUTOTUNE COMPLETE ===");
    for (size_t i = 0; i < kps.size(); i++) {
        Serial.printf("Motor %zu: Kp=%.2f, Ki=%.2f, Kd=%.4f [%s]\n",
                      i, kps[i], kis[i], kds[i], oks[i] ? "OK" : "FAIL");
    }
}

void oledAutotunerAborted() {
    Serial.println("AUTOTUNE: Aborted by user");
}

// ============================================================
// Main auto-tuner state machine
// ============================================================

void autoTunerStart() {
    gAtState = ATState::AT_WAIT_RELEASE;
    Serial.println("AutoTuner: Waiting for BOOT release...");
}

// Variabel global untuk menyimpan nilai Kp Ki Kd manual dari Serial
static bool gUseCustomInitGains = false;
static float gCustomInitKp = 0.0f;
static float gCustomInitKi = 0.0f;
static float gCustomInitKd = 0.0f;

// Start auto-tuner for a single motor index (non-blocking) with optional custom initial gains
void autoTunerStartSingle(int motorIdx, float initKp, float initKi, float initKd) {
    if (motorIdx < 0 || (size_t)motorIdx >= motors.size()) {
        Serial.println("AutoTunerStartSingle: invalid motor index");
        return;
    }

    // Jika parameter manual diberikan (bernilai positif), tandai untuk digunakan
    if (initKp >= 0.0f && initKi >= 0.0f && initKd >= 0.0f) {
        gUseCustomInitGains = true;
        gCustomInitKp = initKp;
        gCustomInitKi = initKi;
        gCustomInitKd = initKd;
        Serial.printf("Using Custom Init Gains -> Kp: %.2f, Ki: %.2f, Kd: %.4f\n", initKp, initKi, initKd);
    } else {
        gUseCustomInitGains = false;
    }

    gTargetMotor = motorIdx;
    gSingleMode = true;
    autoTunerStart();
}

// Abort the running auto-tuner (safe stop)
void autoTunerAbort() {
    gAborted = true;
    gAtState = ATState::AT_DONE;
}

bool autoTunerIsActive() {
    return gAtState != ATState::AT_IDLE;
}

void autoTunerTick(bool bootPressed) {
    switch (gAtState) {

    case ATState::AT_IDLE:
        break;

    case ATState::AT_WAIT_RELEASE:
        if (!bootPressed) {
            gAborted = false;
            gScreenDrawn = false;
            motorStopAll();

            // Initialize results vectors
            gResultsKp.assign(motors.size(), 0.0f);
            gResultsKi.assign(motors.size(), 0.0f);
            gResultsKd.assign(motors.size(), 0.0f);
            gResultsScore.assign(motors.size(), 0.0f);
            gResultsOk.assign(motors.size(), false);
            gResultsCycles.assign(motors.size(), 0);

            if (gSingleMode && gTargetMotor >= 0 && (size_t)gTargetMotor < motors.size()) {
                gMotorIdx = gTargetMotor;
            } else {
                gMotorIdx = 0;
            }

            gAtState = ATState::AT_MOTOR_INIT;
        }
        break;

    case ATState::AT_MOTOR_INIT: {
        if (bootPressed) {
            motorStopAll();
            gAborted = true;
            gAtState = ATState::AT_DONE;
            break;
        }

        pidLoadFromNVS(gMotorIdx, gCurrentKp, gCurrentKi, gCurrentKd);

        // Timpa dengan nilai kustom dari Serial jika disediakan pengguna
        if (gUseCustomInitGains && gSingleMode) {
            gCurrentKp = gCustomInitKp;
            gCurrentKi = gCustomInitKi;
            gCurrentKd = gCustomInitKd;
            Serial.println("AutoTuner: Initialized with CUSTOM gains from Serial CLI.");
        }

        // Cek apakah NVS masih berisi default (belum pernah di-tuning)
        // pidLoadFromNVS return default Kp=0.1, Ki=0.0, Kd=0.0 jika NVS kosong
        // Cek Kp == 0.1 karena default NVS, dan tidak mungkin hasil tuning = 0.1
        bool isFirstTime = (fabsf(gCurrentKp - 0.1f) < 0.0001f);

        if (isFirstTime && !gUseCustomInitGains) {
            float target_vel_rads = AUTOTUNE_TARGET_RPM * kRpmToRadPerSec;
            float kpMin = AutoTunerNS::kMinKpForMotionFraction * maxPwm / target_vel_rads;
            gCurrentKp = kpMin;
            gCurrentKi = 0.5f;
            gCurrentKd = 0.0f;
            Serial.println("AutoTuner: First-time run - using initial safety gains.");
        } else {
            Serial.printf("AutoTuner: Using loaded gains -> Kp=%.2f Ki=%.2f Kd=%.4f\n",
                          gCurrentKp, gCurrentKi, gCurrentKd);
        }

        gBestKp = gCurrentKp;
        gBestKi = gCurrentKi;
        gBestKd = gCurrentKd;
        gBestScore = AutoTunerNS::kBestScoreInitial;
        gCycleCount = 0;
        gLastOledMs = 0;

        precisionInit();
        pidResetOne(gMotorIdx);
        
        oledAutotunerStart("Initialization");
        gAtState = ATState::AT_CYCLE_START;
        break;
    }

    case ATState::AT_CYCLE_START:
        if (bootPressed) {
            motorStopAll();
            gAborted = true;
            gAtState = ATState::AT_DONE;
            break;
        }

        pidSetGains(gMotorIdx, gCurrentKp, gCurrentKi, gCurrentKd);
        pidResetOne(gMotorIdx);
        gMetrics.reset();
        pidResetOne(gMotorIdx);
        gStartMs = millis();
        gLastPidTickMs = millis();
        gAtState = ATState::AT_CYCLE_RUN;
        break;

    case ATState::AT_CYCLE_RUN: {
        if (bootPressed) {
            motorStopAll();
            gAborted = true;
            gAtState = ATState::AT_DONE;
            break;
        }

        uint32_t elapsed = millis() - gStartMs;

        // Jalankan PID dan metrik dengan interval tetap 40ms agar konsisten
        if (millis() - gLastPidTickMs >= AutoTunerNS::kPidTickMs) {
            gLastPidTickMs = millis();
            convertEncoderToRPM();

            float velRpm = getEncoderVelocityRpm(gMotorIdx);

            // Anti-windup startup
            if (elapsed < AutoTunerNS::kBootPressResetMs && fabsf(velRpm) < AutoTunerNS::kTargetRpm * AutoTunerNS::kMinActiveTargetFraction) {
                pidResetOne(gMotorIdx);
            }

            // Run PID controller
            double minIntegral = (gCurrentKi > 0.0001f) ? -1023.0 / gCurrentKi : -2000.0;
            double maxIntegral = (gCurrentKi > 0.0001f) ? 1023.0 / gCurrentKi : 2000.0;
            int pwm = (int)computePID(gMotorIdx, AutoTunerNS::kTargetRpm, velRpm, gCurrentKp, gCurrentKi, gCurrentKd, minIntegral, maxIntegral);
            pwmMotor(gMotorIdx, pwm);

            // Collect metrics
            if (velRpm > gMetrics.peak_rpm) gMetrics.peak_rpm = velRpm;
            gMetrics.total_err += fabsf(velRpm - AutoTunerNS::kTargetRpm);
            gMetrics.samples++;

            // Rise time 10%→90%
            if (!gMetrics.rise_10_done && velRpm >= AutoTunerNS::kTargetRpm * 0.10f) {
                gMetrics.rise_10_t = millis();
                gMetrics.rise_10_done = true;
            }
            if (gMetrics.rise_10_done && !gMetrics.rise_90_done && velRpm >= AutoTunerNS::kTargetRpm * 0.90f) {
                gMetrics.rise_time_ms = millis() - gMetrics.rise_10_t;
                gMetrics.rise_90_done = true;
            }

            // Burst detection
            if (!gMetrics.burst_detected && elapsed < 500 && velRpm > AutoTunerNS::kTargetRpm * AutoTunerNS::kBurstThreshold) {
                gMetrics.burst_detected = true;
                gMetrics.burst_rpm = velRpm;
                gMetrics.burst_t_ms = elapsed;
            }

            // Steady-state window (Welford's online algorithm)
            if (elapsed >= (uint32_t)(AUTOTUNE_RUN_MS * AutoTunerNS::kSteadyStateStartFraction)) {
                gMetrics.ss_samples++;
                float delta = velRpm - gMetrics.ss_mean;
                gMetrics.ss_mean += delta / gMetrics.ss_samples;
                float delta2 = velRpm - gMetrics.ss_mean;
                gMetrics.ss_M2 += delta * delta2;
            }

            oledAutotunerRunning(precisionName(), AutoTunerNS::kTargetRpm, getEncoderVelocityRpm(gMotorIdx), elapsed, AUTOTUNE_RUN_MS);
        }

        if (elapsed >= AUTOTUNE_RUN_MS) {
            motorStopAll();
            gAtState = ATState::AT_CYCLE_FINISH;
        }
        break;
    }

    case ATState::AT_CYCLE_FINISH: {
        float score = calculateScore();

        if (score < gBestScore) {
            gBestScore = score;
            gBestKp = gCurrentKp;
            gBestKi = gCurrentKi;
            gBestKd = gCurrentKd;
            gNoImprove = 0;
        } else {
            gNoImprove++;
        }

        adjustParameters();
        gCycleCount++;

        if (gCycleCount >= AUTOTUNE_MAX_CYCLES) {
            pidSaveToNVS(gMotorIdx, gBestKp, gBestKi, gBestKd);
            gResultsKp[gMotorIdx] = gBestKp;
            gResultsKi[gMotorIdx] = gBestKi;
            gResultsKd[gMotorIdx] = gBestKd;
            gResultsScore[gMotorIdx] = gBestScore;
            gResultsOk[gMotorIdx] = (gBestScore < AutoTunerNS::kBestScoreSavedCutoff);
            gResultsCycles[gMotorIdx] = gCycleCount;

            oledAutotunerCycleResult(gMotorIdx, motors.size(), gBestKp, gBestKi, gBestKd, gResultsOk[gMotorIdx], gCycleCount);
            gStartMs = millis();
            gAtState = ATState::AT_MOTOR_SHOW;
        } else {
            gStartMs = millis();
            gAtState = ATState::AT_CYCLE_COOLDOWN;
        }
        break;
    }

    case ATState::AT_CYCLE_COOLDOWN:
        if (bootPressed) {
            motorStopAll();
            gAborted = true;
            gAtState = ATState::AT_DONE;
            break;
        }
        if (millis() - gStartMs >= AUTOTUNE_COOLDOWN_MS)
            gAtState = ATState::AT_CYCLE_START;
        break;

    case ATState::AT_MOTOR_SHOW:
        if (bootPressed || (millis() - gStartMs >= AUTOTUNE_SHOW_MS)) {
            if (gSingleMode) {
                gAtState = ATState::AT_DONE;
            } else {
                gMotorIdx++;
                gAtState = ((size_t)gMotorIdx < motors.size()) ? ATState::AT_MOTOR_INIT : ATState::AT_DONE;
            }
        }
        break;

    case ATState::AT_DONE:
        motorStopAll();
        if (!gScreenDrawn) {
            if (gAborted) {
                oledAutotunerAborted();
            } else {
                oledAutotunerDone(gResultsKp, gResultsKi, gResultsKd, gResultsOk);
                pidReloadFromNVS();
            }
            gScreenDrawn = true;
        }
        // Reset single-mode flags when done
        if (!gSingleMode) {
            // nothing
        } else {
            gSingleMode = false;
            gTargetMotor = -1;
        }
        if (bootPressed) {
            // Wait for release to return to IDLE
        } else {
            gScreenDrawn = false;
            gAtState = ATState::AT_IDLE;
        }
        break;
    }
}
