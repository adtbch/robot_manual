/*
 * =====================================================================
 * FILE    : autoTuner.ino
 * PERAN   : Auto-Tuner PID Motor (Step Response + Scoring)
 *           Metode: Multi-pass (COARSE -> FINE)
 *           Tuning: Kf -> Kp -> Ki (Kd diabaikan/0)
 * =====================================================================
 */

#include "autoTuner.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "oled.h"

namespace {

enum class TuneState {
    IDLE,
    INIT_MOTOR,
    FIND_KF_START,
    FIND_KF_WAIT,
    EVAL_START,
    EVAL_RUN,
    COOLDOWN,
    DONE,
    ERROR_WAIT
};

enum class PassLevel {
    COARSE_KP,
    FINE_KP,
    COARSE_KI,
    FINE_KI,
    FINISHED
};

TuneState state = TuneState::IDLE;
PassLevel currentPass = PassLevel::COARSE_KP;

int tMotorIdx = -1;

// Konstanta uji
constexpr float TUNE_TARGET_RPM = 200.0f;
constexpr uint32_t TUNE_RUN_MS = 1500;
constexpr uint32_t TUNE_COOLDOWN_MS = 500;

// Parameter pencarian
float testVal = 0.0f;
float testMin = 0.0f;
float testMax = 0.0f;
float testStep = 0.0f;

// Hasil terbaik
float bestKp = 0.0f, bestKi = 0.0f, baseKf = 0.0f;
float bestScore = 999999.0f;
float currentScore = 0.0f;

// Variabel Loop
uint32_t stateStartMs = 0;
bool isSettled = false;
uint32_t settleTimeMs = 0;

// Fungsi helper setup range
void setupPass(PassLevel level) {
    currentPass = level;
    bestScore = 999999.0f; // Reset skor tiap pass
    
    if (level == PassLevel::COARSE_KP) {
        testMin = 0.1f; testMax = 3.0f; testStep = 0.3f; testVal = testMin;
    } else if (level == PassLevel::FINE_KP) {
        testMin = max(0.01f, bestKp - 0.3f); testMax = bestKp + 0.3f; testStep = 0.05f; testVal = testMin;
    } else if (level == PassLevel::COARSE_KI) {
        testMin = 0.01f; testMax = 0.5f; testStep = 0.05f; testVal = testMin;
    } else if (level == PassLevel::FINE_KI) {
        testMin = max(0.001f, bestKi - 0.05f); testMax = bestKi + 0.05f; testStep = 0.01f; testVal = testMin;
    }
}

// Fungsi Scoring
void updateScore(float rpm, uint32_t elapsed) {
    float err = TUNE_TARGET_RPM - rpm;
    float absErr = fabsf(err);
    
    // IAE (Integral Absolute Error)
    currentScore += absErr * 0.04f;

    // Overshoot penalty
    if (err < -5.0f) {
        currentScore += fabsf(err) * 3.0f; // Penalti berat jika melampaui target
    }

    // Settling time: dianggap stabil jika masuk range +- 5% (10 RPM)
    if (!isSettled && absErr <= 10.0f && elapsed > 200) {
        isSettled = true;
        settleTimeMs = elapsed;
    }
}

// Draw to OLED (Progress)
void drawProgress(const char* passName, float val, float rpm, int prog, int total) {
    char buf1[30];
    char buf2[30];
    snprintf(buf1, sizeof(buf1), "%s: %.2f", passName, val);
    snprintf(buf2, sizeof(buf2), "RPM:%d [%d/%d]", (int)rpm, prog, total);
    oledShowStatus(buf1, buf2);
}

} // anonymous namespace

void startAutoTune(int motorIdx) {
    if (motorIdx < 0 || motorIdx >= 4) return;
    tMotorIdx = motorIdx;
    state = TuneState::INIT_MOTOR;
    Serial.printf("\n[AUTOTUNE] Motor %d - Start\n", motorIdx);
}

bool isAutoTunerRunning() { return state != TuneState::IDLE; }

void autoTunerTick() {
    if (state == TuneState::IDLE) return;
    uint32_t now = millis();

    switch (state) {
        case TuneState::INIT_MOTOR:
            motorStopAll();
            pidResetOne(tMotorIdx);
            stateStartMs = now;
            state = TuneState::FIND_KF_START;
            break;

        case TuneState::FIND_KF_START:
            Serial.println("[AUTOTUNE] Step 1: Cari Kf (Open Loop)");
            drawProgress("FIND_KF", 0, 0, 0, 1);
            pwmMotor(tMotorIdx, PWM_MAX / 2); // Gas 50%
            stateStartMs = now;
            state = TuneState::FIND_KF_WAIT;
            break;

        case TuneState::FIND_KF_WAIT:
            if (now - stateStartMs > 1000) {
                float maxRpm = getEncoderVelocityRpm(tMotorIdx);
                pwmMotor(tMotorIdx, 0);
                
                if (maxRpm < 10.0f) {
                    Serial.println("[AUTOTUNE] ERROR: RPM terlalu kecil (Encoder slip?)");
                    extern void oledShowStatus(const char*, const char*);
                    oledShowStatus("ERROR TUNE", "RPM < 10 !!");
                    stateStartMs = now;
                    state = TuneState::ERROR_WAIT;
                    return;
                }
                baseKf = (PWM_MAX / 2.0f) / maxRpm;
                Serial.printf("[AUTOTUNE] Kf base = %.3f\n", baseKf);
                
                setupPass(PassLevel::COARSE_KP);
                stateStartMs = now;
                state = TuneState::COOLDOWN;
            }
            break;

        case TuneState::ERROR_WAIT:
            // Tahan layar error selama 3 detik
            if (now - stateStartMs > 3000) {
                state = TuneState::IDLE;
            }
            break;

        case TuneState::COOLDOWN:
            if (now - stateStartMs > TUNE_COOLDOWN_MS) {
                if (currentPass == PassLevel::FINISHED) state = TuneState::DONE;
                else state = TuneState::EVAL_START;
            }
            break;

        case TuneState::EVAL_START: {
            currentScore = 0.0f;
            isSettled = false;
            settleTimeMs = TUNE_RUN_MS; // default max
            
            float kP = (currentPass == PassLevel::COARSE_KP || currentPass == PassLevel::FINE_KP) ? testVal : bestKp;
            float kI = (currentPass == PassLevel::COARSE_KI || currentPass == PassLevel::FINE_KI) ? testVal : bestKi;
            
            pidSetGains(tMotorIdx, kP, kI, 0.0f, baseKf);
            pidResetOne(tMotorIdx);
            
            Serial.printf("  Test Kp=%.2f Ki=%.3f ... ", kP, kI);
            
            stateStartMs = now;
            state = TuneState::EVAL_RUN;
            break;
        }

        case TuneState::EVAL_RUN:
            {
                uint32_t elapsed = now - stateStartMs;
                float rpm = getEncoderVelocityRpm(tMotorIdx);
                
                // Bypass Ramping di pid.ino (Tembak Step murni)
                float dt = 0.04f; 
                extern PIDState pidStates[];
                int pOut = pidCompute(pidStates[tMotorIdx], TUNE_TARGET_RPM, rpm, dt);
                pwmMotor(tMotorIdx, pOut);
                
                updateScore(rpm, elapsed);
                
                // Update OLED Progress
                int prog = ((testVal - testMin) / testStep) + 1;
                int total = ((testMax - testMin) / testStep) + 1;
                const char* passName = 
                    (currentPass == PassLevel::COARSE_KP) ? "COARSE Kp" :
                    (currentPass == PassLevel::FINE_KP) ? "FINE Kp" :
                    (currentPass == PassLevel::COARSE_KI) ? "COARSE Ki" : "FINE Ki";
                    
                drawProgress(passName, testVal, rpm, prog, total);
            }

            if (now - stateStartMs > TUNE_RUN_MS) {
                pwmMotor(tMotorIdx, 0); // Stop
                
                // Tambah penalti untuk settling time yang lama
                currentScore += (settleTimeMs / 100.0f) * 2.0f;
                
                Serial.printf("Score: %.1f (Settle: %d ms)\n", currentScore, settleTimeMs);
                
                if (currentScore < bestScore) {
                    bestScore = currentScore;
                    if (currentPass == PassLevel::COARSE_KP || currentPass == PassLevel::FINE_KP) bestKp = testVal;
                    if (currentPass == PassLevel::COARSE_KI || currentPass == PassLevel::FINE_KI) bestKi = testVal;
                }
                
                testVal += testStep;
                
                // Lanjut next test atau pindah Pass
                if (testVal > testMax + 0.001f) {
                    Serial.printf(">> Best Pass: Kp=%.2f Ki=%.2f\n", bestKp, bestKi);
                    if (currentPass == PassLevel::COARSE_KP) setupPass(PassLevel::FINE_KP);
                    else if (currentPass == PassLevel::FINE_KP) setupPass(PassLevel::COARSE_KI);
                    else if (currentPass == PassLevel::COARSE_KI) setupPass(PassLevel::FINE_KI);
                    else setupPass(PassLevel::FINISHED);
                }
                
                stateStartMs = now;
                state = TuneState::COOLDOWN;
            }
            break;

        case TuneState::DONE:
            Serial.println("\n[AUTOTUNE] FINAL RESULT:");
            Serial.printf("Motor %d -> Kp: %.2f | Ki: %.3f | Kd: 0.00 | Kf: %.3f\n", 
                           tMotorIdx, bestKp, bestKi, baseKf);
                           
            // Save & Terapkan
            pidSaveToNVS(tMotorIdx, bestKp, bestKi, 0.0f, baseKf);
            pidSetGains(tMotorIdx, bestKp, bestKi, 0.0f, baseKf);
            
            char buf[30];
            snprintf(buf, sizeof(buf), "M%d TUNED!", tMotorIdx);
            oledShowStatus("DONE!", buf);
            
            state = TuneState::IDLE;
            break;
    }
}