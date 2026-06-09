/*
 * =====================================================================
 * FILE    : semi_auto_presets.ino
 * PERAN   : Preset semi-otomatis (servo + motor grip height).
 *           Share + Circle/Square → simpan sudut servo + tinggi motor.
 *           Circle/Square → gerakkan servo dulu, baru motor.
 *
 *           ** HANYA AKTIF DI MODE_GRIPPING **
 *
 * DATA TERSIMPAN (per preset):
 *   - Servo rotation angle (derajat)
 *   - Motor grip height (encoder count axis X)
 *
 * PROTOCOL SAVE:
 *   Controller kirim Share+Circle/Square → Master simpan ke NVS.
 *   Controller kedip LED PS4 sebagai indikasi (tanpa konfirmasi).
 *
 * SEQUENCE RECALL:
 *   1. Servo rotation bergerak ke sudut tersimpan
 *   2. Setelah servo sampai (atau timeout 2s), motor grip bergerak
 *   3. Setelah motor sampai (atau timeout 3s), selesai
 * =====================================================================
 */

#include "robot_config.h"

// ============================================================
// NVS KEYS
// ============================================================
static const char* NVS_SEMI_AUTO = "semi_auto";

// ============================================================
// DEFAULT VALUES
// ============================================================
static const int16_t DEFAULT_SERVO_ANGLE = 0;
static const long    DEFAULT_MOTOR_HEIGHT = 0;

// ============================================================
// PRESET DATA
// ============================================================
struct SemiAutoPreset {
    int16_t servoAngle;
    long    motorHeight;
};

static SemiAutoPreset gPresetCircle = { DEFAULT_SERVO_ANGLE, DEFAULT_MOTOR_HEIGHT };
static SemiAutoPreset gPresetSquare = { DEFAULT_SERVO_ANGLE, DEFAULT_MOTOR_HEIGHT };

// ============================================================
// SEQUENCE STATE
// ============================================================
enum class SemiAutoState : uint8_t {
    IDLE,
    MOVING_SERVO,
    WAITING_SERVO,
    MOVING_MOTOR,
    DONE
};

static SemiAutoState gSeqState = SemiAutoState::IDLE;
static SemiAutoPreset gActivePreset = {};
static unsigned long gSeqStartMs = 0;
static const unsigned long SERVO_TIMEOUT_MS = 2000;
static const unsigned long MOTOR_TIMEOUT_MS = 3000;

// ============================================================
// NVS FUNCTIONS
// ============================================================

void semiAutoPresetsLoad() {
    Preferences prefs;
    prefs.begin(NVS_SEMI_AUTO, true);
    gPresetCircle.servoAngle  = prefs.getShort("c_servo", DEFAULT_SERVO_ANGLE);
    gPresetCircle.motorHeight = prefs.getLong("c_motor",  DEFAULT_MOTOR_HEIGHT);
    gPresetSquare.servoAngle  = prefs.getShort("s_servo", DEFAULT_SERVO_ANGLE);
    gPresetSquare.motorHeight = prefs.getLong("s_motor",  DEFAULT_MOTOR_HEIGHT);
    prefs.end();
}

static void semiAutoPresetsSave() {
    Preferences prefs;
    prefs.begin(NVS_SEMI_AUTO, false);
    prefs.putShort("c_servo", gPresetCircle.servoAngle);
    prefs.putLong("c_motor",  gPresetCircle.motorHeight);
    prefs.putShort("s_servo", gPresetSquare.servoAngle);
    prefs.putLong("s_motor",  gPresetSquare.motorHeight);
    prefs.end();
}

// ============================================================
// SAVE PRESETS
// ============================================================

void semiAutoPresetSaveCircle(int servoAngle, long motorHeight) {
    gPresetCircle.servoAngle  = (int16_t)servoAngle;
    gPresetCircle.motorHeight = motorHeight;
    semiAutoPresetsSave();
    Serial.printf("[SEMI-AUTO] Circle saved: servo=%d° motor=%ld\n", servoAngle, motorHeight);
}

void semiAutoPresetSaveSquare(int servoAngle, long motorHeight) {
    gPresetSquare.servoAngle  = (int16_t)servoAngle;
    gPresetSquare.motorHeight = motorHeight;
    semiAutoPresetsSave();
    Serial.printf("[SEMI-AUTO] Square saved: servo=%d° motor=%ld\n", servoAngle, motorHeight);
}

// ============================================================
// RECALL PRESETS — mulai sequence servo → motor
// ============================================================

void semiAutoPresetRecallCircle() {
    if (gSeqState != SemiAutoState::IDLE) return;
    gActivePreset = gPresetCircle;
    gSeqState = SemiAutoState::MOVING_SERVO;
    gSeqStartMs = millis();
    Serial.printf("[SEMI-AUTO] Circle recall: servo=%d° motor=%ld\n",
                  gActivePreset.servoAngle, gActivePreset.motorHeight);
}

void semiAutoPresetRecallSquare() {
    if (gSeqState != SemiAutoState::IDLE) return;
    gActivePreset = gPresetSquare;
    gSeqState = SemiAutoState::MOVING_SERVO;
    gSeqStartMs = millis();
    Serial.printf("[SEMI-AUTO] Square recall: servo=%d° motor=%ld\n",
                  gActivePreset.servoAngle, gActivePreset.motorHeight);
}

// ============================================================
// TICK — dipanggil setiap loop()
// State machine: Servo → delay → Motor → selesai
// ============================================================

void semiAutoPresetTick() {
    if (gSeqState == SemiAutoState::IDLE) return;

    unsigned long nowMs = millis();

    switch (gSeqState) {

    case SemiAutoState::MOVING_SERVO: {
        // Gerakkan servo ke sudut tersimpan
        extern int gRotationAngle;
        extern void setServoAngle(int idServo, int angle);
        gRotationAngle = gActivePreset.servoAngle;
        setServoAngle(0, gRotationAngle);  // Servo 0 = rotation
        gSeqState = SemiAutoState::WAITING_SERVO;
        gSeqStartMs = nowMs;
        break;
    }

    case SemiAutoState::WAITING_SERVO: {
        // Tunggu servo sampai (timeout 2 detik)
        if ((nowMs - gSeqStartMs) >= SERVO_TIMEOUT_MS) {
            gSeqState = SemiAutoState::MOVING_MOTOR;
            gSeqStartMs = nowMs;
        }
        break;
    }

    case SemiAutoState::MOVING_MOTOR: {
        // Gerakkan motor grip ke tinggi tersimpan
        extern long getEncoderCount(uint8_t motorIndex);
        extern void pwmMotor(int idMotor, int pwmValue);
        extern void stopAllMotorTargets();

        long currentEnc = getEncoderCount(1);  // Motor 1 = axis X (grip)
        long targetEnc  = gActivePreset.motorHeight;
        long error      = targetEnc - currentEnc;

        if (abs(error) < 10 || (nowMs - gSeqStartMs) >= MOTOR_TIMEOUT_MS) {
            // Sampai atau timeout → stop
            pwmMotor(1, 0);
            gSeqState = SemiAutoState::DONE;
            gSeqStartMs = nowMs;
            Serial.println("[SEMI-AUTO] Sequence complete");
        } else {
            int speed = (error > 0) ? 300 : -300;  // GRIPPER_SPEED_DEFAULT
            pwmMotor(1, speed);
        }
        break;
    }

    case SemiAutoState::DONE: {
        // Brief delay lalu kembali ke IDLE
        if ((nowMs - gSeqStartMs) >= 200) {
            gSeqState = SemiAutoState::IDLE;
        }
        break;
    }

    default:
        gSeqState = SemiAutoState::IDLE;
        break;
    }
}

// ============================================================
// CHECK — apakah sequence sedang berjalan?
// ============================================================

bool semiAutoPresetIsActive() {
    return gSeqState != SemiAutoState::IDLE;
}
