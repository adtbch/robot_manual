/*
 * =====================================================================
 * FILE    : gripper_control.ino
 * PERAN   : Toggle servo gripper buka/tutup dengan tombol Circle PS4.
 *
 * PERILAKU:
 *   - Boot → gripper posisi BUKA (0 derajat)
 *   - Tekan Circle → toggle (BUKA ↔ TUTUP)
 *   - Tekan Circle lagi → toggle balik
 *
 * DEBOUNCE:
 *   - Tunggu 300ms setelah press sebelum bisa trigger lagi
 * =====================================================================
 */

#include "robot_config.h"

#define GRIPPER_SERVO_ID  1   // index di vector servos (servoGrib)
#define GRIPPER_ANGLE_OPEN   0
#define GRIPPER_ANGLE_CLOSED 90

#define BTN_CIRCLE_MASK (1u << 1)

static bool gGripperOpen = true;
static unsigned long gLastToggleMs = 0;
static const unsigned long DEBOUNCE_MS = 300;

// =====================================================================
//  INIT — set gripper ke posisi buka saat boot
// =====================================================================

void gripper_init() {
    setServoAngle(GRIPPER_SERVO_ID, GRIPPER_ANGLE_OPEN);
    gGripperOpen = true;
    Serial.println("[GRIPPER] Init — posisi BUKA");
}

// =====================================================================
//  TICK — dipanggil setiap loop() dengan ControlPacket
// =====================================================================

void gripper_tick(const ControlPacket &pkt) {
    // Hanya proses saat tombol baru ditekan (bukan tahan)
    static bool lastCircle = false;
    bool circleNow = (pkt.buttons & BTN_CIRCLE_MASK) != 0;

    // Edge detection: baru saja ditekan (rising edge)
    if (circleNow && !lastCircle) {
        unsigned long nowMs = millis();
        if (nowMs - gLastToggleMs >= DEBOUNCE_MS) {
            gLastToggleMs = nowMs;
            gGripperOpen = !gGripperOpen;

            int angle = gGripperOpen ? GRIPPER_ANGLE_OPEN : GRIPPER_ANGLE_CLOSED;
            setServoAngle(GRIPPER_SERVO_ID, angle);

            Serial.printf("[GRIPPER] %s\n", gGripperOpen ? "BUKA" : "TUTUP");
        }
    }

    lastCircle = circleNow;
}
