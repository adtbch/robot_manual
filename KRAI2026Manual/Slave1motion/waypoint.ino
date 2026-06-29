/*
 * =====================================================================
 * FILE    : waypoint.ino
 * PERAN   : PID waypoint controller — gerak ke (x, y, yaw).
 *
 *   Position P-controller:
 *     errX/Y (meter) → vx/vy (RPM, field-centric)
 *     → driveFieldCentricWithYawCorrection() yang sudah ada
 *
 *   Yaw dihandle oleh pidKinematicYaw (sudah ada di kinematik.ino).
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#include "waypoint.h"
#include "encoder.h"
#include "mpu.h"
#include "kinematik.h"
#include "motor.h"
#include <Preferences.h>

// =====================================================================
//  STATE
// =====================================================================

namespace {

static constexpr const char* WP_NVS_NS         = "waypoint";
static constexpr float       WP_DEFAULT_KP      = 200.0f;  // RPM per meter
static constexpr float       WP_DEFAULT_TOL_POS = 0.05f;   // 5 cm
static constexpr float       WP_DEFAULT_TOL_YAW = 3.0f;    // 3 derajat

WaypointState wpState = WaypointState::IDLE;

} // namespace

// Tunable (NVS-stored)
float wpKpXY       = WP_DEFAULT_KP;
float wpTolPos_m   = WP_DEFAULT_TOL_POS;
float wpTolYaw_deg = WP_DEFAULT_TOL_YAW;
float wpMaxSpeed   = WP_DEFAULT_MAX_RPM;

// Target saat ini — dibaca oleh serial.ino untuk status print
float wpTargetX_m    = 0.0f;
float wpTargetY_m    = 0.0f;
float wpTargetYaw_deg = 0.0f;

// =====================================================================
//  NVS
// =====================================================================

void initWaypointPid() {
    Preferences prefs;
    prefs.begin(WP_NVS_NS, true);
    wpKpXY       = prefs.getFloat("kp",      WP_DEFAULT_KP);
    wpTolPos_m   = prefs.getFloat("tol_pos", WP_DEFAULT_TOL_POS);
    wpTolYaw_deg = prefs.getFloat("tol_yaw", WP_DEFAULT_TOL_YAW);
    prefs.end();
    Serial.printf("[WP] Kp=%.1f TolPos=%.3fm TolYaw=%.1fdeg\n",
                  wpKpXY, wpTolPos_m, wpTolYaw_deg);
}

void saveWaypointPid() {
    Preferences prefs;
    prefs.begin(WP_NVS_NS, false);
    prefs.putFloat("kp",      wpKpXY);
    prefs.putFloat("tol_pos", wpTolPos_m);
    prefs.putFloat("tol_yaw", wpTolYaw_deg);
    prefs.end();
    Serial.printf("[WP] Saved: Kp=%.1f TolPos=%.3fm TolYaw=%.1fdeg\n",
                  wpKpXY, wpTolPos_m, wpTolYaw_deg);
}

// =====================================================================
//  PUBLIC API
// =====================================================================

void cancelWaypoint() {
    wpState = WaypointState::IDLE;
    rpmMotor(0, 0, 0, 0);
}

bool          isWaypointActive()  { return wpState == WaypointState::RUNNING; }
bool          isWaypointReached() { return wpState == WaypointState::REACHED; }
WaypointState getWaypointState()  { return wpState; }

// =====================================================================
//  TICK — panggil setiap loop()
// =====================================================================

void waypointTick(float x_cm, float y_cm, float yaw_deg, float maxSpeed) {
    // Konversi cm → meter (odomX/Y dalam meter)
    float x_m = x_cm * 0.01f;
    float y_m = y_cm * 0.01f;

    // Target berubah → restart
    if (x_m != wpTargetX_m || y_m != wpTargetY_m || yaw_deg != wpTargetYaw_deg) {
        wpTargetX_m = x_m; wpTargetY_m = y_m; wpTargetYaw_deg = yaw_deg;
        wpState = WaypointState::RUNNING;
    }
    if (wpState != WaypointState::RUNNING) return;

    static Jeda jeda;
    if (!jeda.check(40)) return;  // 25 Hz

    const float errX = wpTargetX_m  - odomX;
    const float errY = wpTargetY_m  - odomY;
    const float dist = sqrtf(errX * errX + errY * errY);

    if (dist < wpTolPos_m) {
        wpState = WaypointState::REACHED;
        rpmMotor(0, 0, 0, 0);
        Serial.printf("[WP] Reached! pos=(%.3f,%.3f)m yaw=%.1fdeg\n",
                      odomX, odomY, getYaw());
        return;
    }

    // P-controller: error (m) → velocity (RPM) di frame field
    const int vx = (int)constrain(wpKpXY * errX, -maxSpeed, maxSpeed);
    const int vy = (int)constrain(wpKpXY * errY, -maxSpeed, maxSpeed);

    driveFieldCentricWithYawCorrection(vx, vy, (int)wpTargetYaw_deg);
}
