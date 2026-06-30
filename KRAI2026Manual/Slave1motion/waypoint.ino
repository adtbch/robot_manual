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

struct WpPoint {
    float x_m;
    float y_m;
    float yaw_deg;
};

bool     wpComboActive = false;
uint8_t  wpComboIndex  = 0;
WpPoint  wpComboPts[2];

void applyWaypointTarget(float x_m, float y_m, float yaw_deg) {
    wpTargetX_m     = x_m;
    wpTargetY_m     = y_m;
    wpTargetYaw_deg = yaw_deg;
}

bool isWithinWaypointTol(float x_m, float y_m, float yaw_deg) {
    const float errX = x_m - odomX;
    const float errY = y_m - odomY;
    const float dist = sqrtf(errX * errX + errY * errY);
    return dist < wpTolPos_m && fabsf(getYawError(yaw_deg)) < wpTolYaw_deg;
}

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
    wpComboActive = false;
    wpComboIndex  = 0;
    wpState       = WaypointState::IDLE;
    rpmMotor(0, 0, 0, 0);
}

void startWaypoint(float x_cm, float y_cm, float yaw_deg, float maxRpm) {
    wpComboActive = false;
    wpComboIndex  = 0;
    wpMaxSpeed    = maxRpm;
    const float x_m = x_cm * 0.01f;
    const float y_m = y_cm * 0.01f;
    if (isWithinWaypointTol(x_m, y_m, yaw_deg)) {
        wpState = WaypointState::REACHED;
        rpmMotor(0, 0, 0, 0);
        Serial.printf("[WP] Already at target pos=(%.3f,%.3f)m yaw=%.1fdeg\n",
            odomX, odomY, getYaw());
            return;
        }
    applyWaypointTarget(x_m, y_m, yaw_deg);
    wpState = WaypointState::RUNNING;
}

void startWaypointCombo(float x1_cm, float y1_cm, float yaw1_deg,
                        float x2_cm, float y2_cm, float yaw2_deg, float maxRpm) {
    wpComboPts[0] = {x1_cm * 0.01f, y1_cm * 0.01f, yaw1_deg};
    wpComboPts[1] = {x2_cm * 0.01f, y2_cm * 0.01f, yaw2_deg};
    wpComboActive = true;
    wpComboIndex  = 0;
    wpMaxSpeed    = maxRpm;
    applyWaypointTarget(wpComboPts[0].x_m, wpComboPts[0].y_m, wpComboPts[0].yaw_deg);
    wpState = WaypointState::RUNNING;
    Serial.printf("[WP] Combo start P1=(%.0f,%.0f)cm yaw=%.1f → P2=(%.0f,%.0f)cm yaw=%.1f\n",
                  x1_cm, y1_cm, yaw1_deg, x2_cm, y2_cm, yaw2_deg);
}

bool          isWaypointActive()      { return wpState == WaypointState::RUNNING; }
bool          isWaypointReached()     { return wpState == WaypointState::REACHED; }
WaypointState getWaypointState()      { return wpState; }
bool          isWaypointComboActive() { return wpComboActive; }
uint8_t       getWaypointComboStep()  { return wpComboActive ? (uint8_t)(wpComboIndex + 1) : 0; }

// =====================================================================
//  TICK — panggil setiap loop()
// =====================================================================

void waypointTick(float x_m, float y_m, float yaw_deg, float maxSpeed) {
    // if (wpState != WaypointState::RUNNING) return;

    wpTargetX_m     = x_m;
    wpTargetY_m     = y_m;
    wpTargetYaw_deg = yaw_deg;
    
    static Jeda jeda;
    if (!jeda.check(40)) return;  // 25 Hz

    float errX = wpTargetX_m - odomX;
    float errY = wpTargetY_m - odomY;

    if (isWithinWaypointTol(wpTargetX_m, wpTargetY_m, wpTargetYaw_deg)) {
        if (wpComboActive && wpComboIndex == 0) {
            wpComboIndex = 1;
            applyWaypointTarget(wpComboPts[1].x_m, wpComboPts[1].y_m, wpComboPts[1].yaw_deg);
            Serial.printf("[WP] Point 1 OK → P2 (%.0f,%.0f)cm yaw=%.1fdeg\n",
                          wpComboPts[1].x_m * 100.0f, wpComboPts[1].y_m * 100.0f,
                          wpComboPts[1].yaw_deg);
            // Recalc for new target; if also in range, next tick will catch it
            errX = wpTargetX_m - odomX;
            errY = wpTargetY_m - odomY;
        } else {
            wpComboActive = false;
            wpComboIndex  = 0;
            wpState = WaypointState::REACHED;
            rpmMotor(0, 0, 0, 0);
            Serial.printf("[WP] Reached! pos=(%.3f,%.3f)m yaw=%.1fdeg\n",
                          odomX, odomY, getYaw());
            Serial1.printf("WP: REACHED\n");
            return;
        }
    }

    const int vx = (int)constrain(wpKpXY * errX, -maxSpeed, maxSpeed);
    const int vy = (int)constrain(wpKpXY * errY, -maxSpeed, maxSpeed);

    driveFieldCentricWithYawCorrection(vx, vy, (int)wpTargetYaw_deg);
}
