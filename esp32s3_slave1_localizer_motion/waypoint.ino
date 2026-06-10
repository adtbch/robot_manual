#include "robot_config.h"

static bool waypointActive = false;
static float wpTargetX_mm = 0.0f;
static float wpTargetY_mm = 0.0f;
static float wpTargetYaw_deg = 0.0f;
static int wpMaxSpeedXY = 500;
static int wpMaxSpeedYaw = 200;
static float wpThresholdXY_mm = 10.0f;
static float wpThresholdYaw_deg = 3.0f;

PIDState pidWaypointX;
PIDState pidWaypointY;

void initWaypointPid() {
  pidWaypointX.kp = 2.0f; pidWaypointX.ki = 0.0f; pidWaypointX.kd = 0.0f;
  pidWaypointY.kp = 2.0f; pidWaypointY.ki = 0.0f; pidWaypointY.kd = 0.0f;
  pidWaypointX.reset();
  pidWaypointY.reset();
}

void setWaypoint(float targetX_mm, float targetY_mm, float targetYaw_deg,
                 int maxSpeedXY, int maxSpeedYaw,
                 float thresholdXY_mm, float thresholdYaw_deg) {
  wpTargetX_mm = targetX_mm;
  wpTargetY_mm = targetY_mm;
  wpTargetYaw_deg = targetYaw_deg;
  wpMaxSpeedXY = maxSpeedXY;
  wpMaxSpeedYaw = maxSpeedYaw;
  wpThresholdXY_mm = thresholdXY_mm;
  wpThresholdYaw_deg = thresholdYaw_deg;
  pidWaypointX.reset();
  pidWaypointY.reset();
  waypointActive = true;
}

void cancelWaypoint() {
  waypointActive = false;
  motorStopAll();
}

bool isWaypointActive() {
  return waypointActive;
}

void getWaypointTarget(float &x_mm, float &y_mm, float &yaw_deg) {
  x_mm = wpTargetX_mm;
  y_mm = wpTargetY_mm;
  yaw_deg = wpTargetYaw_deg;
}

void getWaypointStatus(float &deltaX_mm, float &deltaY_mm, float &distance_mm, int &vx_cmd, int &vy_cmd, int &vtheta_cmd) {
  deltaX_mm = wpTargetX_mm - robotPosX_mm;
  deltaY_mm = wpTargetY_mm - robotPosY_mm;
  distance_mm = sqrtf(deltaX_mm * deltaX_mm + deltaY_mm * deltaY_mm);
  vx_cmd = constrain((int)(pidWaypointX.kp * deltaX_mm), -wpMaxSpeedXY, wpMaxSpeedXY);
  vy_cmd = constrain((int)(pidWaypointY.kp * deltaY_mm), -wpMaxSpeedXY, wpMaxSpeedXY);

  float currentYaw = getYaw();
  float yawError = wpTargetYaw_deg - currentYaw;
  while (yawError > 180.0f) yawError -= 360.0f;
  while (yawError < -180.0f) yawError += 360.0f;
  vtheta_cmd = constrain(pidComputeYaw(pidKinematicYaw, wpTargetYaw_deg, currentYaw, 0.04f), -wpMaxSpeedYaw, wpMaxSpeedYaw);
}

bool executewaypoint() {
  if (!waypointActive) return true;

  float deltaX = wpTargetX_mm - robotPosX_mm;
  float deltaY = wpTargetY_mm - robotPosY_mm;
  float distance = sqrtf(deltaX * deltaX + deltaY * deltaY);
  float currentYaw = getYaw();

  float yawError = wpTargetYaw_deg - currentYaw;
  while (yawError > 180.0f) yawError -= 360.0f;
  while (yawError < -180.0f) yawError += 360.0f;

  int vx = constrain((int)(pidWaypointX.kp * deltaX), -wpMaxSpeedXY, wpMaxSpeedXY);
  int vy = constrain((int)(pidWaypointY.kp * deltaY), -wpMaxSpeedXY, wpMaxSpeedXY);
  int vtheta = constrain(pidComputeYaw(pidKinematicYaw, wpTargetYaw_deg, currentYaw, 0.04f), -wpMaxSpeedYaw, wpMaxSpeedYaw);

  driveFieldCentric(vx, vy, vtheta);

  if (distance < wpThresholdXY_mm && fabsf(yawError) <= wpThresholdYaw_deg) {
    waypointActive = false;
    motorStopAll();
    return true;
  }

  return false;
}

bool executewaypoint(float targetX_mm, float targetY_mm, float targetYaw_deg,
                     int maxSpeedXY, int maxSpeedYaw,
                     float thresholdXY_mm, float thresholdYaw_deg) {
  setWaypoint(targetX_mm, targetY_mm, targetYaw_deg, maxSpeedXY, maxSpeedYaw, thresholdXY_mm, thresholdYaw_deg);
  return executewaypoint();
}
