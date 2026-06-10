#include "robot_config.h"

float robotPosX_mm = 0.0f;
float robotPosY_mm = 0.0f;

void resetOdometry() {
  robotPosX_mm = 0.0f;
  robotPosY_mm = 0.0f;
}

void setOdometry(float x_mm, float y_mm) {
  robotPosX_mm = x_mm;
  robotPosY_mm = y_mm;
}

void updateOdometry() {
  static uint32_t lastOdometryMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastOdometryMs < 40) return;
  float dt = (lastOdometryMs > 0) ? (nowMs - lastOdometryMs) * 0.001f : 0.04f;
  lastOdometryMs = nowMs;

  float rpmFR = getEncoderVelocityRpm(0);
  float rpmFL = getEncoderVelocityRpm(1);
  float rpmBR = getEncoderVelocityRpm(2);
  float rpmBL = getEncoderVelocityRpm(3);

  float vx_robot_rpm = (rpmFR + rpmFL + rpmBR + rpmBL) * 0.25f;
  float vy_robot_rpm = (rpmFR - rpmFL - rpmBR + rpmBL) * 0.25f;

  float wheelCircum_m = 2.0f * PI * radiusRoda;
  float vx_robot_ms = vx_robot_rpm * wheelCircum_m / 60.0f;
  float vy_robot_ms = vy_robot_rpm * wheelCircum_m / 60.0f;

  float yawRad = getYaw() * DEG_TO_RAD;
  float c = cosf(yawRad);
  float s = sinf(yawRad);

  float vx_field_ms = vx_robot_ms * c + vy_robot_ms * s;
  float vy_field_ms = -vx_robot_ms * s + vy_robot_ms * c;

  robotPosX_mm += vx_field_ms * dt * 1000.0f;
  robotPosY_mm += vy_field_ms * dt * 1000.0f;
}
