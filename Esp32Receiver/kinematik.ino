float clampCommand(float value) {
	return constrain(value, -kPwmMax, kPwmMax);
}

void moveXYW(float xPwm, float yPwm, float wPwm) {
	// Inverse kinematics mecanum langsung dalam domain PWM.
	float speedFrontLeft = xPwm - yPwm - wPwm;
	float speedFrontRight = xPwm + yPwm + wPwm;
	float speedRearLeft = xPwm + yPwm - wPwm;
	float speedRearRight = xPwm - yPwm + wPwm;

	float maxAbsSpeed = fabsf(speedFrontLeft);
	maxAbsSpeed = max(maxAbsSpeed, fabsf(speedFrontRight));
	maxAbsSpeed = max(maxAbsSpeed, fabsf(speedRearLeft));
	maxAbsSpeed = max(maxAbsSpeed, fabsf(speedRearRight));
	if (maxAbsSpeed < 1.0f) {
		maxAbsSpeed = 1.0f;
	}

	if (maxAbsSpeed > kPwmMax) {
		const float scale = static_cast<float>(kPwmMax) / maxAbsSpeed;
		speedFrontLeft *= scale;
		speedFrontRight *= scale;
		speedRearLeft *= scale;
		speedRearRight *= scale;
	}

	const float pwmFrontLeft = clampCommand(speedFrontLeft);
	const float pwmFrontRight = clampCommand(speedFrontRight);
	const float pwmRearLeft = clampCommand(speedRearLeft);
	const float pwmRearRight = clampCommand(speedRearRight);

	driveMotorNormalized(frontLeft, pwmFrontLeft);
	driveMotorNormalized(frontRight, pwmFrontRight);
	driveMotorNormalized(rearLeft, pwmRearLeft);
	driveMotorNormalized(rearRight, pwmRearRight);
}

void moveX(float xPwm) {
	moveXYW(xPwm, 0.0f, 0.0f);
}

void moveXY(float xPwm, float yPwm) {
	moveXYW(xPwm, yPwm, 0.0f);
}

void applyMecanumCommand(float vx, float vy, float wz) {
	moveXYW(vx, vy, wz);
}

bool readVelocityCommand(float &vx, float &vy, float &wz) {
	if (!Serial.available()) {
		return false;
	}

	String line = Serial.readStringUntil('\n');
	line.trim();
	if (line.length() == 0) {
		return false;
	}

	if (line.equalsIgnoreCase("STOP")) {
		vx = 0.0f;
		vy = 0.0f;
		wz = 0.0f;
		return true;
	}

	const int firstSpace = line.indexOf(' ');
	const int secondSpace = line.indexOf(' ', firstSpace + 1);
	if (firstSpace < 0 || secondSpace < 0) {
		return false;
	}

	vx = line.substring(0, firstSpace).toFloat();
	vy = line.substring(firstSpace + 1, secondSpace).toFloat();
	wz = line.substring(secondSpace + 1).toFloat();
	return true;
}
