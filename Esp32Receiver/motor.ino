bool initMotorPwm(const Motor &motor) {
	const double forwardSetup = ledcSetup(motor.channelForward, kPwmFreqHz, kPwmResolutionBits);
	const double reverseSetup = ledcSetup(motor.channelReverse, kPwmFreqHz, kPwmResolutionBits);
	ledcAttachPin(motor.pinForward, motor.channelForward);
	ledcAttachPin(motor.pinReverse, motor.channelReverse);

	ledcWrite(motor.channelForward, 0);
	ledcWrite(motor.channelReverse, 0);

	const bool forwardOk = forwardSetup > 0;
	const bool reverseOk = reverseSetup > 0;
	return forwardOk && reverseOk;
}

bool initAllMotorsPwm() {
	bool ok = true;
	ok = initMotorPwm(frontLeft) && ok;
	ok = initMotorPwm(frontRight) && ok;
	ok = initMotorPwm(rearLeft) && ok;
	ok = initMotorPwm(rearRight) && ok;
	return ok;
}

void driveMotorNormalized(const Motor &motor, float normalizedSpeed) {
	float speed = constrain(normalizedSpeed, -kPwmMax, kPwmMax);
	if (motor.invert) {
		speed = -speed;
	}

	const uint32_t duty = static_cast<uint32_t>(fabsf(speed));

	if (speed >= 0.0f) {
		ledcWrite(motor.channelForward, duty);
		ledcWrite(motor.channelReverse, 0);
	} else {
		ledcWrite(motor.channelForward, 0);
		ledcWrite(motor.channelReverse, duty);
	}
}

void stopAllMotors() {
	driveMotorNormalized(frontLeft, 0.0f);
	driveMotorNormalized(frontRight, 0.0f);
	driveMotorNormalized(rearLeft, 0.0f);
	driveMotorNormalized(rearRight, 0.0f);
}