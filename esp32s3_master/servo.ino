// KODE KHUSUS ARDUINO CORE ESP32 v2.x.x
#include "robot_config.h" 
// Motor config vector (definition)
std::vector<servoConfig> servos = {
    {servoRotation, 4}, 
    {servoGrib, 5},   
};/

void setupServos() {
  for (size_t i = 0; i < servos.size(); i++) {
    ledcSetup(servos[i].ledc_channel, servoFrequency, servoResolution);
    ledcAttachPin(servos[i].pin, servos[i].ledc_channel);
  }
}

void setServoAngle(int idServo, int angle) {
  // 1. Ubah sudut (0-180) menjadi lebar pulsa waktu (500us sampai 2400us)
  // Rentang standar servo: 500us (0 derajat) hingga 2400us (180 derajat)
  long pulseWidth = map(angle, 0, 180, 500, 2400);
  
  // 2. Petakan lebar pulsa ke nilai Duty Cycle resolusi 14-bit (0 hingga 16383)
  // Rumus: (pulseWidth / 20000us) * 16383
  long duty = (pulseWidth * 16383) / 20000;
  
  ledcWrite(servos[idServo].ledc_channel, duty);
}
