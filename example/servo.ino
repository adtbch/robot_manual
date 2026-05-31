// KODE KHUSUS ARDUINO CORE ESP32 v2.x.x
const int servoPin1 = 2;    
const int servoPin2 = 42;   

const int ledcChannel1 = 0; 
const int ledcChannel2 = 1; 

const int freq = 50;       // Frekuensi 50Hz (Periode 20ms = 20000us)
const int resolution = 14; // Mengubah ke resolusi 14-bit agar hitungan map() lebih pas

void setup() {
  // Setup PWM untuk Core v2.x.x
  ledcSetup(ledcChannel1, freq, resolution);
  ledcAttachPin(servoPin1, ledcChannel1);
  
  ledcSetup(ledcChannel2, freq, resolution);
  ledcAttachPin(servoPin2, ledcChannel2);
}

void loop() {
  // Gerakkan ke 0 derajat
  setServoAngle(ledcChannel1, 0);
  setServoAngle(ledcChannel2, 0);
  delay(1500);
  
  // Gerakkan ke 90 derajat
  setServoAngle(ledcChannel1, 90);
  setServoAngle(ledcChannel2, 90);
  delay(1500);
  
  // Gerakkan ke 180 derajat
  setServoAngle(ledcChannel1, 180);
  setServoAngle(ledcChannel2, 180);
  delay(1500);
}

void setServoAngle(int channel, int angle) {
  // 1. Ubah sudut (0-180) menjadi lebar pulsa waktu (500us sampai 2400us)
  // Rentang standar servo: 500us (0 derajat) hingga 2400us (180 derajat)
  long pulseWidth = map(angle, 0, 180, 500, 2400);
  
  // 2. Petakan lebar pulsa ke nilai Duty Cycle resolusi 14-bit (0 hingga 16383)
  // Rumus: (pulseWidth / 20000us) * 16383
  long duty = (pulseWidth * 16383) / 20000;
  
  ledcWrite(channel, duty);
}
