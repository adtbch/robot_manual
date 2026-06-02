// Sesuaikan nomor pin Hardware Serial 2 (Serial2) pada ESP32 Anda
// Biasanya default ESP32: RX2 = Pin 16, TX2 = Pin 17
#define RXD2 12
#define TXD2 11
#define PIN_SET 19  // MISALNYA pin SET WSN-31 terhubung ke GPIO 4 ESP32

void setup() {
  Serial.begin(9600);   // Komunikasi ESP32 ke PC
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // Komunikasi ke WSN-31
  
  // Mengatur GPIO ESP32 menjadi OUTPUT LOW (Sama dengan menyambung ke GND)
  pinMode(PIN_SET, OUTPUT);
  digitalWrite(PIN_SET, LOW); 
  
  Serial.println("WSN-31 otomatis masuk Mode Konfigurasi...");
}

void loop() {
  // Meneruskan ketikan dari PC ke WSN-31
  if (Serial.available()) {
    char c = Serial.read();
    
    // Jika Anda menekan Enter, kirim karakter penutup lengkap ke WSN-31
    if (c == '\n' || c == '\r') {
      Serial2.print("\r\n"); 
    } else {
      Serial2.write(c); // Kirim karakter biasa (A, T, dll)
    }
  }
  
  // Meneruskan balasan dari WSN-31 ke layar PC
  if (Serial2.available()) {
    Serial.write(Serial2.read());
  }
}
