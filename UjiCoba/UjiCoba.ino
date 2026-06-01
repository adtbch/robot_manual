/*
 * ESP32 Komunikasi 2 Arah dengan ESP-NOW
 * 
 * Fungsi: Mengirim DAN menerima pesan antar ESP32 menggunakan ESP-NOW (wireless)
 * File ini diupload ke KEDUA ESP32 (Unit A dan Unit B)
 * 
 * Cara Pakai:
 * 1. Upload sketch ini ke ESP32 pertama (Unit A)
 * 2. Buka Serial Monitor Unit A, catat MAC Address yang muncul
 * 3. Edit baris broadcastAddress[] dengan MAC Address Unit A
 * 4. Upload sketch ke ESP32 kedua (Unit B)
 * 5. Buka Serial Monitor kedua ESP32 (115200 baud)
 * 6. Ketik pesan di salah satu Serial Monitor dan tekan Enter
 * 7. Pesan akan dikirim via ESP-NOW (wireless, tanpa kabel!)
 * 
 * Trial Error:
 * - Pastikan kedua ESP32 menggunakan MAC Address yang benar
 * - Jarak maksimal sekitar 100-200 meter (tergantung kondisi)
 * - Tidak perlu WiFi router, komunikasi langsung ESP32 ke ESP32
 */

#include <esp_now.h>
#include <WiFi.h>

// ============================================
// KONFIGURASI MAC ADDRESS
// ============================================
// GANTI dengan MAC Address ESP32 lawan bicara
// Cara dapat MAC: Upload sketch ini, buka Serial Monitor, catat MAC yang muncul
uint8_t broadcastAddress[] = {0xA4, 0xCB, 0x8F, 0xD9, 0x2B, 0xA0};

// ============================================
// STRUKTUR DATA
// ============================================
// Struktur untuk mengirim pesan
typedef struct struct_message {
    char text[200];  // Pesan teks maksimal 200 karakter
} struct_message;

// Variabel untuk pesan yang akan dikirim dan diterima
struct_message outgoingMessage;
struct_message incomingMessage;

// ============================================
// VARIABEL GLOBAL
// ============================================
String inputBuffer = "";           // Buffer untuk input dari Serial Monitor
unsigned long sentCount = 0;       // Counter pesan yang dikirim
unsigned long receivedCount = 0;   // Counter pesan yang diterima
String lastStatus = "";            // Status pengiriman terakhir

esp_now_peer_info_t peerInfo;

// ============================================
// CALLBACK FUNCTIONS
// ============================================

// Callback ketika data berhasil dikirim
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\n[STATUS KIRIM] ");
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Berhasil terkirim!");
    lastStatus = "Delivery Success :)";
  } else {
    Serial.println("Gagal terkirim!");
    lastStatus = "Delivery Fail :(";
  }
}

// Callback ketika data diterima
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));
  receivedCount++;
  
  Serial.print("[TERIMA #");
  Serial.print(receivedCount);
  Serial.print("] ");
  Serial.println(incomingMessage.text);
}

// ============================================
// SETUP - Inisialisasi
// ============================================
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  delay(1000);
  
  Serial.println();
  Serial.println("===========================================");
  Serial.println("  ESP32 ESP-NOW KOMUNIKASI 2 ARAH");
  Serial.println("===========================================");
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  
  // Tampilkan MAC Address ESP32 ini
  Serial.print("MAC Address ESP32 ini: ");
  Serial.println(WiFi.macAddress());
  Serial.println("CATAT MAC ini untuk dimasukkan ke ESP32 lain!");
  Serial.println("-------------------------------------------");

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println("ESP-NOW berhasil diinisialisasi");

  // Register callback untuk kirim data
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer (ESP32 lawan bicara)
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  Serial.println("Peer berhasil ditambahkan");
  
  // Register callback untuk terima data
  esp_now_register_recv_cb(OnDataRecv);
  
  Serial.println("-------------------------------------------");
  Serial.println("Ketik pesan dan tekan Enter untuk mengirim");
  Serial.println("===========================================");
  Serial.println();
}
 
// ============================================
// LOOP - Program Utama
// ============================================
void loop() {
  // ========================================
  // BAGIAN 1: KIRIM PESAN
  // Baca input dari Serial Monitor dan kirim via ESP-NOW
  // ========================================
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    // Jika Enter ditekan, kirim pesan
    if (inChar == '\n' || inChar == '\r') {
      if (inputBuffer.length() > 0) {
        // Copy pesan ke struktur outgoing
        inputBuffer.toCharArray(outgoingMessage.text, 200);
        
        // Kirim via ESP-NOW
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingMessage, sizeof(outgoingMessage));
        
        sentCount++;
        
        // Tampilkan konfirmasi
        Serial.print("[KIRIM #");
        Serial.print(sentCount);
        Serial.print("] ");
        Serial.println(inputBuffer);
        
        if (result == ESP_OK) {
          Serial.println("-> Mengirim...");
        } else {
          Serial.println("-> Error saat mengirim!");
        }
        
        // Kosongkan buffer
        inputBuffer = "";
      }
    } else {
      // Tambahkan karakter ke buffer
      inputBuffer += inChar;
    }
  }
  
  // Delay kecil untuk stabilitas
  delay(10);
}