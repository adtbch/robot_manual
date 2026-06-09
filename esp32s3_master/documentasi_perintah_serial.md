# Dokumentasi Perintah Serial USB (ESP32-S3 Master)

Dokumentasi ini menjelaskan cara mengontrol motor dan servo pada robot arm secara manual menggunakan koneksi Serial USB (115200 baud).

---

## 🔌 Cara Menghubungkan
1. Hubungkan ESP32-S3 Master ke PC menggunakan kabel data USB.
2. Buka **Serial Monitor** (Arduino IDE, VS Code, PuTTY, dll).
3. Set Baudrate ke: **`115200`**
4. Set Line Ending ke: **`Newline (\n)`** atau **`Both NL & CR`**.

---

## 📝 Format Perintah
Perintah dikirimkan dalam bentuk teks/string dengan format case-insensitive (huruf besar/kecil tidak berpengaruh).

### 1. Kontrol Motor (Axis X & Z)
Menggerakkan motor ke posisi encoder target menggunakan feedback real-time.
*   **Format:** `motorX <posisi>` / `motorZ <posisi>`
*   **Contoh:**
    *   `motorX 100` $\rightarrow$ Menggerakkan axis X ke koordinat encoder `100`.
    *   `motorZ 1500` $\rightarrow$ Menggerakkan axis Z ke koordinat encoder `1500`.

### 2. Kontrol Servo
Mengatur sudut servo (0 hingga 180 derajat).
*   **Format:** `servo1 <sudut>` / `servo2 <sudut>`
*   **Contoh:**
    *   `servo1 90` $\rightarrow$ Set servo rotation (servo 1) ke `90` derajat (posisi tengah).
    *   `servo2 180` $\rightarrow$ Set servo gripper (servo 2) ke `180` derajat.

### 3. Cek Status Sensor & Motor
Mendapatkan posisi encoder saat ini serta status pergerakan motor.
*   **Format:** `status`
*   **Contoh output:**
    ```text
    === Robot Status ===
    Motor X position: 102 (target: 100)
    Motor Z position: 1498 (target: 1500)
    ```

### 4. Emergency Stop
Menghentikan paksa semua motor dan membatalkan seluruh target posisi aktif.
*   **Format:** `stop`
*   **Contoh:**
    *   `stop` $\rightarrow$ Motor langsung berhenti seketika.

---

## ⚠️ Catatan Penting
*   **Homing & Centering:** Saat pertama kali menyala (setup), robot akan otomatis melakukan homing (mundur sampai menyentuh limit switch), lalu bergerak ke posisi tengah (`CENTER_POSITION`). Tunggu sampai muncul pesan **`Robot ready!`** di Serial Monitor sebelum mengirimkan perintah.
*   **Proportional Speed:** Kecepatan motor akan melambat secara otomatis saat mendekati target koordinat untuk mencegah overshoot/benturan fisik.
*   **Toleransi Posisi:** Target dianggap tercapai jika selisih posisi encoder dengan target kurang dari `30` count (`MOTOR_POSITION_TOLERANCE`).
servo2 0