/*
 * =====================================================================
 * FILE    : oled.ino
 * PERAN   : OLED display — multi-mode tampilan.
 *           Boot button untuk ganti mode.
 *
 * MODE 0  : Yaw (compass dial + degree)
 * MODE 1  : Debug (encoder count + RPM + odometry)
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#include "oled.h"
#include "button.h"
#include "i2c_bus.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =====================================================================
//  STATE
// =====================================================================

namespace {

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

OledMode currentMode = OLED_MODE_YAW;
bool oledReady = false;

} // anonymous namespace

// =====================================================================
//  DISPLAY — MODE 0: YAW
// =====================================================================

namespace {

void drawYawMode() {
    float yawDeg = getYaw();
    float slopeDeg = getSlopeDeg();  // roll: maju nanjak +, mundur nanjak −

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // Compass dial
    int cx = 40, cy = 32, r = 22;
    display.drawCircle(cx, cy, r, SSD1306_WHITE);

    // Cardinal markers
    display.fillCircle(cx, cy - r + 2, 2, SSD1306_WHITE); // N
    display.fillCircle(cx, cy + r - 2, 2, SSD1306_WHITE); // S
    display.fillCircle(cx - r + 2, cy, 2, SSD1306_WHITE); // W
    display.fillCircle(cx + r - 2, cy, 2, SSD1306_WHITE); // E

    // Arrow
    float rad = yawDeg * (PI / 180.0f);
    int ax = cx + (int)(r * 0.7f * sin(rad));
    int ay = cy - (int)(r * 0.7f * cos(rad));
    display.drawLine(cx, cy, ax, ay, SSD1306_WHITE);
    display.fillCircle(ax, ay, 3, SSD1306_WHITE);

    // Yaw text
    display.setCursor(68, 8);
    display.setTextSize(2);
    display.print((int)yawDeg);
    display.setTextSize(1);
    display.print(" deg");

    // Cardinal direction
    display.setCursor(68, 30);
    display.setTextSize(1);
    if (yawDeg > -22.5f && yawDeg <= 22.5f)        display.print("NORTH");
    else if (yawDeg > 22.5f && yawDeg <= 67.5f)     display.print("NE");
    else if (yawDeg > 67.5f && yawDeg <= 112.5f)    display.print("EAST");
    else if (yawDeg > 112.5f && yawDeg <= 157.5f)   display.print("SE");
    else if (yawDeg > 157.5f || yawDeg <= -157.5f)  display.print("SOUTH");
    else if (yawDeg > -157.5f && yawDeg <= -112.5f) display.print("SW");
    else if (yawDeg > -112.5f && yawDeg <= -67.5f)  display.print("WEST");
    else if (yawDeg > -67.5f && yawDeg <= -22.5f)   display.print("NW");

    // Bottom bar — S=slope (roll), P=pitch
    display.drawFastHLine(0, 54, 128, SSD1306_WHITE);
    display.setCursor(0, 56);
    display.print("S:");
    display.print((int)slopeDeg);
    display.print(" P:");
    display.print((int)getPitch());
}

// =====================================================================
//  DISPLAY — MODE 1: DEBUG (ENC + RPM + ODO)
// =====================================================================

void drawDebugMode() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    // Header
    display.setCursor(0, 0);
    display.print("== DEBUG == Yaw:");
    display.print((int)getYaw());

    // Encoder RPM (4 motor)
    display.setCursor(0, 12);
    display.print("FR:");
    display.print((int)getEncoderVelocityRpm(0));
    display.print(" FL:");
    display.print((int)getEncoderVelocityRpm(1));

    display.setCursor(0, 22);
    display.print("BR:");
    display.print((int)getEncoderVelocityRpm(2));
    display.print(" BL:");
    display.print((int)getEncoderVelocityRpm(3));

    // External encoder count
    display.setCursor(0, 34);
    display.print("EXT:");
    for (int i = 0; i < 4; i++) {
        display.print((int32_t)getExtEncoderCount(i));
        if (i < 3) display.print(",");
    }

    // Odometry
    display.setCursor(0, 46);
    display.print("ODO X:");
    display.print(odomX, 2);
    display.print(" Y:");
    display.print(odomY, 2);

    display.setCursor(0, 56);
    display.print("T:");
    display.print(odomTheta, 1);
    display.print("deg  MODE:DBG");
}

} // anonymous namespace

// =====================================================================
//  PUBLIC API
// =====================================================================

bool setupOLED() {
    // Probe I2C dulu tanpa spam NACK error
    uint8_t foundAddr = 0;
    Wire.beginTransmission(OLED_I2C_ADDR);
    if (Wire.endTransmission() == 0) {
        foundAddr = OLED_I2C_ADDR;
    } else {
        Wire.beginTransmission(0x3D);
        if (Wire.endTransmission() == 0) {
            foundAddr = 0x3D;
        }
    }

    if (foundAddr == 0) {
        Serial.println("OLED: not found on I2C (0x3C & 0x3D)");
        return false;
    }

    Serial.printf("OLED: found at 0x%02X\n", foundAddr);
    if (!display.begin(SSD1306_SWITCHCAPVCC, foundAddr)) {
        Serial.println("OLED: begin() failed");
        return false;
    }
    display.setRotation(2); // Memutar layar 180 derajat
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("OLED OK");
    display.display();
    delay(500);

    oledReady = true;
    Serial.println("OLED: READY");
    return true;
}

void updateOLED() {
    if (!oledReady) { Serial.println("OLED: not ready"); return; }

    // display.display() kirim ~1KB via I2C → ~80ms; throttle 100ms agar loop() tidak tercekik
    static Jeda jedaOled;
    if (!jedaOled.check(100)) return;

    // Prioritas: kalau tombol sedang ditahan ≥ 3 detik, tampilkan pesan khusus
    if (isButtonLongHolding()) {
        if (!I2cBus::acquire(I2cBus::Owner::OLED)) { Serial.println("OLED: acquire fail (hold)"); return; }
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.println("AUTOTUNE");
        display.setTextSize(1);
        display.setCursor(0, 30);
        display.println("Tahan 3 detik...");
        display.println("Lepas utk mulai");
        display.display();
        I2cBus::release(I2cBus::Owner::OLED);
        return;
    }

    // Jika autotuner sedang jalan, biarkan autotuner yang menggambar layar
    if (isAutoTunerRunning()) {
        return;
    }

    // Ganti mode HANYA jika tombol ditekan singkat (< 3 detik)
    if (isButtonShortPressed()) {
        currentMode = (OledMode)((currentMode + 1) % OLED_MODE_COUNT);
    }

    if (!I2cBus::acquire(I2cBus::Owner::OLED)) { Serial.println("OLED: acquire fail"); return; }

    switch (currentMode) {
        case OLED_MODE_YAW:   drawYawMode();   break;
        case OLED_MODE_DEBUG: drawDebugMode(); break;
        default: break;
    }

    display.display();
    I2cBus::release(I2cBus::Owner::OLED);
}

void oledShowStatus(const char* line1, const char* line2) {
    if (!oledReady) return;
    if (!I2cBus::acquire(I2cBus::Owner::OLED)) return;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println(line1);
    if (line2) {
        display.setTextSize(1);
        display.println(line2);
    }
    display.display();
    I2cBus::release(I2cBus::Owner::OLED);
}
