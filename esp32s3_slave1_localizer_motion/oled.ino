// ============================================================
// OLED Display - Yaw and Robot Status
// ============================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

bool setupOLED() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("OLED: not found at 0x3C");
        // Try alternative I2C address
        if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
            Serial.println("OLED: not found at 0x3D either");
            return false;
        }
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("OLED OK");
    display.display();
    delay(500);
    return true;
}

void displayYaw(float yawDeg, const char* status) {
    display.clearDisplay();

    // --- Yaw compass dial ---
    int cx = 40, cy = 32, r = 22;
    display.drawCircle(cx, cy, r, SSD1306_WHITE);

    // Cardinal markers (N, E, S, W)
    display.fillCircle(cx, cy - r + 2, 2, SSD1306_WHITE); // N
    display.fillCircle(cx, cy + r - 2, 2, SSD1306_WHITE); // S
    display.fillCircle(cx - r + 2, cy, 2, SSD1306_WHITE); // W
    display.fillCircle(cx + r - 2, cy, 2, SSD1306_WHITE); // E

    // Arrow pointing at yaw
    float rad = yawDeg * (PI / 180.0f);
    int ax = cx + (int)(r * 0.7f * sin(rad));
    int ay = cy - (int)(r * 0.7f * cos(rad));
    display.drawLine(cx, cy, ax, ay, SSD1306_WHITE);
    display.fillCircle(ax, ay, 3, SSD1306_WHITE);

    // --- Yaw text (right side) ---
    display.setCursor(68, 8);
    display.setTextSize(2);
    display.print((int)yawDeg);
    display.setTextSize(1);
    display.print(" deg");

    // Cardinal direction text
    display.setCursor(68, 30);
    display.setTextSize(1);
    if (yawDeg > -22.5f && yawDeg <= 22.5f)       display.print("NORTH");
    else if (yawDeg > 22.5f && yawDeg <= 67.5f)    display.print("NE");
    else if (yawDeg > 67.5f && yawDeg <= 112.5f)   display.print("EAST");
    else if (yawDeg > 112.5f && yawDeg <= 157.5f)  display.print("SE");
    else if (yawDeg > 157.5f || yawDeg <= -157.5f) display.print("SOUTH");
    else if (yawDeg > -157.5f && yawDeg <= -112.5f)display.print("SW");
    else if (yawDeg > -112.5f && yawDeg <= -67.5f) display.print("WEST");
    else if (yawDeg > -67.5f && yawDeg <= -22.5f)  display.print("NW");

    // --- Status bar (bottom) ---
    display.drawFastHLine(0, 54, 128, SSD1306_WHITE);
    display.setCursor(0, 56);
    display.setTextSize(1);
    if (status) display.print(status);

    display.display();
}
