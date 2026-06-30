/*
 * =====================================================================
 * FILE    : uji coba.ino
 * PERAN   : Test servo via serial command.
 *
 * COMMANDS:
 *   servo <angle>   — Set servo sudut (0-180)
 *   help            — Tampilkan daftar command
 *
 * BOARD   : ESP32-S3
 * =====================================================================
 */

// Pin servo (sesuaikan dengan hardware)
constexpr uint8_t SERVO_PIN = 17;

// PWM config (standard servo 50Hz)
constexpr int SERVO_FREQ  = 50;
constexpr int SERVO_RES   = 14;   // 2^14 = 16384
constexpr int DUTY_MIN    = 500;  // us (0 derajat)
constexpr int DUTY_MAX    = 2400; // us (180 derajat)

// Serial buffer
constexpr size_t CMD_BUF_SIZE = 64;
char cmdBuf[CMD_BUF_SIZE];
uint8_t cmdBufIdx = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("=== Uji Servo ===");
    Serial.println("Ketik 'servo <angle>' (contoh: servo 90)");
    Serial.println("Ketik 'help' untuk daftar command");

    // Attach servo
    ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES);

    // Set ke 0 derajat saat boot
    setServoAngle(0);
    Serial.println("Servo: 0 derajat");
}

void loop() {
    while (Serial.available() > 0) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (cmdBufIdx > 0) {
                cmdBuf[cmdBufIdx] = '\0';
                parseCommand(cmdBuf);
                cmdBufIdx = 0;
            }
        } else if (cmdBufIdx < CMD_BUF_SIZE - 1) {
            cmdBuf[cmdBufIdx++] = c;
        } else {
            Serial.println("Error: command terlalu panjang");
            cmdBufIdx = 0;
        }
    }
}

void parseCommand(char* cmd) {
    char* token = strtok(cmd, " ");
    if (token == nullptr) return;

    // Lowercase
    for (char* p = token; *p; ++p) *p = tolower(*p);

    // SERVO <angle>
    if (strcmp(token, "servo") == 0) {
        char* val = strtok(nullptr, " ");
        if (val != nullptr) {
            int angle = constrain(atoi(val), 0, 180);
            setServoAngle(angle);
            Serial.printf("Servo: %d derajat\n", angle);
        } else {
            Serial.println("Usage: servo <angle>  (0-180)");
        }
    }
    // HELP
    else if (strcmp(token, "help") == 0) {
        Serial.println("--- Daftar Command ---");
        Serial.println("  servo <angle>   (0-180)");
        Serial.println("  help            tampilkan ini");
    }
    // UNKNOWN
    else {
        Serial.printf("Unknown: '%s'\n", token);
    }
}

void setServoAngle(int angle) {
    angle = constrain(angle, 0, 180);
    long pulseWidth = map(angle, 0, 180, DUTY_MIN, DUTY_MAX);
    long duty = (pulseWidth * ((1L << SERVO_RES) - 1)) / 20000;
    ledcWrite(SERVO_PIN, duty);
}
