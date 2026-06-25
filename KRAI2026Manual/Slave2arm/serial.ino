/*
 * =====================================================================
 * FILE    : serial.ino
 * PERAN   : 1) Inisialisasi UART1 (→ master).
 *           2) Unified command handler: PC (USB Serial) & master (UART1).
 *              Response balik ke sumber yang mengirim.
 *
 * UART1   : masterSerial (RX=36, TX=35) → master
 *
 * COMMANDS:
 *   motor1-4 <pwm>       (-1023..1023)
 *   motorstop             stop semua motor
 *   pne1-4 <on|off|t>    pneumatic on/off/toggle
 *   pneall                semua pneumatic OFF
 *   enc                   baca semua encoder
 *   encreset              reset semua encoder
 *   limit                 baca semua limit switch
 *   prox                  baca semua proximity
 *   status                tampilkan semua status
 *   stop                  stop semua motor + pneumatic off
 *   help                  tampilkan daftar command
 *
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#include "serial.h"
#include "motor.h"
#include "encoder.h"
#include "limit_switch.h"
#include "proximity.h"
#include "pneumatic.h"

// =====================================================================
//  UART SETUP
// =====================================================================

HardwareSerial masterSerial(1);

void setupSerial() {
    masterSerial.begin(MASTER_BAUD, SERIAL_8N1, MASTER_RX, MASTER_TX);
}

// =====================================================================
//  SERIAL COMMAND HANDLER — unified PC & master
// =====================================================================

namespace {

constexpr size_t SERIAL_CMD_BUF_SIZE = 64;

// Dua buffer terpisah: USB Serial & masterSerial
char pcBuf[SERIAL_CMD_BUF_SIZE];
uint8_t pcBufIdx = 0;

char masterBuf[SERIAL_CMD_BUF_SIZE];
uint8_t masterBufIdx = 0;

// ── Help ─────────────────────────────────────────────────────────
void printHelp(Print& out) {
    out.println("--- Daftar Command ---");
    out.println("  motor <id> <pwm>   (contoh: motor 1 500)");
    out.println("  motorstop          stop semua motor");
    out.println("  pne <id> <on|off|t>(contoh: pne 2 on)");
    out.println("  pneall             semua pneumatic OFF");
    out.println("  enc                baca encoder");
    out.println("  encreset           reset encoder");
    out.println("  limit              baca limit switch");
    out.println("  prox               baca proximity");
    out.println("  status             semua status");
    out.println("  stop               stop semua");
    out.println("  help               tampilkan ini");
}

// ── Parser utama — output ke Print& out ──────────────────────────
void parseAndExecuteCommand(char* cmd, Print& out) {
    char* token = strtok(cmd, " ");
    if (token == nullptr) return;

    // Lowercase
    for (char* p = token; *p; ++p) *p = tolower(*p);

    // ── MOTOR <id> <pwm> ────────────────────────────────────────
    if (strcmp(token, "motor") == 0) {
        char* id = strtok(nullptr, " ");
        char* val = strtok(nullptr, " ");
        if (id != nullptr && val != nullptr) {
            int pwm = constrain(atoi(val), -PWM_MAX, PWM_MAX);
            pwmMotor(id[0], pwm);
            out.printf("Motor '%c' PWM: %d\n", id[0], pwm);
        } else {
            out.println("Usage: motor <id> <pwm>  (contoh: motor 1 500)");
        }
        return;
    }

    // ── MOTORSTOP ───────────────────────────────────────────────
    if (strcmp(token, "motorstop") == 0) {
        motorStopAll();
        out.println("Semua motor STOP");
        return;
    }

    // ── PNE <id> <on|off|t> ─────────────────────────────────────
    if (strcmp(token, "pne") == 0) {
        char* id = strtok(nullptr, " ");
        char* val = strtok(nullptr, " ");
        if (id != nullptr && val != nullptr) {
            for (char* p = val; *p; ++p) *p = tolower(*p);
            if (strcmp(val, "on") == 0) {
                pneumaticOn(id[0]);
                out.printf("Pneumatic '%c': ON\n", id[0]);
            } else if (strcmp(val, "off") == 0) {
                pneumaticOff(id[0]);
                out.printf("Pneumatic '%c': OFF\n", id[0]);
            } else if (strcmp(val, "t") == 0 || strcmp(val, "toggle") == 0) {
                pneumaticToggle(id[0]);
                out.printf("Pneumatic '%c': %s\n", id[0],
                           pneumaticState(id[0]) ? "ON" : "OFF");
            } else {
                out.println("Usage: pne <id> <on|off|t>  (contoh: pne 2 on)");
            }
        } else {
            out.println("Usage: pne <id> <on|off|t>  (contoh: pne 2 on)");
        }
        return;
    }

    // ── PNEALL ──────────────────────────────────────────────────
    if (strcmp(token, "pneall") == 0) {
        pneumaticAllOff();
        out.println("Semua pneumatic OFF");
        return;
    }

    // ── ENC ─────────────────────────────────────────────────────
    if (strcmp(token, "enc") == 0) {
        for (size_t i = 0; i < ENCODER_COUNT; i++) {
            out.printf("  Encoder%zu: %ld\n", i + 1, getEncoderCount(i));
        }
        return;
    }

    // ── ENCRESET ────────────────────────────────────────────────
    if (strcmp(token, "encreset") == 0) {
        for (size_t i = 0; i < ENCODER_COUNT; i++) {
            resetEncoderCount(i);
        }
        out.println("Semua encoder di-reset");
        return;
    }

    // ── LIMIT ───────────────────────────────────────────────────
    if (strcmp(token, "limit") == 0) {
        for (size_t i = 0; i < LIMIT_COUNT; i++) {
            out.printf("  Limit%zu: %s\n", i + 1,
                       readLimitSwitch(i) ? "TRIGGERED" : "clear");
        }
        return;
    }

    // ── PROX ────────────────────────────────────────────────────
    if (strcmp(token, "prox") == 0) {
        for (size_t i = 0; i < PROXIMITY_COUNT; i++) {
            out.printf("  Prox%zu: %s\n", i + 1,
                       readProximity(i) ? "DETECTED" : "clear");
        }
        return;
    }

    // ── STATUS ──────────────────────────────────────────────────
    if (strcmp(token, "status") == 0) {
        out.println("=== STATUS ===");
        for (size_t i = 0; i < ENCODER_COUNT; i++) {
            out.printf("  Enc%zu    : %ld\n", i + 1, getEncoderCount(i));
        }
        for (size_t i = 0; i < LIMIT_COUNT; i++) {
            out.printf("  Lim%zu    : %s\n", i + 1,
                       readLimitSwitch(i) ? "TRIGGERED" : "clear");
        }
        for (size_t i = 0; i < PROXIMITY_COUNT; i++) {
            out.printf("  Prox%zu   : %s\n", i + 1,
                       readProximity(i) ? "DETECTED" : "clear");
        }
        for (size_t i = 0; i < PNEUMATIC_COUNT; i++) {
            out.printf("  Pne%zu    : %s\n", i + 1,
                       pneumaticState(i) ? "ON" : "OFF");
        }
        out.println("==============");
        return;
    }

    // ── STOP ────────────────────────────────────────────────────
    if (strcmp(token, "stop") == 0) {
        motorStopAll();
        pneumaticAllOff();
        out.println("Semua motor STOP, pneumatic OFF");
        return;
    }

    // ── HELP ────────────────────────────────────────────────────
    if (strcmp(token, "help") == 0) {
        printHelp(out);
        return;
    }

    // ── UNKNOWN ─────────────────────────────────────────────────
    out.printf("Unknown: '%s' (ketik 'help' untuk daftar)\n", token);
}

} // anonymous namespace

// =====================================================================
//  SETUP COMMAND HANDLER
// =====================================================================

void setupSerialCommand() {
    Serial.println("=== Serial Command Ready (PC + Master) ===");
    printHelp(Serial);
}

// =====================================================================
//  TICK — panggil di loop(), handle kedua sumber
// =====================================================================

void serialCommandTick() {
    // PC (USB Serial) → response ke Serial
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (pcBufIdx > 0) {
                pcBuf[pcBufIdx] = '\0';
                parseAndExecuteCommand(pcBuf, Serial);
                pcBufIdx = 0;
            }
        } else if (pcBufIdx < SERIAL_CMD_BUF_SIZE - 1) {
            pcBuf[pcBufIdx++] = c;
        } else {
            Serial.println("Error: command terlalu panjang");
            pcBufIdx = 0;
        }
    }

    // Master (UART1) → response ke masterSerial
    while (masterSerial.available() > 0) {
        char c = masterSerial.read();
        if (c == '\n' || c == '\r') {
            if (masterBufIdx > 0) {
                masterBuf[masterBufIdx] = '\0';
                parseAndExecuteCommand(masterBuf, masterSerial);
                masterBufIdx = 0;
            }
        } else if (masterBufIdx < SERIAL_CMD_BUF_SIZE - 1) {
            masterBuf[masterBufIdx++] = c;
        } else {
            masterSerial.println("Error: command terlalu panjang");
            masterBufIdx = 0;
        }
    }
}
