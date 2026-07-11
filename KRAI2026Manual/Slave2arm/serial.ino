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
#include "forest_config.h"
#include "motor_y_level_proxy.h"

// =====================================================================
//  UART SETUP
// =====================================================================

HardwareSerial masterSerial(1);

void setupSerial() {
    masterSerial.setRxBufferSize(2048);
    masterSerial.setTxBufferSize(2048);
    masterSerial.begin(MASTER_BAUD, SERIAL_8N1, MASTER_RX, MASTER_TX);
}

// =====================================================================
//  UART TX PROXY — web task core 0 queue, loop() core 1 kirim
// =====================================================================

namespace {

char     gMasterUartPendingLine[80];
volatile bool gMasterUartPending = false;

bool waitMasterUartSent(uint32_t timeoutMs) {
    const uint32_t t0 = millis();
    while (gMasterUartPending && (millis() - t0 < timeoutMs)) {
        vTaskDelay(1);
    }
    return !gMasterUartPending;
}

} // anonymous namespace

void masterUartProxyTick() {
    if (!gMasterUartPending) return;
    masterSerial.println(gMasterUartPendingLine);
    masterSerial.flush();
    gMasterUartPending = false;
}

void masterUartSendLine(const char* line) {
    strncpy(gMasterUartPendingLine, line, sizeof(gMasterUartPendingLine) - 1);
    gMasterUartPendingLine[sizeof(gMasterUartPendingLine) - 1] = '\0';
    gMasterUartPending = true;
    waitMasterUartSent(50);
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
    out.println("  motor <id> <pwm>   (x=run smp limit, k=run trus, y=pwm)");
    out.println("  motorstop          stop semua motor");
    out.println("  motorrunstop       stop motor X/K run");
    out.println("  motortarget <enc>  set encoder target motor Y (alias: motorpid)");
    out.println("  motortargetstop    stop motor Y (alias: motorpidstop)");
    out.println("  motory <u|d> <step> jog encoder Y lokal");
    out.println("  pne <r|l|lk|rk> <on|off|t>");
    out.println("  pneall             semua pneumatic OFF");
    out.println("  prox               baca proximity R/L");
    out.println("  enc                baca encoder");
    out.println("  encreset           reset encoder");
    out.println("  limit              baca limit switch");
    out.println("  status             semua status");
    out.println("  stop               stop semua");
    out.println("  help               tampilkan ini");
}

// ── Null print — buang semua output (untuk command dari master) ─
struct NullPrint : Print {
    size_t write(uint8_t) override { return 1; }
    size_t write(const uint8_t*, size_t n) override { return n; }
};

NullPrint gNullPrint;

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
            const int pwm = constrain(atoi(val), -PWM_MAX, PWM_MAX);
            const char motorId = id[0];
            if (executeMotorCommand(motorId, pwm)) {
                if ((motorId == 'x' || motorId == 'k') && pwm == 0)
                    out.printf("Motor '%c' STOP\n", motorId);
                else if (motorId == 'x' || motorId == 'k')
                    out.printf("Motor '%c' RUN: %d\n", motorId, pwm);
                else
                    out.printf("Motor '%c' PWM: %d\n", motorId, pwm);
            } else {
                out.printf("Motor id '%c' tidak valid\n", motorId);
            }
        } else {
            out.println("Usage: motor <id> <pwm>  (x/k=run smp limit, y=pwm)");
        }
        return;
    }

    // ── MOTORSTOP ───────────────────────────────────────────────
    if (strcmp(token, "motorstop") == 0) {
        motorStopAll();
        out.println("Semua motor STOP");
        return;
    }

    // ── MOTOR RUN STOP (x/k) ────────────────────────────────────
    if (strcmp(token, "motorrunstop") == 0) {
        motorRunStopAll();
        out.println("Motor X/K STOP");
        return;
    }

    // ── MOTOR TARGET (bang-bang encoder Y) ──────────────────────
    if (strcmp(token, "motortarget") == 0 || strcmp(token, "motorpid") == 0) {
        char* val = strtok(nullptr, " ");
        if (val != nullptr) {
            long target = atol(val);
            motorYSetTarget(target);
            out.printf("Motor Y target: %ld\n", target);
        } else {
            out.println("Usage: motortarget <encoder>  (contoh: motortarget 500)");
        }
        return;
    }

    if (strcmp(token, "motortargetstop") == 0 || strcmp(token, "motorpidstop") == 0) {
        motorYStop();
        out.println("Motor Y STOP");
        return;
    }

    // ── MOTORY <u|d> <step> — jog encoder Y lokal ──────────────
    if (strcmp(token, "motory") == 0) {
        char* dir = strtok(nullptr, " ");
        char* val = strtok(nullptr, " ");
        if (dir != nullptr && val != nullptr) {
            long step = atol(val);
            if (dir[0] == 'u') motorYAdjustTarget(step);
            else if (dir[0] == 'd') motorYAdjustTarget(-step);
            out.printf("Motor Y jog %s %ld (enc: %ld)\n",
                       dir[0] == 'u' ? "UP" : "DOWN", step, getEncoderCount('y'));
        } else {
            out.println("Usage: motory <u|d> <step>  (contoh: motory u 15)");
        }
        return;
    }

    // ── PNE <id> <on|off|t> ─────────────────────────────────────
    if (strcmp(token, "pne") == 0) {
        char* id = strtok(nullptr, " ");
        char* val = strtok(nullptr, " ");
        if (id != nullptr && val != nullptr) {
            for (char* p = val; *p; ++p) *p = tolower(*p);
            if (strcmp(val, "on") == 0) {
                pneumaticOn(id);
                out.printf("Pneumatic '%s': ON\n", id);
            } else if (strcmp(val, "off") == 0) {
                pneumaticOff(id);
                out.printf("Pneumatic '%s': OFF\n", id);
            } else if (strcmp(val, "t") == 0 || strcmp(val, "toggle") == 0) {
                pneumaticToggle(id);
                out.printf("Pneumatic '%s': %s\n", id,
                           pneumaticState(id) ? "ON" : "OFF");
            } else {
                out.println("Usage: pne <r|l|lk|rk> <on|off|t>  (contoh: pne r on)");
            }
        } else {
            out.println("Usage: pne <r|l|lk|rk> <on|off|t>  (contoh: pne r on)");
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
        out.printf("  ArmBox_Depan   : %s\n", readLimitSwitch(LIMIT_ARMBOX_DEPAN)    ? "TRIGGERED" : "clear");
        out.printf("  ArmBox_Belakang: %s\n", readLimitSwitch(LIMIT_ARMBOX_BELAKANG) ? "TRIGGERED" : "clear");
        out.printf("  ArmBox_Turun   : %s\n", readLimitSwitch(LIMIT_ARMBOX_TURUN)    ? "TRIGGERED" : "clear");
        return;
    }

    // ── PROX ────────────────────────────────────────────────────
    if (strcmp(token, "prox") == 0) {
        out.printf("  ProxR: %s\n", readProximity('r') ? "DETECTED" : "clear");
        out.printf("  ProxL: %s\n", readProximity('l') ? "DETECTED" : "clear");
        return;
    }

    // ── STATUS ──────────────────────────────────────────────────
    if (strcmp(token, "status") == 0) {
        out.println("=== STATUS ===");
        out.printf("  EncXKanan  : %ld\n", getEncoderCount('x'));
        out.printf("  EncYKanan  : %ld (target: %ld, %s)\n",
                   getEncoderCount('y'), motorYGetTarget(),
                   motorYIsActive() ? "ACTIVE" : "idle");
        out.printf("  EncXKiri   : %ld\n", getEncoderCount('k'));
        out.printf("  ArmBox_Depan   : %s\n", readLimitSwitch(LIMIT_ARMBOX_DEPAN)    ? "TRIGGERED" : "clear");
        out.printf("  ArmBox_Belakang: %s\n", readLimitSwitch(LIMIT_ARMBOX_BELAKANG) ? "TRIGGERED" : "clear");
        out.printf("  ArmBox_Turun   : %s\n", readLimitSwitch(LIMIT_ARMBOX_TURUN)    ? "TRIGGERED" : "clear");
        out.printf("  ProxR     : %s\n", readProximity('r') ? "DETECTED" : "clear");
        out.printf("  ProxL     : %s\n", readProximity('l') ? "DETECTED" : "clear");
        out.printf("  PneR      : %s\n", pneumaticState("r") ? "ON" : "OFF");
        out.printf("  PneL      : %s\n", pneumaticState("l") ? "ON" : "OFF");
        out.printf("  PneLK     : %s\n", pneumaticState("lk") ? "ON" : "OFF");
        out.printf("  PneRK     : %s\n", pneumaticState("rk") ? "ON" : "OFF");
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

    // Master (UART1) → execute only, no reply
    while (masterSerial.available() > 0) {
        char c = masterSerial.read();
        if (c == '\n' || c == '\r') {
            if (masterBufIdx > 0) {
                masterBuf[masterBufIdx] = '\0';
                if (!parseMasterMotorLevelLine(masterBuf)) {
                    if (!parseMasterForestLine(masterBuf)) {
                        parseAndExecuteCommand(masterBuf, gNullPrint);
                    }
                }
                masterBufIdx = 0;
            }
        } else if (masterBufIdx < SERIAL_CMD_BUF_SIZE - 1) {
            masterBuf[masterBufIdx++] = c;
        } else {
            masterBufIdx = 0;
        }
    }
}
