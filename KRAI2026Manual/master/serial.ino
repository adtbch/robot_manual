/*
 * =====================================================================
 * FILE    : serial.ino
 * PERAN   : 1) Inisialisasi UART1 (→ slave1) dan UART2 (→ slave2).
 *           2) Unified command handler: PC, slave1, slave2 — semua
 *              command sama, response balik ke sumber yang mengirim.
 *
 * UART1   : slave1Serial (RX=45, TX=48) → slave1
 * UART2   : slave2Serial (RX=47, TX=21) → slave2
 *
 * COMMANDS:
 *   motor1 <pwm>        - Set motor 1 PWM (-1023..1023)
 *   motor2 <pwm>        - Set motor 2 PWM (-1023..1023)
 *   motorstop           - Stop semua motor
 *   servo1 <angle>      - Set servo 1 sudut (0-180)
 *   servo2 <angle>      - Set servo 2 sudut (0-180)
 *   servo3 <angle>      - Set servo 3 sudut (0-180)
 *   flash             flash lamp fire
 *   enc                 - Baca semua encoder
 *   encreset            - Reset semua encoder
 *   limit               - Baca semua limit switch
 *   prox                - Baca proximity sensor
 *   gripper reset       - Reset auto gripper ke IDLE
 *   status              - Tampilkan semua status
 *   stop                - Stop semua motor + servo tengah
 *   help                - Tampilkan daftar command
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "serial.h"
#include "motor.h"
#include "servo.h"
#include "encoder.h"
#include "limit_switch.h"
#include "relay.h"
#include "proximity.h"
#include "odom.h"
#include "forest.h"

// =====================================================================
//  UART SETUP
// =====================================================================

HardwareSerial slave1Serial(1);
HardwareSerial slave2Serial(2);

bool  gOdomValid = false;
float gOdomX_m   = 0.0f;
float gOdomY_m   = 0.0f;
float gOdomW_deg = 0.0f;

void setupSerial() {
    // UART1 — ke slave1
    slave1Serial.begin(SLAVE_BAUD, SERIAL_8N1, SLAVE1_RX, SLAVE1_TX);

    // UART2 — ke slave2
    slave2Serial.begin(SLAVE_BAUD, SERIAL_8N1, SLAVE2_RX, SLAVE2_TX);
}

// =====================================================================
//  SERIAL COMMAND HANDLER — unified PC, slave1, slave2
// =====================================================================

namespace {

constexpr size_t SERIAL_CMD_BUF_SIZE = 64;

// Tiga buffer terpisah: USB Serial, slave1Serial, slave2Serial
char pcBuf[SERIAL_CMD_BUF_SIZE];
uint8_t pcBufIdx = 0;

char slave1Buf[SERIAL_CMD_BUF_SIZE];
uint8_t slave1BufIdx = 0;

char slave2Buf[SERIAL_CMD_BUF_SIZE];
uint8_t slave2BufIdx = 0;

// ── Help ─────────────────────────────────────────────────────────
void printHelp(Print& out) {
    out.println("--- Daftar Command ---");
    out.println("  motor <id> <pwm>  (contoh: motor x 500)");
    out.println("  motorstop        stop semua motor");
    out.println("  motorlevel <0-5> motor Y ke level preset");
    out.println("  motorlevel cfg <v0>..<v5>  set 6 level encoder (Preferences)");
    out.println("  motorlevel get           baca 6 level dari master");
    out.println("  motortarget <x|y> <enc>  set encoder target (alias: motorpid)");
    out.println("  motortargetstop <x|y>    stop positioning (alias: motorpidstop)");
    out.println("  servo <id> <angle> (contoh: servo d 90)");
    out.println("  flash            flash lamp fire");
    out.println("  enc               baca encoder");
    out.println("  encreset          reset encoder");
    out.println("  limit             baca limit switch");
    out.println("  prox              baca proximity");
    out.println("  gripper <reset|homing>  reset atau homing gripper");
    out.println("  status            semua status");
    out.println("  odom [poll]       odometri slave1 (cache / poll)");
    out.println("  odomrec           tampilkan titik terekam");
    out.println("  odomrec clear     hapus record odom");
    out.println("  odomgoto <1-4>    gerak ke titik odom terekam");
    out.println("  forest cfg <d1> <d2>  set tujuan forest 1/2");
    out.println("  forest get            baca dest1 dest2 dari NVS");
    out.println("  forest goto <1|2>     jalankan tujuan");
    out.println("  forest exit           keluar forest");
    out.println("  forest cancel         batalkan forest");
    out.println("  stop              stop semua");
    out.println("  help              tampilkan ini");
}

// ── Parser respons dari slave1 (bukan command ke master) ─────────
bool parseSlave1Response(char* line) {
    char* token = strtok(line, " ");
    if (token == nullptr) return false;

    for (char* p = token; *p; ++p) *p = tolower(*p);

    if (strcmp(token, "odomtomaster") == 0) {
        char* xStr = strtok(nullptr, " ");
        char* yStr = strtok(nullptr, " ");
        char* wStr = strtok(nullptr, " ");
        if (xStr == nullptr || yStr == nullptr || wStr == nullptr) {
            return false;
        }
        gOdomX_m   = atof(xStr);
        gOdomY_m   = atof(yStr);
        gOdomW_deg = atof(wStr);
        gOdomValid = true;
        odomOnSampleReceived(gOdomX_m, gOdomY_m, gOdomW_deg);
        Serial.printf("[Slave1] odomToMaster: x=%.3fm y=%.3fm w=%.1fdeg\n",
                      gOdomX_m, gOdomY_m, gOdomW_deg);
        return true;
    }

    return false;
}

// ── Parser utama — output ke Print& out ──────────────────────────
void parseAndExecuteCommand(char* cmd, Print& out) {
    char* token = strtok(cmd, " ");
    if (token == nullptr) return;

    // Lowercase untuk case-insensitive
    for (char* p = token; *p; ++p) *p = tolower(*p);

    // ── BOX — dari slave2arm (proximity detect) ─────────────────
    if (strcmp(token, "boxr") == 0 || strcmp(token, "boxl") == 0) {
        char side = token[3];  // 'r' atau 'l'
        motorYSetTarget(gMotorYLevelEnc[1]);
        boxSetPending(side);
        out.printf("Box %c: motor Y naik ke level 1\n", side);
        return;
    }

    // ── MOTORLEVEL — motor Y level preset / cfg / get ───────────
    if (strcmp(token, "motorlevel") == 0) {
        char* sub = strtok(nullptr, " ");
        if (sub == nullptr) {
            out.println("Usage: motorlevel <0-5> | cfg <v0>..<v5> | get");
            return;
        }
        for (char* p = sub; *p; ++p) *p = tolower(*p);

        if (strcmp(sub, "cfg") == 0) {
            long levels[6];
            for (int i = 0; i < 6; i++) {
                char* v = strtok(nullptr, " ");
                if (v == nullptr) {
                    out.println("motorlevel err usage");
                    return;
                }
                levels[i] = atol(v);
            }
            if (motorYLevelSave(levels)) {
                out.println("motorlevel ok");
            } else {
                out.println("motorlevel err invalid");
            }
            return;
        }

        if (strcmp(sub, "get") == 0) {
            out.printf("motorlevel lvl %ld %ld %ld %ld %ld %ld\n",
                       gMotorYLevelEnc[0], gMotorYLevelEnc[1], gMotorYLevelEnc[2],
                       gMotorYLevelEnc[3], gMotorYLevelEnc[4], gMotorYLevelEnc[5]);
            return;
        }

        const int level = constrain(atoi(sub), 0, MOTOR_Y_LEVEL_MAX);
        motorYSetTarget(gMotorYLevelEnc[level]);
        out.printf("Motor Y level: %d (enc %ld)\n", level, gMotorYLevelEnc[level]);
        return;
    }

    // ── MOTOR <id> <pwm> ────────────────────────────────────────
    else if (strcmp(token, "motor") == 0) {
        char* id = strtok(nullptr, " ");
        char* val = strtok(nullptr, " ");
        if (id != nullptr && val != nullptr) {
            int pwm = constrain(atoi(val), -PWM_MAX, PWM_MAX);
            pwmMotor(id[0], pwm);
            out.printf("Motor '%c' PWM: %d\n", id[0], pwm);
        } else {
            out.println("Usage: motor <id> <pwm>  (contoh: motor x 500)");
        }
    }

    // ── MOTORSTOP ───────────────────────────────────────────────
    else if (strcmp(token, "motorstop") == 0) {
        motorStopAll();
        out.println("Semua motor STOP");
    }

    // ── MOTORTARGET <id> <target> ───────────────────────────────
    else if (strcmp(token, "motortarget") == 0 || strcmp(token, "motorpid") == 0) {
        char* id = strtok(nullptr, " ");
        char* val = strtok(nullptr, " ");
        if (id != nullptr && val != nullptr) {
            long target = atol(val);
            if (id[0] == 'x') {
                motorXSetTarget(target);
                out.printf("Motor X target: %ld\n", target);
            } else if (id[0] == 'y') {
                motorYSetTarget(target);
                out.printf("Motor Y target: %ld\n", target);
            } else {
                out.println("Usage: motortarget <x|y> <target>");
            }
        } else {
            out.println("Usage: motortarget <x|y> <target>  (contoh: motortarget y 500)");
        }
    }

    // ── MOTORTARGETSTOP <id> ────────────────────────────────────
    else if (strcmp(token, "motortargetstop") == 0 || strcmp(token, "motorpidstop") == 0) {
        char* id = strtok(nullptr, " ");
        if (id != nullptr) {
            if (id[0] == 'x') {
                motorXStop();
                out.println("Motor X STOP");
            } else if (id[0] == 'y') {
                motorYStop();
                out.println("Motor Y STOP");
            } else {
                out.println("Usage: motortargetstop <x|y>");
            }
        } else {
            out.println("Usage: motortargetstop <x|y>");
        }
    }

    // ── SERVO <id> <angle> ─────────────────────────────────────
    else if (strcmp(token, "servo") == 0) {
        char* id = strtok(nullptr, " ");
        char* val = strtok(nullptr, " ");
        if (id != nullptr && val != nullptr) {
            int angle = constrain(atoi(val), 0, 180);
            setServoAngle(id[0], angle);
            out.printf("Servo '%c': %d derajat\n", id[0], angle);
        } else {
            out.println("Usage: servo <id> <angle>  (contoh: servo d 90)");
        }
    }

    // ── FLASH ───────────────────────────────────────────────────
    else if (strcmp(token, "flash") == 0) {
        flashFire();
        out.println("Flash: FIRED");
    }

    // ── ENC ─────────────────────────────────────────────────────
    else if (strcmp(token, "enc") == 0) {
        for (size_t i = 0; i < ENCODER_COUNT; i++) {
            out.printf("  Encoder%zu: %ld\n", i + 1, getEncoderCount(i));
        }
    }

    // ── ENCRESET ────────────────────────────────────────────────
    else if (strcmp(token, "encreset") == 0) {
        for (size_t i = 0; i < ENCODER_COUNT; i++) {
            resetEncoderCount(i);
        }
        out.println("Semua encoder di-reset");
    }

    // ── LIMIT ───────────────────────────────────────────────────
    else if (strcmp(token, "limit") == 0) {
        out.printf("  LimY_Bawah       : %s\n", readLimitSwitch(LIMIT_Y_BAWAH)         ? "TRIGGERED" : "clear");
        out.printf("  LimX_Mundur      : %s\n", readLimitSwitch(LIMIT_X_MUNDUR)        ? "TRIGGERED" : "clear");
        out.printf("  LimArmBox_Depan  : %s\n", readLimitSwitch(LIMIT_ARMBOX_DEPAN)    ? "TRIGGERED" : "clear");
        out.printf("  LimArmBox_Belakang: %s\n", readLimitSwitch(LIMIT_ARMBOX_BELAKANG) ? "TRIGGERED" : "clear");
    }

    // ── PROX ────────────────────────────────────────────────────
    else if (strcmp(token, "prox") == 0) {
        out.printf("  Proximity: %s\n",
                   readProximity() ? "DETECTED" : "clear");
    }

    // ── GRIPPER ─────────────────────────────────────────────────
    else if (strcmp(token, "gripper") == 0) {
        char* sub = strtok(nullptr, " ");
        if (sub != nullptr && strcmp(sub, "reset") == 0) {
            gripperReset();
            out.println("Gripper: RESET ke IDLE");
        } else if (sub != nullptr && strcmp(sub, "homing") == 0) {
            setHomingAll();
            out.println("Gripper: homing motor (limit) + servo awal");
        } else {
            out.println("Usage: gripper <reset|homing>");
        }
    }

    // ── STATUS ──────────────────────────────────────────────────
    else if (strcmp(token, "status") == 0) {
        out.println("=== STATUS ===");
        out.printf("  Prox    : %s\n", readProximity() ? "DETECTED" : "clear");
        out.printf("  EncX    : %ld\n", getEncoderCount('x'));
        out.printf("  EncY    : %ld\n", getEncoderCount('y'));
        out.printf("  LimY_Bawah       : %s\n", readLimitSwitch(LIMIT_Y_BAWAH)         ? "TRIGGERED" : "clear");
        out.printf("  LimX_Mundur      : %s\n", readLimitSwitch(LIMIT_X_MUNDUR)        ? "TRIGGERED" : "clear");
        out.printf("  LimArmBox_Depan  : %s\n", readLimitSwitch(LIMIT_ARMBOX_DEPAN)    ? "TRIGGERED" : "clear");
        out.printf("  LimArmBox_Belakang: %s\n", readLimitSwitch(LIMIT_ARMBOX_BELAKANG) ? "TRIGGERED" : "clear");
        out.println("==============");
    }

    // ── STOP ────────────────────────────────────────────────────
    else if (strcmp(token, "stop") == 0) {
        motorStopAll();
        setServoAngle('d', 90);
        setServoAngle('t', 90);
        setServoAngle('b', 90);
        out.println("Semua motor STOP, servo tengah (90)");
    }

    // ── ODOM (slave1) ───────────────────────────────────────────
    else if (strcmp(token, "odom") == 0) {
        char* sub = strtok(nullptr, " ");
        if (sub != nullptr) {
            for (char* p = sub; *p; ++p) *p = tolower(*p);
        }
        if (sub != nullptr && strcmp(sub, "poll") == 0) {
            sendShowOdomCommand();
            out.println("Odom poll dikirim ke slave1");
        } else if (gOdomValid) {
            out.printf("Odom: x=%.3fm y=%.3fm w=%.1fdeg\n",
                       gOdomX_m, gOdomY_m, gOdomW_deg);
        } else {
            sendShowOdomCommand();
            out.println("Odom: belum ada data — poll dikirim ke slave1");
        }
    }

    // ── ODOM RECORD ─────────────────────────────────────────────
    else if (strcmp(token, "odomrec") == 0) {
        char* sub = strtok(nullptr, " ");
        if (sub != nullptr) {
            for (char* p = sub; *p; ++p) *p = tolower(*p);
        }
        if (sub != nullptr && strcmp(sub, "clear") == 0) {
            odomRecordClear();
            out.println("OdomRecord cleared");
        } else {
            odomRecordPrint(out);
        }
    }

    // ── ODOM GOTO <1-4> ─────────────────────────────────────────
    else if (strcmp(token, "odomgoto") == 0) {
        char* val = strtok(nullptr, " ");
        if (val == nullptr) {
            out.println("Usage: odomgoto <1-4>");
            return;
        }
        const uint8_t slot = (uint8_t)atoi(val);
        if (odomGoto(slot)) {
            const OdomWaypoint& wp = gOdomWaypoints[slot - 1];
            out.printf("OdomGoto #%u: x=%.1f y=%.1f yaw=%.0f speed=%d\n",
                       slot, wp.x_cm, wp.y_cm, wp.yaw_deg, wp.maxspeed_rpm);
        } else if (slot < 1 || slot > ODOM_WP_COUNT) {
            out.println("OdomGoto err: usage odomgoto <1-4>");
        } else {
            out.printf("OdomGoto err: slot %u belum terekam (%u/4)\n", slot, gOdomWpFilled);
        }
    }

    // ── FOREST ──────────────────────────────────────────────────
    else if (strcmp(token, "forest") == 0) {
        char* sub = strtok(nullptr, " ");
        if (sub == nullptr) {
            out.println("Usage: forest cfg|get|goto|exit|cancel");
            return;
        }
        for (char* p = sub; *p; ++p) *p = tolower(*p);

        if (strcmp(sub, "cfg") == 0) {
            char* d1s = strtok(nullptr, " ");
            char* d2s = strtok(nullptr, " ");
            if (d1s != nullptr && d2s != nullptr) {
                forestSetDestinations((uint8_t)atoi(d1s), (uint8_t)atoi(d2s));
                out.println("forest ok");
            } else {
                out.println("forest err usage");
            }
            return;
        }

        if (strcmp(sub, "get") == 0) {
            out.printf("forest dest %u %u %u\n",
                       gForestDest1, gForestDest2, gForestDest1Done ? 1u : 0u);
            return;
        }

        if (strcmp(sub, "goto") == 0) {
            char* slotStr = strtok(nullptr, " ");
            if (slotStr == nullptr) {
                out.println("forest err usage");
                return;
            }
            const uint8_t slot = (uint8_t)atoi(slotStr);
            if (slot != 1 && slot != 2) {
                out.println("forest err usage");
                return;
            }
            if (slot == 2 && !gForestDest1Done) {
                out.println("forest err dest1_not_done");
                return;
            }
            if (forestGotoSlot(slot)) {
                out.println("forest ok");
            } else if (slot == 2 && !gForestDest1Done) {
                out.println("forest err dest1_not_done");
            } else {
                out.println("forest err invalid_id");
            }
            return;
        }

        if (strcmp(sub, "exit") == 0) {
            forestTriggerExit();
            out.println("forest ok");
            return;
        }

        if (strcmp(sub, "cancel") == 0) {
            cancelForestGoto();
            out.println("forest ok");
            return;
        }

        out.println("forest err usage");
        return;
    }

    // ── HELP ────────────────────────────────────────────────────
    else if (strcmp(token, "help") == 0) {
        printHelp(out);
    }

    // ── UNKNOWN ─────────────────────────────────────────────────
    else {
        out.printf("Unknown: '%s' (ketik 'help' untuk daftar)\n", token);
    }
}

} // anonymous namespace

// =====================================================================
//  SETUP COMMAND HANDLER
// =====================================================================

void setupSerialCommand() {
    Serial.println("=== Serial Command Ready (PC + Slave1 + Slave2) ===");
    printHelp(Serial);
}

// =====================================================================
//  TICK — panggil di loop(), handle ketiga sumber
// =====================================================================

void serialCommandTick() {
    // ── PC (USB Serial) ─────────────────────────────────────────
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

    // ── Slave1 (UART1) — respons telemetry, bukan command handler ─
    while (slave1Serial.available() > 0) {
        char c = slave1Serial.read();
        if (c == '\n' || c == '\r') {
            if (slave1BufIdx > 0) {
                slave1Buf[slave1BufIdx] = '\0';
                if (!parseSlave1Status(slave1Buf)) {
                    parseAndExecuteCommand(slave1Buf, slave1Serial);
                }
                slave1BufIdx = 0;
            }
        } else if (slave1BufIdx < SERIAL_CMD_BUF_SIZE - 1) {
            slave1Buf[slave1BufIdx++] = c;
        } else {
            Serial.println("[Slave1] Error: line terlalu panjang");
            slave1BufIdx = 0;
        }
    }

    // ── Slave2 (UART2) — sensor data + command ──────────────────
    while (slave2Serial.available() > 0) {
        char c = slave2Serial.read();
        if (c == '\n' || c == '\r') {
            if (slave2BufIdx > 0) {
                slave2Buf[slave2BufIdx] = '\0';
                // Sensor data (prox/enc/limit/pne) → update state
                if (!parseSlave2Sensor(slave2Buf)) {
                    // Bukan sensor → command biasa
                    parseAndExecuteCommand(slave2Buf, slave2Serial);
                }
                slave2BufIdx = 0;
            }
        } else if (slave2BufIdx < SERIAL_CMD_BUF_SIZE - 1) {
            slave2Buf[slave2BufIdx++] = c;
        } else {
            slave2Serial.println("Error: command terlalu panjang");
            slave2BufIdx = 0;
        }
    }
}
