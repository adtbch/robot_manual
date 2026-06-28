/*
 * =====================================================================
 * FILE    : serial.ino
 * PERAN   : Inisialisasi UART1 + UART2, relay WSN-31 ke master,
 *           unified command handler (USB Serial + UART master).
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#include "serial.h"
#include "pid.h"
#include "motor.h"
#include "mpu.h"
#include "autoTuner.h"
#include "kinematik.h"
#include "waypoint.h"
#include "encoder.h"

extern bool testYawMode;
extern int  testYawTarget;

namespace {

constexpr size_t SERIAL_CMD_BUF_SIZE = 96;

char pcBuf[SERIAL_CMD_BUF_SIZE];
uint8_t pcBufIdx = 0;

char masterBuf[SERIAL_CMD_BUF_SIZE];
uint8_t masterBufIdx = 0;

void printHelp(Print& out) {
    out.println("Commands:");
    out.println("  tune <idx> <kp> <ki> <kf> <db>         — set motor PID");
    out.println("  save                                    — save motor PID to NVS");
    out.println("  tuneyaw <kp> <ki> <kd>                  — set yaw PID");
    out.println("  saveyaw / loadyaw / showyaw             — yaw PID NVS");
    out.println("  rpm <fr> <fl> <br> <bl>                 — USB: raw PWM | Master: PID RPM");
    out.println("  kn <vx> <vy> <yaw>                      — field-cent RPM + yaw correction");
    out.println("  stop                                    — stop all motors");
    out.println("  autotune <motor|all>                    — run auto-tuner");
    out.println("  calibrate / calclear                    — gyro calibration NVS");
    out.println("  goto <x_cm> <y_cm> <yaw_deg>           — gerak ke waypoint");
    out.println("  wp cancel|status                        — batal/status waypoint");
    out.println("  tunewp <kp> [tol_pos_cm] [tol_yaw_deg] — tuning waypoint");
    out.println("  savewp                                  — simpan waypoint params ke NVS");
    out.println("  odom                                    — tampilkan odometri");
    out.println("  odomreset                               — reset odometri ke (0,0,0)");
}

void parseAndExecuteCommand(char* cmd, Print& out, bool fromMasterUart) {
    char* token = strtok(cmd, " ");
    if (token == nullptr) return;

    for (char* p = token; *p; ++p) *p = tolower(*p);

    if (strcmp(token, "tune") == 0) {
        char* idxStr = strtok(nullptr, " ");
        char* kpStr = strtok(nullptr, " ");
        char* kiStr = strtok(nullptr, " ");
        char* kfStr = strtok(nullptr, " ");
        char* dbStr = strtok(nullptr, " ");
        if (idxStr != nullptr && kpStr != nullptr && kiStr != nullptr &&
            kfStr != nullptr && dbStr != nullptr) {
            const int idx = atoi(idxStr);
            const float kp = atof(kpStr);
            const float ki = atof(kiStr);
            const float kf = atof(kfStr);
            const float deadband = atof(dbStr);
            pidSetGains(idx, kp, ki, kf, deadband);
            if (!fromMasterUart) out.printf("Motor %d PID updated: Kp=%.2f Ki=%.2f Kf=%.2f Db=%.2f\n",
                       idx, kp, ki, kf, deadband);
        } else {
            if (!fromMasterUart) out.println("Format: tune <motorIdx> <kp> <ki> <kf> <db>");
        }
    } else if (strcmp(token, "save") == 0) {
        for (int i = 0; i < 4; i++) {
            pidSaveToNVS(i, pidStates[i].kp, pidStates[i].ki,
                         pidStates[i].kf, pidStates[i].deadband);
        }
        if (!fromMasterUart) out.println("Semua konstanta PID tersimpan ke NVS.");
    } else if (strcmp(token, "rpm") == 0) {
        char* frStr = strtok(nullptr, " ");
        char* flStr = strtok(nullptr, " ");
        char* brStr = strtok(nullptr, " ");
        char* blStr = strtok(nullptr, " ");
        if (frStr != nullptr && flStr != nullptr && brStr != nullptr && blStr != nullptr) {
            const int fr = atoi(frStr);
            const int fl = atoi(flStr);
            const int br = atoi(brStr);
            const int bl = atoi(blStr);
            if (fromMasterUart) {
                rpmMotor(fr, fl, br, bl);
                if (!fromMasterUart) out.printf("RPM: FR=%d FL=%d BR=%d BL=%d\n", fr, fl, br, bl);
            } else {
                pwmMotor(0, fr);
                pwmMotor(1, fl);
                pwmMotor(2, br);
                pwmMotor(3, bl);
                if (!fromMasterUart) out.printf("Test PWM: FR=%d FL=%d BR=%d BL=%d\n", fr, fl, br, bl);
            }
        } else {
            if (!fromMasterUart) out.println("Format: rpm <fr> <fl> <br> <bl>");
        }
    } else if (strcmp(token, "kn") == 0) {
        char* vxStr = strtok(nullptr, " ");
        char* vyStr = strtok(nullptr, " ");
        char* yawStr = strtok(nullptr, " ");
        if (vxStr != nullptr && vyStr != nullptr && yawStr != nullptr) {
            const int vx = atoi(vxStr);
            const int vy = atoi(vyStr);
            const int yaw = atoi(yawStr);
            driveFieldCentricWithYawCorrection(vx, vy, yaw);
            if (!fromMasterUart) out.printf("KN: vx=%d vy=%d yaw=%d\n", vx, vy, yaw);
        } else {
            if (!fromMasterUart) out.println("Format: kn <vx> <vy> <yawDeg>");
        }
    } else if (strcmp(token, "goto") == 0) {
        char* xStr   = strtok(nullptr, " ");
        char* yStr   = strtok(nullptr, " ");
        char* yawStr = strtok(nullptr, " ");
        if (xStr != nullptr && yStr != nullptr && yawStr != nullptr) {
            const float x_m   = atof(xStr)   * 0.01f;  // cm → m
            const float y_m   = atof(yStr)   * 0.01f;
            const float yaw   = atof(yawStr);
            testYawMode = false;
            setWaypoint(x_m, y_m, yaw);
            if (!fromMasterUart) out.printf("WP set: x=%.3fm y=%.3fm yaw=%.1fdeg\n", x_m, y_m, yaw);
        } else {
            if (!fromMasterUart) out.println("Format: goto <x_cm> <y_cm> <yaw_deg>");
        }
    } else if (strcmp(token, "wp") == 0) {
        char* arg = strtok(nullptr, " ");
        if (arg != nullptr && strcmp(arg, "cancel") == 0) {
            cancelWaypoint();
            if (!fromMasterUart) out.println("WP dibatalkan.");
        } else if (arg != nullptr && strcmp(arg, "status") == 0) {
            const char* states[] = {"IDLE", "RUNNING", "REACHED"};
            if (!fromMasterUart) out.printf("WP state: %s | target=(%.3f,%.3f)m yaw=%.1fdeg\n",
                       states[(int)getWaypointState()],
                       wpTargetX_m, wpTargetY_m, wpTargetYaw_deg);
            if (!fromMasterUart) out.printf("Odom: x=%.3fm y=%.3fm yaw=%.1fdeg\n",
                       odomX, odomY, getYaw());
        } else {
            if (!fromMasterUart) out.println("Format: wp cancel | wp status");
        }
    } else if (strcmp(token, "tunewp") == 0) {
        char* kpStr      = strtok(nullptr, " ");
        char* tolPosStr  = strtok(nullptr, " ");
        char* tolYawStr  = strtok(nullptr, " ");
        if (kpStr != nullptr) {
            wpKpXY = atof(kpStr);
            if (tolPosStr != nullptr) wpTolPos_m   = atof(tolPosStr) * 0.01f;
            if (tolYawStr != nullptr) wpTolYaw_deg = atof(tolYawStr);
            if (!fromMasterUart) out.printf("WP params: Kp=%.1f TolPos=%.3fm TolYaw=%.1fdeg\n",
                       wpKpXY, wpTolPos_m, wpTolYaw_deg);
        } else {
            if (!fromMasterUart) out.printf("WP params: Kp=%.1f TolPos=%.3fm TolYaw=%.1fdeg\n",
                       wpKpXY, wpTolPos_m, wpTolYaw_deg);
            if (!fromMasterUart) out.println("Format: tunewp <kp> [tol_pos_cm] [tol_yaw_deg]");
        }
    } else if (strcmp(token, "savewp") == 0) {
        saveWaypointPid();
    } else if (strcmp(token, "odom") == 0) {
        if (!fromMasterUart) out.printf("Odom: x=%.3fm y=%.3fm theta=%.1fdeg | yaw=%.1fdeg\n",
                   odomX, odomY, odomTheta, getYaw());
    } else if (strcmp(token, "odomreset") == 0) {
        resetOdometry();
        if (!fromMasterUart) out.println("Odometri direset ke (0,0,0).");
    } else if (strcmp(token, "stop") == 0) {
        cancelWaypoint();
        testYawMode = false;
        if (!fromMasterUart) out.println("Semua motor BERHENTI.");
    } else if (strcmp(token, "autotune") == 0) {
        char* arg = strtok(nullptr, " ");
        if (arg == nullptr) {
            if (!fromMasterUart) out.println("Format: autotune <motor|all>");
        } else if (strcmp(arg, "all") == 0) {
            startAutoTuneAll();
        } else {
            startAutoTune(atoi(arg));
        }
    } else if (strcmp(token, "calibrate") == 0) {
        motorStopAll();
        calibrateGyro();
    } else if (strcmp(token, "calclear") == 0) {
        calibClearNVS();
    } else if (strcmp(token, "tuneyaw") == 0) {
        char* kpStr = strtok(nullptr, " ");
        char* kiStr = strtok(nullptr, " ");
        char* kdStr = strtok(nullptr, " ");
        if (kpStr != nullptr && kiStr != nullptr && kdStr != nullptr) {
            const float kp = atof(kpStr);
            const float ki = atof(kiStr);
            const float kd = atof(kdStr);
            pidKinematicYaw.kp = kp;
            pidKinematicYaw.ki = ki;
            pidKinematicYaw.kd = kd;
            pidKinematicYaw.reset();
            if (!fromMasterUart) out.printf("Yaw PID updated: Kp=%.3f Ki=%.3f Kd=%.3f\n", kp, ki, kd);
        } else {
            if (!fromMasterUart) out.println("Format: tuneyaw <kp> <ki> <kd>");
        }
    } else if (strcmp(token, "saveyaw") == 0) {
        saveYawPid();
    } else if (strcmp(token, "loadyaw") == 0) {
        initYawPid();
    } else if (strcmp(token, "showyaw") == 0) {
        showYawPid();
    } else {
        if (!fromMasterUart) printHelp(out);
    }
}

void readSerialLine(Stream& port, char* buf, uint8_t& idx, Print& out, bool fromMasterUart) {
    while (port.available() > 0) {
        const char c = port.read();
        if (c == '\n' || c == '\r') {
            if (idx > 0) {
                buf[idx] = '\0';
                parseAndExecuteCommand(buf, out, fromMasterUart);
                idx = 0;
            }
        } else if (idx < SERIAL_CMD_BUF_SIZE - 1) {
            buf[idx++] = c;
        } else {
            if (!fromMasterUart) out.println("Error: command terlalu panjang");
            idx = 0;
        }
    }
}

}  // namespace

// =====================================================================
//  SETUP
// =====================================================================

void setupSerial() {
    pinMode(WSN_SET_PIN, OUTPUT);
    digitalWrite(WSN_SET_PIN, HIGH);

    Serial2.begin(SERIAL_WSN_BAUD, SERIAL_8N1, SERIAL_WSN_RX, SERIAL_WSN_TX);
    Serial1.begin(SERIAL_MASTER_BAUD, SERIAL_8N1, SERIAL_MASTER_RX, SERIAL_MASTER_TX);

    Serial.printf("[Serial] Master UART RX=%d TX=%d @ %lu baud | WSN RX=%d TX=%d\n",
                  SERIAL_MASTER_RX, SERIAL_MASTER_TX, (unsigned long)SERIAL_MASTER_BAUD,
                  SERIAL_WSN_RX, SERIAL_WSN_TX);
}

// =====================================================================
//  RELAY — forwarding WSN-31 → Master
// =====================================================================

void serialRelayTick() {
    while (Serial2.available()) {
        const uint8_t b = Serial2.read();
        Serial1.write(b);
    }
}

// =====================================================================
//  COMMAND HANDLER — USB Serial + UART master
// =====================================================================

void serialCommandTick() {
    readSerialLine(Serial, pcBuf, pcBufIdx, Serial, false);
    readSerialLine(Serial1, masterBuf, masterBufIdx, Serial1, true);
}
