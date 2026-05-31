// ============================================================
// SERIAL COMMANDS INTERFACE - Implementation
// Fully Non-Blocking State Machine (No delay())
// ============================================================

#include "serial_commands.h"

// ============================================================
// Global Variables
// ============================================================
static volatile bool stopRequested = false;

// --- Non-blocking state machine for timed motor moves ---
enum class CmdState : uint8_t {
    CMD_IDLE = 0,
    CMD_MOVE_RPM,
    CMD_MOVE_PWM,
    CMD_SEQ_RPM,
    CMD_SEQ_PWM,
    CMD_ROBOT,
    CMD_FIELD,
    // Sub-states for SEQ
    CMD_SEQ_RUNNING,
};

static CmdState g_cmdState = CmdState::CMD_IDLE;
static unsigned long g_cmdStartMs = 0;
static unsigned long g_cmdDurationMs = 0;
static int g_cmdIdx = 0;
static float g_cmdValue = 0.0f;
static int g_cmdPwm = 0;
static int g_cmdVx = 0, g_cmdVy = 0, g_cmdVtheta = 0;
static int g_seqMaxIdx = 0;

// ============================================================
// Helper Functions
// ============================================================

static void toUpperInPlace(char *s) {
    for (; *s; ++s) *s = toupper((unsigned char)*s);
}

void serialEmergencyStop() {
    stopRequested = true;
    g_cmdState = CmdState::CMD_IDLE;
    autoTunerAbort();
    motorStopAll();
    Serial.println("EMERGENCY STOP!");
}

// ============================================================
// Non-blocking timed motor tick — called from loop()
// ============================================================

void serialCommandsTick() {
    if (stopRequested) {
        stopRequested = false;
        g_cmdState = CmdState::CMD_IDLE;
        motorStopAll();
        return;
    }

    switch (g_cmdState) {

    case CmdState::CMD_IDLE:
        // Nothing to do
        break;

    // ------------------------------------------------
    case CmdState::CMD_MOVE_RPM: {
        if (millis() - g_cmdStartMs >= g_cmdDurationMs) {
            motorStopAll();
            g_cmdState = CmdState::CMD_IDLE;
            Serial.println("Done.");
            break;
        }
        int rpm1 = (g_cmdIdx == 0) ? (int)g_cmdValue : 0;
        int rpm2 = (g_cmdIdx == 1) ? (int)g_cmdValue : 0;
        int rpm3 = (g_cmdIdx == 2) ? (int)g_cmdValue : 0;
        int rpm4 = (g_cmdIdx == 3) ? (int)g_cmdValue : 0;
        rpmMotor(rpm1, rpm2, rpm3, rpm4);
        break;
    }

    // ------------------------------------------------
    case CmdState::CMD_MOVE_PWM: {
        if (millis() - g_cmdStartMs >= g_cmdDurationMs) {
            pwmMotor(g_cmdIdx, 0);
            g_cmdState = CmdState::CMD_IDLE;
            Serial.println("Done.");
            break;
        }
        pwmMotor(g_cmdIdx, g_cmdPwm);
        break;
    }

    // ------------------------------------------------
    case CmdState::CMD_SEQ_RUNNING: {
        if (stopRequested || g_cmdIdx > g_seqMaxIdx) {
            motorStopAll();
            g_cmdState = CmdState::CMD_IDLE;
            Serial.println("Sequential test complete.");
            break;
        }

        // If current motor's time has expired, move to next
        if (millis() - g_cmdStartMs >= g_cmdDurationMs) {
            motorStopAll();
            g_cmdIdx++;
            if (g_cmdIdx <= g_seqMaxIdx) {
                g_cmdStartMs = millis();
                Serial.printf("Testing motor %d...\n", g_cmdIdx);
            }
            break;
        }

        // Run current motor
        if (g_cmdValue != 0.0f) {
            int r1 = (g_cmdIdx == 0) ? (int)g_cmdValue : 0;
            int r2 = (g_cmdIdx == 1) ? (int)g_cmdValue : 0;
            int r3 = (g_cmdIdx == 2) ? (int)g_cmdValue : 0;
            int r4 = (g_cmdIdx == 3) ? (int)g_cmdValue : 0;
            rpmMotor(r1, r2, r3, r4);
        } else {
            pwmMotor(g_cmdIdx, g_cmdPwm);
        }
        break;
    }

    // ------------------------------------------------
    case CmdState::CMD_ROBOT: {
        if (millis() - g_cmdStartMs >= g_cmdDurationMs) {
            motorStopAll();
            g_cmdState = CmdState::CMD_IDLE;
            Serial.println("Done.");
            break;
        }
        driveRobotCentric(g_cmdVx, g_cmdVy, g_cmdVtheta);
        break;
    }

    // ------------------------------------------------
    case CmdState::CMD_FIELD: {
        if (millis() - g_cmdStartMs >= g_cmdDurationMs) {
            motorStopAll();
            g_cmdState = CmdState::CMD_IDLE;
            Serial.println("Done.");
            break;
        }
        driveFieldCentric(g_cmdVx, g_cmdVy, g_cmdVtheta);
        break;
    }

    }
}

// ============================================================
// Continuous motor command tick (MOTOR / MOTORS) — called from loop()
// ============================================================

// These are stored as globals for continuous running
static int g_contIdx = -1;
static int g_contRpm[4] = {0, 0, 0, 0};

void serialContinuousTick() {
    if (g_contIdx < 0) return;
    if (autoTunerIsActive()) return; // Don't interfere with autotune

    if (g_contIdx == 4) {
        // MOTORS mode — all 4
        rpmMotor(g_contRpm[0], g_contRpm[1], g_contRpm[2], g_contRpm[3]);
    } else {
        // Single MOTOR mode
        int rpm1 = (g_contIdx == 0) ? g_contRpm[0] : 0;
        int rpm2 = (g_contIdx == 1) ? g_contRpm[1] : 0;
        int rpm3 = (g_contIdx == 2) ? g_contRpm[2] : 0;
        int rpm4 = (g_contIdx == 3) ? g_contRpm[3] : 0;
        rpmMotor(rpm1, rpm2, rpm3, rpm4);
    }
}

void serialContinuousStop() {
    g_contIdx = -1;
    g_contRpm[0] = g_contRpm[1] = g_contRpm[2] = g_contRpm[3] = 0;
    motorStopAll();
}

// ============================================================
// Public API — Serial command starters
// ============================================================

void serialMoveRpm(int idx, float rpm, unsigned long durationMs) {
    if (idx < 0 || (size_t)idx >= motors.size()) return;
    Serial.printf("Moving motor %d to %.1f RPM for %lu ms\n", idx, rpm, durationMs);
    g_cmdState = CmdState::CMD_MOVE_RPM;
    g_cmdIdx = idx;
    g_cmdValue = rpm;
    g_cmdDurationMs = durationMs;
    g_cmdStartMs = millis();
}

void serialMovePwm(int idx, int pwm, unsigned long durationMs) {
    if (idx < 0 || (size_t)idx >= motors.size()) return;
    Serial.printf("Moving motor %d with PWM %d for %lu ms\n", idx, pwm, durationMs);
    g_cmdState = CmdState::CMD_MOVE_PWM;
    g_cmdIdx = idx;
    g_cmdPwm = pwm;
    g_cmdDurationMs = durationMs;
    g_cmdStartMs = millis();
}

void serialSeqRpm(float rpm, unsigned long durationMs) {
    Serial.printf("Sequential test: %.1f RPM for %lu ms per motor\n", rpm, durationMs);
    g_cmdState = CmdState::CMD_SEQ_RUNNING;
    g_cmdIdx = 0;
    g_cmdValue = rpm;
    g_cmdPwm = 0;
    g_cmdDurationMs = durationMs;
    g_cmdStartMs = millis();
    g_seqMaxIdx = motors.size() - 1;
    Serial.printf("Testing motor %d...\n", 0);
}

void serialSeqPwm(int pwm, unsigned long durationMs) {
    Serial.printf("Sequential PWM test: %d for %lu ms per motor\n", pwm, durationMs);
    g_cmdState = CmdState::CMD_SEQ_RUNNING;
    g_cmdIdx = 0;
    g_cmdValue = 0.0f;
    g_cmdPwm = pwm;
    g_cmdDurationMs = durationMs;
    g_cmdStartMs = millis();
    g_seqMaxIdx = motors.size() - 1;
    Serial.printf("Testing motor %d...\n", 0);
}

void serialTestRobotCentric(int vx, int vy, int vtheta, unsigned long durationMs) {
    Serial.printf("Robot-Centric: vx=%d vy=%d w=%d for %lu ms\n", vx, vy, vtheta, durationMs);
    g_cmdState = CmdState::CMD_ROBOT;
    g_cmdVx = vx;
    g_cmdVy = vy;
    g_cmdVtheta = vtheta;
    g_cmdDurationMs = durationMs;
    g_cmdStartMs = millis();
}

void serialTestFieldCentric(int vx, int vy, int vtheta, unsigned long durationMs) {
    Serial.printf("Field-Centric: vx=%d vy=%d w=%d for %lu ms\n", vx, vy, vtheta, durationMs);
    g_cmdState = CmdState::CMD_FIELD;
    g_cmdVx = vx;
    g_cmdVy = vy;
    g_cmdVtheta = vtheta;
    g_cmdDurationMs = durationMs;
    g_cmdStartMs = millis();
}

void serialAutoTuneSingle(int motorIdx, float initKp, float initKi, float initKd) {
    if (motorIdx < 0 || (size_t)motorIdx >= motors.size()) return;
    // Stop any continuous motor first
    serialContinuousStop();
    stopRequested = false;
    Serial.printf("Starting auto-tune for motor %d...\n", motorIdx);
    autoTunerStartSingle(motorIdx, initKp, initKi, initKd);
}

void serialAutoTuneAll() {
    serialContinuousStop();
    stopRequested = false;
    Serial.println("Starting auto-tune for all motors...");
    autoTunerStart();
}

void printSerialUsage() {
    Serial.println("\n=== SERIAL COMMANDS ===");
    Serial.println("AUTOTUNE:");
    Serial.println("  AUTOTUNE RPM <idx>                          - autotune single motor (0-3)");
    Serial.println("  AUTOTUNE RPM <idx> <Kp> <Ki> <Kd>          - autotune with custom initial gains");
    Serial.println("  AUTOTUNE RPM ALL                            - autotune all motors");
    Serial.println("\nMOTOR CONTROL:");
    Serial.println("  MOTOR <idx> <rpm>                           - set single motor RPM (continuous)");
    Serial.println("  MOTORS <r1> <r2> <r3> <r4>                 - set all 4 motors RPM (continuous)");
    Serial.println("  MOVE RPM <idx> <rpm> <ms>                  - run motor for duration");
    Serial.println("  MOVE PWM <idx> <pwm> <ms>                  - run motor PWM for duration");
    Serial.println("  SEQ RPM <rpm> <ms>                         - test motors one-by-one");
    Serial.println("  SEQ PWM <pwm> <ms>                         - test motors PWM one-by-one");
    Serial.println("\nKINEMATIK:");
    Serial.println("  ROBOT <vx> <vy> <w> <ms>                   - robot-centric movement");
    Serial.println("  FIELD <vx> <vy> <w> <ms>                   - field-centric movement");
    Serial.println("\nOTHER:");
    Serial.println("  STOP                                       - emergency stop");
    Serial.println("  HELP                                       - show this help");
}

// ============================================================
// Main Serial Command Processor — called from loop()
// Parses serial input and starts the appropriate state machine
// ============================================================

void processSerialCommands() {
    if (!Serial.available()) return;

    char buf[128];
    size_t len = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
    buf[len] = '\0';

    // Trim whitespace
    char *p = buf;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0') return;

    // Make uppercase copy for command comparison
    char cmd[128];
    strncpy(cmd, p, sizeof(cmd));
    cmd[sizeof(cmd)-1] = '\0';
    toUpperInPlace(cmd);

    // ==================== STOP ====================
    if (strncmp(cmd, "STOP", 4) == 0) {
        serialEmergencyStop();
        return;
    }

    // ==================== HELP ====================
    if (strncmp(cmd, "HELP", 4) == 0) {
        printSerialUsage();
        return;
    }

    // Stop any timed state-machine command when a new command arrives
    if (g_cmdState != CmdState::CMD_IDLE) {
        motorStopAll();
        g_cmdState = CmdState::CMD_IDLE;
    }

    // ==================== AUTOTUNE ====================
    if (strncmp(cmd, "AUTOTUNE", 8) == 0) {
        char *tok = strtok(p, " \t");
        tok = strtok(NULL, " \t");
        if (!tok) {
            Serial.println("Usage: AUTOTUNE RPM <idx|ALL>");
            return;
        }
        toUpperInPlace(tok);
        if (strcmp(tok, "RPM") == 0) {
            char *arg = strtok(NULL, " \t");
            if (!arg) {
                Serial.println("Usage: AUTOTUNE RPM <idx|ALL>");
                return;
            }
            toUpperInPlace(arg);
            if (strcmp(arg, "ALL") == 0) {
                serialAutoTuneAll();
            } else {
                int idx = atoi(arg);
                char *skp = strtok(NULL, " \t");
                char *ski = strtok(NULL, " \t");
                char *skd = strtok(NULL, " \t");
                if (skp && ski && skd) {
                    serialAutoTuneSingle(idx, atof(skp), atof(ski), atof(skd));
                } else {
                    serialAutoTuneSingle(idx);
                }
            }
        }
        return;
    }

    // ==================== MOTOR (single - continuous) ====================
    if (strncmp(cmd, "MOTOR ", 6) == 0) {
        char *sidx = strtok(p, " \t"); sidx = strtok(NULL, " \t");
        char *srpm = strtok(NULL, " \t");
        if (!sidx || !srpm) {
            Serial.println("Usage: MOTOR <idx> <rpm>");
            return;
        }
        int idx = atoi(sidx);
        float rpm = atof(srpm);

        serialContinuousStop();
        g_contIdx = idx;
        g_contRpm[0] = (idx == 0) ? (int)rpm : 0;
        g_contRpm[1] = (idx == 1) ? (int)rpm : 0;
        g_contRpm[2] = (idx == 2) ? (int)rpm : 0;
        g_contRpm[3] = (idx == 3) ? (int)rpm : 0;

        Serial.printf("Setting motor %d to %.1f RPM (continuous)\n", idx, rpm);
        Serial.println("Send STOP to stop the motor.");
        return;
    }

    // ==================== MOTORS (all 4 - continuous) ====================
    if (strncmp(cmd, "MOTORS ", 7) == 0) {
        char *sr1 = strtok(p, " \t"); sr1 = strtok(NULL, " \t");
        char *sr2 = strtok(NULL, " \t");
        char *sr3 = strtok(NULL, " \t");
        char *sr4 = strtok(NULL, " \t");
        if (!sr1 || !sr2 || !sr3 || !sr4) {
            Serial.println("Usage: MOTORS <rpm1> <rpm2> <rpm3> <rpm4>");
            return;
        }
        serialContinuousStop();
        g_contIdx = 4; // Special: all 4
        g_contRpm[0] = atoi(sr1);
        g_contRpm[1] = atoi(sr2);
        g_contRpm[2] = atoi(sr3);
        g_contRpm[3] = atoi(sr4);

        Serial.printf("Setting motors: %d %d %d %d RPM (continuous)\n",
                      g_contRpm[0], g_contRpm[1], g_contRpm[2], g_contRpm[3]);
        Serial.println("Send STOP to stop the motors.");
        return;
    }

    // ==================== MOVE ====================
    if (strncmp(cmd, "MOVE", 4) == 0) {
        char *tok = strtok(p, " \t");
        char *mode = strtok(NULL, " \t");
        if (!mode) { Serial.println("Usage: MOVE <RPM|PWM> <idx> <value> <ms>"); return; }
        toUpperInPlace(mode);
        char *sidx = strtok(NULL, " \t");
        char *sval = strtok(NULL, " \t");
        char *sdur = strtok(NULL, " \t");
        if (!sidx || !sval) { Serial.println("Usage: MOVE <RPM|PWM> <idx> <value> <ms>"); return; }
        int idx = atoi(sidx);
        long dur = (sdur) ? atol(sdur) : 2000;

        serialContinuousStop();

        if (strcmp(mode, "RPM") == 0) {
            serialMoveRpm(idx, atof(sval), (unsigned long)dur);
        } else if (strcmp(mode, "PWM") == 0) {
            serialMovePwm(idx, atoi(sval), (unsigned long)dur);
        }
        return;
    }

    // ==================== SEQ ====================
    if (strncmp(cmd, "SEQ", 3) == 0) {
        char *tok = strtok(p, " \t");
        char *mode = strtok(NULL, " \t");
        char *sval = strtok(NULL, " \t");
        char *sdur = strtok(NULL, " \t");
        if (!mode || !sval) { Serial.println("Usage: SEQ <RPM|PWM> <value> <ms>"); return; }
        toUpperInPlace(mode);
        long dur = (sdur) ? atol(sdur) : 2000;

        serialContinuousStop();

        if (strcmp(mode, "RPM") == 0) {
            serialSeqRpm(atof(sval), (unsigned long)dur);
        } else if (strcmp(mode, "PWM") == 0) {
            serialSeqPwm(atoi(sval), (unsigned long)dur);
        }
        return;
    }

    // ==================== ROBOT (Robot-Centric) ====================
    if (strncmp(cmd, "ROBOT", 5) == 0) {
        char *svx = strtok(p, " \t"); svx = strtok(NULL, " \t");
        char *svy = strtok(NULL, " \t");
        char *sw  = strtok(NULL, " \t");
        char *sdur = strtok(NULL, " \t");
        if (!svx || !svy || !sw) { Serial.println("Usage: ROBOT <vx> <vy> <w> <ms>"); return; }
        long dur = (sdur) ? atol(sdur) : 2000;

        serialContinuousStop();
        serialTestRobotCentric(atoi(svx), atoi(svy), atoi(sw), (unsigned long)dur);
        return;
    }

    // ==================== FIELD (Field-Centric) ====================
    if (strncmp(cmd, "FIELD", 5) == 0) {
        char *svx = strtok(p, " \t"); svx = strtok(NULL, " \t");
        char *svy = strtok(NULL, " \t");
        char *sw  = strtok(NULL, " \t");
        char *sdur = strtok(NULL, " \t");
        if (!svx || !svy || !sw) { Serial.println("Usage: FIELD <vx> <vy> <w> <ms>"); return; }
        long dur = (sdur) ? atol(sdur) : 2000;

        serialContinuousStop();
        serialTestFieldCentric(atoi(svx), atoi(svy), atoi(sw), (unsigned long)dur);
        return;
    }

    // ==================== UNKNOWN COMMAND ====================
    Serial.println("Unknown command. Type HELP for usage.");
}
