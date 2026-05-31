# Robot Manual - Copilot Instructions

**Project**: KRAI 2026 Robot Manual Control  
**Platform**: ESP32 (Arduino Core 2.0.17)  
**Language**: C++ (Arduino)  
**Architecture**: Embedded control system with dynamic vector-based motor/encoder management

---

## Table of Contents

1. [Quick Start](#quick-start)
2. [Code Structure](#code-structure)
3. [Naming Conventions](#naming-conventions)
4. [Core Components](#core-components)
5. [Development Workflow](#development-workflow)
6. [Common Tasks](#common-tasks)

---

## Quick Start

**Build & Upload:**
  ```bash
  cd /home/aditya/Documents/KRAI_2026/robot_manual
  arduino-cli compile --fqbn esp32:esp32:esp32s3 .
  ```

**Key Files to Edit:**
- [`robot_config.h`](../robot_config.h) — Hardware pins, constants, motor/encoder configuration
- [`motor.ino`](../motor.ino) — Motor PWM control (low-level)
- [`encoder.ino`](../encoder.ino) — Encoder ISR and velocity calculation
- [`pid_controller.ino`](../pid_controller.ino) — Velocity feedback PID controller
- [`autoTuner.ino`](../autoTuner.ino) — Automatic PID parameter tuning
- [`robot_manual.ino`](../robot_manual.ino) — Main entry point and boot button handler

---

## Code Structure

### Header File: `robot_config.h`
**Purpose**: Centralized configuration for hardware pins, constants, and shared type definitions.

**Contains:**
- Motor/encoder pin definitions (`motorDepanKanan_A`, `encoderMotorBelakangKiri_A`, etc.)
- Hardware constants (`encoderMotorPpr`, `maxPwm`, `pwmFrequency`, `pwmResolution`)
- PID limits (`kpMin`, `kpMax`, `kiMin`, `kiMax`, `kdMin`, `kdMax`)
- Unit converters (`kRadPerSecToRpm`, `kRpmToRadPerSec`)
- Shared struct definitions (`MotorConfig`, `EncoderConfig`)
- Extern declarations for globals (`motors`, `encoders`)
- Auto-tuner settings (`AUTOTUNE_TARGET_VEL`, `AUTOTUNE_RUN_MS`, `AUTOTUNE_MAX_CYCLES`)

### Motor Control: `motor.ino`
**Purpose**: Low-level PWM output to motors via H-bridge.

**Key Functions:**
- `SetupMotors()` — Initialize motor pins and LEDC PWM channels
- `pwmMotor(int idMotor, int pwmValue)` — Send PWM command to one motor
  - Validates motor index bounds
  - Constrains PWM to `[minPwm, maxPwm]`
  - Sets direction pin based on sign (HIGH = backward, LOW = forward)

**Global:**
- `std::vector<MotorConfig> motors` — Dynamically populated from pins in config

### Encoder Reading: `encoder.ino`
**Purpose**: Quadrature encoder decoding and RPM calculation.

**Key Functions:**
- `void IRAM_ATTR Encoder(void *arg)` — ISR callback for encoder pin changes
  - Called on CHANGE event for each encoder's Pin A
  - Uses `attachInterruptArg()` to pass motor index
  - Updates `encoders[idx].count` (quadrature direction detection)
  
- `setupEncoders()` — Initialize encoder pins and attach interrupts
  
- `convertEncoderToRPM()` — Convert pulse counts to RPM (runs in main loop)
  - Called from `robot_manual.ino` loop
  - Updates `motorVelocityRpm` cache every `intervalRpm` ms
  - Uses low-pass filter (alpha = 0.3) for noise reduction
  - Implements Welford's online mean/variance for steady-state metrics
  
- `getEncoderVelocityRpm(int motorIdx)` → float — Get latest RPM value
- `getEncoderVelocityRadS(int motorIdx)` → float — Convert to rad/s

**Globals:**
- `std::vector<EncoderConfig> encoders` — Dynamically sized
- `std::vector<float> motorVelocityRpm` — Cache of latest RPM per motor (updated by `convertEncoderToRPM()`)
- `const unsigned long intervalRpm` = 100 ms — Period for RPM calculation

### PID Controller: `pid_controller.ino`
**Purpose**: Velocity feedback control loop for each motor.

**Key Functions:**
- `pidControllerInit()` — Initialize PID state vectors from NVS
- `pidLoadFromNVS(int motorIdx, float &kp, float &ki, float &kd)` — Read from flash
- `pidSaveToNVS(int motorIdx, float kp, float ki, float kd)` — Write to flash
- `pidSetGains(int motorIdx, float kp, float ki, float kd)` — Update live PID coefficients
- `pidResetOne(int motorIdx)` — Clear integral and derivative history
- `pidCompute(int motorIdx, float targetRPM, float dt)` → int — Run one PID iteration, return PWM command
- `rpmMotorControl(int rpm0, int rpm1, int rpm2, int rpm3)` — Convenience wrapper to control all 4 motors
  - Note: the `rpmMotorControl(...)` helper is a fixed 4-motor convenience wrapper in this codebase. For configurations with more than 4 motors, replace it with a vector-based API (for example `rpmMotorControl(const std::vector<int>& rpms)`) or provide an overload that accepts a dynamic array; also update any call sites accordingly.
- `motorStopAll()` — Set all motors to PWM=0

**Globals:**
- `std::vector<PIDState> pidStates` — Controller state per motor

### Auto-Tuner: `autoTuner.ino`
**Purpose**: Automatically tune PID parameters via relay-based perturbation and scoring.

**Key Functions:**
- `autoTunerStart()` — Transition from IDLE to WAIT_RELEASE
- `autoTunerIsActive()` → bool — Check if tuning is in progress
- `autoTunerTick(bool bootPressed)` — Main state machine (call from loop ~10ms rate or faster)

**State Machine:**
```
AT_IDLE → AT_WAIT_RELEASE → AT_MOTOR_INIT → [AT_CYCLE_START → AT_CYCLE_RUN → AT_CYCLE_FINISH → AT_CYCLE_COOLDOWN]* → AT_MOTOR_SHOW → AT_DONE
```

**Features:**
- Precision stages (COARSE → FINE → ULTRA_FINE)
- Dynamic parameter adjustment based on overshoot, rise time, steady-state error
- Welford's online variance for jitter detection
- 3-precision stage tuning with stagnation detection
- Scores best cycle across 12 max attempts per motor
- Saves best PID to NVS per motor
- OLED output stubs (Serial fallback)

**Globals:**
- `static ATState gAtState` — Current tuning state
- `static int gMotorIdx` — Motor being tuned (0 to motors.size()-1)
- `static float gCurrentKp, gCurrentKi, gCurrentKd` — Live PID gains during a cycle
- `static float gBestKp, gBestKi, gBestKd` — Best gains found so far
- `static CycleMetrics gMetrics` — Performance data for current cycle

### Main Sketch: `robot_manual.ino`
**Purpose**: Application entry point, boot button handler, main loop.

**Boot Button Trigger:**
- Reads BOOT_BUTTON_PIN (GPIO 0)
- On 3-second continuous press: launches auto-tuner
- LED feedback via OLED (or Serial)

**Loop Flow:**
1. Read encoders → `convertEncoderToRPM()`
2. If auto-tuner active: `autoTunerTick(bootPressed)` (skips normal control)
3. Else: Manual/normal mode (placeholder for user control logic)

---

## Naming Conventions

### Variables & Constants
**Style**: `lowerCamelCase`

**Examples**:
- Motor config: `motorDepanKanan`, `motorBelakangKiri`
- Encoder pins: `encoderMotorBelakangKanan_A`, `encoderMotorBelakangKiri_B`
- PID limits: `kpMin`, `kpMax`, `kiMin`, `kiMax`, `kdMin`, `kdMax`
- Encoder: `encoderMotorPpr` (pulses per revolution), `intervalRpm`, `millisRpm`
- Velocity: `motorVelocityRpm`
- Time: `gStartMs`, `gLastPidTickMs`

**Rules**:
- Boolean: prefix `is` or `has` (e.g., `isMotorRunning`, `hasEncoderData`)
- Getter: prefix `get` (e.g., `getEncoderVelocityRpm()`)
- Setter: prefix `set` (e.g., `pidSetGains()`)
- Global state machine: prefix `g` (e.g., `gAtState`, `gMotorIdx`, `gCurrentKp`)

### Macros & #defines
**Style**: `UPPERCASE_WITH_UNDERSCORES` (legacy pattern, OK to keep)

**Examples**:
- `#define BOOT_BUTTON_PIN 0`
- `#define AUTOTUNE_TARGET_VEL 3.0f`
- `#define AUTOTUNE_RUN_MS 10000`
- `#define PID_NVS_NAMESPACE "pid_tuning"`

### Functions
**Style**: `lowerCamelCase`

**Naming Patterns**:
- Hardware init: `SetupMotors()`, `setupEncoders()`, `pidControllerInit()`
- Control loop: `pwmMotor()`, `pidCompute()`, `convertEncoderToRPM()`
- State machine: `autoTunerTick()`, `autoTunerStart()`
- Utilities: `checkInterval()`, `motorStopAll()`

### Types & Structs
**Style**: `PascalCase`

**Examples**:
- `struct MotorConfig { ... }`
- `struct EncoderConfig { ... }`
- `struct PIDState { ... }`
- `struct CycleMetrics { ... }`
- `enum class Precision : uint8_t { COARSE, FINE, ULTRA_FINE }`

---

## Core Components

### Motor Control Loop
```
Target RPM (user) 
  ↓
pidCompute() [PID feedback control]
  ↓
Motor PWM output (via pwmMotor)
  ↓
Encoded tachometer feedback (Encoder ISR)
  ↓
convertEncoderToRPM() [Low-pass filtering, Welford's online stats]
  ↓
getEncoderVelocityRpm() [Velocity cache for next PID iteration]
```

### Dynamic Motor/Encoder Count
- **Motivation**: Support variable robot configurations (4 wheels standard, extensible)
- **Implementation**: `std::vector<MotorConfig>` and `std::vector<EncoderConfig>` sized at runtime
- **Size**: Determined by initialization list in `motor.ino` and `encoder.ino` (currently 4)
 - **Auto-tuner**: Loops from `0` to `motors.size()` (dynamic) — however, the repository's helper functions and NVS keys are configured for 4 motors by default. If you add more than 4 motors you must:
   - Update or replace the `rpmMotorControl(...)` wrapper to accept a dynamic number of motors (e.g. `rpmMotorControl(const std::vector<int>& rpms)`).
   - Expand the NVS key scheme and storage logic so keys are generated per motor (e.g. `kp_<motorIdx>`, `ki_<motorIdx>`, `kd_<motorIdx>`) instead of assuming keys only up to `_3`.

### NVS (Flash Memory) Storage
- **Namespace**: `"pid_tuning"`
- **Keys**: `"kp_0"`, `"ki_0"`, `"kd_0"`, ... `"kp_3"`, `"ki_3"`, `"kd_3"` (per motor)
- **Usage**: Store auto-tuned PID parameters across power cycles
- **Load/Save API**:
  - `pidLoadFromNVS(motorIdx, kp, ki, kd)` — Load default on first boot
  - `pidSaveToNVS(motorIdx, kp, ki, kd)` — Save after tuning complete
  - Note: current key list shows `kp_0 .. kp_3` for the default 4-motor setup. When adding motors, implement programmatic key generation (e.g. `snprintf(key, sizeof(key), "kp_%d", motorIdx)`) so NVS scales with `motors.size()`.

---

## Development Workflow

### Adding a New Motor/Encoder
1. **Update `robot_config.h`**: Add pin definitions
   ```cpp
   #define motorDepanKanan_A   15
   #define motorDepanKanan_B   16
   #define encoderMotorDepanKanan_A   1
   ```

2. **Update `motor.ino`**: Add to the `motors` vector initialization
   ```cpp
   std::vector<MotorConfig> motors = {
       {motorDepanKanan_A, motorDepanKanan_B, 0},
       // ... more motors
   };
   ```

3. **Update `encoder.ino`**: Add to the `encoders` vector initialization
   ```cpp
   std::vector<EncoderConfig> encoders = {
       {encoderMotorDepanKanan_A, encoderMotorDepanKanan_B, 0},
       // ... more encoders
   };
   ```

4. **Compile & verify**: `arduino-cli compile --fqbn esp32:esp32:esp32s3 .`

### Tuning PID Manually
1. Press BOOT button for 3 seconds to start auto-tuner
2. Follow terminal/OLED output
3. Auto-tuner saves best Kp, Ki, Kd to NVS
4. Values are reloaded on next power cycle

### Testing Motor Control
```cpp
// In robot_manual.ino loop:
int targetRpm = 500;  // 500 RPM on motor 0
rpmMotorControl(targetRpm, 0, 0, 0);  // Other motors idle
```

### Monitoring Encoder/Velocity
```cpp
// Get current RPM for motor 0:
float currentRpm = getEncoderVelocityRpm(0);
Serial.printf("Motor 0: %.1f RPM\n", currentRpm);
```

---

## Common Tasks

### Task: Change Motor Direction (Motor Reverse)
**File**: [`motor.ino`](../motor.ino)

Inside `pwmMotor()`, swap the direction pin logic for the target motor:
```cpp
// Example: reverse motor 1 (Front Left)
} else if (pwmValue < 0) {
    ledcWrite(motors[idMotor].ledc_channel, duty);
    // Swap: was HIGH, now LOW (and vice versa)
    digitalWrite(motors[idMotor].pin_direction, LOW);  // ← Change this line
}
```

### Task: Adjust Encoder PPR or Motor Count
**File**: [`robot_config.h`](../robot_config.h)

```cpp
const int encoderMotorPpr = 270;  // Change to your encoder spec
// Recompile and upload
```

### Task: Modify PID Tuning Targets
**File**: [`robot_config.h`](../robot_config.h)

```cpp
#define AUTOTUNE_TARGET_VEL   3.0f   // Target velocity (rad/s)
#define AUTOTUNE_RUN_MS       10000  // Test duration per cycle (ms)
#define AUTOTUNE_MAX_CYCLES   12     // Max tuning iterations per motor
```

**PID Safety Note:** When integrating or modifying PID code, always implement anti-windup and a hardware-failure cutoff. Recommended safeguards:
- Limit the integral term magnitude (integral windup bounds) inside `pidCompute()` to prevent runaway when the actuator saturates.
- If the motor PWM output exceeds a configurable threshold but the encoder reports exactly `0` RPM for a sustained timeout (for example `>= 500 ms`), trigger a safety cutoff: set PWM to `0`, log the fault, and optionally mark the motor as `faulted` to avoid integral accumulation.
- Provide configurable thresholds in `robot_config.h` such as `PID_INTEGRAL_LIMIT` and `ZERO_RPM_TIMEOUT_MS` so they can be tuned per hardware.

### Task: Debug Encoder Velocity
**File**: [`encoder.ino`](../encoder.ino)

Velocity is printed every 100 ms:
```
Encoder Motor 0: 25 pulses | RPM: 150.5
Motor 0 - RPM Raw: 150.2 | RPM Filtered: 150.1
```

Review `convertEncoderToRPM()` output in Serial monitor.

### Task: View Auto-Tuner Progress
**File**: [`autoTuner.ino`](../autoTuner.ino)

OLED stubs print to Serial (see `oledAutotuner*()` functions):
```
[COARSE] Target: 286.4 RPM, Current: 142.5 RPM, Elapsed: 5123/10000 ms
Motor 0: Kp=15.00, Ki=0.50, Kd=0.04 [OK] Cycles: 12
```

---

## Appendix: Project Layout

```
robot_manual/
├── .github/
│   └── copilot-instructions.md        (this file)
├── robot_config.h                    (shared config & types)
├── motor.ino                         (PWM control)
├── encoder.ino                       (ISR + RPM calc)
├── pid_controller.ino                (velocity feedback PID)
├── autoTuner.ino                     (auto-tuning state machine)
├── robot_manual.ino                  (main entry point)
└── error.ino                         (error handling stubs)
```

---

## Questions & Feedback

If instructions are unclear or missing details, please:
1. Note the section or task that was ambiguous
2. Describe what info was needed
3. Suggest clarification
4. Copilot: Use your chat response to ask the user clarifying questions if context is missing.

Example: *"Section 'Debug Encoder Velocity' doesn't explain what to do if RPM values are negative unexpectedly."*
