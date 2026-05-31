#include "motor_arm2_pid_control.h"
#include "pinout.h"
#include "config.h"
#include "uart_slave_receiver.h"
#include "driver/pcnt.h"
#include "driver/mcpwm.h"
#include "driver/ledc.h"

// ==========================================
// IMPLEMENTASI MANIPULATOR 2 (LENGAN KEDUA)
// ==========================================

volatile int16_t arm2_target_putar = 0;
volatile int16_t arm2_target_z     = 0;
volatile int16_t arm2_target_y     = 0;
volatile uint8_t arm2_target_wrist = 0;

// Posisi aktual ticks encoder PCNT
static int16_t current_ticks_putar = 0;
static int16_t current_ticks_z     = 0;
static int16_t current_ticks_y     = 0;

// Variabel komparator PID Posisi Lengan 2
static float err_p_prev = 0, integral_p = 0;
static float err_z_prev = 0, integral_z = 0;
static float err_y_prev = 0, integral_y = 0;
static uint32_t last_pid_time = 0;

// -----------------------------------------
// Init unit hardware PCNT
// -----------------------------------------
static void motor_encoder_pcnt_init(pcnt_unit_t unit, int pin_a, int pin_b) {
    pcnt_config_t cfg = {
        .pulse_gpio_num = pin_a,
        .ctrl_gpio_num  = pin_b,
        .lctrl_mode     = PCNT_MODE_REVERSE,
        .hctrl_mode     = PCNT_MODE_KEEP,
        .pos_mode       = PCNT_COUNT_INC,
        .neg_mode       = PCNT_COUNT_DEC,
        .counter_h_lim  = 32767,
        .counter_l_lim  = -32768,
        .unit           = unit,
        .channel        = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&cfg);
    pcnt_filter_enable(unit);
    pcnt_set_filter_value(unit, 100);
    pcnt_intr_disable(unit);
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

// -----------------------------------------
// Init LEDC PWM Servo Wrist
// -----------------------------------------
static void siapkan_sinyal_pergelangan() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT, // 13-bit resolusi (0 s.d 8191)
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t channel_wrist = {
        .gpio_num       = PIN_SERVO_L2_WRIST,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&channel_wrist);
}

// -----------------------------------------
// Actuate output BTS7960 secara asinkron
// -----------------------------------------
static void putar_motor_pangkal_lengan(float speed) {
#if DRY_RUN_MODE
    return;
#endif
    if (speed > L2_PUTAR_MAX_PWM) speed = L2_PUTAR_MAX_PWM;
    if (speed < -L2_PUTAR_MAX_PWM) speed = -L2_PUTAR_MAX_PWM;

    if (speed > 0) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0.0f);
    } else if (speed < 0) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0f);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, -speed);
    } else {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0f);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0.0f);
    }
}

static void putar_motor_naik_turun(float speed) {
#if DRY_RUN_MODE
    return;
#endif
    if (speed > L2_Z_MAX_PWM) speed = L2_Z_MAX_PWM;
    if (speed < -L2_Z_MAX_PWM) speed = -L2_Z_MAX_PWM;

    if (speed > 0) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, speed);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0.0f);
    } else if (speed < 0) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0f);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, -speed);
    } else {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0f);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0.0f);
    }
}

static void putar_motor_maju_mundur(float speed) {
#if DRY_RUN_MODE
    return;
#endif
    if (speed > L2_Y_MAX_PWM) speed = L2_Y_MAX_PWM;
    if (speed < -L2_Y_MAX_PWM) speed = -L2_Y_MAX_PWM;

    if (speed > 0) {
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, speed);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, 0.0f);
    } else if (speed < 0) {
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0f);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, -speed);
    } else {
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0f);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, 0.0f);
    }
}

void motor_arm2_init() {
    // 1. PCNT Unit encoder Lengan 2
    motor_encoder_pcnt_init(PCNT_UNIT_0, PIN_ENC_L2_PUTAR_A, PIN_ENC_L2_PUTAR_B);
    motor_encoder_pcnt_init(PCNT_UNIT_1, PIN_ENC_L2_Z_A, PIN_ENC_L2_Z_B);
    motor_encoder_pcnt_init(PCNT_UNIT_2, PIN_ENC_L2_Y_A, PIN_ENC_L2_Y_B);

    // 2. MCPWM Unit untuk BTS7960
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_L2_PUTAR_RPWM);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PIN_L2_PUTAR_LPWM);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PIN_L2_Z_RPWM);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, PIN_L2_Z_LPWM);

    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, PIN_L2_Y_RPWM);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, PIN_L2_Y_LPWM);

    mcpwm_config_t pwm_cfg;
    pwm_cfg.frequency = 20000;
    pwm_cfg.cmpr_a = 0.0f;
    pwm_cfg.cmpr_b = 0.0f;
    pwm_cfg.counter_mode = MCPWM_UP_COUNTER;
    pwm_cfg.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_cfg);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_cfg);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_cfg);

    // 3. LEDC Servo Wrist
    siapkan_sinyal_pergelangan();

    last_pid_time = millis();
}

// -----------------------------------------
// Set Posisi Servo Wrist (0 s.d 180 derajat)
// -----------------------------------------
void servo_arm2_wrist_set_angle(uint8_t degree) {
    if (degree > 180) degree = 180;
    uint32_t duty = 205 + ((uint32_t)degree * (1024 - 205) / 180);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// -----------------------------------------
// Hentikan pergerakan Lengan 2 seketika
// -----------------------------------------
void motor_arm2_stop_emergency() {
    putar_motor_pangkal_lengan(0.0f);
    putar_motor_naik_turun(0.0f);
    putar_motor_maju_mundur(0.0f);

    integral_p = 0.0f; err_p_prev = 0.0f;
    integral_z = 0.0f; err_z_prev = 0.0f;
    integral_y = 0.0f; err_y_prev = 0.0f;
}

// -----------------------------------------
// Loop PID Posisi Lengan 2 (Core 1)
// Dipanggil setiap 10ms dari task
// -----------------------------------------
void motor_arm2_pid_loop_update() {
    uint32_t now = millis();
    float dt = (float)(now - last_pid_time) / 1000.0f;
    if (dt <= 0.0f) return;
    last_pid_time = now;

    // Failsafe Watchdog: Jika tidak ada paket serial valid dari Master, matikan motor
    if (now - last_packet_ms > FAILSAFE_TIMEOUT_MS) {
        motor_arm2_stop_emergency();
        return;
    }

    // 1. Baca data encoder ticks dari register hardware PCNT
    int16_t raw_p = 0, raw_z = 0, raw_y = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &raw_p);
    pcnt_get_counter_value(PCNT_UNIT_1, &raw_z);
    pcnt_get_counter_value(PCNT_UNIT_2, &raw_y);

    current_ticks_putar = raw_p;
    current_ticks_z     = raw_z;
    current_ticks_y     = raw_y;

    // 2. PID KONTROL KETIGA MOTOR LENGAN 2

    // Sumbu Putar (Base)
    float error_p = arm2_target_putar - current_ticks_putar;
    integral_p += error_p * dt;
    float deriv_p = (error_p - err_p_prev) / dt;
    err_p_prev = error_p;
    float pwm_putar = (error_p * L2_PUTAR_KP) + (integral_p * L2_PUTAR_KI) + (deriv_p * L2_PUTAR_KD);

    if (abs((int)error_p) < L2_PUTAR_DEADBAND) {
        pwm_putar = 0.0f;
        integral_p = 0.0f;
    }

    // Sumbu Z (Naik-Turun)
    float error_z = arm2_target_z - current_ticks_z;
    integral_z += error_z * dt;
    float deriv_z = (error_z - err_z_prev) / dt;
    err_z_prev = error_z;
    float pwm_z = (error_z * L2_Z_KP) + (integral_z * L2_Z_KI) + (deriv_z * L2_Z_KD);

    if (abs((int)error_z) < L2_Z_DEADBAND) {
        pwm_z = 0.0f;
        integral_z = 0.0f;
    }

    // Sumbu Y (Maju-Mundur)
    float error_y = arm2_target_y - current_ticks_y;
    integral_y += error_y * dt;
    float deriv_y = (error_y - err_y_prev) / dt;
    err_y_prev = error_y;
    float pwm_y = (error_y * L2_Y_KP) + (integral_y * L2_Y_KI) + (deriv_y * L2_Y_KD);

    if (abs((int)error_y) < L2_Y_DEADBAND) {
        pwm_y = 0.0f;
        integral_y = 0.0f;
    }

    // 3. Actuate motor BTS7960
    putar_motor_pangkal_lengan(pwm_putar);
    putar_motor_naik_turun(pwm_z);
    putar_motor_maju_mundur(pwm_y);

    // 4. Actuate Servo Wrist Pitch
    servo_arm2_wrist_set_angle(arm2_target_wrist);

    #if DEBUG_LENGAN2
        static uint8_t log_div = 0;
        if (++log_div >= 10) {
            log_div = 0;
            Serial.printf("[ARM2] P_Tgt:%d P_Act:%d | Z_Tgt:%d Z_Act:%d | Y_Tgt:%d Y_Act:%d\n",
                          arm2_target_putar, current_ticks_putar,
                          arm2_target_z, current_ticks_z,
                          arm2_target_y, current_ticks_y);
        }
    #endif
}
