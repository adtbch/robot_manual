// Common PID definitions for the project
#pragma once

#include <Arduino.h>
#include <vector>

// Legacy PID structure for backward compatibility
typedef struct {
    double kp;
    double ki;
    double kd;
    double integral;
    double prev_error;
} PID;

void pid_init(PID* p, float kp, float ki, float kd);
double pid_compute(PID* p, double setpoint, double measurement,
                   double minIntegral, double maxIntegral);
void pid_reset(PID* p);

// Enhanced PID Data structure using dynamic std::vector
struct PIDData {
    double error = 0.0;
    double integral = 0.0;
    double derivative = 0.0;
    double previousError = 0.0;
    double previousInput = 0.0;
    double filteredDerivative = 0.0;
    unsigned long lastTime = 0;
    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;
};

extern std::vector<PIDData> pidData;

// Enhanced non-blocking PID compute function
double computePID(int index, double setpoint, double input, double Kp, double Ki, double Kd, double Minintegral, double Maxintegral);
