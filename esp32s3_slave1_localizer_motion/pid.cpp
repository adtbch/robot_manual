#include <Arduino.h>
#include <math.h>

#include "robot_config.h"

// Global vector for dynamic PID storage
std::vector<PIDData> pidData;

// =============================================================
// GENERAL PURPOSE MODULAR & NON-BLOCKING PID CONTROLLER (Vector-based)
// =============================================================
// Can be used for any PID controller (motors, sensors, yaw, distance, etc.)
// Automatically sizes the state memory vector dynamically as needed.

double computePID(int index, double setpoint, double input, double Kp, double Ki, double Kd, double Minintegral, double Maxintegral) {
  // Validate index parameter
  if (index < 0) {
    return 0.0;
  }

  // Ensure the vector is dynamically sized to fit the index to prevent out-of-bounds crash
  if ((size_t)index >= pidData.size()) {
    pidData.resize(index + 1);
  }

  // Save current gains inside the struct for debugging or reference
  pidData[index].Kp = Kp;
  pidData[index].Ki = Ki;
  pidData[index].Kd = Kd;

  // Calculate and store error
  pidData[index].error = setpoint - input;

  // Special angle-wrapping logic for Heading/Yaw PID (Index 4 and 11)
  // Ensures error wraps between -180 and 180 degrees to prevent taking the "long way around"
  if (index == 4 || index == 11) {
    if (pidData[index].error > 180.0) {
      pidData[index].error -= 360.0;
    } else if (pidData[index].error < -180.0) {
      pidData[index].error += 360.0;
    }
  }

  // Calculate dynamic dt (sampling time in seconds) to support non-blocking / variable rate execution
  unsigned long currentTime = millis();
  double dt = 0.0;
  if (pidData[index].lastTime > 0) {
    dt = (currentTime - pidData[index].lastTime) / 1000.0;
  }
  // Clamp dt: jika lastTime == 0 (baru reset) atau dt tidak wajar, pakai 40ms default
  if (pidData[index].lastTime == 0 || dt <= 0.0 || dt > 1.0) {
    dt = 0.04; // 40ms — sesuai kPidTickMs autoTuner
  }
  pidData[index].lastTime = currentTime;

  // DYNAMIC SETPOINT WEIGHTING (2-DOF PID) - Mencegah lonjakan (Spike) awal
  // Memberikan bobot Kp yang lebih lembut saat selisih (error) masih besar/jauh dari target.
  double errorRatio = fabs(pidData[index].error) / fmax(fabs(setpoint), 1.0);
  double dynamicWeight = 0.8; // Default weight untuk jarak jauh (menghaluskan hentakan)

  if (errorRatio < 0.5) {
    // Transisi membesar kembali ke 1.0 (kekuatan penuh) saat sudah dekat target
    double transitionRatio = errorRatio / 0.5;
    dynamicWeight = 1.0 - (0.2 * transitionRatio);
  }

  // Hitung Term P menggunakan Setpoint Weighting
  double errorP = (dynamicWeight * setpoint) - input;
  double pTerm = Kp * errorP;

  // Accumulate integral — WAJIB dikali dt agar konsisten terhadap tick rate
  // Tanpa dt, integral tumbuh proporsional jumlah tick, bukan waktu → perilaku berbeda saat rate berubah
  pidData[index].integral += pidData[index].error * dt;
  pidData[index].integral = constrain(pidData[index].integral, Minintegral, Maxintegral);

  // Calculate derivative (discrete difference) — Derivative on Measurement
  // (mencegah Derivative Kick saat target berubah mendadak)
  // FIX: guard pakai flag firstRun, bukan cek != 0.0
  // Cek != 0.0 akan skip derivative saat motor benar-benar diam (input = 0 RPM)
  double dTerm = 0.0;
  if (pidData[index].lastTime > 0 && pidData[index].previousInput != -99999.0) {
    // Dibagi dt agar magnitude derivative tidak tergantung tick rate
    dTerm = Kd * -(input - pidData[index].previousInput) / dt;
  }
  pidData[index].previousInput = input;

  // Compute final PID output
  double output = pTerm + (Ki * pidData[index].integral) + dTerm;

  // Save current error as previousError for the next computation cycle
  pidData[index].previousError = pidData[index].error;

  return output;
}
