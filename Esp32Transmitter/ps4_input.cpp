#include "ps4_input.h"
#include "transmitter_state.h"

#include <string.h>
#include <Arduino.h>
#include <PS4Controller.h>

namespace {
// ─── Mahony filter state ───────────────────────────────────────────────────
// Quaternion orientasi: q = q0 + q1·i + q2·j + q3·k  (unit quaternion).
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
// Akumulasi error integral untuk estimasi bias gyro.
float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
uint32_t lastImuMs = 0;
bool mahonyInitialized = false;

// ─── Kalibrasi bias gyro ──────────────────────────────────────────────────
// Rata-rata gyro saat diam untuk eliminasi DC offset (penyebab utama yaw drift).
constexpr uint16_t kCalibSamples = 64;   // jumlah sampel kalibrasi
float gyrBiasX = 0.0f, gyrBiasY = 0.0f, gyrBiasZ = 0.0f;
float gyrSumX = 0.0f, gyrSumY = 0.0f, gyrSumZ = 0.0f;
uint16_t calibCount = 0;
bool calibDone = false;
// ──────────────────────────────────────────────────────────────────────────

// Mahony complementary filter: fusi gyro + accelerometer.
// gx/gy/gz dalam rad/s, ax/ay/az dalam unit apa pun — akan dinormalisasi.
void updateMahonyFilter(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float dt) {
	// 1. Normalisasi accelerometer (abaikan jika magnitude dekat 0).
	const float accNorm = sqrtf(ax * ax + ay * ay + az * az);
	if (accNorm < 0.01f) {
		// Sensor dalam free-fall atau noise — skip koreksi accel.
		goto integrate_only;
	}
	{
		const float invNorm = 1.0f / accNorm;
		ax *= invNorm;
		ay *= invNorm;
		az *= invNorm;

		// 2. Estimasi arah gravitasi dari quaternion saat ini.
		//    vx/vy/vz = baris ketiga rotation matrix (kolom gravitasi world [0,0,1] dirotasi ke frame sensor).
		const float vx = 2.0f * (q1 * q3 - q0 * q2);
		const float vy = 2.0f * (q0 * q1 + q2 * q3);
		const float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

		// 3. Error = cross product(accel_terukur, estimasi_gravitasi).
		//    Error menunjukkan seberapa jauh estimasi kita dari referensi accel.
		const float ex = (ay * vz - az * vy);
		const float ey = (az * vx - ax * vz);
		const float ez = (ax * vy - ay * vx);

		// 4. Integral term: estimasi bias gyro (Ki).
		exInt += ex * kMahonyKi * dt;
		eyInt += ey * kMahonyKi * dt;
		ezInt += ez * kMahonyKi * dt;

		// 5. Koreksi laju rotasi gyro dengan Kp * error + integral bias.
		gx += kMahonyKp * ex + exInt;
		gy += kMahonyKp * ey + eyInt;
		gz += kMahonyKp * ez + ezInt;
	}

integrate_only:
	// 6. Integrasi quaternion: q_dot = 0.5 * q ⊗ omega.
	const float halfDt = 0.5f * dt;
	q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfDt;
	q1 += ( q0 * gx + q2 * gz - q3 * gy) * halfDt;
	q2 += ( q0 * gy - q1 * gz + q3 * gx) * halfDt;
	q3 += ( q0 * gz + q1 * gy - q2 * gx) * halfDt;

	// 7. Normalisasi quaternion.
	const float qNorm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	if (qNorm > 0.001f) {
		const float invQNorm = 1.0f / qNorm;
		q0 *= invQNorm;
		q1 *= invQNorm;
		q2 *= invQNorm;
		q3 *= invQNorm;
	}
}

// Konversi quaternion → Euler angles (ZYX convention, dalam derajat).
// roll = rotasi di sumbu X (depan/belakang miring).
// pitch = rotasi di sumbu Y (kiri/kanan miring).
// yaw = rotasi di sumbu Z (putar horizontal).
void quaternionToEuler(float &rollDeg, float &pitchDeg, float &yawDeg) {
	// Roll (phi): atan2(2(q0*q1 + q2*q3), 1 - 2(q1² + q2²))
	rollDeg  = atan2f(2.0f * (q0*q1 + q2*q3),
	                  1.0f - 2.0f * (q1*q1 + q2*q2)) * (180.0f / M_PI);
	// Pitch (theta): asin(2(q0*q2 - q3*q1))  — clamp untuk stabilitas numerik.
	const float sinP = 2.0f * (q0*q2 - q3*q1);
	pitchDeg = asinf(constrain(sinP, -1.0f, 1.0f)) * (180.0f / M_PI);
	// Yaw (psi): atan2(2(q0*q3 + q1*q2), 1 - 2(q2² + q3²))
	yawDeg   = atan2f(2.0f * (q0*q3 + q1*q2),
	                  1.0f - 2.0f * (q2*q2 + q3*q3)) * (180.0f / M_PI);
}
}  // namespace

void onPs4Connected() {
	callbackConnected = true;
}

void onPs4Disconnected() {
	callbackConnected = false;
}

// Reset Mahony filter ke kondisi awal dan mulai kalibrasi gyro.
// Panggil saat PS4 baru terkoneksi atau tekan tombol Options.
void resetYaw() {
	q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
	exInt = 0.0f; eyInt = 0.0f; ezInt = 0.0f;
	mahonyInitialized = false;
	// Reset kalibrasi gyro.
	gyrBiasX = 0.0f; gyrBiasY = 0.0f; gyrBiasZ = 0.0f;
	gyrSumX = 0.0f; gyrSumY = 0.0f; gyrSumZ = 0.0f;
	calibCount = 0;
	calibDone = false;
}

uint32_t buildButtonsMask() {
	uint32_t mask = 0;
	if (PS4.Up()) mask |= kBtnUp;
	if (PS4.Right()) mask |= kBtnRight;
	if (PS4.Down()) mask |= kBtnDown;
	if (PS4.Left()) mask |= kBtnLeft;
	if (PS4.Square()) mask |= kBtnSquare;
	if (PS4.Cross()) mask |= kBtnCross;
	if (PS4.Circle()) mask |= kBtnCircle;
	if (PS4.Triangle()) mask |= kBtnTriangle;
	if (PS4.UpRight()) mask |= kBtnUpRight;
	if (PS4.DownRight()) mask |= kBtnDownRight;
	if (PS4.UpLeft()) mask |= kBtnUpLeft;
	if (PS4.DownLeft()) mask |= kBtnDownLeft;
	if (PS4.L1()) mask |= kBtnL1;
	if (PS4.R1()) mask |= kBtnR1;
	if (PS4.L2()) mask |= kBtnL2;
	if (PS4.R2()) mask |= kBtnR2;
	if (PS4.Share()) mask |= kBtnShare;
	if (PS4.Options()) mask |= kBtnOptions;
	if (PS4.L3()) mask |= kBtnL3;
	if (PS4.R3()) mask |= kBtnR3;
	if (PS4.PSButton()) mask |= kBtnPs;
	if (PS4.Touchpad()) mask |= kBtnTouchpad;
	return mask;
}

int16_t applyDeadband(const int raw) {
	if (abs(raw) < kStickDeadband) {
		return 0;
	}
	return raw;
}

int16_t mapStickToPwm(const int raw) {
	const int clean = applyDeadband(raw);
	if (clean == 0) {
		return 0;
	}
	return static_cast<int16_t>(map(clean, -128, 127, -kPwmMax, kPwmMax));
}

int batteryPercent(const uint8_t rawBattery) {
	int pct = static_cast<int>(rawBattery) * 100 / 11;
	return constrain(pct, 0, 100);
}

void updateBatteryLed(const bool force) {
	if (!kEnablePs4ControllerOutput) {
		return;
	}

	if (!PS4.isConnected()) {
		return;
	}

	const bool lowBattery = batteryPercent(PS4.Battery()) <= kBatteryLowPercent;
	if (!force && ledStateInitialized && lowBattery == lastBatteryLow) {
		return;
	}

	lastBatteryLow = lowBattery;
	ledStateInitialized = true;

	if (lowBattery) {
		PS4.setLed(32, 0, 0);
	} else {
		PS4.setLed(0, 32, 0);
	}
	PS4.setRumble(0, 0);
	PS4.sendToController();
}

void buildPacketFromPs4(ControlPacket &packet, const bool connected) {
	memset(&packet, 0, sizeof(packet));
	packet.magic = kPacketMagic;
	packet.connected = connected ? 1 : 0;

	if (!connected) {
		return;
	}

	packet.x = mapStickToPwm(-PS4.LStickY());
	packet.y = mapStickToPwm(PS4.LStickX());
	packet.w = mapStickToPwm(PS4.RStickX());
	packet.lx = PS4.LStickX();
	packet.ly = PS4.LStickY();
	packet.rx = PS4.RStickX();
	packet.ry = PS4.RStickY();
	packet.l2Value = PS4.L2Value();
	packet.r2Value = PS4.R2Value();

	if (kUseProcessedYaw) {
		// ─── Mahony 6-DOF IMU fusion ─────────────────────────────────────────
		const uint32_t nowMs = millis();
		if (!mahonyInitialized) {
			mahonyInitialized = true;
			lastImuMs = nowMs;
		} else {
			// Limit to maximum ~250Hz update rate (jeda 4ms).
			// Ini mencegah ESP32 membaca paket yang sama berulang kali dan merusak kalibrasi.
			if (nowMs - lastImuMs < 4) {
				goto skip_filter;
			}
			// dt dalam detik (clamp agar tidak blow up saat ada jeda besar).
			const float dt = constrain(
				static_cast<float>(nowMs - lastImuMs) * 0.001f, 0.001f, 0.1f);
			lastImuMs = nowMs;

			// Baca gyro raw (sebelum remap).
			const float rawGx = static_cast<float>(PS4.GyrX());
			const float rawGy = static_cast<float>(PS4.GyrY());
			const float rawGz = static_cast<float>(PS4.GyrZ());

			// Fase kalibrasi: kumpulkan sampel gyro saat diam untuk cari rata-rata bias.
			if (!calibDone) {
				gyrSumX += rawGx;
				gyrSumY += rawGy;
				gyrSumZ += rawGz;
				calibCount++;
				// 64 sampel @ 4ms/sampel = butuh ~0.25 detik kalibrasi
				if (calibCount >= kCalibSamples) {
					gyrBiasX = gyrSumX / static_cast<float>(kCalibSamples);
					gyrBiasY = gyrSumY / static_cast<float>(kCalibSamples);
					gyrBiasZ = gyrSumZ / static_cast<float>(kCalibSamples);
					calibDone = true;
					Serial.printf("\n[IMU] Kalibrasi selesai! Gyro bias: x=%.1f y=%.1f z=%.1f\n\n", 
						gyrBiasX, gyrBiasY, gyrBiasZ);
				}
				// Selama kalibrasi, skip filter update.
				goto skip_filter;
			}

			{
				// Kurangi bias hasil kalibrasi.
				float dgX = rawGx - gyrBiasX;
				float dgY = rawGy - gyrBiasY;
				float dgZ = rawGz - gyrBiasZ;

				// DEADBAND: Jika getaran sangat kecil (noise), anggap 0 mutlak agar tidak ada drift saat diam.
				const float deadband = 8.0f;
				if (abs(dgX) < deadband) dgX = 0;
				if (abs(dgY) < deadband) dgY = 0;
				if (abs(dgZ) < deadband) dgZ = 0;

				// Remap sumbu PS4 → Mahony: PS4-Y (gravity) → Mahony-Z.
				const float gyrScale = kPs4GyrScale * (M_PI / 180.0f);
				const float gx =  dgX * gyrScale;
				const float gy = -dgZ * gyrScale;
				const float gz =  dgY * gyrScale;

				// Konversi accel raw → g, dengan remap sumbu yang sama.
				const float ax =  static_cast<float>(PS4.AccX()) * kPs4AccScale;
				const float ay = -static_cast<float>(PS4.AccZ()) * kPs4AccScale;
				const float az =  static_cast<float>(PS4.AccY()) * kPs4AccScale;

				updateMahonyFilter(gx, gy, gz, ax, ay, az, dt);
			}
		}

	skip_filter:
		// Reset yaw saat tombol Options ditekan.
		if (PS4.Options()) {
			resetYaw();
		}

		// Konversi quaternion → Euler angles.
		float rollDeg, pitchDeg, yawDeg;
		quaternionToEuler(rollDeg, pitchDeg, yawDeg);

		// Simpan dalam int16_t dengan resolusi 0,1° (rentang ±3276.7°).
		// gyrX = roll, gyrY = pitch, gyrZ = yaw.
		packet.gyrX = static_cast<int16_t>(rollDeg  * 10.0f);
		packet.gyrY = static_cast<int16_t>(pitchDeg * 10.0f);
		packet.gyrZ = static_cast<int16_t>(yawDeg   * 10.0f);
	} else {
		// Kirim data raw gyroscope PS4.
		packet.gyrX = PS4.GyrX();
		packet.gyrY = PS4.GyrY();
		packet.gyrZ = PS4.GyrZ();
	}

	packet.buttons = buildButtonsMask();
}

bool packetChanged(const ControlPacket &a, const ControlPacket &b) {
	return a.connected != b.connected ||
		a.x != b.x || a.y != b.y || a.w != b.w ||
		a.l2Value != b.l2Value || a.r2Value != b.r2Value ||
		a.buttons != b.buttons ||
		a.gyrX != b.gyrX || a.gyrY != b.gyrY || a.gyrZ != b.gyrZ;
}
