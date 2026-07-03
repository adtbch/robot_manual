Yang Diubah
Fix #1 — serial.ino slave1 (baris 95–100)
Hapus guard RUNNING:
// SEBELUM
if (getWaypointState() != WaypointState::RUNNING) {
    testYawMode = false;
    startWaypoint(x_cm, y_cm, yaw, speed);
}

// SESUDAH
// Update target selalu — waypointTick follow target terbaru
testYawMode = false;
startWaypoint(x_cm, y_cm, yaw, speed);
Efek: tiap goto dari master langsung update target, tidak dibuang.
Fix #2a — waypoint.ino startWaypoint() (baris 124–141)
applyWaypointTarget dipindah ke atas — target diupdate dulu, baru cek toleransi:
// SEBELUM: cek tol → apply target → set RUNNING
// SESUDAH: apply target → cek tol → REACHED/RUNNING
applyWaypointTarget(x_m, y_m, yaw_deg);  // ← selalu update
if (isWithinWaypointTol(...)) { ... REACHED ... return; }
wpState = WaypointState::RUNNING;         // ← bahkan jika sebelumnya RUNNING
Fix #2b — waypoint.ino waypointTick() (baris 172–173)
Hapus Jeda 40ms internal:
// DIHAPUS:
// static Jeda jeda;
// if (!jeda.check(40)) return;  // 25 Hz
Efek: waypointTick tidak punya throttle sendiri. Rate efektif dikontrol oleh rpmMotor (throttle 40ms sekali, sudah cukup). Latency GOTO turun dari worst-case 80ms → max 40ms.