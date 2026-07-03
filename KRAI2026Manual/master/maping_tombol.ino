/*
 * =====================================================================
 * FILE    : mamping_tombol.ino
 * PERAN   : Dokumentasi mapping tombol PS4 → aksi robot (Master).
 *           Hanya comment & dokumentasi. Tidak ada kode eksekusi.
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 *
 * MODE INPUT
 * ───────────
 *   SHARE (tekan)      : Toggle MODE_DPAD ↔ MODE_ANALOG
 *                         (DPAD = tombol panah, ANALOG = stick kiri)
 *   L1+R1+L2+R2 (hold) : Toggle gModeInvert (balik arah lx/ly)
 *
 * SPEED MODIFIER (berlaku untuk semua sub-modul)
 * ──────────────
 *   R1 (hold) : FAST mode  — RPM=150, yaw step 25ms, gripper step=40
 *   L1 (hold) : SLOW mode  — RPM=25,  yaw step 150ms, gripper step=5
 *   none      : NORMAL     — RPM=75,  yaw step 50ms, gripper step=15
 *
 * =====================================================================
 *  1. MOTION CONTROL — movement (motion_control.ino)
 * =====================================================================
 *   KN-only (velocity langsung, no GOTO/waypoint)
 *
 *   DPAD/Stick kiri    : Mecanum field-centric
 *     ▲ / ly positif   : maju
 *     ▼ / ly negatif   : mundur
 *     ◄ / lx negatif   : geser kiri
 *     ► / lx positif   : geser kanan
 *
 *   Stick kanan (rx)   : Yaw target ±1° per step
 *     rx positif        : yaw +1° (putar kanan)
 *     rx negatif        : yaw -1° (putar kiri)
 *
 *   L2 + stick kanan   : Snap yaw kardinal
 *     rx positif        : yaw = +90°
 *     rx negatif        : yaw = -90°
 *     ry positif        : yaw = 0°
 *     ry negatif        : yaw = 180°
 *
 *   Square (hold)      : Force vx=0, vy=0 (robot diam di tempat)
 *   R2                 : Block yaw update (biarkan yaw manual hold)
 *
 * =====================================================================
 *  2. GRIPPER CONTROL — arm capit (gripper_control.ino)
 * =====================================================================
 *   OPTIONS (tekan)    : Setup Zone 1 (homing + posisi awal)
 *   Cross (tekan)      : Toggle servo B (capit buka/tutup)
 *
 *   R2 (hold) + stick  : Manual jog motor X/Y
 *     Stik kiri ly     : Motor Y (naik/turun) — step encoder
 *       ▲ / ly +       : naik (+step)
 *       ▼ / ly -       : turun (-step)
 *     Stik kiri lx     : Motor X (maju/mundur) — step encoder
 *       ► / lx +       : maju (+step)
 *       ◄ / lx -       : mundur (-step), cek limit X mundur
 *
 *   Segitiga + R2 hold + stik : Servo B manual (saat READY_TO_STAB)
 *     ▲ / ly +         : servo T naik (+5/10/20°)
 *     ▼ / ly -         : servo T turun (-5/10/20°)
 *
 *   R2 + stick kanan ry : Motor Y jog di slave2arm (armbox)
 *     ry < 0 (atas)    : naik (+step)
 *     ry > 0 (bawah)   : turun (-step)
 *     Proteksi limit switch slave2LimitTurun()
 *
 *   R2 + L2 trigger max : Flash lamp (l2Value≥250 && r2Value≥250)
 *
 * =====================================================================
 *  3. ARM BOX CONTROL — kotak senjata (armBox_control.ino)
 * =====================================================================
 *   Circle (hold) + stik kiri :
 *     ► / lx positif   : armBoxFBToggle('r') — toggle motor k depan/belakang
 *     ◄ / lx negatif   : armBoxFBToggle('l') — toggle motor x depan/belakang
 *     ▲ / ly positif   : armBoxDone('l') — kotak kiri selesai
 *     ▼ / ly negatif   : armBoxDone('r') — kotak kanan selesai
 *
 * =====================================================================
 *  4. FOREST CONTROL — navigasi forest (forest_control.ino)
 * =====================================================================
 *   Square (hold) + DPAD :
 *     ▲ (tekan)        : forestGotoSlot(1) + motor Y level 4
 *     ◄ (tekan)        : forestGotoSlot(2)
 *     ▼ (tekan)        : forestTriggerExit() + zoneState=2
 *
 * =====================================================================
 *  5. ODOM RECORD — record waypoint (odom.ino)
 * =====================================================================
 *   TOUCHPAD + SHARE (hold)  : Enter record mode
 *   L3 + R3 (tekan)          : Exit record mode
 *
 *   Dalam record mode (TOUCHPAD+SHARE dihold):
 *     R1 + TOUCHPAD   : Record ZONE1_0
 *     L1 + TOUCHPAD   : Record ZONE1_1
 *     R2 + TOUCHPAD   : Record ZONE1_2
 *     L2 + TOUCHPAD   : Record ZONE1_3
 *     R1 + TRIANGLE   : Record APPROACH_0
 *     L1 + SQUARE     : Record APPROACH_1
 *     R2 + TRIANGLE   : Record APPROACH_2
 *     L2 + SQUARE     : Record APPROACH_3
 *     Cross + TOUCHPAD : Record FOREST_2
 *     Square + TOUCHPAD: Record FOREST_6
 *     Circle + TOUCHPAD: Record FOREST_7
 *     Triangle + TOUCHPAD: Record FOREST_11
 *
 * =====================================================================
 *  6. AUTO GRIPPER — state machine (otomat.ino / gripper.ino)
 * =====================================================================
 *   Proximity (detect) : Auto state IDLE → CLOSING → UP → STRAIGHTEN → READY_TO_STAB
 *
 *   OPTIONS (tombol)   : setupZone1() = servo homing + motor Y level 0 + motor X enc 200
 *   Cross (tombol)     : Toggle servo B (capit) 0°/90°
 *
 * =====================================================================
 *  7. AUTO ARM BOX — state machine (otomat.ino)
 * =====================================================================
 *   slave2ProxR() detect : Auto R → pneumatic on → motor Y level 4 → arm maju
 *   slave2ProxL() detect : Auto L → pneumatic on → motor Y level 4 → arm maju
 *   armBoxDone('r')       : motor Y level 5 → pneumatic off → kembali
 *   armBoxDone('l')       : motor Y level 5 → pneumatic off → kembali
 *
 * =====================================================================
 *  8. HOMING — blocking setup (gripper.ino)
 * =====================================================================
 *   Serial command "setHomingAll" :
 *     Motor Y (pwm 400) → sampai limit Y bawah → reset enc → level 0
 *     Motor X (pwm 400) → sampai limit X mundur → reset enc
 *     Servo T = 70°, Servo B = 0°, Servo D = 0°
 *
 * =====================================================================
 *  9. SETUP ZONE 1 (otomat.ino / gripper_control.ino)
 * =====================================================================
 *   OPTIONS + zoneState==1  : Servo homing + odomGoto(1) + motor Y level 0 + motor X target 200
 *   OPTIONS + zoneState==2  : Toggle modeKinematics (tidak dipakai — KN-only sekarang)
 *
 * =====================================================================
 *  10. BOOT BUTTON — hardware (waypoint.ino)
 * =====================================================================
 *   BOOT (GPIO0, LOW) : Toggle alliance color RED ↔ BLUE
 *
 * =====================================================================
 *  11. GRIPPER ZONE 1 — auto state (otomat.ino)
 * =====================================================================
 *   Proximity detect (D) → IDLE:
 *     Servo D = 90° (tutup capit)
 *     Delay 300ms
 *     Servo T = 90° (lengan naik)
 *     Motor Y → level 1
 *     Tunggu motor Y sampai level 1
 *     Motor X → enc 0
 *     odomGoto(2)
 *     Servo T = 0° (lengan siap stab)
 *     odomGoto(3)
 *     State = READY_TO_STAB
 */
