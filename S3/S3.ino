#include "usb_host.h"

GamepadState gp;
GamepadHost usb;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-S3 USB Gamepad PS4");
  usb.begin();
}

void loop() {
  usb.task();
  if (gp.connected) {
    Serial.printf("U:%d D:%d L:%d R:%d SQ:%d X:%d CI:%d TR:%d L1:%d R1:%d L3:%d R3:%d SH:%d OP:%d TP:%d\n",
      gp.up, gp.down, gp.left, gp.right,
      gp.square, gp.cross, gp.circle, gp.triangle,
      gp.l1, gp.r1, gp.l3, gp.r3, gp.share, gp.options, gp.touchpad);
    Serial.printf("LX:%4d LY:%4d RX:%4d RY:%4d L2:%3d R2:%3d\n",
      gp.lx, gp.ly, gp.rx, gp.ry, gp.l2a, gp.r2a);
    delay(60);
  }
}
