#ifndef GAMEPAD_H
#define GAMEPAD_H

#include <Arduino.h>

struct GamepadState {
  bool square, cross, circle, triangle;
  bool l1, r1, l2, r2;
  bool share, options, touchpad, l3, r3;
  bool up, down, left, right;
  int8_t lx, ly, rx, ry;
  uint8_t l2a, r2a;
  bool connected;
};

extern GamepadState gp;

void parsePacket(uint8_t *d, int len) {
  if (len < 10 || d[0] != 0x01) return;

  gp.lx = d[1] - 128;
  gp.ly = d[2] - 128;
  gp.rx = d[3] - 128;
  gp.ry = d[4] - 128;

  uint8_t hat_raw = d[5] & 0x0F;
  gp.up    = (hat_raw == 0 || hat_raw == 1 || hat_raw == 7);
  gp.right = (hat_raw == 1 || hat_raw == 2 || hat_raw == 3);
  gp.down  = (hat_raw == 3 || hat_raw == 4 || hat_raw == 5);
  gp.left  = (hat_raw == 5 || hat_raw == 6 || hat_raw == 7);

  gp.square   = d[5] & 0x10;
  gp.cross    = d[5] & 0x20;
  gp.circle   = d[5] & 0x40;
  gp.triangle = d[5] & 0x80;

  gp.l1      = d[6] & 0x01;
  gp.r1      = d[6] & 0x02;
  gp.l2      = d[6] & 0x04;
  gp.r2      = d[6] & 0x08;
  gp.share   = d[6] & 0x10;
  gp.options = d[6] & 0x20;
  gp.l3      = d[6] & 0x40;
  gp.r3      = d[6] & 0x80;

  gp.touchpad = d[7] & 0x02;

  gp.l2a = d[8];
  gp.r2a = d[9];
}

#endif
