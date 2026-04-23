#pragma once
#include <Arduino.h>

static constexpr uint8_t RADIO_CHANNEL = 76;
static constexpr uint8_t RADIO_ADDRESS[6] = "CTRL1";

enum ControlMode : uint8_t {
  MODE_MANUAL = 0,
  MODE_AUTO   = 1
};

enum ButtonMask : uint8_t {
  BTN_MASK_LEFT_FWD  = 1 << 0,
  BTN_MASK_LEFT_REV  = 1 << 1,
  BTN_MASK_RIGHT_FWD = 1 << 2,
  BTN_MASK_RIGHT_REV = 1 << 3,
  BTN_MASK_A1        = 1 << 4,
  BTN_MASK_A2        = 1 << 5
};

struct __attribute__((packed)) Packet {
  uint8_t mode;
  uint8_t buttons_bitmask;
};

static inline bool packetEquals(const Packet &a, const Packet &b) {
  return a.mode == b.mode && a.buttons_bitmask == b.buttons_bitmask;
}

static inline bool anyDriveButtons(uint8_t mask) {
  return (mask & (BTN_MASK_LEFT_FWD | BTN_MASK_LEFT_REV |
                  BTN_MASK_RIGHT_FWD | BTN_MASK_RIGHT_REV)) != 0;
}