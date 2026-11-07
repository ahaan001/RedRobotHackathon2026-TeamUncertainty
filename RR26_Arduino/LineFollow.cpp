/**
 * @file  LineFollow.cpp
 * @brief QTR-8RC reading, per-channel normalization, weighted line position,
 *        and proportional steering.
 *
 * RedRobot::ReadLineSensor returns RC decay time in microseconds per channel.
 * More reflection (white) discharges faster and reads lower; black tape reads
 * higher; a channel that sees nothing runs to the 10 ms timeout (~10000).
 * LINE_DARK_IS_HIGH in Config.h flips the math if the bench test disagrees.
 */
#include <Arduino.h>
#include "RedRobot.h"
#include "Robot.h"

namespace {
  uint16_t s_white[6];
  uint16_t s_black[6];
  LineFollow::Reading s_last;
  int s_lastSteerSign = 1;

  int normalize(int i, uint16_t raw) {
    if (raw >= LINE_SATURATED_US) return -1;             // sees nothing
    int span = (int)s_black[i] - (int)s_white[i];
    if (span < 50) span = 50;                             // uncalibrated guard
    int n = ((int)raw - (int)s_white[i]) * 1000 / span;   // white -> 0, black -> 1000
    if (!LINE_DARK_IS_HIGH) n = 1000 - n;
    return clampInt(n, 0, 1000);
  }
}

namespace LineFollow {

  void begin() {
    setCalibration(LINE_WHITE_DEFAULT, LINE_BLACK_DEFAULT);
    memset(&s_last, 0, sizeof(s_last));
  }

  void setCalibration(const uint16_t white[6], const uint16_t black[6]) {
    for (int i = 0; i < 6; i++) { s_white[i] = white[i]; s_black[i] = black[i]; }
  }

  void printCalibration() {
    Serial.print("[LF] white:");
    for (int i = 0; i < 6; i++) Serial.printf(" %5u", s_white[i]);
    Serial.print("  black:");
    for (int i = 0; i < 6; i++) Serial.printf(" %5u", s_black[i]);
    Serial.println();
  }

  const Reading& read() {
    Reading& r = s_last;
    RedRobot::ReadLineSensor(r.raw);

    r.sum = 0;
    r.saturated = 0;
    int darkCount = 0;
    long weighted = 0;
    for (int i = 0; i < 6; i++) {
      int idx = LINE_REVERSED ? (5 - i) : i;
      int n = normalize(idx, r.raw[idx]);
      if (n < 0) { r.saturated++; n = 0; }
      r.norm[i] = n;
      r.sum += n;
      weighted += (long)n * i * 1000;
      if (n > LINE_END_MARKER_LEVEL) darkCount++;
    }

    bool offFloor = r.saturated > LINE_SATURATED_MAX;
    r.onLine    = !offFloor && r.sum > LINE_ON_THRESHOLD;
    r.endMarker = !offFloor && darkCount >= LINE_END_MARKER_COUNT;
    r.position  = r.onLine ? (int)(weighted / r.sum) : 2500;
    return r;
  }

  const Reading& last() { return s_last; }

  // Slide the robot across the tape by hand while this runs. Tracks min and max
  // per channel, ignoring saturated samples, then stores them as white and black.
  void calibrate(uint32_t ms) {
    uint16_t lo[6], hi[6];
    for (int i = 0; i < 6; i++) { lo[i] = 65535; hi[i] = 0; }
    uint32_t t0 = millis();
    int samples = 0;
    while (millis() - t0 < ms) {
      uint16_t v[6];
      RedRobot::ReadLineSensor(v);
      for (int i = 0; i < 6; i++) {
        if (v[i] >= LINE_SATURATED_US) continue;
        if (v[i] < lo[i]) lo[i] = v[i];
        if (v[i] > hi[i]) hi[i] = v[i];
      }
      samples++;
    }
    for (int i = 0; i < 6; i++) {
      if (lo[i] == 65535 || hi[i] == 0 || hi[i] - lo[i] < 50) continue;  // channel never saw both
      s_white[i] = lo[i];
      s_black[i] = hi[i];
    }
    Serial.printf("[LF] calibrated from %d samples\n", samples);
    printCalibration();
  }

  void steer(const Reading& r, int base, int kp, int& left, int& right) {
    // error in -1000..1000 (position 0..5000 about 2500), steer in percent
    int error = r.position - 2500;
    int st = (error * kp) / 2500;
    s_lastSteerSign = (st >= 0) ? 1 : -1;
    left  = clampInt(base + st, -100, 100);
    right = clampInt(base - st, -100, 100);
  }

  int lastSteerSign() { return s_lastSteerSign; }
}
