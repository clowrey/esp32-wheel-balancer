#pragma once
// Drives the touchscreen UI on a PC with the demo wheel: the same ModelCore and BalancerUi code
// as the firmware, a frame in memory, and a finger that taps by label. Used by the simulator
// (tools/ui_sim/sim.cpp) and the host tests (tests/test_ui.cpp).

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "ui.h"
#include "ui_core.h"

namespace bal {

struct Driver {
  ModelCore m;
  BalancerUi ui;
  std::vector<uint16_t> pixels;
  Canvas c;
  HitTrace trace;
  uint32_t now = 1;
  int frames_drawn = 0;

  Driver() : pixels(static_cast<size_t>(SCREEN_W) * SCREEN_H), c(pixels.data(), SCREEN_W, SCREEN_H) {
    m.set_simulate(true);
    ui.bind(&m);
    this->frame();
  }

  // One frame with the finger up (or down at x, y)
  void frame(bool down = false, int x = 0, int y = 0) {
    m.tick(now);
    Input in;
    in.down = down;
    in.x = x;
    in.y = y;
    if (ui.frame(c, in, now, &trace)) frames_drawn++;
  }

  // Lets time pass: the demo wheel runs at 50 steps a second, the UI draws 5 frames a second
  void advance(double seconds) {
    const int steps = static_cast<int>(seconds * 50);
    for (int i = 0; i < steps; i++) {
      now += 20;
      m.tick(now);
      if (i % 10 == 9) this->frame();
    }
    this->frame();
  }

  // Is something with this label on the screen now, and can it be tapped?
  bool has(const char *label, int nth = 0) const { return trace.find(label, nth) != nullptr; }

  // A tap in the middle of what was drawn with this label: the finger comes down, then goes up
  bool tap(const char *label, int nth = 0) {
    const Hit *h = trace.find(label, nth);
    if (h == nullptr) {
      std::printf("tap: no \"%s\" on the screen\n", label);
      return false;
    }
    const int x = h->x + h->w / 2, y = h->y + h->h / 2;
    now += 30;
    this->frame(true, x, y);
    now += 30;
    this->frame(false, x, y);
    return true;
  }

  // Types on the number pad
  void type(const char *keys) {
    for (const char *k = keys; *k; k++) {
      const char key[2] = {*k, 0};
      this->tap(key);
    }
  }

  // A measurement run the way an operator does it with the demo wheel: spin it, let it coast
  // through the test speed, tap LOCK SPEED near `lock_rpm` if it hasn't locked by itself.
  void run(double lock_rpm = 245, std::vector<uint16_t> *mid_shot = nullptr) {
    for (int pass = 0; pass < 30 && !m.run_stats().ready; pass++) {
      m.sim_spin();
      advance(0.3);
      const double T = m.job().test_rpm() > 0 ? m.job().test_rpm() : lock_rpm;
      while (m.live().rpm == 0 || m.live().rpm > T * 0.95) {
        advance(0.1);
        if (m.job().test_rpm() <= 0 && m.live().rpm > 0 && m.live().rpm <= lock_rpm && has("LOCK SPEED")) tap("LOCK SPEED");
        if (mid_shot != nullptr && m.run_stats().revs >= 12) {
          *mid_shot = pixels;
          mid_shot = nullptr;
        }
      }
    }
    advance(0.3);
  }
};

// The demo wheel has no sensors: readings for the screens that show raw data. Each sensor
// sees gravity on one axis and a wobble at the wheel's speed plus a little noise.
inline void fake_sensor_readings(LiveData &lv, double rpm = 300) {
  const double hz = rpm / 60.0, fs = 250.0;
  uint32_t seed = 12345;
  auto noise = [&]() {
    seed = seed * 1664525u + 1013904223u;
    return ((seed >> 8) & 0xFFFF) / 65535.0 - 0.5;
  };
  for (int ch = 0; ch < 2; ch++) {
    const double amp = ch == 0 ? 0.012 : 0.007, lag = ch == 0 ? 0.3 : 1.4;
    lv.raw_n[ch] = LiveData::kRawPts;
    for (int i = 0; i < LiveData::kRawPts; i++)
      lv.raw[ch][i] = static_cast<float>((ch == 0 ? -0.98 : 0.02) + amp * std::sin(2 * M_PI * hz * i / fs + lag) +
                                         0.004 * noise());
  }
  const float mean[2][3] = {{0.03f, -0.98f, 0.12f}, {0.97f, 0.02f, -0.18f}};
  const float rms[2][3] = {{0.0021f, 0.0034f, 0.0089f}, {0.0012f, 0.0016f, 0.0051f}};
  for (int ch = 0; ch < 2; ch++)
    for (int k = 0; k < 3; k++) {
      lv.axis_mean[ch][k] = mean[ch][k];
      lv.axis_rms[ch][k] = rms[ch][k];
    }
}

// The frame as an image file (PPM, the simplest image format there is)
inline bool write_ppm(const std::string &path, const std::vector<uint16_t> &px) {
  FILE *f = std::fopen(path.c_str(), "wb");
  if (f == nullptr) return false;
  std::fprintf(f, "P6\n%d %d\n255\n", SCREEN_W, SCREEN_H);
  std::vector<uint8_t> row(static_cast<size_t>(SCREEN_W) * 3);
  for (int y = 0; y < SCREEN_H; y++) {
    for (int x = 0; x < SCREEN_W; x++) {
      const Rgb p = from565(px[static_cast<size_t>(y) * SCREEN_W + x]);  // as the panel shows it
      row[static_cast<size_t>(x) * 3] = static_cast<uint8_t>(p >> 16);
      row[static_cast<size_t>(x) * 3 + 1] = static_cast<uint8_t>(p >> 8);
      row[static_cast<size_t>(x) * 3 + 2] = static_cast<uint8_t>(p);
    }
    std::fwrite(row.data(), 1, row.size(), f);
  }
  std::fclose(f);
  return true;
}

}  // namespace bal
