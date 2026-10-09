#pragma once
// Acquisition (PLAN §4, Phase 1): PCM1808 over I2S1 plus MCPWM capture of
// the tach and of every 256th LRCK edge, all on one hardware timebase.
// Runs its own FreeRTOS task; hands finished revolutions to the main loop
// through a queue. ESP-IDF only.

#ifdef USE_ESP32

#include <atomic>
#include <cstdint>

#include "dsp_revolution.h"
#include "dsp_timebase.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

struct i2s_channel_obj_t;
struct mcpwm_cap_timer_t;
struct mcpwm_cap_channel_t;

namespace bal {

struct AcqConfig {
  int i2s_port = 1;
  int pin_mclk = 1, pin_bclk = 2, pin_ws = 3, pin_din = 4;
  int pin_tach = 32;
  int pin_lrck_capture = -1;  // -1: capture LRCK on pin_ws itself (loopback through the pad)
  bool tach_falling = false;
  bool tach_pullup = true;
  uint32_t sample_rate = 16000;
  bool use_apll = false;
  double adc_full_scale_v = 1.5;  // PCM1808 at 5 V AVDD: 0.6*VCC p-p -> +/-1.5 V
  double sensitivity_mv_g[2] = {100.0, 100.0};
  double gain[2] = {10.0, 10.0};
  bool invert[2] = {false, false};
  bool swap_channels = false;
  RevolutionConfig rev;  // fs is overwritten with sample_rate
  int task_core = 1;
  int task_priority = 10;
};

// Live numbers for the status bar / debug page. Written by the acquisition
// task, read by the main loop (all fields are individually atomic).
struct AcqStatus {
  std::atomic<uint32_t> frames{0};
  std::atomic<uint32_t> tach_pulses{0};
  std::atomic<uint32_t> revs{0};
  std::atomic<uint32_t> dma_overflows{0};
  std::atomic<uint32_t> resyncs{0};
  std::atomic<uint32_t> glitches{0};
  std::atomic<uint32_t> last_tach_ms{0};
  std::atomic<float> measured_fs{0};
  std::atomic<float> rms_g[2];
  std::atomic<float> peak_frac[2];  // largest |sample| / full scale over the last second
  std::atomic<bool> aligned{false};
  std::atomic<bool> running{false};
};

class Acquisition {
 public:
  bool start(const AcqConfig &cfg);
  void stop();
  bool running() const { return status_.running.load(); }

  // Non-blocking: next finished revolution, if any.
  bool pop(RevResult &out);
  const AcqStatus &status() const { return status_; }

  // Scale factors can change at runtime (settings page).
  void set_channel(int ch, double sensitivity_mv_g, double gain, bool invert);

  // --- internals used by the ISR trampolines ---
  struct CapEvent {
    uint8_t kind;      // 0 tach, 1 lrck, 2 soft catch
    uint32_t ticks;    // capture timer value
    uint32_t frames;   // frames completed (soft catch only)
  };
  QueueHandle_t cap_queue_ = nullptr;
  std::atomic<uint32_t> isr_frames_done_{0};
  mcpwm_cap_channel_t *soft_chan_ = nullptr;
  std::atomic<uint32_t> isr_overflows_{0};

 private:
  static void task_trampoline_(void *arg);
  void task_();
  bool setup_i2s_();
  bool setup_capture_();
  void handle_cap_(const CapEvent &e);

  AcqConfig cfg_;
  AcqStatus status_;
  RevolutionAnalyzer analyzer_;
  TachTimebase timebase_;
  QueueHandle_t rev_queue_ = nullptr;
  TaskHandle_t task_ = nullptr;
  i2s_channel_obj_t *rx_ = nullptr;
  mcpwm_cap_timer_t *cap_timer_ = nullptr;
  mcpwm_cap_channel_t *tach_chan_ = nullptr;
  mcpwm_cap_channel_t *lrck_chan_ = nullptr;
  uint32_t cap_resolution_ = 0;
  volatile float scale_[2] = {0, 0};
  int64_t frame_origin_ = 0;   // analyzer index = frame index - frame_origin_
  uint32_t overflows_seen_ = 0;
  // Running stats for the status page.
  double sq_sum_[2] = {0, 0};
  float peak_[2] = {0, 0};
  uint32_t stat_frames_ = 0;
};

}  // namespace bal

#endif  // USE_ESP32
