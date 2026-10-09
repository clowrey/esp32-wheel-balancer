#include "acq_capture.h"

#ifdef USE_ESP32

#include <cmath>
#include <cstring>

#include "driver/i2s_std.h"
#include "driver/mcpwm_cap.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace bal {

static const char *const TAG = "balancer.acq";

namespace {

constexpr uint32_t kLrckPrescale = 256;  // one LRCK capture every 256 frames
constexpr size_t kFramesPerRead = 256;

struct CapCtx {
  Acquisition *acq;
  uint8_t kind;
};
CapCtx g_ctx[3];

bool IRAM_ATTR on_capture(mcpwm_cap_channel_handle_t, const mcpwm_capture_event_data_t *ed, void *user) {
  auto *ctx = static_cast<CapCtx *>(user);
  Acquisition::CapEvent e;
  e.kind = ctx->kind;
  e.ticks = ed->cap_value;
  e.frames = ctx->acq->isr_frames_done_.load(std::memory_order_relaxed);
  BaseType_t woken = pdFALSE;
  xQueueSendFromISR(ctx->acq->cap_queue_, &e, &woken);
  return woken == pdTRUE;
}

bool IRAM_ATTR on_i2s_recv(i2s_chan_handle_t, i2s_event_data_t *ev, void *user) {
  auto *acq = static_cast<Acquisition *>(user);
  acq->isr_frames_done_.fetch_add(ev->size / 8, std::memory_order_relaxed);
  // Timestamp the DMA buffer completion on the capture timer. Lets the task
  // pin frame indices to capture ticks (PLAN §4).
  if (acq->soft_chan_ != nullptr) mcpwm_capture_channel_trigger_soft_catch(acq->soft_chan_);
  return false;
}

bool IRAM_ATTR on_i2s_overflow(i2s_chan_handle_t, i2s_event_data_t *, void *user) {
  static_cast<Acquisition *>(user)->isr_overflows_.fetch_add(1, std::memory_order_relaxed);
  return false;
}

}  // namespace

void Acquisition::set_channel(int ch, double sens_mv_g, double gain, bool invert) {
  if (ch < 0 || ch > 1) return;
  cfg_.sensitivity_mv_g[ch] = sens_mv_g;
  cfg_.gain[ch] = gain;
  cfg_.invert[ch] = invert;
  // int32 full scale -> volts -> g
  const double v_per_count = cfg_.adc_full_scale_v / 2147483648.0;
  const double v_per_g = (sens_mv_g / 1000.0) * gain;
  float s = v_per_g > 0 ? static_cast<float>(v_per_count / v_per_g) : 0.0f;
  scale_[ch] = invert ? -s : s;
}

bool Acquisition::start(const AcqConfig &cfg) {
  if (status_.running.load()) return true;
  cfg_ = cfg;
  cfg_.rev.fs = cfg_.sample_rate;
  analyzer_.configure(cfg_.rev);
  for (int c = 0; c < 2; c++) {
    status_.rms_g[c].store(0);
    status_.peak_frac[c].store(0);
    set_channel(c, cfg_.sensitivity_mv_g[c], cfg_.gain[c], cfg_.invert[c]);
  }
  cap_queue_ = xQueueCreate(256, sizeof(CapEvent));
  rev_queue_ = xQueueCreate(8, sizeof(RevResult));
  if (cap_queue_ == nullptr || rev_queue_ == nullptr) {
    ESP_LOGE(TAG, "queue alloc failed");
    return false;
  }
  if (!setup_i2s_()) return false;
  if (!setup_capture_()) return false;
  if (i2s_channel_enable(rx_) != ESP_OK) {
    ESP_LOGE(TAG, "i2s enable failed");
    return false;
  }
  status_.running.store(true);
  xTaskCreatePinnedToCore(task_trampoline_, "balancer_acq", 8192, this, cfg_.task_priority, &task_,
                          cfg_.task_core);
  ESP_LOGI(TAG, "acquisition running: fs=%u Hz, capture timer %u Hz", (unsigned) cfg_.sample_rate,
           (unsigned) cap_resolution_);
  return true;
}

void Acquisition::stop() {
  status_.running.store(false);
  // The task exits on its own; peripherals stay allocated (the device only
  // ever stops on reboot).
}

bool Acquisition::setup_i2s_() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(static_cast<i2s_port_t>(cfg_.i2s_port), I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = 16;  // 16 x 256 frames = 256 ms of slack at 16 kHz
  chan_cfg.dma_frame_num = kFramesPerRead;
  if (i2s_new_channel(&chan_cfg, nullptr, &rx_) != ESP_OK) {
    ESP_LOGE(TAG, "i2s_new_channel failed");
    return false;
  }
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(cfg_.sample_rate),
      // PCM1808: 24-bit I2S, 64 BCK per frame -> read 32-bit slots, data MSB-aligned.
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
      .gpio_cfg =
          {
              .mclk = static_cast<gpio_num_t>(cfg_.pin_mclk),
              .bclk = static_cast<gpio_num_t>(cfg_.pin_bclk),
              .ws = static_cast<gpio_num_t>(cfg_.pin_ws),
              .dout = I2S_GPIO_UNUSED,
              .din = static_cast<gpio_num_t>(cfg_.pin_din),
              .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false},
          },
  };
  std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
  if (cfg_.use_apll) std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_APLL;
  if (i2s_channel_init_std_mode(rx_, &std_cfg) != ESP_OK) {
    ESP_LOGE(TAG, "i2s_channel_init_std_mode failed");
    return false;
  }
  i2s_event_callbacks_t cbs = {};
  cbs.on_recv = on_i2s_recv;
  cbs.on_recv_q_ovf = on_i2s_overflow;
  if (i2s_channel_register_event_callback(rx_, &cbs, this) != ESP_OK) {
    ESP_LOGE(TAG, "i2s callback registration failed");
    return false;
  }
  return true;
}

bool Acquisition::setup_capture_() {
  mcpwm_capture_timer_config_t tcfg = {};
  tcfg.group_id = 0;
  tcfg.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;  // PLL_F160M on the P4
  tcfg.resolution_hz = 80000000;                 // 12.5 ns per tick, wraps every ~54 s
  if (mcpwm_new_capture_timer(&tcfg, &cap_timer_) != ESP_OK) {
    ESP_LOGE(TAG, "mcpwm capture timer failed");
    return false;
  }
  mcpwm_capture_timer_get_resolution(cap_timer_, &cap_resolution_);
  timebase_.configure(static_cast<double>(cap_resolution_), cfg_.sample_rate, kLrckPrescale);

  auto make = [&](int gpio, uint8_t kind, uint32_t prescale, bool pos, bool neg, bool pullup,
                  bool loopback, mcpwm_cap_channel_handle_t *out) -> bool {
    mcpwm_capture_channel_config_t c = {};
    c.gpio_num = gpio;
    c.prescale = prescale;
    c.flags.pos_edge = pos;
    c.flags.neg_edge = neg;
    c.flags.pull_up = pullup;
    c.flags.io_loop_back = loopback;
    if (mcpwm_new_capture_channel(cap_timer_, &c, out) != ESP_OK) {
      ESP_LOGE(TAG, "capture channel %u failed", kind);
      return false;
    }
    g_ctx[kind] = {this, kind};
    mcpwm_capture_event_callbacks_t cb = {.on_cap = on_capture};
    mcpwm_capture_channel_register_event_callbacks(*out, &cb, &g_ctx[kind]);
    mcpwm_capture_channel_enable(*out);
    return true;
  };
  // Tach: one edge per revolution.
  if (!make(cfg_.pin_tach, 0, 1, !cfg_.tach_falling, cfg_.tach_falling, cfg_.tach_pullup, false, &tach_chan_))
    return false;
  // LRCK: every 256th rising edge. If no separate pin is given, read the WS
  // pad itself; the I2S peripheral keeps driving it.
  const int lrck_pin = cfg_.pin_lrck_capture >= 0 ? cfg_.pin_lrck_capture : cfg_.pin_ws;
  if (!make(lrck_pin, 1, kLrckPrescale, true, false, false, cfg_.pin_lrck_capture < 0, &lrck_chan_))
    return false;
  // Software catch, triggered from the I2S DMA-complete interrupt.
  if (!make(-1, 2, 1, true, false, false, false, &soft_chan_)) return false;

  mcpwm_capture_timer_enable(cap_timer_);
  mcpwm_capture_timer_start(cap_timer_);
  return true;
}

void Acquisition::task_trampoline_(void *arg) {
  static_cast<Acquisition *>(arg)->task_();
  vTaskDelete(nullptr);
}

bool Acquisition::pop(RevResult &out) {
  if (rev_queue_ == nullptr) return false;
  return xQueueReceive(rev_queue_, &out, 0) == pdTRUE;
}

void Acquisition::handle_cap_(const CapEvent &e) {
  switch (e.kind) {
    case 0:
      status_.tach_pulses.fetch_add(1, std::memory_order_relaxed);
      status_.last_tach_ms.store(static_cast<uint32_t>(esp_timer_get_time() / 1000));
      timebase_.on_tach(e.ticks);
      break;
    case 1:
      timebase_.on_lrck(e.ticks);
      break;
    case 2:
      timebase_.on_soft(e.ticks, e.frames);
      break;
  }
}

void Acquisition::task_() {
  static int32_t buf[kFramesPerRead * 2];
  static float ch0[kFramesPerRead], ch1[kFramesPerRead];
  int64_t next_frame = 0;  // frame index of the next sample we will read
  frame_origin_ = 0;
  RevResult rev;
  uint32_t last_stat_ms = 0;

  while (status_.running.load()) {
    size_t bytes = 0;
    const int64_t t_before = esp_timer_get_time();
    if (i2s_channel_read(rx_, buf, sizeof(buf), &bytes, pdMS_TO_TICKS(200)) != ESP_OK || bytes == 0) continue;
    const int64_t waited_us = esp_timer_get_time() - t_before;
    const size_t n = bytes / 8;

    // Continuity check: if we blocked, this buffer just completed, so the
    // ISR count tells us exactly which frames these are.
    const uint32_t done = isr_frames_done_.load();
    const uint32_t ovf = isr_overflows_.load();
    if (waited_us > 2000) {
      const int64_t first = static_cast<int64_t>(done) - static_cast<int64_t>(n);
      if (first != next_frame || ovf != overflows_seen_) {
        if (next_frame != 0) {
          ESP_LOGW(TAG, "sample stream gap (expected frame %lld, got %lld), restarting analysis",
                   (long long) next_frame, (long long) first);
        }
        next_frame = first;
        frame_origin_ = first;
        analyzer_.reset();
        overflows_seen_ = ovf;
        status_.dma_overflows.store(ovf);
      }
    }

    const float s0 = scale_[cfg_.swap_channels ? 1 : 0];
    const float s1 = scale_[cfg_.swap_channels ? 0 : 1];
    for (size_t i = 0; i < n; i++) {
      const int32_t l = buf[2 * i], r = buf[2 * i + 1];
      const int32_t a = cfg_.swap_channels ? r : l;
      const int32_t b = cfg_.swap_channels ? l : r;
      ch0[i] = static_cast<float>(a) * s0;
      ch1[i] = static_cast<float>(b) * s1;
      sq_sum_[0] += static_cast<double>(ch0[i]) * ch0[i];
      sq_sum_[1] += static_cast<double>(ch1[i]) * ch1[i];
      const float fa = std::fabs(static_cast<float>(a)) / 2147483648.0f;
      const float fb = std::fabs(static_cast<float>(b)) / 2147483648.0f;
      if (fa > peak_[0]) peak_[0] = fa;
      if (fb > peak_[1]) peak_[1] = fb;
    }
    stat_frames_ += n;
    analyzer_.push_samples(ch0, ch1, n);
    next_frame += static_cast<int64_t>(n);
    status_.frames.store(static_cast<uint32_t>(next_frame));

    CapEvent e;
    while (xQueueReceive(cap_queue_, &e, 0) == pdTRUE) handle_cap_(e);
    double tach_frame;
    while (timebase_.pop_tach(tach_frame)) {
      const double idx = tach_frame - static_cast<double>(frame_origin_);
      if (idx >= 0) analyzer_.push_tach(idx);
    }
    status_.aligned.store(timebase_.aligned());
    status_.resyncs.store(timebase_.realigns());
    status_.measured_fs.store(static_cast<float>(timebase_.measured_fs()));

    while (analyzer_.pop(rev)) {
      status_.revs.fetch_add(1, std::memory_order_relaxed);
      if (xQueueSend(rev_queue_, &rev, 0) != pdTRUE) {
        RevResult drop;  // consumer is behind: keep the newest
        xQueueReceive(rev_queue_, &drop, 0);
        xQueueSend(rev_queue_, &rev, 0);
      }
    }
    status_.glitches.store(analyzer_.glitches());

    const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    if (now_ms - last_stat_ms >= 1000 && stat_frames_ > 0) {
      for (int c = 0; c < 2; c++) {
        status_.rms_g[c].store(static_cast<float>(std::sqrt(sq_sum_[c] / stat_frames_)));
        status_.peak_frac[c].store(peak_[c]);
        sq_sum_[c] = 0;
        peak_[c] = 0;
      }
      stat_frames_ = 0;
      last_stat_ms = now_ms;
    }
  }
}

}  // namespace bal

#endif  // USE_ESP32
