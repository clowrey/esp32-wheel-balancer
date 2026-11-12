#include "acq_mpu.h"

#ifdef USE_ESP32

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "driver/i2c_master.h"
#include "driver/mcpwm_cap.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace bal {

static const char *const TAG = "balancer.mpu";
static const char *const TAG_RAW = "mpu.raw";

namespace {

// MPU-6050 register map (the MPU-6500/9250 family shares these).
constexpr uint8_t kRegSmplrtDiv = 0x19;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegAccelConfig2 = 0x1D;  // MPU-6500 family only
constexpr uint8_t kRegIntPinCfg = 0x37;
constexpr uint8_t kRegIntEnable = 0x38;
constexpr uint8_t kRegAccelXoutH = 0x3B;
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegPwrMgmt2 = 0x6C;
constexpr uint8_t kRegWhoAmI = 0x75;

// A 6-byte read takes about 0.2 ms at 400 kHz, so this only expires when the
// bus is in trouble. Keep it generous: when ESP-IDF's i2c_master gives up on a
// transfer, a late receive interrupt can still land and write through a stale
// pointer (seen on the P4 as a store access fault in i2c_isr_receive_handler,
// which rebooted the board while a sensor's wiring was flaky).
constexpr int kI2cTimeoutMs = 50;

// Low-pass choices: DLPF_CFG value and the accelerometer's group delay,
// which is removed from the phase. The MPU-6050
// column is from its register map. The MPU-6500-family column is less
// certain (register-map revisions disagree); it only shifts the displayed
// phase, which the trial runs calibrate out.
struct FilterRow {
  int hz;
  uint8_t cfg;
  double delay_6050_s;   // MPU-6050 (CONFIG register)
  double delay_6500_s;   // MPU-6500 family (ACCEL_CONFIG2)
};
constexpr FilterRow kFilters[] = {
    {184, 1, 0.0020, 0.00580}, {94, 2, 0.0030, 0.00780}, {44, 3, 0.0049, 0.01180},
    {21, 4, 0.0085, 0.01980},  {10, 5, 0.0138, 0.03570}, {5, 6, 0.0190, 0.06696},
};

struct Chip {
  uint8_t who;
  const char *name;
  bool family_6500;
};
constexpr Chip kChips[] = {
    {0x68, "MPU-6050", false}, {0x72, "MPU-6050 (0x72)", false}, {0x70, "MPU-6500", true},
    {0x71, "MPU-9250", true},  {0x73, "MPU-9255", true},         {0x74, "MPU-6515", true},
    {0x12, "ICM-20602", true}, {0x11, "ICM-20600", true},        {0x98, "ICM-20689", true},
};

struct CapCtx {
  MpuAcquisition *acq;
  uint8_t kind;
};
CapCtx g_mpu_ctx[3];

bool IRAM_ATTR on_mpu_capture(mcpwm_cap_channel_handle_t, const mcpwm_capture_event_data_t *ed, void *user) {
  auto *ctx = static_cast<CapCtx *>(user);
  MpuAcquisition::Event e;
  e.kind = ctx->kind;
  e.ticks = ed->cap_value;
  e.count = ctx->acq->edge_count_[ctx->kind].fetch_add(1, std::memory_order_relaxed) + 1;
  BaseType_t woken = pdFALSE;
  xQueueSendFromISR(ctx->acq->evq_, &e, &woken);
  return woken == pdTRUE;
}

}  // namespace

bool MpuAcquisition::write_reg_(int s, uint8_t reg, uint8_t value) {
  const uint8_t buf[2] = {reg, value};
  return i2c_master_transmit(dev_[s], buf, sizeof(buf), 50) == ESP_OK;
}

bool MpuAcquisition::read_regs_(int s, uint8_t reg, uint8_t *buf, size_t len) {
  return i2c_master_transmit_receive(dev_[s], &reg, 1, buf, len, kI2cTimeoutMs) == ESP_OK;
}

void MpuAcquisition::set_invert(int ch, bool invert) {
  if (ch < 0 || ch > 1) return;
  invert_[ch].store(invert);
}

bool MpuAcquisition::new_bus_(int b, int sda, int scl, bool low_power) {
  i2c_master_bus_config_t bcfg = {};
  bcfg.sda_io_num = static_cast<gpio_num_t>(sda);
  bcfg.scl_io_num = static_cast<gpio_num_t>(scl);
  if (low_power) {
    // The P4's third controller (LP I2C) only reaches GPIO0..15. The two
    // HP controllers are taken by ESPHome's bus (touch) and sensor A.
    bcfg.i2c_port = LP_I2C_NUM_0;
    bcfg.lp_source_clk = LP_I2C_SCLK_DEFAULT;
  } else {
    bcfg.i2c_port = cfg_.i2c_port;
    bcfg.clk_source = I2C_CLK_SRC_DEFAULT;
  }
  bcfg.glitch_ignore_cnt = 7;
  bcfg.flags.enable_internal_pullup = true;  // the GY-521 boards carry their own pull-ups as well
  if (i2c_new_master_bus(&bcfg, &bus_[b]) != ESP_OK) {
    ESP_LOGE(TAG, "I2C bus %d on SDA GPIO%d / SCL GPIO%d failed%s", b, sda, scl,
             low_power ? " (the second bus needs pins GPIO0..15)" : "");
    return false;
  }
  // The sensors stay powered through a reboot of the P4 and one may have been
  // cut off mid-transfer, holding SDA low. Clock it free before probing.
  i2c_master_bus_reset(bus_[b]);
  return true;
}

bool MpuAcquisition::setup_i2c_() {
  if (!new_bus_(0, cfg_.pin_sda, cfg_.pin_scl, false)) return false;
  // Sensor B on its own bus keeps a misbehaving sensor from stalling the other.
  if (cfg_.sensors > 1 && cfg_.pin_sda_b >= 0) {
    if (!new_bus_(1, cfg_.pin_sda_b, cfg_.pin_scl_b, true)) return false;
    bus_of_[1] = 1;
  }
  for (int s = 0; s < cfg_.sensors; s++) {
    // A sensor that doesn't answer yet is still added: the acquisition task
    // keeps clearing the bus and setting it up until it does, so a sensor
    // that was stuck at boot no longer needs a power cycle.
    if (!probe_(s, 3)) {
      ESP_LOGE(TAG, "sensor %c: nothing answers at 0x%02X yet (check wiring; AD0 high selects 0x69); will keep trying",
               'A' + s, cfg_.address[s]);
    }
    i2c_device_config_t dcfg = {};
    dcfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dcfg.device_address = cfg_.address[s];
    dcfg.scl_speed_hz = cfg_.i2c_hz;
    if (i2c_master_bus_add_device(bus_[bus_of_[s]], &dcfg, &dev_[s]) != ESP_OK) {
      ESP_LOGE(TAG, "sensor %c: add device failed", 'A' + s);
      return false;
    }
  }
  return true;
}

bool MpuAcquisition::probe_(int s, int tries) {
  for (int i = 0; i < tries; i++) {
    if (i > 0) {
      // Nine clocks and a STOP release a sensor that is holding SDA low.
      i2c_master_bus_reset(bus_[bus_of_[s]]);
      vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (i2c_master_probe(bus_[bus_of_[s]], cfg_.address[s], 50) == ESP_OK) return true;
  }
  return false;
}

bool MpuAcquisition::setup_sensor_(int s) {
  // Reset, then wake on the gyro PLL clock (much steadier than the internal RC).
  write_reg_(s, kRegPwrMgmt1, 0x80);
  vTaskDelay(pdMS_TO_TICKS(100));
  if (!write_reg_(s, kRegPwrMgmt1, 0x01)) {
    ESP_LOGE(TAG, "sensor %c: no response after reset", 'A' + s);
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(30));

  uint8_t who = 0;
  read_regs_(s, kRegWhoAmI, &who, 1);
  bool family_6500 = false;
  chip_[s] = "unknown (treated as MPU-6050)";
  for (const Chip &c : kChips) {
    if (c.who == who) {
      chip_[s] = c.name;
      family_6500 = c.family_6500;
    }
  }
  ESP_LOGI(TAG, "sensor %c at 0x%02X: WHO_AM_I 0x%02X, %s", 'A' + s, cfg_.address[s], who, chip_[s]);

  const FilterRow *f = &kFilters[2];
  for (const FilterRow &row : kFilters)
    if (row.hz == cfg_.filter_hz) f = &row;
  uint8_t range_bits = 0;
  switch (cfg_.range_g) {
    case 4:
      range_bits = 1;
      break;
    case 8:
      range_bits = 2;
      break;
    case 16:
      range_bits = 3;
      break;
    default:
      break;
  }
  const uint8_t div = static_cast<uint8_t>(1000 / cfg_.sample_rate - 1);  // 1 kHz base with the filter on

  bool ok = true;
  ok = ok && write_reg_(s, kRegPwrMgmt2, 0x00);
  ok = ok && write_reg_(s, kRegConfig, f->cfg);
  ok = ok && write_reg_(s, kRegSmplrtDiv, div);
  ok = ok && write_reg_(s, kRegGyroConfig, 0x00);
  ok = ok && write_reg_(s, kRegAccelConfig, static_cast<uint8_t>(range_bits << 3));
  if (family_6500) ok = ok && write_reg_(s, kRegAccelConfig2, f->cfg);
  // INT: active high, push-pull, 50 us pulse per reading.
  ok = ok && write_reg_(s, kRegIntPinCfg, 0x10);
  ok = ok && write_reg_(s, kRegIntEnable, 0x01);
  if (!ok) {
    ESP_LOGE(TAG, "sensor %c: configuration failed", 'A' + s);
    return false;
  }
  delay_s_[s] = family_6500 ? f->delay_6500_s : f->delay_6050_s;
  return true;
}

bool MpuAcquisition::setup_capture_() {
  mcpwm_capture_timer_config_t tcfg = {};
  tcfg.group_id = 0;
  tcfg.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
  tcfg.resolution_hz = 80000000;  // 12.5 ns per tick, wraps every ~54 s
  if (mcpwm_new_capture_timer(&tcfg, &cap_timer_) != ESP_OK) {
    ESP_LOGE(TAG, "mcpwm capture timer failed");
    return false;
  }
  mcpwm_capture_timer_get_resolution(cap_timer_, &cap_resolution_);

  auto make = [&](int gpio, uint8_t kind, bool pos, bool neg, bool pullup, bool pulldown) -> bool {
    mcpwm_capture_channel_config_t c = {};
    c.gpio_num = gpio;
    c.prescale = 1;
    c.flags.pos_edge = pos;
    c.flags.neg_edge = neg;
    c.flags.pull_up = pullup;
    c.flags.pull_down = pulldown;
    if (mcpwm_new_capture_channel(cap_timer_, &c, &chan_[kind]) != ESP_OK) {
      ESP_LOGE(TAG, "capture channel %u on GPIO%d failed", kind, gpio);
      return false;
    }
    g_mpu_ctx[kind] = {this, kind};
    mcpwm_capture_event_callbacks_t cb = {.on_cap = on_mpu_capture};
    mcpwm_capture_channel_register_event_callbacks(chan_[kind], &cb, &g_mpu_ctx[kind]);
    mcpwm_capture_channel_enable(chan_[kind]);
    return true;
  };
  // Tach: one edge per revolution.
  if (!make(cfg_.pin_tach, 0, !cfg_.tach_falling, cfg_.tach_falling, cfg_.tach_pullup, false)) return false;
  // Data-ready of each sensor; pulled down so an unplugged sensor stays quiet.
  for (int s = 0; s < cfg_.sensors; s++)
    if (!make(cfg_.pin_int[s], static_cast<uint8_t>(1 + s), true, false, false, true)) return false;

  mcpwm_capture_timer_enable(cap_timer_);
  mcpwm_capture_timer_start(cap_timer_);
  return true;
}

bool MpuAcquisition::start(const MpuConfig &cfg) {
  if (status_.running.load()) return true;
  cfg_ = cfg;
  if (cfg_.sensors < 1) cfg_.sensors = 1;
  if (cfg_.sensors > 2) cfg_.sensors = 2;
  for (int c = 0; c < 2; c++) {
    status_.rms_g[c].store(0);
    status_.peak_frac[c].store(0);
    invert_[c].store(cfg_.invert[c]);
  }
  for (auto &e : edge_count_) e.store(0);
  g_per_count_ = static_cast<float>(cfg_.range_g) / 32768.0f;

  evq_ = xQueueCreate(256, sizeof(Event));
  rev_queue_ = xQueueCreate(8, sizeof(RevResult));
  if (evq_ == nullptr || rev_queue_ == nullptr) {
    ESP_LOGE(TAG, "queue alloc failed");
    return false;
  }
  if (!setup_capture_()) return false;
  if (!setup_i2c_()) return false;
  for (int s = 0; s < cfg_.sensors; s++) {
    if (!setup_sensor_(s)) {
      chip_[s] = "not answering (retrying)";
      ESP_LOGW(TAG, "sensor %c: not set up yet; retrying every 3 s", 'A' + s);
    }
  }

  // Analysis runs on the resampler's grid. Slow rotors: keep revolutions down
  // to 30 rpm, and size the ring for two of the slowest. The filter delay is
  // taken off each reading's timestamp, so the analyzer has none to remove.
  for (int s = 0; s < 2; s++)
    delay_ticks_[s] = static_cast<uint32_t>(delay_s_[s] * static_cast<double>(cap_resolution_) + 0.5);
  cfg_.rev.delay_s = 0;
  cfg_.rev.fs = cfg_.sample_rate;
  cfg_.rev.min_rpm = 30.0;
  const double fastest = 60.0 * cfg_.sample_rate / 8.0;  // at least 8 samples per revolution
  if (cfg_.rev.max_rpm > fastest) cfg_.rev.max_rpm = fastest;
  cfg_.rev.ring_size = static_cast<size_t>(4.0 * cfg_.sample_rate * 60.0 / cfg_.rev.min_rpm);
  analyzer_.configure(cfg_.rev);
  resampler_.configure(static_cast<double>(cap_resolution_), cfg_.sample_rate, cfg_.sensors);
  xQueueReset(evq_);  // edges that arrived while the sensors were being set up are stale

  status_.running.store(true);
  xTaskCreatePinnedToCore(task_trampoline_, "balancer_mpu", 8192, this, cfg_.task_priority, &task_handle_,
                          cfg_.task_core);
  ESP_LOGI(TAG, "acquisition running: %d sensor(s) at %u Hz, %d Hz filter (delay %.1f ms), +/-%d g", cfg_.sensors,
           (unsigned) cfg_.sample_rate, cfg_.filter_hz, delay_s_[0] * 1e3, cfg_.range_g);
  return true;
}

void MpuAcquisition::task_trampoline_(void *arg) {
  static_cast<MpuAcquisition *>(arg)->task_();
  vTaskDelete(nullptr);
}

int MpuAcquisition::copy_raw(int ch, float *out, int max) const {
  if (ch < 0 || ch >= cfg_.sensors || max <= 0) return 0;
  const uint32_t n = raw_n_[ch].load(std::memory_order_acquire);
  const int count = static_cast<int>(std::min<uint32_t>(n, std::min<uint32_t>(max, kRawLen)));
  // Oldest first. The task may overwrite the oldest few while this copies;
  // that only nudges a display trace.
  for (int i = 0; i < count; i++) out[i] = raw_[ch][(n - count + i) % kRawLen];
  return count;
}

void MpuAcquisition::set_stream(bool on) {
  if (on && !stream_.load()) {
    ESP_LOGI(TAG_RAW, "MPU CFG range=%d rate=%u filter=%d sensors=%d axisA=%d axisB=%d clock=%u", cfg_.range_g,
             (unsigned) cfg_.sample_rate, cfg_.filter_hz, cfg_.sensors, cfg_.axis[0], cfg_.axis[1],
             (unsigned) cap_resolution_);
    stream_reset_.store(true);
  }
  stream_.store(on);
}

void MpuAcquisition::stream_push_(int s, uint32_t ticks, const int16_t xyz[3]) {
  // Extend the 32-bit capture timer (wraps every ~54 s at 80 MHz) to 64 bits.
  // Both sensors share one extension; a signed step tolerates their edges
  // arriving slightly out of order.
  if (stream_reset_.exchange(false)) {
    stream_ticks_init_ = false;
    batch_[0].n = batch_[1].n = 0;
  }
  if (!stream_ticks_init_) {
    stream_last_ticks_ = ticks;
    stream_ticks64_ = ticks;
    stream_ticks_init_ = true;
  }
  stream_ticks64_ += static_cast<int32_t>(ticks - stream_last_ticks_);
  stream_last_ticks_ = ticks;
  StreamBatch &bt = batch_[s];
  const uint64_t us = stream_ticks64_ / (cap_resolution_ / 1000000u);
  if (bt.n == 0) bt.t0_us = us;
  bt.dt_us[bt.n] = static_cast<uint32_t>(us - bt.t0_us);
  for (int k = 0; k < 3; k++) bt.xyz[bt.n][k] = xyz[k];
  if (++bt.n < kBatch) return;
  // One log line per batch: "MPU A <t0_us> <n> dt,x,y,z ..." in raw counts.
  char line[kBatch * 26 + 48];
  int p = snprintf(line, sizeof(line), "MPU %c %llu %d", 'A' + s, static_cast<unsigned long long>(bt.t0_us), bt.n);
  for (int i = 0; i < bt.n && p < static_cast<int>(sizeof(line)) - 26; i++)
    p += snprintf(line + p, sizeof(line) - p, " %u,%d,%d,%d", (unsigned) bt.dt_us[i], bt.xyz[i][0], bt.xyz[i][1],
                  bt.xyz[i][2]);
  ESP_LOGI(TAG_RAW, "%s", line);
  bt.n = 0;
}

bool MpuAcquisition::pop(RevResult &out) {
  if (rev_queue_ == nullptr) return false;
  return xQueueReceive(rev_queue_, &out, 0) == pdTRUE;
}

void MpuAcquisition::task_() {
  static float f0[64], f1[64];
  Event e;
  RevResult rev;
  uint64_t frames = 0;
  const uint32_t start_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
  uint32_t last_stat_ms = start_ms, last_warn_ms = start_ms, warned_errors = 0;
  uint32_t last_reading_ms[2] = {start_ms, start_ms};
  uint32_t last_reinit_ms[2] = {start_ms, start_ms};

  while (status_.running.load()) {
    const bool got = xQueueReceive(evq_, &e, pdMS_TO_TICKS(100)) == pdTRUE;
    const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    if (got && e.kind == 0) {
      status_.tach_pulses.fetch_add(1, std::memory_order_relaxed);
      status_.last_tach_ms.store(now_ms);
      resampler_.push_tach(e.ticks);
    } else if (got) {
      const int s = e.kind - 1;
      uint8_t b[6];
      if (edge_count_[e.kind].load(std::memory_order_relaxed) != e.count) {
        // The task fell behind and a newer reading has replaced this one.
        missed_++;
      } else if (fail_streak_[s] >= kFailBackoff && now_ms < next_try_ms_[s]) {
        // This sensor keeps failing: only try it every kBackoffMs, so its
        // timeouts can't starve the other sensor.
        backoff_skips_[s]++;
      } else if (!read_regs_(s, kRegAccelXoutH, b, sizeof(b))) {
        i2c_errors_++;
        sensor_errors_[s]++;
        if (++fail_streak_[s] >= kFailBackoff) {
          next_try_ms_[s] = now_ms + kBackoffMs;
          if (fail_streak_[s] == kFailBackoff)
            ESP_LOGW(TAG, "sensor %c: %d reads in a row failed; trying it every %d ms until it answers", 'A' + s,
                     kFailBackoff, kBackoffMs);
        }
      } else if (edge_count_[e.kind].load(std::memory_order_relaxed) != e.count) {
        // A new reading landed during the read: no telling which one we got.
        missed_++;
      } else {
        if (fail_streak_[s] >= kFailBackoff) ESP_LOGI(TAG, "sensor %c: reads working again", 'A' + s);
        fail_streak_[s] = 0;
        int16_t xyz[3];
        for (int k = 0; k < 3; k++)
          xyz[k] = static_cast<int16_t>((static_cast<uint16_t>(b[2 * k]) << 8) | b[2 * k + 1]);
        const int16_t raw = xyz[cfg_.axis[s]];
        float g = static_cast<float>(raw) * g_per_count_;
        if (invert_[s].load(std::memory_order_relaxed)) g = -g;
        // Recent readings for the LIVE screen's raw trace.
        const uint32_t w = raw_n_[s].load(std::memory_order_relaxed);
        raw_[s][w % kRawLen] = g;
        raw_n_[s].store(w + 1, std::memory_order_release);
        if (stream_.load(std::memory_order_relaxed)) stream_push_(s, e.ticks, xyz);
        sum_[s] += g;
        sq_sum_[s] += static_cast<double>(g) * g;
        stat_n_[s]++;
        const float frac = std::fabs(static_cast<float>(raw)) / 32768.0f;
        if (frac > peak_[s]) peak_[s] = frac;
        last_reading_ms[s] = now_ms;
        resampler_.push(s, e.ticks - delay_ticks_[s], g);  // when the motion happened, not when it was reported
      }
    }

    if (resampler_.take_restart()) {
      analyzer_.reset();  // the grid has a new origin
      frames = 0;
    }
    size_t n;
    while ((n = resampler_.pop_frames(f0, f1, 64)) > 0) {
      analyzer_.push_samples(f0, f1, n);
      frames += n;
    }
    double tach_index;
    while (resampler_.pop_tach(tach_index)) analyzer_.push_tach(tach_index);
    while (analyzer_.pop(rev)) {
      status_.revs.fetch_add(1, std::memory_order_relaxed);
      if (xQueueSend(rev_queue_, &rev, 0) != pdTRUE) {
        RevResult drop;  // consumer is behind: keep the newest
        xQueueReceive(rev_queue_, &drop, 0);
        xQueueSend(rev_queue_, &rev, 0);
      }
    }

    if (now_ms - last_stat_ms >= 1000) {
      last_stat_ms = now_ms;
      bool alive = resampler_.started();
      for (int c = 0; c < 2; c++) {
        if (c < cfg_.sensors) alive = alive && (now_ms - last_reading_ms[c] < 300);
        if (stat_n_[c] > 0) {
          // AC part only: gravity and tilt sit on every reading.
          const double mean = sum_[c] / stat_n_[c];
          const double var = sq_sum_[c] / stat_n_[c] - mean * mean;
          status_.rms_g[c].store(static_cast<float>(std::sqrt(var > 0 ? var : 0)));
        } else {
          status_.rms_g[c].store(0);
        }
        status_.peak_frac[c].store(peak_[c]);
        sum_[c] = sq_sum_[c] = 0;
        peak_[c] = 0;
        stat_n_[c] = 0;
      }
      // A sensor that lost power or was re-plugged comes back asleep with its
      // interrupt off, and one that hung mid-transfer holds SDA low until the
      // bus is clocked free: clear the bus and set it up again (this stalls
      // the other sensor for ~0.15 s). This also picks up a sensor that did
      // not answer at boot.
      for (int c = 0; c < cfg_.sensors; c++) {
        if (now_ms - last_reading_ms[c] > 1500 && now_ms - last_reinit_ms[c] > 3000) {
          last_reinit_ms[c] = now_ms;
          if (!probe_(c, 3)) {
            if (!silent_logged_[c]) ESP_LOGW(TAG, "sensor %c is silent and not answering; still trying", 'A' + c);
            silent_logged_[c] = true;
          } else if (setup_sensor_(c)) {
            delay_ticks_[c] = static_cast<uint32_t>(delay_s_[c] * static_cast<double>(cap_resolution_) + 0.5);
            ESP_LOGW(TAG, "sensor %c was silent and has been set up again", 'A' + c);
            silent_logged_[c] = false;
            fail_streak_[c] = 0;
          }
        }
      }
      status_.aligned.store(alive);
      status_.frames.store(static_cast<uint32_t>(frames));
      status_.measured_fs.store(static_cast<float>(resampler_.measured_rate(0)));
      status_.resyncs.store(resampler_.restarts());
      status_.dma_overflows.store(missed_ + i2c_errors_);
      status_.glitches.store(analyzer_.glitches());
      if (i2c_errors_ != warned_errors && now_ms - last_warn_ms >= 5000) {
        ESP_LOGW(TAG,
                 "failed sensor reads so far: A %u, B %u (skipped while backing off: A %u, B %u); check that "
                 "sensor's wiring, or try a lower frequency",
                 (unsigned) sensor_errors_[0], (unsigned) sensor_errors_[1], (unsigned) backoff_skips_[0],
                 (unsigned) backoff_skips_[1]);
        warned_errors = i2c_errors_;
        last_warn_ms = now_ms;
      }
    }
  }
}

}  // namespace bal

#endif  // USE_ESP32
