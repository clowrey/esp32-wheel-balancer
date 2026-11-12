#include "acq_mpu.h"

#ifdef USE_ESP32

#include <cmath>
#include <cstring>

#include "driver/i2c_master.h"
#include "driver/mcpwm_cap.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace bal {

static const char *const TAG = "balancer.mpu";

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

constexpr int kI2cTimeoutMs = 5;

// Low-pass choices: DLPF_CFG value and the accelerometer's group delay,
// which is removed from the phase like the PCM1808's delay is.
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

bool MpuAcquisition::setup_i2c_() {
  i2c_master_bus_config_t bcfg = {};
  bcfg.i2c_port = cfg_.i2c_port;
  bcfg.sda_io_num = static_cast<gpio_num_t>(cfg_.pin_sda);
  bcfg.scl_io_num = static_cast<gpio_num_t>(cfg_.pin_scl);
  bcfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bcfg.glitch_ignore_cnt = 7;
  bcfg.flags.enable_internal_pullup = true;  // the GY-521 boards carry their own pull-ups as well
  if (i2c_new_master_bus(&bcfg, &bus_) != ESP_OK) {
    ESP_LOGE(TAG, "I2C bus on SDA GPIO%d / SCL GPIO%d failed", cfg_.pin_sda, cfg_.pin_scl);
    return false;
  }
  for (int s = 0; s < cfg_.sensors; s++) {
    if (i2c_master_probe(bus_, cfg_.address[s], 50) != ESP_OK) {
      ESP_LOGE(TAG, "sensor %c: nothing answers at 0x%02X (check wiring; AD0 high selects 0x69)", 'A' + s,
               cfg_.address[s]);
      return false;
    }
    i2c_device_config_t dcfg = {};
    dcfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dcfg.device_address = cfg_.address[s];
    dcfg.scl_speed_hz = cfg_.i2c_hz;
    if (i2c_master_bus_add_device(bus_, &dcfg, &dev_[s]) != ESP_OK) {
      ESP_LOGE(TAG, "sensor %c: add device failed", 'A' + s);
      return false;
    }
  }
  return true;
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
  if (s == 0) cfg_.rev.delay_s = family_6500 ? f->delay_6500_s : f->delay_6050_s;
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
  for (int s = 0; s < cfg_.sensors; s++)
    if (!setup_sensor_(s)) return false;

  // Analysis runs on the resampler's grid. Slow rotors: keep revolutions down
  // to 30 rpm, and size the ring for two of the slowest.
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
           (unsigned) cfg_.sample_rate, cfg_.filter_hz, cfg_.rev.delay_s * 1e3, cfg_.range_g);
  return true;
}

void MpuAcquisition::task_trampoline_(void *arg) {
  static_cast<MpuAcquisition *>(arg)->task_();
  vTaskDelete(nullptr);
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
  uint32_t last_stat_ms = 0, last_warn_ms = 0, warned_errors = 0;
  uint32_t last_reading_ms[2] = {0, 0};

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
      } else if (!read_regs_(s, kRegAccelXoutH, b, sizeof(b))) {
        i2c_errors_++;
      } else if (edge_count_[e.kind].load(std::memory_order_relaxed) != e.count) {
        // A new reading landed during the read: no telling which one we got.
        missed_++;
      } else {
        const int a = cfg_.axis[s];
        const int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(b[2 * a]) << 8) | b[2 * a + 1]);
        float g = static_cast<float>(raw) * g_per_count_;
        if (invert_[s].load(std::memory_order_relaxed)) g = -g;
        sum_[s] += g;
        sq_sum_[s] += static_cast<double>(g) * g;
        stat_n_[s]++;
        const float frac = std::fabs(static_cast<float>(raw)) / 32768.0f;
        if (frac > peak_[s]) peak_[s] = frac;
        last_reading_ms[s] = now_ms;
        resampler_.push(s, e.ticks, g);
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
      status_.aligned.store(alive);
      status_.frames.store(static_cast<uint32_t>(frames));
      status_.measured_fs.store(static_cast<float>(resampler_.measured_rate(0)));
      status_.resyncs.store(resampler_.restarts());
      status_.dma_overflows.store(missed_ + i2c_errors_);
      status_.glitches.store(analyzer_.glitches());
      if (i2c_errors_ != warned_errors && now_ms - last_warn_ms >= 5000) {
        ESP_LOGW(TAG, "%u failed sensor reads so far (check the I2C wiring; try a lower frequency)",
                 (unsigned) i2c_errors_);
        warned_errors = i2c_errors_;
        last_warn_ms = now_ms;
      }
    }
  }
}

}  // namespace bal

#endif  // USE_ESP32
