#pragma once
// Acquisition from one or two MPU-6050-family accelerometers on I2C, for
// slow rotors on a soft rig (tires). ESP-IDF only.
//
// Each sensor raises INT when a reading is ready. Those edges and the tach
// are latched by one MCPWM capture timer; the task reads the sensor after
// each edge and StreamResampler (dsp_resample.h) puts both sensors and the
// tach on one uniform grid, and RevolutionAnalyzer takes it from there.

#ifdef USE_ESP32

#include <atomic>
#include <cstdint>

#include "dsp_resample.h"
#include "dsp_revolution.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

struct i2c_master_bus_t;
struct i2c_master_dev_t;
struct mcpwm_cap_timer_t;
struct mcpwm_cap_channel_t;

namespace bal {

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
  // Every axis of each sensor over the last second, for the axis picker:
  // mean (mostly gravity, so it shows which axis is vertical) and AC rms.
  std::atomic<float> axis_mean[2][3] = {};
  std::atomic<float> axis_rms[2][3] = {};
  std::atomic<float> peak_frac[2];  // largest |sample| / full scale over the last second
  std::atomic<uint32_t> read_errors[2] = {};  // I2C reads that failed, per sensor
  std::atomic<uint32_t> missed[2] = {};       // readings overwritten before they could be read, per sensor
  std::atomic<bool> aligned{false};
  std::atomic<bool> running{false};
};

struct MpuConfig {
  int i2c_port = -1;            // -1: first free port (ESPHome's own bus has port 0)
  int pin_sda = -1, pin_scl = -1;  // sensor A's bus (and B's, unless B has its own)
  // Optional separate bus for sensor B, on the P4's LP I2C controller, which
  // only reaches GPIO0..15. -1: B shares sensor A's bus.
  int pin_sda_b = -1, pin_scl_b = -1;
  uint32_t i2c_hz = 400000;
  int sensors = 2;              // 1: sensor A only (single-plane balancing)
  uint8_t address[2] = {0x68, 0x69};
  int pin_int[2] = {-1, -1};
  int axis[2] = {0, 0};         // 0 X, 1 Y, 2 Z: the axis that points along the measured motion
  bool invert[2] = {false, false};
  int range_g = 2;              // 2, 4, 8, 16
  int filter_hz = 44;           // sensor low-pass: 5, 10, 21, 44, 94, 184
  uint32_t sample_rate = 250;   // sensor output rate and analysis grid, Hz (divides 1000)
  int pin_tach = -1;
  bool tach_falling = false;
  bool tach_pullup = true;
  RevolutionConfig rev;         // fs is filled in at start; delay_s is not used (see delay_s_)
  int task_core = 1;
  int task_priority = 10;
};

class MpuAcquisition {
 public:
  bool start(const MpuConfig &cfg);
  void stop() { status_.running.store(false); }

  bool pop(RevResult &out);
  const AcqStatus &status() const { return status_; }
  // Only the polarity can change at runtime: the sensor already reports g.
  void set_invert(int ch, bool invert);
  // Which sensor axis (0 X, 1 Y, 2 Z) feeds the measurement; changeable at runtime.
  void set_axis(int ch, int axis);
  const char *chip_name(int ch) const { return chip_[ch]; }
  double filter_delay_s(int ch) const { return delay_s_[ch]; }

  // The latest readings of one sensor's measured axis in g, oldest first.
  // Returns how many were copied (up to max and kRawLen).
  static constexpr int kRawLen = 1024;
  int copy_raw(int ch, float *out, int max) const;
  // Stream every reading (all three axes, raw counts, capture timestamps) to
  // the log for a PC to record; see tools/mpu_log.py. Lines are tagged
  // "mpu.raw": "MPU A <t0_us> <n> dt_us,x,y,z ..." plus one "MPU CFG" line.
  void set_stream(bool on);
  bool streaming() const { return stream_.load(); }

  // Recorder: the last few minutes of every raw reading and tach pulse, in
  // PSRAM, for downloading and offline analysis (balancer.cpp serves it).
  struct RecRecord {
    uint32_t t_us;   // capture time, us since acquisition started (wraps after ~71 min)
    uint8_t kind;    // 0 tach pulse, 1 sensor A, 2 sensor B
    uint8_t axis;    // the axis that sensor was measuring (0 X, 1 Y, 2 Z)
    int16_t xyz[3];  // raw counts (zero for a tach pulse)
  };
  static_assert(sizeof(RecRecord) == 12, "recorder file format");
  bool set_recorder_minutes(float minutes);  // call after start(); allocates the ring
  bool recording() const { return rec_ != nullptr; }
  // Records [first, first + count) are stable to read now; index modulo capacity.
  size_t recorder_snapshot(uint64_t *first) const;
  const RecRecord &recorder_at(uint64_t i) const { return rec_[i % rec_cap_]; }
  size_t recorder_capacity() const { return rec_cap_; }
  const MpuConfig &config() const { return cfg_; }
  uint32_t capture_hz() const { return cap_resolution_; }
  // --- internals used by the ISR trampoline ---
  struct Event {
    uint8_t kind;     // 0 tach, 1 sensor A ready, 2 sensor B ready
    uint32_t ticks;   // capture timer value at the edge
    uint32_t count;   // per-source edge counter
  };
  QueueHandle_t evq_ = nullptr;
  std::atomic<uint32_t> edge_count_[3];

 private:
  static void task_trampoline_(void *arg);
  void task_();
  bool setup_i2c_();
  bool probe_(int s, int tries);  // probe, clearing the bus between tries
  bool setup_sensor_(int s);
  bool setup_capture_();
  bool write_reg_(int s, uint8_t reg, uint8_t value);
  bool read_regs_(int s, uint8_t reg, uint8_t *buf, size_t len);

  MpuConfig cfg_;
  AcqStatus status_;
  StreamResampler resampler_;
  RevolutionAnalyzer analyzer_;
  QueueHandle_t rev_queue_ = nullptr;
  TaskHandle_t task_handle_ = nullptr;
  i2c_master_bus_t *bus_[2] = {nullptr, nullptr};
  int bus_of_[2] = {0, 0};  // which bus each sensor is on
  bool new_bus_(int b, int sda, int scl, bool low_power);
  i2c_master_dev_t *dev_[2] = {nullptr, nullptr};
  mcpwm_cap_timer_t *cap_timer_ = nullptr;
  mcpwm_cap_channel_t *chan_[3] = {nullptr, nullptr, nullptr};
  uint32_t cap_resolution_ = 0;
  const char *chip_[2] = {"?", "?"};
  float g_per_count_ = 1.0f / 16384.0f;
  // Each sensor's low-pass group delay. A reading is stamped this much
  // earlier than its data-ready edge, so two different chips still line up.
  double delay_s_[2] = {0, 0};
  uint32_t delay_ticks_[2] = {0, 0};
  std::atomic<bool> invert_[2];
  std::atomic<int> axis_[2] = {};
  double ax_sum_[2][3] = {}, ax_sq_[2][3] = {};
  // Running stats for the status page.
  double sum_[2] = {0, 0}, sq_sum_[2] = {0, 0};
  float peak_[2] = {0, 0};
  uint32_t stat_n_[2] = {0, 0};
  uint32_t missed_ = 0;
  uint32_t i2c_errors_ = 0;
  uint32_t sensor_errors_[2] = {0, 0};
  uint32_t missed_s_[2] = {0, 0};
  // A sensor that fails kFailBackoff reads in a row is only tried every
  // kBackoffMs until it answers, so its timeouts can't starve the other one.
  static constexpr int kFailBackoff = 5;
  static constexpr int kBackoffMs = 200;
  int fail_streak_[2] = {0, 0};
  uint32_t next_try_ms_[2] = {0, 0};
  uint32_t backoff_skips_[2] = {0, 0};
  bool silent_logged_[2] = {false, false};

  // Recent readings of the measured axis (g, polarity applied) for the LIVE
  // screen. Written by the task only; raw_n_ counts every reading.
  float raw_[2][kRawLen] = {};
  std::atomic<uint32_t> raw_n_[2] = {};

  // Raw stream to the PC log (see set_stream).
  // Readings per log line. ESPHome cuts a message from another task at about
  // 255 characters; 6 readings at their longest (27 characters each) fit.
  static constexpr int kBatch = 6;
  struct StreamBatch {
    uint64_t t0_us = 0;
    int n = 0;
    uint32_t dt_us[kBatch];
    int16_t xyz[kBatch][3];
  };
  void stream_push_(int s, uint32_t ticks, const int16_t xyz[3]);
  std::atomic<bool> stream_{false};
  std::atomic<bool> stream_reset_{false};
  StreamBatch batch_[2];
  bool stream_ticks_init_ = false;
  uint32_t stream_last_ticks_ = 0;
  uint64_t stream_ticks64_ = 0;

  void record_(uint8_t kind, uint32_t ticks, const int16_t *xyz);
  RecRecord *rec_ = nullptr;
  size_t rec_cap_ = 0;
  std::atomic<uint64_t> rec_n_{0};
  bool rec_ticks_init_ = false;
  uint32_t rec_last_ticks_ = 0;
  uint64_t rec_ticks64_ = 0;
};

}  // namespace bal

#endif  // USE_ESP32
