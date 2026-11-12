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
  std::atomic<float> peak_frac[2];  // largest |sample| / full scale over the last second
  std::atomic<bool> aligned{false};
  std::atomic<bool> running{false};
};

struct MpuConfig {
  int i2c_port = -1;            // -1: first free port (ESPHome's own bus has port 0)
  int pin_sda = -1, pin_scl = -1;
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
  const char *chip_name(int ch) const { return chip_[ch]; }
  double filter_delay_s(int ch) const { return delay_s_[ch]; }

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
  i2c_master_bus_t *bus_ = nullptr;
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
  // Running stats for the status page.
  double sum_[2] = {0, 0}, sq_sum_[2] = {0, 0};
  float peak_[2] = {0, 0};
  uint32_t stat_n_[2] = {0, 0};
  uint32_t missed_ = 0;
  uint32_t i2c_errors_ = 0;
};

}  // namespace bal

#endif  // USE_ESP32
