#pragma once
// ESPHome glue: owns acquisition, persistence and the UI; the balancing
// behaviour itself lives in bal::ModelCore (ui_core.h).

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

#include <memory>

#include "acq_capture.h"
#include "acq_mpu.h"
#include "ui.h"
#include "ui_core.h"

namespace esphome {
namespace balancer {

// Everything kept in flash. Bump kStateMagic when the layout changes.
struct SavedState {
  uint32_t magic;
  bal::UiSettings settings;
  bal::JobConfig job;
  uint8_t has_machine;
  bal::InfluenceModel machine;
  double machine_rpm;
  bal::JobConfig machine_cfg;
};
static constexpr uint32_t kStateMagic = 0xBA1A0001;

class BalancerComponent : public Component, public bal::ModelCore {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  // --- YAML configuration ---
  void set_i2s(int port, int mclk, int bclk, int ws, int din, int sample_rate, bool use_apll);
  void set_lrck_capture_pin(int pin) { acq_cfg_.pin_lrck_capture = pin; }
  void set_tach(int pin, bool falling, bool pullup);
  void set_adc_full_scale(float volts) { acq_cfg_.adc_full_scale_v = volts; }
  void set_channel_default(int ch, float sens_mv_g, float gain, bool invert);
  void set_swap_channels(bool swap) { acq_cfg_.swap_channels = swap; }
  void set_adc_delay_us(float us) { acq_cfg_.rev.delay_s = us * 1e-6; }
  void set_task_core(int core) {
    acq_cfg_.task_core = core;
    mpu_cfg_.task_core = core;
  }
  // Digital sensors (sensor_type: mpu6050) in place of the PCM1808.
  void set_mpu_bus(int sda, int scl, int frequency, int sample_rate, int filter_hz, int range_g);
  void set_mpu_sensor(int ch, int address, int int_pin, int axis);
  void set_preset(int preset) { preset_ = static_cast<bal::Preset>(preset); }
  void set_max_speed_change(float fraction) { max_speed_change_ = fraction; }
  void set_rpm_sensor(sensor::Sensor *s) { rpm_sensor_ = s; }
  void set_amp_sensor_a(sensor::Sensor *s) { amp_sensor_[0] = s; }
  void set_amp_sensor_b(sensor::Sensor *s) { amp_sensor_[1] = s; }
  void set_phase_sensor_a(sensor::Sensor *s) { phase_sensor_[0] = s; }
  void set_phase_sensor_b(sensor::Sensor *s) { phase_sensor_[1] = s; }

 protected:
  void persist_settings() override { save_(); }
  void persist_job_defaults() override { save_(); }
  void persist_machine() override { save_(); }
  void on_settings_changed() override;

 private:
  void save_();
  void update_acq_status_();
  void publish_sensors_();

  bool acq_pop_(bal::RevResult &out);

  bal::AcqConfig acq_cfg_;
  bal::MpuConfig mpu_cfg_;
  bool use_mpu_ = false;
  bal::Preset preset_ = bal::Preset::MOTOR_ROTOR;
  float max_speed_change_ = -1;  // < 0: keep the preset's value
#ifdef USE_ESP32
  // Only the acquisition that the YAML selects is created.
  std::unique_ptr<bal::Acquisition> acq_;
  std::unique_ptr<bal::MpuAcquisition> mpu_;
#endif
  bal::BalancerUi ui_;
  bal::RevResult rev_buf_;
  ESPPreferenceObject pref_;
  SavedState saved_{};
  bool acq_started_ = false;
  uint32_t last_ui_ms_ = 0;
  uint32_t last_pub_ms_ = 0;
  sensor::Sensor *rpm_sensor_{nullptr};
  sensor::Sensor *amp_sensor_[2]{nullptr, nullptr};
  sensor::Sensor *phase_sensor_[2]{nullptr, nullptr};
};

}  // namespace balancer
}  // namespace esphome
