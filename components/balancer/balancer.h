#pragma once
// ESPHome glue: owns acquisition, persistence and the UI; the balancing
// behaviour itself lives in bal::ModelCore (ui_core.h). The UI (ui.h) draws
// into a frame in PSRAM; the rows that changed are sent to the display, and
// the touch panel's state is handed to the UI each frame.

#include "esphome/components/display/display.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/touchscreen/touchscreen.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

#include <memory>

#include "acq_mpu.h"
#include "ui.h"
#include "ui_core.h"

namespace esphome {
namespace balancer {

// Everything kept in flash (NVS); it survives reboots and firmware updates.
// Changing this layout (any struct inside it) makes older saved data fail to
// load and the person's calibrated settings fall back to defaults, so don't
// change it casually: add a migration from the previous layout instead.
struct SavedState {
  uint32_t magic;
  bal::UiSettings settings;
  bal::JobConfig job;
  uint8_t has_machine;
  bal::InfluenceModel machine;
  double machine_rpm;
  bal::JobConfig machine_cfg;
};
static constexpr uint32_t kStateMagic = 0xBA1A0004;  // 3: UiSettings.axis[]; 4: mass_oz

class BalancerComponent : public Component, public bal::ModelCore, public touchscreen::TouchListener {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  // --- YAML configuration ---
  void set_display(display::Display *d) { display_ = d; }
  void set_touchscreen(touchscreen::Touchscreen *t) { touchscreen_ = t; }
  void set_tach(int pin, bool falling, bool pullup);
  void set_task_core(int core) { mpu_cfg_.task_core = core; }
  void set_mpu_bus(int sda, int scl, int frequency, int sample_rate, int filter_hz, int range_g);
  void set_mpu_sensor(int ch, int address, int int_pin, int axis, bool invert);
  void set_mpu_bus_b(int sda, int scl) {
    mpu_cfg_.pin_sda_b = sda;
    mpu_cfg_.pin_scl_b = scl;
  }
  void set_max_speed_change(float fraction) { max_speed_change_ = fraction; }
  void set_rpm_sensor(sensor::Sensor *s) { rpm_sensor_ = s; }
  void set_amp_sensor_a(sensor::Sensor *s) { amp_sensor_[0] = s; }
  void set_amp_sensor_b(sensor::Sensor *s) { amp_sensor_[1] = s; }
  void set_phase_sensor_a(sensor::Sensor *s) { phase_sensor_[0] = s; }
  void set_phase_sensor_b(sensor::Sensor *s) { phase_sensor_[1] = s; }
  void set_pc_log_at_boot(bool on) { pc_log_at_boot_ = on; }
  void set_weight_unit_oz(bool oz) { settings_.mass_oz = oz; }
  void set_record_minutes(float m) { record_minutes_ = m; }
  void set_tach_teach_pin(int pin) { teach_pin_ = pin; }

  // --- UiModel ---
  std::string network_status() const override;
  bool pc_log() const override;
  void set_pc_log(bool on) override;
  bool has_tach_teach() const override { return teach_pin_ >= 0; }
  void set_tach_teach(bool on) override;
  bool tach_teach() const override { return teach_on_; }

  // --- the touch panel (touchscreen::TouchListener) ---
  void touch(touchscreen::TouchPoint tp) override;
  void update(const touchscreen::TouchPoints_t &points) override;
  void release() override;

 protected:
  void persist_settings() override { save_(); }
  void persist_job_defaults() override { save_(); }
  void persist_machine() override { save_(); }
  void on_settings_changed() override;

 private:
  void save_();
  void update_acq_status_();
  void refresh_network_(uint32_t now);
  void publish_sensors_();

  bool acq_pop_(bal::RevResult &out);
  void draw_frame_(uint32_t now);
  void send_rows_(int y0, int y1);

  display::Display *display_{nullptr};
  touchscreen::Touchscreen *touchscreen_{nullptr};
  uint16_t *frame_{nullptr};  // RGB565, SCREEN_W x SCREEN_H, in PSRAM
  // The touch panel as the next frame will see it. A finger that came down and
  // went up again between two frames is still seen: down for one frame, then up.
  bal::Input input_;
  bool touch_event_{false}, down_seen_{true}, release_waiting_{false};
  uint32_t last_frame_ms_{0}, last_touch_ms_{0}, last_report_ms_{0}, started_ms_{0};
  uint32_t draw_us_{0}, push_us_{0}, worst_us_{0}, frames_drawn_{0}, rows_sent_{0};

  bal::MpuConfig mpu_cfg_;
  float max_speed_change_ = -1;  // < 0: keep the default
#ifdef USE_ESP32
  std::unique_ptr<bal::MpuAcquisition> mpu_;  // not created in demo mode
#endif
  bal::BalancerUi ui_;
  bal::RevResult rev_buf_;
  ESPPreferenceObject pref_;
  SavedState saved_{};
  bool acq_started_ = false;
  bool pc_log_at_boot_ = false;
  float record_minutes_ = 0;  // raw recorder length in PSRAM (0: off)
  // What network_status() returns: made in loop() at most every 10 s, and only
  // while the settings screen shows it (see refresh_network_).
  std::string net_text_ = "Wi-Fi: checking...";
  bool net_valid_ = false;
  uint32_t net_asked_ms_ = 0;
  int teach_pin_ = -1;        // drives the tach sensor's teach input through a transistor (high = teach)
  bool teach_on_ = false;
  // The 20 s limit released the teach input: it stays off until the finger is
  // lifted or comes down again, so the UI cannot switch it straight back on.
  bool teach_timed_out_ = false;
  uint32_t teach_since_ms_ = 0;
  uint32_t last_pub_ms_ = 0;
  sensor::Sensor *rpm_sensor_{nullptr};
  sensor::Sensor *amp_sensor_[2]{nullptr, nullptr};
  sensor::Sensor *phase_sensor_[2]{nullptr, nullptr};
};

}  // namespace balancer
}  // namespace esphome
