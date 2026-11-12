#include "balancer.h"

#include <cmath>

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace balancer {

static const char *const TAG = "balancer";

void BalancerComponent::set_tach(int pin, bool falling, bool pullup) {
  mpu_cfg_.pin_tach = pin;
  mpu_cfg_.tach_falling = falling;
  mpu_cfg_.tach_pullup = pullup;
}

void BalancerComponent::set_mpu_bus(int sda, int scl, int frequency, int sample_rate, int filter_hz, int range_g) {
  mpu_cfg_.pin_sda = sda;
  mpu_cfg_.pin_scl = scl;
  mpu_cfg_.i2c_hz = frequency;
  mpu_cfg_.sample_rate = sample_rate;
  mpu_cfg_.filter_hz = filter_hz;
  mpu_cfg_.range_g = range_g;
  mpu_cfg_.sensors = 0;
}

void BalancerComponent::set_mpu_sensor(int ch, int address, int int_pin, int axis, bool invert) {
  if (ch < 0 || ch > 1) return;
  mpu_cfg_.address[ch] = static_cast<uint8_t>(address);
  mpu_cfg_.pin_int[ch] = int_pin;
  mpu_cfg_.axis[ch] = axis;
  settings_.invert[ch] = invert;
  if (mpu_cfg_.sensors < ch + 1) mpu_cfg_.sensors = ch + 1;
}

void BalancerComponent::setup() {
  // Starting values (ModelCore), then whatever was saved on top.
  if (mpu_cfg_.sensors < 2) job_.set_max_planes(1);
  if (max_speed_change_ > 0) {
    set_max_period_change(max_speed_change_);
    configure_measurement_();
  }
  pref_ = global_preferences->make_preference<SavedState>(fnv1_hash("balancer_state_tire"));
  if (pref_.load(&saved_) && saved_.magic == kStateMagic) {
    settings_ = saved_.settings;
    job_.reset(saved_.job);
    if (saved_.has_machine) load_machine(saved_.machine, saved_.machine_rpm, saved_.machine_cfg);
    ESP_LOGI(TAG, "restored settings%s", saved_.has_machine ? " and saved machine" : "");
  }
  for (int c = 0; c < 2; c++) mpu_cfg_.invert[c] = settings_.invert[c];
  configure_measurement_();

#ifdef USE_ESP32
  if (!simulate_) {
    mpu_ = std::make_unique<bal::MpuAcquisition>();
    acq_started_ = mpu_->start(mpu_cfg_);
    if (!acq_started_) {
      ESP_LOGE(TAG, "acquisition failed to start (check the sensor wiring and tach pin)");
      this->status_set_error(LOG_STR("acquisition failed"));
    }
  }
#endif
}

void BalancerComponent::on_settings_changed() {
#ifdef USE_ESP32
  for (int c = 0; c < 2; c++)
    if (mpu_) mpu_->set_invert(c, settings_.invert[c]);
#endif
}

bool BalancerComponent::acq_pop_(bal::RevResult &out) {
#ifdef USE_ESP32
  if (mpu_) return mpu_->pop(out);
#endif
  return false;
}

void BalancerComponent::save_() {
  saved_.magic = kStateMagic;
  saved_.settings = settings_;
  saved_.job = job_.config();
  saved_.has_machine = has_machine_ ? 1 : 0;
  saved_.machine = machine_;
  saved_.machine_rpm = machine_rpm_;
  saved_.machine_cfg = machine_cfg_;
  if (pref_.save(&saved_) && global_preferences->sync())
    ESP_LOGI(TAG, "state saved");
  else
    ESP_LOGW(TAG, "saving state failed");
}

void BalancerComponent::update_acq_status_() {
#ifdef USE_ESP32
  if (!acq_started_) return;
  const bal::AcqStatus &st = mpu_->status();
  live_.acq_ok = st.running.load();
  live_.aligned = st.aligned.load();
  live_.fs = st.measured_fs.load();
  live_.resyncs = st.resyncs.load();
  live_.overflows = st.dma_overflows.load();
  live_.glitches = st.glitches.load();
  live_.tach_pulses = st.tach_pulses.load();
  for (int c = 0; c < 2; c++) {
    live_.rms_g[c] = st.rms_g[c].load();
    live_.peak_frac[c] = st.peak_frac[c].load();
  }
#endif
}

void BalancerComponent::loop() {
  const uint32_t now = millis();
  if (acq_started_) {
    for (int i = 0; i < 8 && acq_pop_(rev_buf_); i++) feed_rev(rev_buf_, now);
  }
  tick(now);

  if (!ui_.built() && lv_display_get_default() != nullptr) {
    ui_.build(this);
    ESP_LOGI(TAG, "UI ready");
  }
  if (ui_.built() && now - last_ui_ms_ >= 200) {
    last_ui_ms_ = now;
    update_acq_status_();
    ui_.update();
  }
  if (now - last_pub_ms_ >= 1000) {
    last_pub_ms_ = now;
    publish_sensors_();
  }
}

void BalancerComponent::publish_sensors_() {
  const bal::LiveData &lv = live_;
  if (rpm_sensor_ != nullptr) rpm_sensor_->publish_state(lv.rpm);
  for (int c = 0; c < 2; c++) {
    const bool have = lv.tach_ok && lv.revs > 0;
    if (amp_sensor_[c] != nullptr) amp_sensor_[c]->publish_state(have ? std::abs(lv.v[c]) : NAN);
    if (phase_sensor_[c] != nullptr) phase_sensor_[c]->publish_state(have ? bal::arg_deg(lv.v[c]) : NAN);
  }
}

void BalancerComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Balancer:");
  ESP_LOGCONFIG(TAG, "  Mode: %s", simulate_ ? "DEMO (simulated wheel)" : "sensors");
  ESP_LOGCONFIG(TAG, "  Tach: GPIO%d, %s edge%s", mpu_cfg_.pin_tach, mpu_cfg_.tach_falling ? "falling" : "rising",
                mpu_cfg_.tach_pullup ? ", pull-up" : "");
  static const char kAxis[] = "XYZ";
  ESP_LOGCONFIG(TAG, "  Sensors: MPU-6050 family on I2C, SDA GPIO%d, SCL GPIO%d, %u Hz", mpu_cfg_.pin_sda,
                mpu_cfg_.pin_scl, (unsigned) mpu_cfg_.i2c_hz);
  ESP_LOGCONFIG(TAG, "  Readings: %u Hz, %d Hz low-pass, +/-%d g", (unsigned) mpu_cfg_.sample_rate, mpu_cfg_.filter_hz,
                mpu_cfg_.range_g);
  for (int c = 0; c < mpu_cfg_.sensors; c++) {
#ifdef USE_ESP32
    const char *chip = mpu_ ? mpu_->chip_name(c) : "not started";
#else
    const char *chip = "";
#endif
    ESP_LOGCONFIG(TAG, "  Sensor %c: address 0x%02X, INT GPIO%d, %c axis%s, %s", c == 0 ? 'A' : 'B',
                  mpu_cfg_.address[c], mpu_cfg_.pin_int[c], kAxis[mpu_cfg_.axis[c]],
                  settings_.invert[c] ? ", inverted" : "", chip);
  }
  if (mpu_cfg_.sensors < 2) ESP_LOGCONFIG(TAG, "  One sensor: single-plane balancing only");
}

}  // namespace balancer
}  // namespace esphome
