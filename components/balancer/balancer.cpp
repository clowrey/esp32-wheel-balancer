#include "balancer.h"

#include <cmath>

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace balancer {

static const char *const TAG = "balancer";

void BalancerComponent::set_i2s(int port, int mclk, int bclk, int ws, int din, int sample_rate, bool use_apll) {
  acq_cfg_.i2s_port = port;
  acq_cfg_.pin_mclk = mclk;
  acq_cfg_.pin_bclk = bclk;
  acq_cfg_.pin_ws = ws;
  acq_cfg_.pin_din = din;
  acq_cfg_.sample_rate = sample_rate;
  acq_cfg_.use_apll = use_apll;
}

void BalancerComponent::set_tach(int pin, bool falling, bool pullup) {
  acq_cfg_.pin_tach = pin;
  acq_cfg_.tach_falling = falling;
  acq_cfg_.tach_pullup = pullup;
  mpu_cfg_.pin_tach = pin;
  mpu_cfg_.tach_falling = falling;
  mpu_cfg_.tach_pullup = pullup;
}

void BalancerComponent::set_mpu_bus(int sda, int scl, int frequency, int sample_rate, int filter_hz, int range_g) {
  use_mpu_ = true;
  mpu_cfg_.pin_sda = sda;
  mpu_cfg_.pin_scl = scl;
  mpu_cfg_.i2c_hz = frequency;
  mpu_cfg_.sample_rate = sample_rate;
  mpu_cfg_.filter_hz = filter_hz;
  mpu_cfg_.range_g = range_g;
  mpu_cfg_.sensors = 0;
}

void BalancerComponent::set_mpu_sensor(int ch, int address, int int_pin, int axis) {
  if (ch < 0 || ch > 1) return;
  mpu_cfg_.address[ch] = static_cast<uint8_t>(address);
  mpu_cfg_.pin_int[ch] = int_pin;
  mpu_cfg_.axis[ch] = axis;
  if (mpu_cfg_.sensors < ch + 1) mpu_cfg_.sensors = ch + 1;
}

void BalancerComponent::set_channel_default(int ch, float sens_mv_g, float gain, bool invert) {
  if (ch < 0 || ch > 1) return;
  acq_cfg_.sensitivity_mv_g[ch] = sens_mv_g;
  acq_cfg_.gain[ch] = gain;
  acq_cfg_.invert[ch] = invert;
  settings_.sens_mv_g[ch] = sens_mv_g;
  settings_.gain[ch] = gain;
  settings_.invert[ch] = invert;
}

void BalancerComponent::setup() {
  // Starting values for this kind of work, then whatever was saved on top.
  // Each preset has its own slot in flash, so switching firmware between a
  // motor rotor and a tire never mixes their settings.
  set_digital_sensor(use_mpu_);
  if (use_mpu_ && mpu_cfg_.sensors < 2) job_.set_max_planes(1);
  apply_preset(preset_);
  if (max_speed_change_ > 0) set_max_period_change(max_speed_change_);
  pref_ = global_preferences->make_preference<SavedState>(
      fnv1_hash(preset_ == bal::Preset::TIRE ? "balancer_state_tire" : "balancer_state"));
  if (pref_.load(&saved_) && saved_.magic == kStateMagic) {
    settings_ = saved_.settings;
    job_.reset(saved_.job);
    if (saved_.has_machine) load_machine(saved_.machine, saved_.machine_rpm, saved_.machine_cfg);
    ESP_LOGI(TAG, "restored settings%s", saved_.has_machine ? " and saved machine" : "");
  }
  for (int c = 0; c < 2; c++) {
    acq_cfg_.sensitivity_mv_g[c] = settings_.sens_mv_g[c];
    acq_cfg_.gain[c] = settings_.gain[c];
    acq_cfg_.invert[c] = settings_.invert[c];
    mpu_cfg_.invert[c] = settings_.invert[c];
  }
  configure_measurement_();

#ifdef USE_ESP32
  if (!simulate_) {
    if (use_mpu_) {
      mpu_ = std::make_unique<bal::MpuAcquisition>();
      acq_started_ = mpu_->start(mpu_cfg_);
    } else {
      acq_ = std::make_unique<bal::Acquisition>();
      acq_started_ = acq_->start(acq_cfg_);
    }
    if (!acq_started_) {
      ESP_LOGE(TAG, "acquisition failed to start (check the %s and tach pins)", use_mpu_ ? "sensor wiring" : "I2S");
      this->status_set_error(LOG_STR("acquisition failed"));
    }
  }
#endif
}

void BalancerComponent::on_settings_changed() {
#ifdef USE_ESP32
  for (int c = 0; c < 2; c++) {
    if (mpu_) mpu_->set_invert(c, settings_.invert[c]);
    if (acq_) acq_->set_channel(c, settings_.sens_mv_g[c], settings_.gain[c], settings_.invert[c]);
  }
#endif
}

bool BalancerComponent::acq_pop_(bal::RevResult &out) {
#ifdef USE_ESP32
  if (mpu_) return mpu_->pop(out);
  if (acq_) return acq_->pop(out);
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
  const bal::AcqStatus &st = mpu_ ? mpu_->status() : acq_->status();
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
  ESP_LOGCONFIG(TAG, "  Mode: %s", simulate_ ? "DEMO (simulated rotor)" : "sensors");
  ESP_LOGCONFIG(TAG, "  Preset: %s", preset_ == bal::Preset::TIRE ? "tire" : "motor rotor");
  ESP_LOGCONFIG(TAG, "  Tach: GPIO%d, %s edge%s", acq_cfg_.pin_tach, acq_cfg_.tach_falling ? "falling" : "rising",
                acq_cfg_.tach_pullup ? ", pull-up" : "");
  if (use_mpu_) {
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
    return;
  }
  ESP_LOGCONFIG(TAG, "  I2S%d: MCLK GPIO%d, BCLK GPIO%d, LRCK GPIO%d, DIN GPIO%d, %u Hz%s", acq_cfg_.i2s_port,
                acq_cfg_.pin_mclk, acq_cfg_.pin_bclk, acq_cfg_.pin_ws, acq_cfg_.pin_din,
                (unsigned) acq_cfg_.sample_rate, acq_cfg_.use_apll ? " (APLL)" : "");
  ESP_LOGCONFIG(TAG, "  LRCK capture: %s", acq_cfg_.pin_lrck_capture < 0 ? "on the LRCK pad" : "separate pin");
  for (int c = 0; c < 2; c++)
    ESP_LOGCONFIG(TAG, "  Channel %c: %.2f mV/g, gain %.2f%s", c == 0 ? 'A' : 'B', settings_.sens_mv_g[c],
                  settings_.gain[c], settings_.invert[c] ? ", inverted" : "");
  ESP_LOGCONFIG(TAG, "  ADC full scale %.3f V, chain delay %.1f us", acq_cfg_.adc_full_scale_v,
                acq_cfg_.rev.delay_s * 1e6);
}

}  // namespace balancer
}  // namespace esphome
