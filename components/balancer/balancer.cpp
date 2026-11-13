#include "balancer.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#ifdef USE_WIFI
#include "esphome/components/wifi/wifi_component.h"
#endif
#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif
#ifdef USE_ESP32
#include "esphome/components/web_server_base/web_server_base.h"
#include <esp_http_server.h>
#endif

#ifdef USE_ESP32
#include "driver/gpio.h"
#endif

namespace esphome {
namespace balancer {

static const char *const TAG = "balancer";

#ifdef USE_ESP32
// GET /recording.bin: the recorder's raw readings, oldest first, for
// tools/mpu_recording.py. A 64-byte header, then 12-byte records
// (MpuAcquisition::RecRecord). Sent in chunks straight from PSRAM.
class RecordingHandler : public AsyncWebHandler {
 public:
  explicit RecordingHandler(bal::MpuAcquisition *acq) : acq_(acq) {}
  bool canHandle(AsyncWebServerRequest *request) const override {
    if (request->method() != HTTP_GET) return false;
    char buf[AsyncWebServerRequest::URL_BUF_SIZE];
    const auto url = request->url_to(buf);
    return url == "/recording.bin";
  }
  void handleRequest(AsyncWebServerRequest *request) override {
    httpd_req_t *req = *request;
    uint64_t first = 0;
    const size_t count = acq_->recorder_snapshot(&first);
    const bal::MpuConfig &cfg = acq_->config();
    struct __attribute__((packed)) Header {
      char magic[4];
      uint32_t header_len, record_size, count, sample_rate, range_g, capture_hz, sensors;
      uint8_t axis_a, axis_b, filter_hz, reserved0;
      uint8_t reserved[28];
    } hd = {};
    static_assert(sizeof(Header) == 64, "recording header");
    memcpy(hd.magic, "WBR1", 4);
    hd.header_len = sizeof(Header);
    hd.record_size = sizeof(bal::MpuAcquisition::RecRecord);
    hd.count = static_cast<uint32_t>(count);
    hd.sample_rate = cfg.sample_rate;
    hd.range_g = static_cast<uint32_t>(cfg.range_g);
    hd.capture_hz = acq_->capture_hz();
    hd.sensors = static_cast<uint32_t>(cfg.sensors);
    hd.axis_a = static_cast<uint8_t>(cfg.axis[0]);
    hd.axis_b = static_cast<uint8_t>(cfg.axis[1]);
    hd.filter_hz = static_cast<uint8_t>(cfg.filter_hz);
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"recording.bin\"");
    if (httpd_resp_send_chunk(req, reinterpret_cast<const char *>(&hd), sizeof(hd)) != ESP_OK) return;
    static constexpr size_t kChunk = 680;  // records per chunk (~8 KB)
    std::unique_ptr<bal::MpuAcquisition::RecRecord[]> buf(new bal::MpuAcquisition::RecRecord[kChunk]);
    for (size_t i = 0; i < count; i += kChunk) {
      const size_t n = std::min(kChunk, count - i);
      for (size_t k = 0; k < n; k++) buf[k] = acq_->recorder_at(first + i + k);
      if (httpd_resp_send_chunk(req, reinterpret_cast<const char *>(buf.get()), n * sizeof(buf[0])) != ESP_OK) return;
    }
    httpd_resp_send_chunk(req, nullptr, 0);
    ESP_LOGI(TAG, "recording sent: %u readings", (unsigned) count);
  }

 protected:
  bal::MpuAcquisition *acq_;
};
#endif

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
  settings_.axis[ch] = axis;
  settings_.invert[ch] = invert;
  if (mpu_cfg_.sensors < ch + 1) mpu_cfg_.sensors = ch + 1;
}

// Longest a teach can be held: a dropped touch release must not leave the
// sensor stuck in teach mode.
static constexpr uint32_t kTeachMaxMs = 20000;

void BalancerComponent::set_tach_teach(bool on) {
  if (teach_pin_ < 0 || on == teach_on_ || (on && teach_timed_out_)) return;
#ifdef USE_ESP32
  gpio_set_level(static_cast<gpio_num_t>(teach_pin_), on ? 1 : 0);
#endif
  teach_on_ = on;
  teach_since_ms_ = millis();
  ESP_LOGI(TAG, "tach teach input %s", on ? "ON" : "off");
}

void BalancerComponent::setup() {
#ifdef USE_ESP32
  if (teach_pin_ >= 0) {
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << teach_pin_;
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_down_en = GPIO_PULLDOWN_ENABLE;  // transistor stays off while the pin floats at boot
    gpio_set_level(static_cast<gpio_num_t>(teach_pin_), 0);
    gpio_config(&io);
    gpio_set_level(static_cast<gpio_num_t>(teach_pin_), 0);
  }
#endif
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
  for (int c = 0; c < 2; c++) {
    mpu_cfg_.invert[c] = settings_.invert[c];
    if (settings_.axis[c] >= 0 && settings_.axis[c] <= 2) mpu_cfg_.axis[c] = settings_.axis[c];
  }
  configure_measurement_();

#ifdef USE_ESP32
  if (!simulate_) {
    mpu_ = std::make_unique<bal::MpuAcquisition>();
    acq_started_ = mpu_->start(mpu_cfg_);
    if (!acq_started_) {
      ESP_LOGE(TAG, "acquisition failed to start (check the sensor wiring and tach pin)");
      this->status_set_error(LOG_STR("acquisition failed"));
    } else {
      if (pc_log_at_boot_) set_pc_log(true);
      if (record_minutes_ > 0 && mpu_->set_recorder_minutes(record_minutes_) &&
          web_server_base::global_web_server_base != nullptr) {
        web_server_base::global_web_server_base->init();
        web_server_base::global_web_server_base->add_handler(new RecordingHandler(mpu_.get()));  // NOLINT
      }
    }
  }
#endif

  // The screen: a frame in PSRAM that the UI draws into
  RAMAllocator<uint16_t> allocator(RAMAllocator<uint16_t>::ALLOC_EXTERNAL);
  frame_ = allocator.allocate(static_cast<size_t>(bal::SCREEN_W) * bal::SCREEN_H);
  if (frame_ == nullptr) {
    ESP_LOGE(TAG, "no memory for the screen's frame");
  } else {
    memset(frame_, 0, static_cast<size_t>(bal::SCREEN_W) * bal::SCREEN_H * sizeof(uint16_t));
    ui_.bind(this);
    if (touchscreen_ != nullptr) touchscreen_->register_listener(this);
  }
  started_ms_ = millis();
}

// ---- touch ----

void BalancerComponent::touch(touchscreen::TouchPoint tp) {
  last_touch_ms_ = millis();
  teach_timed_out_ = false;
  input_.down = true;
  input_.x = tp.x;
  input_.y = tp.y;
  down_seen_ = false;
  release_waiting_ = false;
  touch_event_ = true;
}

void BalancerComponent::update(const touchscreen::TouchPoints_t &points) {
  if (points.empty() || !input_.down) return;
  last_touch_ms_ = millis();
  // Until a frame has seen the finger down, keep where it came down: that is
  // where the press starts, and a move from there is a drag, not a tap.
  if (!down_seen_) return;
  if (input_.x != points[0].x || input_.y != points[0].y) {
    input_.x = points[0].x;
    input_.y = points[0].y;
    touch_event_ = true;
  }
}

void BalancerComponent::release() {
  last_touch_ms_ = millis();
  teach_timed_out_ = false;
  if (!down_seen_) {
    // No frame has seen the finger down yet: lift it after the next one
    release_waiting_ = true;
  } else {
    input_.down = false;
  }
  touch_event_ = true;
}

// ---- frames ----

// With a finger on the panel a frame is drawn this often; otherwise a few times a second
static constexpr uint32_t kTouchFrameMs = 30, kQuietFrameMs = 200, kAfterTouchMs = 400;

void BalancerComponent::draw_frame_(uint32_t now) {
  const uint32_t t0 = micros();
  update_acq_status_();
  bal::Canvas canvas(frame_, bal::SCREEN_W, bal::SCREEN_H);
  // While the panel may still be starting, every frame is drawn and sent whole
  if (now - started_ms_ < 3000) ui_.redraw();
  const bool drew = ui_.frame(canvas, input_, now);
  const uint32_t t1 = micros();
  if (drew) {
    for (int i = 0; i < ui_.redrawn_count; i++) send_rows_(ui_.redrawn[i].y0, ui_.redrawn[i].y1);
    const uint32_t t2 = micros();
    draw_us_ = t1 - t0;
    push_us_ = t2 - t1;
    worst_us_ = std::max(worst_us_, t2 - t0);
    frames_drawn_++;
  }
  if (now - last_report_ms_ >= 60000) {
    ESP_LOGD(TAG, "screen: %u frames drawn in %u s, %u rows sent; last: draw %u us, send %u us; slowest %u us",
             (unsigned) frames_drawn_, (unsigned) ((now - last_report_ms_) / 1000), (unsigned) rows_sent_,
             (unsigned) draw_us_, (unsigned) push_us_, (unsigned) worst_us_);
    last_report_ms_ = now;
    frames_drawn_ = rows_sent_ = worst_us_ = 0;
  }
}

void BalancerComponent::send_rows_(int y0, int y1) {
  if (display_ == nullptr) return;
  // The frame is RGB565 in this chip's byte order, which is what the panel's buffer holds
  display_->draw_pixels_at(0, y0, bal::SCREEN_W, y1 - y0,
                           reinterpret_cast<const uint8_t *>(frame_ + static_cast<size_t>(y0) * bal::SCREEN_W),
                           display::COLOR_ORDER_RGB, display::COLOR_BITNESS_565, false, 0, 0, 0);
  rows_sent_ += y1 - y0;
}

void BalancerComponent::on_settings_changed() {
#ifdef USE_ESP32
  for (int c = 0; c < 2; c++) {
    if (!mpu_) continue;
    mpu_->set_invert(c, settings_.invert[c]);
    mpu_->set_axis(c, settings_.axis[c]);
    mpu_cfg_.axis[c] = settings_.axis[c];  // for the config dump
  }
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
    live_.read_errors[c] = st.read_errors[c].load();
    for (int k = 0; k < 3; k++) {
      live_.axis_mean[c][k] = st.axis_mean[c][k].load();
      live_.axis_rms[c][k] = st.axis_rms[c][k].load();
    }
    live_.missed[c] = st.missed[c].load();
    live_.raw_n[c] = mpu_->copy_raw(c, live_.raw[c], bal::LiveData::kRawPts);
  }
#endif
}

std::string BalancerComponent::network_status() const { return net_text_; }

// The SSID and signal come from the Wi-Fi co-processor: a blocking request over
// SDIO that can stall for seconds when the link is busy (asked 5x a second it
// froze the loop until the task watchdog reset the board). So the text is made
// here, at most every 10 s and only while the settings screen shows it, and the
// UI only ever reads the finished string (the same within a frame).
void BalancerComponent::refresh_network_(uint32_t now) {
  if (ui_.current() != bal::BalancerUi::SETTINGS) return;
  if (net_valid_ && now - net_asked_ms_ < 10000) return;
  net_valid_ = true;
  net_asked_ms_ = now;
  std::string out;
#ifdef USE_WIFI
  auto *w = wifi::global_wifi_component;
  if (w != nullptr && w->is_connected()) {
    char ssid[wifi::SSID_BUFFER_SIZE];
    w->wifi_ssid_to(ssid);
    out += str_sprintf("Wi-Fi: connected to %s, %d dBm\n", ssid, (int) w->wifi_rssi());
    std::string ips;
    for (auto &ip : w->get_ip_addresses()) {
      if (!ip.is_set() || !ip.is_ip4()) continue;
      char buf[network::IP_ADDRESS_BUFFER_SIZE];
      if (!ips.empty()) ips += ", ";
      ips += ip.str_to(buf);
    }
    out += "IP: " + (ips.empty() ? std::string("none yet") : ips) + "\n";
  } else {
    out += "Wi-Fi: NOT connected\nIP: none\n";
  }
#else
  out += "Wi-Fi: not in this firmware\n";
#endif
#ifdef USE_API
  const bool ha = api::global_api_server != nullptr && api::global_api_server->is_connected();
  out += ha ? "Home Assistant: connected" : "Home Assistant: not connected";
#endif
  net_text_ = out;
}

bool BalancerComponent::pc_log() const {
#ifdef USE_ESP32
  if (mpu_) return mpu_->streaming();
#endif
  return false;
}

void BalancerComponent::set_pc_log(bool on) {
#ifdef USE_ESP32
  if (mpu_) {
    mpu_->set_stream(on);
    ESP_LOGI(TAG, "raw stream to the PC log %s", on ? "on (tools/mpu_log.py)" : "off");
  }
#endif
}

void BalancerComponent::loop() {
  const uint32_t now = millis();
  if (acq_started_) {
    for (int i = 0; i < 8 && acq_pop_(rev_buf_); i++) feed_rev(rev_buf_, now);
  }
  tick(now);
  if (teach_on_ && now - teach_since_ms_ > kTeachMaxMs) {
    ESP_LOGW(TAG, "tach teach held for %u s: releasing it", (unsigned) (kTeachMaxMs / 1000));
    set_tach_teach(false);
    teach_timed_out_ = true;
  }

  if (frame_ != nullptr) {
    const bool busy = input_.down || now - last_touch_ms_ < kAfterTouchMs;
    if (touch_event_ || now - last_frame_ms_ >= (busy ? kTouchFrameMs : kQuietFrameMs)) {
      touch_event_ = false;
      last_frame_ms_ = now;
      refresh_network_(now);
      draw_frame_(now);
      if (input_.down) down_seen_ = true;
      if (release_waiting_) {
        // A tap shorter than a frame: the frame just drawn saw the finger down;
        // the release goes in at once, before another touch can replace it.
        release_waiting_ = false;
        input_.down = false;
        draw_frame_(now);
      }
    }
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
  if (teach_pin_ >= 0) ESP_LOGCONFIG(TAG, "  Tach teach output: GPIO%d (high = teach)", teach_pin_);
  static const char kAxis[] = "XYZ";
  ESP_LOGCONFIG(TAG, "  Sensors: MPU-6050 family on I2C, SDA GPIO%d, SCL GPIO%d, %u Hz", mpu_cfg_.pin_sda,
                mpu_cfg_.pin_scl, (unsigned) mpu_cfg_.i2c_hz);
  if (mpu_cfg_.sensors > 1 && mpu_cfg_.pin_sda_b >= 0)
    ESP_LOGCONFIG(TAG, "  Sensor B on its own bus (LP I2C): SDA GPIO%d, SCL GPIO%d", mpu_cfg_.pin_sda_b,
                  mpu_cfg_.pin_scl_b);
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
