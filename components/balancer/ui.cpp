// The UI's engine: one frame (touch in, changed rows out), the top bar, the pop-ups and the
// actions the screens share. The screens themselves are in ui_screens.cpp.
#include "ui.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace bal {

// A finger that moves further than this from where it came down is dragging, not tapping
static const int DRAG_START = 12;

// ============================================================================
// The frame

void BalancerUi::show(Screen s) { current_ = s; }

void BalancerUi::tick_(uint32_t now) {
  now_ms_ = now;
  if (!toast_.empty() && static_cast<int32_t>(now - toast_until_) >= 0) toast_.clear();
  if (det_ch_ >= 0 && det_running_) {
    const LiveData &lv = model_->live();
    for (int k = 0; k < 3; k++) det_peak_[k] = std::max(det_peak_[k], lv.axis_rms[det_ch_][k]);
    if (static_cast<int32_t>(now - det_end_ms_) >= 0) finish_detect_();
  }
  if (current_ == MEASURE) {
    // The first run locks the test speed by itself once it is held steady: say so once.
    const double T = model_->job().test_rpm();
    if (T > 0 && m_prev_test_rpm_ <= 0 && model_->measuring() && model_->auto_lock_hi() > 0)
      toast_msg_(fmt("Speed locked at %.0f rpm. Keep it within +/-%.0f%% for every run.", T,
                     model_->settings().window * 100));
    m_prev_test_rpm_ = T;
  }
}

void BalancerUi::draw_(Frame &f) {
  f.c.reset_clip();
  f.c.clear(kBg);
  // Under a pop-up the screen is drawn but does not react
  const bool popup = pad_.open;
  f.active = !popup;
  switch (current_) {
    case HOME: home_(f); break;
    case SETUP: setup_(f); break;
    case MEASURE: measure_(f); break;
    case RESULT: result_(f); break;
    case LIVE: live_(f); break;
    case SETTINGS: settings_(f); break;
    default: break;
  }
  if (pad_.open) {
    f.active = true;
    number_pad_(f);
  }
  f.active = false;
  this->draw_toast_(f);
}

bool BalancerUi::frame(Canvas &c, const Input &input, uint32_t now_ms, HitTrace *trace) {
  if (model_ == nullptr) return false;
  this->tick_(now_ms);
  const bool pressed = input.down && !was_down_, released = !input.down && was_down_;
  if (pressed) {
    press_x_ = input.x;
    press_y_ = input.y;
    dragging_ = false;
  } else if (input.down && (std::abs(input.x - press_x_) > DRAG_START || std::abs(input.y - press_y_) > DRAG_START)) {
    dragging_ = true;
  }
  if (input.down) {
    last_x_ = input.x;
    last_y_ = input.y;
  }

  // First a dry run: the widgets see the touch and react, and what the frame would look like
  // is summed up without drawing it.
  auto make = [&](HitTrace *record) {
    Frame f{c, now_ms};
    f.down = input.down;
    f.dragging = dragging_;
    f.tx = last_x_;
    f.ty = last_y_;
    f.press_x = press_x_;
    f.press_y = press_y_;
    f.trace = record;
    if (record != nullptr) record->count = 0;
    return f;
  };
  Frame f = make(trace);
  f.released = released;
  c.begin_dry();
  this->draw_(f);
  const uint32_t *sums = c.end_dry();
  if (f.changed) {
    // Something was tapped: the frame to show is the one after it, without the tap in it
    Frame after = make(trace);
    c.begin_dry();
    this->draw_(after);
    sums = c.end_dry();
  }
  was_down_ = input.down;

  // Which bands would look different: runs of them, a run taking in a gap of one band
  const int bands = c.bands();
  redrawn_count = 0;
  for (int band = 0; band < bands; band++) {
    if (drawn_ && sums[band] == drawn_sums_[band]) continue;
    const int y0 = band * Canvas::BAND_ROWS, y1 = std::min(c.height(), y0 + Canvas::BAND_ROWS);
    if (redrawn_count > 0 && (y0 - redrawn[redrawn_count - 1].y1 <= Canvas::BAND_ROWS || redrawn_count == MAX_RUNS)) {
      redrawn[redrawn_count - 1].y1 = y1;
    } else {
      redrawn[redrawn_count++] = {y0, y1};
    }
    drawn_sums_[band] = sums[band];
  }
  drawn_ = true;
  for (int i = 0; i < redrawn_count; i++) {
    c.set_window(redrawn[i].y0, redrawn[i].y1);
    Frame draw = make(nullptr);
    this->draw_(draw);
  }
  c.clear_window();
  return redrawn_count > 0;
}

// ============================================================================
// Shared pieces

void BalancerUi::bar_(Frame &f, const char *title) {
  Canvas &c = f.c;
  const LiveData &lv = model_->live();
  c.fill(0, 0, SCREEN_W, 56, kPanel);
  text_mid(c, FONT_M28, 16, 28, title, kText);
  text_mid(c, FONT_M28, 552, 28, lv.rpm > 0 ? fmt("%.0f rpm", lv.rpm) : "-- rpm", kText, CENTER);
  std::string st;
  if (model_->simulate()) st = "DEMO  ";
  for (int ch = 0; ch < 2; ch++)
    st += fmt("%c %.3f g%s  ", ch == 0 ? 'A' : 'B', lv.rms_g[ch], lv.peak_frac[ch] > 0.95f ? " CLIP" : "");
  st += lv.tach_ok ? "tach ok" : "no tach";
  const bool clip = lv.peak_frac[0] > 0.95f || lv.peak_frac[1] > 0.95f;
  text_mid(c, FONT_R20, SCREEN_W - 16, 28, st.c_str(), clip ? kRed : (lv.tach_ok ? kMuted : kAmber), RIGHT);
}

void BalancerUi::draw_toast_(Frame &f) {
  if (toast_.empty()) return;
  Canvas &c = f.c;
  const int w = 800, h = 64, x = (SCREEN_W - w) / 2, y = SCREEN_H - 90 - h;
  c.round_rect(x, y, w, h, 10, 0x111827);
  c.round_frame(x, y, w, h, 10, 2, kAmber);
  const int lines = Canvas::text_width(FONT_R20, toast_.c_str()) > w - 30 ? 2 : 1;
  text_block(c, FONT_R20, x + w / 2, y + h / 2 - lines * FONT_R20.line_height() / 2, w - 30, toast_.c_str(), kText, 2,
             0, CENTER);
}

// Balance weights in the unit chosen on the settings screen. Computation stays in grams;
// ounces are only for showing and typing.
static constexpr double kGramsPerOz = 28.349523125;

std::string BalancerUi::mass_(double g) const {
  if (model_ != nullptr && model_->settings().mass_oz) return fmt("%.2f oz", g / kGramsPerOz);
  return fmt("%.*f g", g >= 10 ? 1 : 2, g);
}

// The nearest stack of 1 oz and 1/4 oz stick-on weights, e.g. "1 oz + 2 x 1/4 oz".
std::string BalancerUi::mass_pieces_(double g) const {
  if (model_ == nullptr || !model_->settings().mass_oz) return "";
  const int q = static_cast<int>(std::lround(g / kGramsPerOz * 4));
  if (q <= 0) return "less than 1/8 oz: none needed";
  const int ones = q / 4, quarters = q % 4;
  std::string s;
  if (ones > 0) s += fmt("%d oz", ones);
  if (quarters > 0) s += fmt("%s%d x 1/4 oz", s.empty() ? "" : " + ", quarters);
  return s + fmt(" (%.2f oz)", q / 4.0);
}

bool BalancerUi::number_row_(Frame &f, int x, int y, int w, const char *name, const char *unit, int decimals,
                             double &value, double lo, double hi) {
  if (!setting_row(f, x, y, w, name, fmt("%.*f %s", decimals, value, unit))) return false;
  double *target = &value;
  ask_number_(name, value, decimals, [target, lo, hi](double v) { *target = std::min(hi, std::max(lo, v)); });
  return true;
}

// ============================================================================
// Number pad

void BalancerUi::ask_number_(const char *title, double current, int decimals, std::function<void(double)> on_ok) {
  pad_.open = true;
  pad_.fresh = true;
  pad_.title = title;
  snprintf(pad_.entry, sizeof(pad_.entry), "%.*f", decimals, current);
  pad_.ok = std::move(on_ok);
}

void BalancerUi::number_pad_(Frame &f) {
  Canvas &c = f.c;
  c.fill_blend(0, 0, SCREEN_W, SCREEN_H, 0x000000, 178);
  const int px = 212, py = 30, pw = 600, ph = 540;
  panel(c, px, py, pw, ph, kPanel);
  text_mid(c, FONT_M28, px + 24, py + 36, pad_.title.c_str(), kText);
  c.round_rect(px + 24, py + 70, pw - 48, 64, 8, kPanel2);
  c.round_frame(px + 24, py + 70, pw - 48, 64, 8, 2, pad_.fresh ? kGrid : kAccent);
  text_mid(c, FONT_M28, px + pw - 44, py + 102, pad_.entry, kText, RIGHT);

  static const char *const kKeys[4][4] = {
      {"1", "2", "3", "Del"}, {"4", "5", "6", "-"}, {"7", "8", "9", "."}, {"Cancel", "0", "OK", ""}};
  const int kx = px + 24, ky = py + 150, kw = 132, kh = 84, gap = 8;
  for (int r = 0; r < 4; r++) {
    for (int k = 0; k < 4; k++) {
      const char *key = kKeys[r][k];
      if (!key[0]) continue;
      const bool ok = strcmp(key, "OK") == 0;
      const int w = ok ? 2 * kw + gap : kw;
      const Rgb fill = ok ? kGreen : (r == 3 && k == 0 ? kGrid : kPanel2);
      if (!button(f, kx + k * (kw + gap), ky + r * (kh + gap), w, kh, key, fill, true, FONT_M28)) continue;
      char *e = pad_.entry;
      size_t len = strlen(e);
      if (ok) {
        char *end = nullptr;
        const double v = std::strtod(e, &end);
        if (end != e && pad_.ok) pad_.ok(v);
        pad_.open = false;
      } else if (strcmp(key, "Cancel") == 0) {
        pad_.open = false;
      } else if (strcmp(key, "Del") == 0) {
        if (pad_.fresh) {
          e[0] = 0;
        } else if (len > 0) {
          e[len - 1] = 0;
        }
      } else if (strcmp(key, "-") == 0) {
        if (pad_.fresh) {
          strcpy(e, "-");
        } else if (e[0] == '-') {
          memmove(e, e + 1, len);
        } else if (len + 1 < sizeof(pad_.entry)) {
          memmove(e + 1, e, len + 1);
          e[0] = '-';
        }
      } else if (strcmp(key, ".") == 0) {
        if (pad_.fresh) {
          strcpy(e, "0.");
        } else if (strchr(e, '.') == nullptr && len + 2 < sizeof(pad_.entry)) {
          strcat(e, len == 0 || (len == 1 && e[0] == '-') ? "0." : ".");
        }
      } else if (pad_.fresh) {
        strcpy(e, key);
      } else if (len + 1 < sizeof(pad_.entry)) {
        strcat(e, key);
      }
      pad_.fresh = false;
    }
  }
}

// ============================================================================
// Actions

void BalancerUi::on_start_() {
  BalanceJob &job = model_->job();
  JobConfig cfg = job.config();
  job.reset(cfg);
  job.start();
  model_->start_run();
  m_prev_test_rpm_ = 0;
  show(MEASURE);
}

void BalancerUi::on_accept_() {
  std::string err;
  const bool ok = model_->accept_run(&err);
  if (!ok && !err.empty()) toast_msg_(err);
  const JobStep st = model_->job().step();
  if (st == JobStep::TRIAL1 || st == JobStep::TRIAL2) {
    model_->start_run();
  } else if (st == JobStep::RESULT || st == JobStep::VERIFIED) {
    show(RESULT);
  }
}

void BalancerUi::on_verify_() {
  model_->job().begin_verify();
  model_->start_run();
  m_prev_test_rpm_ = 0;
  show(MEASURE);
}

}  // namespace bal
