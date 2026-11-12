#include "ui.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>

namespace bal {

namespace {

// ---- look ------------------------------------------------------------------
constexpr uint32_t kBg = 0x0f1419, kPanel = 0x1b232c, kPanel2 = 0x26313d, kText = 0xe6edf3, kMuted = 0x8b98a5;
constexpr uint32_t kAccent = 0x3b82f6, kGreen = 0x22c55e, kAmber = 0xf59e0b, kRed = 0xef4444;
constexpr uint32_t kChA = 0x38bdf8, kChB = 0xfb923c;
constexpr int kW = 1024, kH = 600, kBarH = 56;

lv_color_t col(uint32_t hex) { return lv_color_hex(hex); }
const lv_font_t *F14 = &lv_font_montserrat_14;
const lv_font_t *F20 = &lv_font_montserrat_20;
const lv_font_t *F28 = &lv_font_montserrat_28;
const lv_font_t *F40 = &lv_font_montserrat_40;
const lv_font_t *F48 = &lv_font_montserrat_48;

std::string fmt(const char *f, ...) __attribute__((format(printf, 1, 2)));
std::string fmt(const char *f, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, f);
  vsnprintf(buf, sizeof(buf), f, ap);
  va_end(ap);
  return buf;
}

// ---- callbacks with captured state ------------------------------------------
struct Cb {
  std::function<void()> fn;
};
void cb_click(lv_event_t *e) {
  auto *cb = static_cast<Cb *>(lv_event_get_user_data(e));
  if (cb != nullptr && cb->fn) cb->fn();
}
void cb_free(lv_event_t *e) { delete static_cast<Cb *>(lv_event_get_user_data(e)); }
void on_click(lv_obj_t *obj, std::function<void()> fn) {
  auto *cb = new Cb{std::move(fn)};
  lv_obj_add_event_cb(obj, cb_click, LV_EVENT_CLICKED, cb);
  lv_obj_add_event_cb(obj, cb_free, LV_EVENT_DELETE, cb);
}

// ---- small builders ----------------------------------------------------------
lv_obj_t *box(lv_obj_t *parent, int x, int y, int w, int h, uint32_t bg = kPanel, int radius = 10) {
  lv_obj_t *o = lv_obj_create(parent);
  lv_obj_set_pos(o, x, y);
  lv_obj_set_size(o, w, h);
  lv_obj_set_style_bg_color(o, col(bg), 0);
  lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(o, 0, 0);
  lv_obj_set_style_radius(o, radius, 0);
  lv_obj_set_style_pad_all(o, 0, 0);
  lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
  return o;
}
lv_obj_t *clear_box(lv_obj_t *parent, int x, int y, int w, int h) {
  lv_obj_t *o = box(parent, x, y, w, h);
  lv_obj_set_style_bg_opa(o, LV_OPA_TRANSP, 0);
  return o;
}
lv_obj_t *text(lv_obj_t *parent, const char *t, const lv_font_t *font, uint32_t color, int x = 0, int y = 0) {
  lv_obj_t *l = lv_label_create(parent);
  lv_label_set_text(l, t);
  lv_obj_set_style_text_font(l, font, 0);
  lv_obj_set_style_text_color(l, col(color), 0);
  lv_obj_set_pos(l, x, y);
  return l;
}
lv_obj_t *circle(lv_obj_t *parent, int cx, int cy, int d, uint32_t fill, lv_opa_t opa = LV_OPA_COVER) {
  lv_obj_t *o = lv_obj_create(parent);
  lv_obj_set_size(o, d, d);
  lv_obj_set_pos(o, cx - d / 2, cy - d / 2);
  lv_obj_set_style_radius(o, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(o, col(fill), 0);
  lv_obj_set_style_bg_opa(o, opa, 0);
  lv_obj_set_style_border_width(o, 0, 0);
  lv_obj_set_style_pad_all(o, 0, 0);
  lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_remove_flag(o, LV_OBJ_FLAG_CLICKABLE);
  return o;
}
lv_obj_t *ring(lv_obj_t *parent, int cx, int cy, int d, uint32_t color, int width) {
  lv_obj_t *o = circle(parent, cx, cy, d, kPanel2, LV_OPA_TRANSP);
  lv_obj_set_style_border_color(o, col(color), 0);
  lv_obj_set_style_border_width(o, width, 0);
  return o;
}
lv_obj_t *line(lv_obj_t *parent, lv_point_precise_t *pts, int n, uint32_t color, int width) {
  lv_obj_t *l = lv_line_create(parent);
  lv_line_set_points(l, pts, n);
  lv_obj_set_pos(l, 0, 0);
  lv_obj_set_style_line_color(l, col(color), 0);
  lv_obj_set_style_line_width(l, width, 0);
  lv_obj_set_style_line_rounded(l, true, 0);
  lv_obj_remove_flag(l, LV_OBJ_FLAG_CLICKABLE);
  return l;
}
void set_visible(lv_obj_t *o, bool v) {
  if (o == nullptr) return;
  if (v)
    lv_obj_remove_flag(o, LV_OBJ_FLAG_HIDDEN);
  else
    lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
}
void set_enabled(lv_obj_t *o, bool en) {
  if (o == nullptr) return;
  if (en)
    lv_obj_remove_state(o, LV_STATE_DISABLED);
  else
    lv_obj_add_state(o, LV_STATE_DISABLED);
}
// Screen position of angle `deg` (clockwise from 12 o'clock) at radius r.
lv_value_precise_t px(int c, double r, double deg) {
  return static_cast<lv_value_precise_t>(std::lround(c + r * std::sin(deg / kDegPerRad)));
}
lv_value_precise_t py(int c, double r, double deg) {
  return static_cast<lv_value_precise_t>(std::lround(c - r * std::cos(deg / kDegPerRad)));
}
double nice_scale(double v) {
  if (!(v > 0)) return 1.0;
  const double e = std::pow(10.0, std::floor(std::log10(v)));
  for (double m : {1.0, 1.5, 2.0, 3.0, 5.0, 7.5, 10.0})
    if (v <= m * e) return m * e;
  return 10 * e;
}
const char *unit_of(UiModel *m) { return quantity_unit(m->settings().quantity); }
int decimals_for(double v) { return v < 0.1 ? 4 : (v < 1 ? 3 : (v < 10 ? 2 : 1)); }

}  // namespace

// ============================================================================
// DialView

void DialView::create(lv_obj_t *parent, int x, int y, int size) {
  size_ = size;
  root_ = clear_box(parent, x, y, size, size);
  const int c = size / 2;
  const double r = size / 2.0 - 22;
  lv_obj_t *disc = circle(root_, c, c, static_cast<int>(2 * r), kPanel2);
  lv_obj_set_style_border_color(disc, col(kMuted), 0);
  lv_obj_set_style_border_width(disc, 3, 0);
  for (int k = 0; k < 12; k++) {
    const double a = k * 30.0;
    const double r_in = (k % 3 == 0) ? r - 16 : r - 9;
    tick_pts_[k][0] = {px(c, r_in, a), py(c, r_in, a)};
    tick_pts_[k][1] = {px(c, r - 2, a), py(c, r - 2, a)};
    line(root_, tick_pts_[k], 2, kMuted, k % 3 == 0 ? 3 : 2);
  }
  // Angle labels just outside the rim.
  const char *names[4] = {"0", "90", "180", "270"};
  for (int k = 0; k < 4; k++) {
    lv_obj_t *l = text(root_, names[k], F14, kMuted);
    const double a = k * 90.0;
    lv_obj_update_layout(l);
    const int w = lv_obj_get_width(l), h = lv_obj_get_height(l);
    lv_obj_set_pos(l, px(c, r + 12, a) - w / 2 + (k == 0 ? 18 : 0), py(c, r + 12, a) - h / 2);
  }
  // Tach mark (reflective tape) at 0 deg.
  lv_obj_t *tape = box(root_, c - 4, c - static_cast<int>(r) - 6, 8, 14, kAccent, 2);
  (void) tape;
  // Fixed positions (created lazily up to 16).
  for (int i = 0; i < 16; i++) {
    lv_obj_t *s = circle(root_, c, c, 12, kMuted);
    lv_obj_add_flag(s, LV_OBJ_FLAG_HIDDEN);
    spots_.push_back(s);
    lv_obj_t *l = text(root_, "", F14, kText);
    lv_obj_add_flag(l, LV_OBJ_FLAG_HIDDEN);
    spot_labels_.push_back(l);
  }
  arrow_ = line(root_, arrow_pts_, 2, kAccent, 3);
  marker_pts_[0] = {static_cast<lv_value_precise_t>(c), static_cast<lv_value_precise_t>(c)};
  marker_pts_[1] = marker_pts_[0];
  marker_line_ = line(root_, marker_pts_, 2, kAmber, 4);
  marker_ = circle(root_, c, c, 26, kAmber);
  lv_obj_set_style_border_color(marker_, col(kText), 0);
  lv_obj_set_style_border_width(marker_, 2, 0);
}

void DialView::set(bool visible, double angle, bool remove, int positions, double offset, int split_a,
                   int split_b, bool rotation_ccw) {
  set_visible(marker_, visible);
  set_visible(marker_line_, visible);
  const int c = size_ / 2;
  const double r = size_ / 2.0 - 22;
  // Rotation arrow: arc outside the rim between 200 and 240 deg, head at the
  // end the rotor turns towards.
  const double a0 = rotation_ccw ? 245.0 : 205.0, a1 = rotation_ccw ? 205.0 : 245.0;
  const double ra = r + 9;
  for (int i = 0; i < 8; i++) {
    const double a = a0 + (a1 - a0) * i / 7.0;
    arrow_pts_[i] = {px(c, ra, a), py(c, ra, a)};
  }
  const double dir = rotation_ccw ? -1.0 : 1.0;  // screen direction of travel at the head
  const double ah = a1 - dir * 7.0;
  arrow_pts_[8] = {px(c, ra + 6, ah), py(c, ra + 6, ah)};
  arrow_pts_[9] = arrow_pts_[7];
  arrow_pts_[10] = {px(c, ra - 6, ah), py(c, ra - 6, ah)};
  lv_line_set_points(arrow_, arrow_pts_, 11);

  const uint32_t mcol = remove ? kRed : kAmber;
  if (visible) {
    const double rm = r * 0.72;
    marker_pts_[1] = {px(c, rm, angle), py(c, rm, angle)};
    lv_line_set_points(marker_line_, marker_pts_, 2);
    lv_obj_set_pos(marker_, static_cast<int>(marker_pts_[1].x) - 13, static_cast<int>(marker_pts_[1].y) - 13);
    lv_obj_set_style_bg_color(marker_, col(mcol), 0);
    lv_obj_set_style_line_color(marker_line_, col(mcol), 0);
  }
  const int n = std::min<int>(positions, static_cast<int>(spots_.size()));
  for (size_t i = 0; i < spots_.size(); i++) {
    const bool show = static_cast<int>(i) < n && positions >= 3;
    set_visible(spots_[i], show);
    set_visible(spot_labels_[i], false);
    if (!show) continue;
    const double a = offset + i * 360.0 / positions;
    const double rs = r * 0.9;
    lv_obj_set_pos(spots_[i], static_cast<int>(px(c, rs, a)) - 6, static_cast<int>(py(c, rs, a)) - 6);
    const bool used = visible && (static_cast<int>(i) == split_a || static_cast<int>(i) == split_b);
    lv_obj_set_style_bg_color(spots_[i], col(used ? mcol : kMuted), 0);
    if (used) {
      lv_label_set_text(spot_labels_[i], fmt("#%d", static_cast<int>(i) + 1).c_str());
      lv_obj_set_pos(spot_labels_[i], static_cast<int>(px(c, r * 0.62, a)) - 10, static_cast<int>(py(c, r * 0.62, a)) - 8);
      set_visible(spot_labels_[i], true);
    }
  }
}

// ============================================================================
// PolarView

void PolarView::create(lv_obj_t *parent, int x, int y, int size, lv_color_t color) {
  size_ = size;
  root_ = clear_box(parent, x, y, size, size);
  const int c = size / 2;
  const int r = size / 2 - 4;
  lv_obj_t *outer = ring(root_, c, c, 2 * r, kMuted, 2);
  (void) outer;
  ring(root_, c, c, r, 0x3a4654, 1);
  axis_pts_[0][0] = {static_cast<lv_value_precise_t>(c), static_cast<lv_value_precise_t>(c - r)};
  axis_pts_[0][1] = {static_cast<lv_value_precise_t>(c), static_cast<lv_value_precise_t>(c + r)};
  axis_pts_[1][0] = {static_cast<lv_value_precise_t>(c - r), static_cast<lv_value_precise_t>(c)};
  axis_pts_[1][1] = {static_cast<lv_value_precise_t>(c + r), static_cast<lv_value_precise_t>(c)};
  line(root_, axis_pts_[0], 2, 0x3a4654, 1);
  line(root_, axis_pts_[1], 2, 0x3a4654, 1);
  text(root_, "0", F14, kMuted, c + 6, 2);
  for (auto &d : dots_) {
    d = circle(root_, c, c, 8, 0);
    lv_obj_set_style_bg_color(d, color, 0);
    lv_obj_set_style_bg_opa(d, LV_OPA_50, 0);
    lv_obj_add_flag(d, LV_OBJ_FLAG_HIDDEN);
  }
  mean_pts_[0] = {static_cast<lv_value_precise_t>(c), static_cast<lv_value_precise_t>(c)};
  mean_pts_[1] = mean_pts_[0];
  mean_line_ = line(root_, mean_pts_, 2, kText, 4);
  mean_head_ = circle(root_, c, c, 14, kText);

}

void PolarView::set(const std::vector<cplx> &dots, cplx mean, bool have_mean, const char *unit) {
  double mx = have_mean ? std::abs(mean) : 0;
  for (const auto &d : dots) mx = std::max(mx, std::abs(d));
  const double scale = nice_scale(mx * 1.1);
  const int c = size_ / 2;
  const double r = size_ / 2.0 - 4;
  const int n = std::min<int>(kDots, static_cast<int>(dots.size()));
  const int start = static_cast<int>(dots.size()) - n;
  for (int i = 0; i < kDots; i++) {
    if (i >= n) {
      lv_obj_add_flag(dots_[i], LV_OBJ_FLAG_HIDDEN);
      continue;
    }
    const cplx d = dots[start + i];
    const double rr = std::min(1.0, std::abs(d) / scale) * r;
    const double a = arg_deg(d);
    lv_obj_set_pos(dots_[i], static_cast<int>(px(c, rr, a)) - 4, static_cast<int>(py(c, rr, a)) - 4);
    lv_obj_remove_flag(dots_[i], LV_OBJ_FLAG_HIDDEN);
  }
  set_visible(mean_line_, have_mean);
  set_visible(mean_head_, have_mean);
  if (have_mean) {
    const double rr = std::min(1.0, std::abs(mean) / scale) * r;
    const double a = arg_deg(mean);
    mean_pts_[1] = {px(c, rr, a), py(c, rr, a)};
    lv_line_set_points(mean_line_, mean_pts_, 2);
    lv_obj_set_pos(mean_head_, static_cast<int>(mean_pts_[1].x) - 7, static_cast<int>(mean_pts_[1].y) - 7);
  }
  if (scale_label_ != nullptr)
    lv_label_set_text(scale_label_, fmt("ring %.*f %s", decimals_for(scale), scale, unit).c_str());
}

// ============================================================================
// BalancerUi: plumbing

lv_obj_t *BalancerUi::make_screen_(Screen s, const char *title) {
  lv_obj_t *scr = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(scr, col(kBg), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_t *bar = box(scr, 0, 0, kW, kBarH, kPanel, 0);
  bar_[s].title = text(bar, title, F28, kText, 16, 12);
  bar_[s].rpm = text(bar, "", F28, kText, 0, 12);
  lv_obj_align(bar_[s].rpm, LV_ALIGN_TOP_MID, 40, 12);
  bar_[s].status = text(bar, "", F20, kMuted, 0, 17);
  lv_obj_align(bar_[s].status, LV_ALIGN_TOP_RIGHT, -16, 17);
  scr_[s] = scr;
  return scr;
}

lv_obj_t *BalancerUi::button_(lv_obj_t *parent, const char *t, const lv_font_t *font, std::function<void()> fn,
                              lv_color_t color) {
  lv_obj_t *b = lv_button_create(parent);
  lv_obj_set_style_bg_color(b, color, 0);
  lv_obj_set_style_bg_color(b, col(0x3a4654), LV_STATE_DISABLED);
  lv_obj_set_style_radius(b, 10, 0);
  lv_obj_set_style_shadow_width(b, 0, 0);
  lv_obj_t *l = lv_label_create(b);
  lv_label_set_text(l, t);
  lv_obj_set_style_text_font(l, font, 0);
  lv_obj_set_style_text_color(l, col(kText), 0);
  lv_obj_set_style_text_color(l, col(kMuted), LV_STATE_DISABLED);
  lv_obj_center(l);
  on_click(b, std::move(fn));
  return b;
}

BalancerUi::Row &BalancerUi::row_(std::vector<Row> &rows, lv_obj_t *parent, const char *name,
                                  std::function<std::string()> get, std::function<void()> on_tap) {
  Row r;
  r.button = lv_button_create(parent);
  lv_obj_set_size(r.button, lv_pct(100), 40);
  lv_obj_set_style_bg_color(r.button, col(kPanel2), 0);
  lv_obj_set_style_radius(r.button, 8, 0);
  lv_obj_set_style_shadow_width(r.button, 0, 0);
  lv_obj_set_style_pad_hor(r.button, 14, 0);
  lv_obj_t *n = lv_label_create(r.button);
  lv_label_set_text(n, name);
  lv_obj_set_style_text_font(n, F20, 0);
  lv_obj_set_style_text_color(n, col(kText), 0);
  lv_obj_align(n, LV_ALIGN_LEFT_MID, 0, 0);
  r.value = lv_label_create(r.button);
  lv_obj_set_style_text_font(r.value, F20, 0);
  lv_obj_set_style_text_color(r.value, col(kChA), 0);
  lv_obj_align(r.value, LV_ALIGN_RIGHT_MID, 0, 0);
  r.get = std::move(get);
  lv_label_set_text(r.value, r.get().c_str());
  on_click(r.button, std::move(on_tap));
  rows.push_back(std::move(r));
  return rows.back();
}

void BalancerUi::number_row_(std::vector<Row> &rows, lv_obj_t *parent, const char *name, const char *unit,
                             int decimals, std::function<double &()> ref, double lo, double hi,
                             std::function<void()> after) {
  std::string nm = name, un = unit;
  row_(
      rows, parent, name, [ref, decimals, un]() { return fmt("%.*f %s", decimals, ref(), un.c_str()); },
      [this, ref, nm, decimals, lo, hi, after, &rows]() {
        ask_number_(nm.c_str(), ref(), decimals, [this, ref, lo, hi, after, &rows](double v) {
          ref() = std::min(hi, std::max(lo, v));
          if (after) after();
          refresh_rows_(rows);
        });
      });
}

void BalancerUi::refresh_rows_(std::vector<Row> &rows) {
  for (auto &r : rows) lv_label_set_text(r.value, r.get().c_str());
  if (suggest_label_ != nullptr && model_ != nullptr) {
    const auto &cfg = model_->job().config();
    const double g = model_->job().suggested_trial_g(plane_sel_);
    const double rpm = model_->job().test_rpm() > 0 ? model_->job().test_rpm() : cfg.service_rpm;
    const std::string why = model_->job().suggested_trial_capped(plane_sel_)
                                ? fmt("(10x the ISO limit for this plane)")
                                : fmt("(10%% of rotor weight at %.0f rpm)", rpm);
    lv_label_set_text(suggest_label_, fmt("Suggested trial for plane %d: %.*f g at %.0f mm\n%s", plane_sel_ + 1,
                                          g >= 10 ? 0 : 2, g, cfg.plane[plane_sel_].trial_radius_mm, why.c_str())
                                          .c_str());
  }
}

void BalancerUi::ask_number_(const char *title, double current, int decimals, std::function<void(double)> on_ok) {
  lv_obj_t *overlay = box(lv_layer_top(), 0, 0, kW, kH, 0x000000, 0);
  lv_obj_set_style_bg_opa(overlay, LV_OPA_70, 0);
  lv_obj_add_flag(overlay, LV_OBJ_FLAG_CLICKABLE);  // swallow touches
  lv_obj_t *p = box(overlay, 212, 30, 600, 540, kPanel);
  text(p, title, F28, kText, 24, 18);
  lv_obj_t *ta = lv_textarea_create(p);
  lv_obj_set_pos(ta, 24, 70);
  lv_obj_set_size(ta, 552, 64);
  lv_textarea_set_one_line(ta, true);
  lv_textarea_set_accepted_chars(ta, "0123456789.-");
  lv_obj_set_style_text_font(ta, F28, 0);
  lv_textarea_set_text(ta, fmt("%.*f", decimals, current).c_str());
  lv_obj_t *kb = lv_keyboard_create(p);
  lv_obj_set_pos(kb, 24, 150);
  lv_obj_set_size(kb, 552, 370);
  lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_NUMBER);
  lv_keyboard_set_textarea(kb, ta);
  lv_obj_set_style_text_font(kb, F28, 0);
  struct Ctx {
    lv_obj_t *overlay, *ta;
    std::function<void(double)> ok;
  };
  auto *ctx = new Ctx{overlay, ta, std::move(on_ok)};
  lv_obj_add_event_cb(
      kb,
      [](lv_event_t *e) {
        auto *c = static_cast<Ctx *>(lv_event_get_user_data(e));
        const lv_event_code_t code = lv_event_get_code(e);
        if (code == LV_EVENT_READY) {
          const char *t = lv_textarea_get_text(c->ta);
          char *end = nullptr;
          const double v = std::strtod(t, &end);
          if (end != t && c->ok) c->ok(v);
          lv_obj_delete_async(c->overlay);
        } else if (code == LV_EVENT_CANCEL) {
          lv_obj_delete_async(c->overlay);
        }
      },
      LV_EVENT_ALL, ctx);
  lv_obj_add_event_cb(
      overlay, [](lv_event_t *e) { delete static_cast<Ctx *>(lv_event_get_user_data(e)); }, LV_EVENT_DELETE, ctx);
}

void BalancerUi::toast_(const std::string &msg) {
  lv_obj_t *t = box(lv_layer_top(), 0, 0, 800, 64, 0x111827);
  lv_obj_set_style_border_color(t, col(kAmber), 0);
  lv_obj_set_style_border_width(t, 2, 0);
  lv_obj_align(t, LV_ALIGN_BOTTOM_MID, 0, -90);
  lv_obj_t *l = text(t, msg.c_str(), F20, kText);
  lv_obj_set_width(l, 770);
  lv_label_set_long_mode(l, LV_LABEL_LONG_MODE_WRAP);
  lv_obj_center(l);
  lv_obj_delete_delayed(t, 3500);
}

void BalancerUi::show(Screen s) {
  current_ = s;
  if (s == SETUP) refresh_rows_(setup_rows_), refresh_rows_(plane_rows_);
  if (s == SETTINGS) refresh_rows_(settings_rows_);
  update();
  lv_screen_load(scr_[s]);
}

void BalancerUi::build(UiModel *model) {
  model_ = model;
  build_home_();
  build_setup_();
  build_measure_();
  build_result_();
  build_live_();
  build_settings_();
  show(HOME);
}

// ============================================================================
// Screens

void BalancerUi::build_home_() {
  lv_obj_t *s = make_screen_(HOME, "Dynamic balancer");
  struct Tile {
    const char *title, *sub;
    int x, y;
    uint32_t color;
    std::function<void()> fn;
  };
  Tile tiles[4] = {
      {"New balance", "Two-plane or static, trial weights", 24, 80, kAccent, [this]() { show(SETUP); }},
      {"Trim run", "One run with the saved machine", 524, 80, 0x1f6f4a,
       [this]() {
         if (model_->start_trim()) {
           model_->start_run();
           show(MEASURE);
         }
       }},
      {"Live vibration", "Speed, 1x, orders, waveform", 24, 310, kPanel2, [this]() { show(LIVE); }},
      {"Settings", "Sensors, units, speed window", 524, 310, kPanel2, [this]() { show(SETTINGS); }},
  };
  for (int i = 0; i < 4; i++) {
    lv_obj_t *b = button_(s, "", F14, tiles[i].fn, col(tiles[i].color));
    lv_obj_set_pos(b, tiles[i].x, tiles[i].y);
    lv_obj_set_size(b, 476, 210);
    lv_obj_t *lbl = lv_obj_get_child(b, 0);
    lv_label_set_text(lbl, tiles[i].title);
    lv_obj_set_style_text_font(lbl, F40, 0);
    lv_obj_align(lbl, LV_ALIGN_CENTER, 0, -18);
    lv_obj_t *sub = lv_label_create(b);
    lv_label_set_text(sub, tiles[i].sub);
    lv_obj_set_style_text_font(sub, F20, 0);
    lv_obj_set_style_text_color(sub, col(0xc7d2dc), 0);
    lv_obj_align(sub, LV_ALIGN_CENTER, 0, 32);
    if (i == 1) home_trim_btn_ = b;
  }
  home_info_ = text(s, "", F20, kMuted, 24, 534);
  home_demo_ = text(s, "", F14, 0xa78bfa, 24, 566);
}

void BalancerUi::build_setup_() {
  lv_obj_t *s = make_screen_(SETUP, "Rotor setup");
  JobConfig &cfg = model_->job().mutable_config();

  lv_obj_t *left = box(s, 16, 66, 488, 458);
  text(left, "ROTOR", F14, kMuted, 16, 10);
  lv_obj_t *lcol = clear_box(left, 16, 34, 456, 418);
  lv_obj_set_flex_flow(lcol, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(lcol, 5, 0);
  auto &rows = setup_rows_;
  row_(rows, lcol, "Planes", [&cfg]() { return cfg.planes == 2 ? std::string("2 (dynamic)") : std::string("1 (static)"); },
       [this, &cfg]() {
         cfg.planes = cfg.planes == 2 ? 1 : 2;
         if (cfg.planes == 1) plane_sel_ = 0;
         rebuild_plane_rows_();
         refresh_rows_(setup_rows_);
       });
  number_row_(rows, lcol, "Rotor mass", "kg", 2, [&cfg]() -> double & { return cfg.rotor_mass_kg; }, 0.01, 10000);
  number_row_(rows, lcol, "Service speed", "rpm", 0, [&cfg]() -> double & { return cfg.service_rpm; }, 10, 100000);
  row_(rows, lcol, "Balance grade", [&cfg]() { return fmt("G%g", cfg.grade); },
       [this, &cfg]() {
         const double grades[] = {1.0, 2.5, 6.3, 16.0, 40.0};
         const int n = 5;
         int i = 0;
         while (i < n && grades[i] != cfg.grade) i++;
         cfg.grade = grades[(i + 1) % n];
         refresh_rows_(setup_rows_);
       });
  row_(rows, lcol, "Angles measured",
       [&cfg]() { return std::string(cfg.angles_with_rotation ? "with rotation" : "against rotation"); },
       [this, &cfg]() {
         cfg.angles_with_rotation = !cfg.angles_with_rotation;
         refresh_rows_(setup_rows_);
       });
  row_(rows, lcol, "Last trial weight", [&cfg]() { return std::string(cfg.keep_last_trial ? "leave on" : "remove"); },
       [this, &cfg]() {
         cfg.keep_last_trial = !cfg.keep_last_trial;
         refresh_rows_(setup_rows_);
       });
  number_row_(rows, lcol, "Drill diameter", "mm", 1, [&cfg]() -> double & { return cfg.drill_diameter_mm; }, 0.5, 50);
  number_row_(rows, lcol, "Material density", "g/cm3", 2, [&cfg]() -> double & { return cfg.density_g_cm3; }, 0.5, 25);

  lv_obj_t *right = box(s, 520, 66, 488, 458);
  text(right, "PLANES", F14, kMuted, 16, 10);
  for (int p = 0; p < 2; p++) {
    plane_tab_[p] = button_(right, p == 0 ? "Plane 1" : "Plane 2", F20, [this, p]() {
      if (p == 1 && model_->job().config().planes < 2) return;
      plane_sel_ = p;
      rebuild_plane_rows_();
    }, col(kPanel2));
    lv_obj_set_pos(plane_tab_[p], 16 + p * 232, 32);
    lv_obj_set_size(plane_tab_[p], 224, 40);
  }
  plane_panel_ = clear_box(right, 16, 80, 456, 320);
  lv_obj_set_flex_flow(plane_panel_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(plane_panel_, 5, 0);
  suggest_label_ = text(right, "", F14, kMuted, 16, 408);
  rebuild_plane_rows_();

  lv_obj_t *b = button_(s, LV_SYMBOL_HOME "  Home", F20, [this]() { show(HOME); }, col(kPanel2));
  lv_obj_set_pos(b, 16, 536);
  lv_obj_set_size(b, 200, 52);
  b = button_(s, "Save as default", F20, [this]() {
    model_->save_job_defaults();
    toast_("Rotor setup saved as the default.");
  }, col(kPanel2));
  lv_obj_set_pos(b, 232, 536);
  lv_obj_set_size(b, 240, 52);
  b = button_(s, "START  " LV_SYMBOL_RIGHT, F28, [this]() { on_start_(); }, col(kGreen));
  lv_obj_set_pos(b, 788, 536);
  lv_obj_set_size(b, 220, 52);
}

void BalancerUi::rebuild_plane_rows_() {
  for (auto &r : plane_rows_) lv_obj_delete(r.button);
  plane_rows_.clear();
  JobConfig &cfg = model_->job().mutable_config();
  const int p = plane_sel_;
  for (int i = 0; i < 2; i++) {
    lv_obj_set_style_bg_color(plane_tab_[i], col(i == p ? kAccent : kPanel2), 0);
    set_enabled(plane_tab_[i], i == 0 || cfg.planes == 2);
  }
  PlaneSetup &ps = cfg.plane[p];
  auto &rows = plane_rows_;
  number_row_(rows, plane_panel_, "Trial mass", "g", 2, [&ps]() -> double & { return ps.trial_mass_g; }, 0.001, 10000,
              [this]() { refresh_rows_(plane_rows_); });
  number_row_(rows, plane_panel_, "Trial angle", "°", 0, [&ps]() -> double & { return ps.trial_angle_deg; }, -360, 720);
  number_row_(rows, plane_panel_, "Trial radius", "mm", 1, [&ps]() -> double & { return ps.trial_radius_mm; }, 1, 5000);
  number_row_(rows, plane_panel_, "Correction radius", "mm", 1, [&ps]() -> double & { return ps.corr_radius_mm; }, 1, 5000);
  row_(rows, plane_panel_, "Correction", [&ps]() { return std::string(ps.remove_mode ? "drill out" : "add weight"); },
       [this, &ps]() {
         ps.remove_mode = !ps.remove_mode;
         refresh_rows_(plane_rows_);
       });
  row_(rows, plane_panel_, "Fixed positions",
       [&ps]() { return ps.positions >= 3 ? fmt("%d spots", ps.positions) : std::string("any angle"); },
       [this, &ps]() {
         ask_number_("Fixed positions (0 = any angle)", ps.positions, 0, [this, &ps](double v) {
           const int n = static_cast<int>(std::lround(v));
           ps.positions = (n >= 3 && n <= 16) ? n : 0;
           refresh_rows_(plane_rows_);
         });
       });
  number_row_(rows, plane_panel_, "Spot #1 angle", "°", 0, [&ps]() -> double & { return ps.positions_offset_deg; },
              -360, 720);
  refresh_rows_(plane_rows_);
}

void BalancerUi::build_measure_() {
  lv_obj_t *s = make_screen_(MEASURE, "Measure");
  lv_obj_t *ip = box(s, 16, 66, 992, 86);
  m_step_ = text(ip, "", F20, kChA, 16, 8);
  m_instr_ = text(ip, "", F20, kText, 16, 36);
  lv_obj_set_width(m_instr_, 960);
  lv_label_set_long_mode(m_instr_, LV_LABEL_LONG_MODE_WRAP);

  lv_obj_t *sp = box(s, 16, 162, 330, 366);
  text(sp, "SPEED", F14, kMuted, 16, 10);
  m_rpm_ = text(sp, "--", F48, kText, 16, 28);
  m_gauge_ = lv_bar_create(sp);
  lv_obj_set_pos(m_gauge_, 16, 94);
  lv_obj_set_size(m_gauge_, 298, 26);
  lv_bar_set_range(m_gauge_, 0, 1000);
  lv_obj_set_style_bg_color(m_gauge_, col(kPanel2), 0);
  lv_obj_set_style_bg_color(m_gauge_, col(kMuted), LV_PART_INDICATOR);
  m_band_ = box(sp, 16, 90, 20, 34, kGreen, 3);
  lv_obj_set_style_bg_opa(m_band_, LV_OPA_40, 0);
  lv_obj_set_style_border_color(m_band_, col(kGreen), 0);
  lv_obj_set_style_border_width(m_band_, 2, 0);
  m_window_ = text(sp, "", F14, kMuted, 16, 128);
  m_state_ = text(sp, "", F20, kMuted, 16, 152);
  lv_obj_set_width(m_state_, 298);
  lv_label_set_long_mode(m_state_, LV_LABEL_LONG_MODE_WRAP);
  m_revs_ = text(sp, "", F20, kText, 16, 202);
  m_revs_bar_ = lv_bar_create(sp);
  lv_obj_set_pos(m_revs_bar_, 16, 232);
  lv_obj_set_size(m_revs_bar_, 298, 12);
  lv_bar_set_range(m_revs_bar_, 0, 100);
  lv_obj_set_style_bg_color(m_revs_bar_, col(kPanel2), 0);
  lv_obj_set_style_bg_color(m_revs_bar_, col(kGreen), LV_PART_INDICATOR);
  m_lock_ = button_(sp, LV_SYMBOL_GPS "  LOCK SPEED", F20, [this]() { model_->lock_speed(0); }, col(kAccent));
  lv_obj_set_pos(m_lock_, 16, 256);
  lv_obj_set_size(m_lock_, 298, 48);
  m_setspeed_ = button_(sp, "Set speed", F20, [this]() {
    const double cur = model_->job().test_rpm() > 0 ? model_->job().test_rpm() : model_->live().rpm;
    ask_number_("Test speed (rpm)", cur, 0, [this](double v) {
      if (v > 0) model_->lock_speed(v);
    });
  }, col(kPanel2));
  lv_obj_set_pos(m_setspeed_, 16, 310);
  lv_obj_set_size(m_setspeed_, 145, 44);
  m_spin_ = button_(sp, "SPIN", F20, [this]() { model_->sim_spin(); }, col(0x7c3aed));
  lv_obj_set_pos(m_spin_, 169, 310);
  lv_obj_set_size(m_spin_, 145, 44);

  const uint32_t chc[2] = {kChA, kChB};
  for (int c = 0; c < 2; c++) {
    lv_obj_t *pp = box(s, 356 + c * 332, 162, 320, 366);
    text(pp, c == 0 ? "SENSOR A (plane 1)" : "SENSOR B (plane 2)", F14, chc[c], 16, 10);
    polar_[c].create(pp, 25, 32, 270, col(chc[c]));
    lv_obj_t *sl = text(pp, "", F14, kMuted, 0, 10);
    lv_obj_align(sl, LV_ALIGN_TOP_RIGHT, -12, 10);
    polar_[c].attach_scale_label(sl);
    m_read_[c] = text(pp, "", F20, kText, 16, 308);
  }

  lv_obj_t *b = button_(s, "Cancel", F20, [this]() {
    model_->cancel_run();
    show(HOME);
  }, col(kPanel2));
  lv_obj_set_pos(b, 16, 536);
  lv_obj_set_size(b, 200, 52);
  b = button_(s, LV_SYMBOL_REFRESH "  Restart run", F20, [this]() { model_->start_run(); }, col(kPanel2));
  lv_obj_set_pos(b, 232, 536);
  lv_obj_set_size(b, 220, 52);
  m_accept_ = button_(s, LV_SYMBOL_OK "  ACCEPT", F28, [this]() { on_accept_(); }, col(kGreen));
  lv_obj_set_pos(m_accept_, 788, 536);
  lv_obj_set_size(m_accept_, 220, 52);
}

void BalancerUi::build_result_() {
  lv_obj_t *s = make_screen_(RESULT, "Correction");
  lv_obj_t *ip = box(s, 16, 66, 992, 60);
  r_instr_ = text(ip, "", F20, kText, 16, 8);
  lv_obj_set_width(r_instr_, 960);
  lv_label_set_long_mode(r_instr_, LV_LABEL_LONG_MODE_WRAP);
  for (int p = 0; p < 2; p++) {
    lv_obj_t *pp = box(s, 16 + p * 338, 136, 330, 392);
    r_title_[p] = text(pp, p == 0 ? "PLANE 1" : "PLANE 2", F14, p == 0 ? kChA : kChB, 16, 10);
    dial_[p].create(pp, 50, 24, 230);
    r_big_[p] = text(pp, "", F28, kText, 16, 258);
    lv_obj_set_width(r_big_[p], 300);
    r_detail_[p] = text(pp, "", F14, kMuted, 16, 330);
    lv_obj_set_width(r_detail_[p], 300);
    lv_label_set_long_mode(r_detail_[p], LV_LABEL_LONG_MODE_WRAP);
  }
  lv_obj_t *info = box(s, 692, 136, 316, 392);
  text(info, "DETAILS", F14, kMuted, 16, 10);
  r_info_ = text(info, "", F14, kText, 16, 34);
  lv_obj_set_width(r_info_, 286);
  lv_label_set_long_mode(r_info_, LV_LABEL_LONG_MODE_WRAP);

  lv_obj_t *b = button_(s, LV_SYMBOL_HOME "  Home", F20, [this]() { show(HOME); }, col(kPanel2));
  lv_obj_set_pos(b, 16, 536);
  lv_obj_set_size(b, 200, 52);
  b = button_(s, LV_SYMBOL_SAVE "  Save machine", F20, [this]() {
    model_->save_machine();
    toast_("Influence coefficients saved. Use Trim run next time on this machine.");
  }, col(kPanel2));
  lv_obj_set_pos(b, 232, 536);
  lv_obj_set_size(b, 250, 52);
  r_verify_ = button_(s, "VERIFY RUN  " LV_SYMBOL_RIGHT, F20, [this]() { on_verify_(); }, col(kGreen));
  lv_obj_set_pos(r_verify_, 768, 536);
  lv_obj_set_size(r_verify_, 240, 52);
}

void BalancerUi::build_live_() {
  lv_obj_t *s = make_screen_(LIVE, "Live vibration");
  lv_obj_t *lp = box(s, 16, 66, 330, 462);
  text(lp, "SPEED", F14, kMuted, 16, 10);
  l_rpm_ = text(lp, "--", F48, kText, 16, 28);
  const uint32_t chc[2] = {kChA, kChB};
  for (int c = 0; c < 2; c++) {
    text(lp, c == 0 ? "SENSOR A" : "SENSOR B", F14, chc[c], 16, 100 + c * 110);
    l_ch_[c] = text(lp, "", F20, kText, 16, 120 + c * 110);
  }
  l_diag_ = text(lp, "", F14, kMuted, 16, 330);
  lv_obj_set_width(l_diag_, 298);
  lv_label_set_long_mode(l_diag_, LV_LABEL_LONG_MODE_WRAP);

  lv_obj_t *wp = box(s, 356, 66, 652, 226);
  text(wp, "AVERAGED REVOLUTION  (tach mark at left edge)", F14, kMuted, 16, 10);
  for (int c = 0; c < 2; c++) {
    for (int i = 0; i <= kWaveBins; i++) l_wave_pts_[c][i] = {0, 0};
    l_wave_[c] = line(wp, l_wave_pts_[c], kWaveBins + 1, chc[c], 2);
  }
  // Raw readings, sharing the bottom-right slot with the orders.
  l_raw_box_ = box(s, 356, 302, 652, 226);
  text(l_raw_box_, "RAW  (mean removed)", F14, kMuted, 16, 10);
  l_raw_info_ = text(l_raw_box_, "", F14, kMuted, 200, 10);
  lv_obj_t *mid = lv_obj_create(l_raw_box_);  // zero line
  lv_obj_set_pos(mid, 16, 125);
  lv_obj_set_size(mid, 620, 1);
  lv_obj_set_style_bg_color(mid, col(kPanel2), 0);
  lv_obj_set_style_border_width(mid, 0, 0);
  for (int c = 0; c < 2; c++) {
    for (int i = 0; i < LiveData::kRawPts; i++) l_raw_pts_[c][i] = {16, 125};
    l_raw_[c] = line(l_raw_box_, l_raw_pts_[c], LiveData::kRawPts, chc[c], 1);
  }
  set_visible(l_raw_box_, false);

  lv_obj_t *op = box(s, 356, 302, 652, 226);
  l_orders_box_ = op;
  text(op, "ORDERS  1x - 10x", F14, kMuted, 16, 10);
  for (int h = 0; h < kMaxOrder; h++) {
    for (int c = 0; c < 2; c++) {
      lv_obj_t *b = lv_bar_create(op);
      lv_obj_set_pos(b, 34 + h * 61 + c * 22, 36);
      lv_obj_set_size(b, 18, 150);
      lv_bar_set_range(b, 0, 1000);
      lv_obj_set_style_bg_color(b, col(kPanel2), 0);
      lv_obj_set_style_bg_color(b, col(chc[c]), LV_PART_INDICATOR);
      lv_obj_set_style_radius(b, 3, 0);
      lv_obj_set_style_radius(b, 3, LV_PART_INDICATOR);
      l_bars_[c][h] = b;
    }
    text(op, fmt("%dx", h + 1).c_str(), F14, kMuted, 40 + h * 61, 194);
  }
  lv_obj_t *b = button_(s, LV_SYMBOL_HOME "  Home", F20, [this]() { show(HOME); }, col(kPanel2));
  lv_obj_set_pos(b, 16, 536);
  lv_obj_set_size(b, 200, 52);
  l_spin_ = button_(s, "SPIN", F20, [this]() { model_->sim_spin(); }, col(0x7c3aed));
  lv_obj_set_pos(l_spin_, 232, 536);
  lv_obj_set_size(l_spin_, 200, 52);
  l_view_btn_ = button_(
      s, "Raw readings", F20,
      [this]() {
        l_show_raw_ = !l_show_raw_;
        update_live_();
      },
      col(kPanel2));
  lv_obj_set_pos(l_view_btn_, 592, 536);
  lv_obj_set_size(l_view_btn_, 200, 52);
  l_pclog_btn_ = button_(
      s, "PC log: off", F20, [this]() { model_->set_pc_log(!model_->pc_log()); update_live_(); }, col(kPanel2));
  lv_obj_set_pos(l_pclog_btn_, 808, 536);
  lv_obj_set_size(l_pclog_btn_, 200, 52);
}

void BalancerUi::build_settings_() {
  lv_obj_t *s = make_screen_(SETTINGS, "Settings");
  lv_obj_t *left = box(s, 16, 66, 560, 458);
  lv_obj_t *colm = clear_box(left, 16, 12, 528, 440);
  lv_obj_set_flex_flow(colm, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(colm, 4, 0);
  UiSettings &st = model_->settings();
  auto &rows = settings_rows_;
  auto apply = [this]() { model_->apply_settings(false); };
  for (int c = 0; c < 2; c++) {
    row_(rows, colm, c == 0 ? "Sensor A polarity" : "Sensor B polarity",
         [&st, c]() { return std::string(st.invert[c] ? "inverted" : "normal"); },
         [this, &st, c, apply]() {
           st.invert[c] = !st.invert[c];
           apply();
           refresh_rows_(settings_rows_);
         });
  }
  row_(rows, colm, "Units", [&st]() { return std::string(quantity_unit(st.quantity)); },
       [this, &st, apply]() {
         st.quantity = static_cast<Quantity>((static_cast<int>(st.quantity) + 1) % 3);
         apply();
         refresh_rows_(settings_rows_);
       });
  row_(rows, colm, "Speed window", [&st]() { return fmt("+/- %.1f %%", st.window * 100); },
       [this, &st, apply]() {
         ask_number_("Speed window (+/- %)", st.window * 100, 1, [this, &st, apply](double v) {
           st.window = std::min(10.0, std::max(0.5, v)) / 100.0;
           apply();
           refresh_rows_(settings_rows_);
         });
       });
  row_(rows, colm, "Revolutions per run", [&st]() { return fmt("%d", st.target_revs); },
       [this, &st, apply]() {
         ask_number_("Minimum revolutions per run", st.target_revs, 0, [this, &st, apply](double v) {
           st.target_revs = static_cast<int>(std::min(1000.0, std::max(5.0, v)));
           apply();
           refresh_rows_(settings_rows_);
         });
       });
  number_row_(rows, colm, "Max phase uncertainty", "°", 1, [&st]() -> double & { return st.max_phase_unc_deg; }, 0.2, 20, apply);

  lv_obj_t *right = box(s, 592, 66, 416, 458);
  lv_obj_t *t = text(right,
                     "Sensors: digital accelerometers,\nalready calibrated in g.\n\n"
                     "Polarity: flip a sensor that is\nmounted facing the other way.\n\n"
                     "Units: what readings and the\npolar plots show. Balancing works\nin any unit.\n\n"
                     "Speed window: revolutions outside\ntarget +/- window are ignored.\n\n"
                     "Changes apply immediately.\nSAVE keeps them after a reboot.",
                     F20, kMuted, 16, 14);
  (void) t;

  lv_obj_t *b = button_(s, LV_SYMBOL_HOME "  Home", F20, [this]() { show(HOME); }, col(kPanel2));
  lv_obj_set_pos(b, 16, 536);
  lv_obj_set_size(b, 200, 52);
  b = button_(s, LV_SYMBOL_SAVE "  SAVE", F28, [this]() {
    model_->apply_settings(true);
    toast_("Settings saved.");
  }, col(kGreen));
  lv_obj_set_pos(b, 788, 536);
  lv_obj_set_size(b, 220, 52);
}

// ============================================================================
// Actions

void BalancerUi::on_start_() {
  BalanceJob &job = model_->job();
  JobConfig cfg = job.config();
  job.reset(cfg);
  job.start();
  model_->start_run();
  show(MEASURE);
}

void BalancerUi::on_accept_() {
  std::string err;
  const bool ok = model_->accept_run(&err);
  if (!ok && !err.empty()) toast_(err);
  const JobStep st = model_->job().step();
  if (st == JobStep::TRIAL1 || st == JobStep::TRIAL2) {
    model_->start_run();
    update();
  } else if (st == JobStep::RESULT || st == JobStep::VERIFIED) {
    show(RESULT);
  }
}

void BalancerUi::on_verify_() {
  model_->job().begin_verify();
  model_->start_run();
  show(MEASURE);
}

// ============================================================================
// Periodic updates

void BalancerUi::update() {
  if (model_ == nullptr) return;
  update_bar_();
  switch (current_) {
    case HOME:
      update_home_();
      break;
    case MEASURE:
      update_measure_();
      break;
    case RESULT:
      update_result_();
      break;
    case LIVE:
      update_live_();
      break;
    default:
      break;
  }
}

void BalancerUi::update_bar_() {
  const LiveData &lv = model_->live();
  Bar &b = bar_[current_];
  if (lv.rpm > 0)
    lv_label_set_text(b.rpm, fmt("%.0f rpm", lv.rpm).c_str());
  else
    lv_label_set_text(b.rpm, "-- rpm");
  std::string st;
  if (model_->simulate()) st = "DEMO  ";
  for (int c = 0; c < 2; c++) {
    st += fmt("%c %.3f g%s  ", c == 0 ? 'A' : 'B', lv.rms_g[c], lv.peak_frac[c] > 0.95f ? " CLIP" : "");
  }
  st += lv.tach_ok ? "tach " LV_SYMBOL_OK : "no tach";
  lv_label_set_text(b.status, st.c_str());
  const bool clip = lv.peak_frac[0] > 0.95f || lv.peak_frac[1] > 0.95f;
  lv_obj_set_style_text_color(b.status, col(clip ? kRed : (lv.tach_ok ? kMuted : kAmber)), 0);
}

void BalancerUi::update_home_() {
  set_enabled(home_trim_btn_, model_->has_machine());
  lv_label_set_text(home_info_, model_->machine_summary().c_str());
  const char *demo = "";
  if (model_->simulate())
    demo = "DEMO MODE: simulated wheel, no sensors. Use SPIN to spin it. Set simulate: false for real sensors.";
  lv_label_set_text(home_demo_, demo);
}

void BalancerUi::update_measure_() {
  BalanceJob &job = model_->job();
  const LiveData &lv = model_->live();
  const MeasureStats &st = model_->run_stats();
  const UiSettings &set = model_->settings();
  lv_label_set_text(m_step_, job_step_name(job.step()));
  lv_label_set_text(m_instr_, job.instruction().c_str());
  lv_label_set_text(m_rpm_, lv.rpm > 0 ? fmt("%.0f", lv.rpm).c_str() : "--");

  const double T = job.test_rpm();
  const int gx = 16, gw = 298;
  if (T > 0) {
    const double lo = T * 0.85, hi = T * 1.15;
    const double v = (lv.rpm - lo) / (hi - lo);
    lv_bar_set_value(m_gauge_, static_cast<int32_t>(std::min(1.0, std::max(0.0, v)) * 1000), LV_ANIM_OFF);
    const double w0 = (T * (1 - set.window) - lo) / (hi - lo), w1 = (T * (1 + set.window) - lo) / (hi - lo);
    lv_obj_set_pos(m_band_, gx + static_cast<int>(w0 * gw), 90);
    lv_obj_set_width(m_band_, std::max(6, static_cast<int>((w1 - w0) * gw)));
    set_visible(m_band_, true);
    lv_label_set_text(m_window_, fmt("window %.0f - %.0f rpm  (+/- %.1f%%)", T * (1 - set.window), T * (1 + set.window),
                                     set.window * 100)
                                     .c_str());
  } else {
    const double top = std::max(100.0, 2.0 * job.config().service_rpm);
    lv_bar_set_value(m_gauge_, static_cast<int32_t>(std::min(1.0, lv.rpm / top) * 1000), LV_ANIM_OFF);
    set_visible(m_band_, false);
    lv_label_set_text(m_window_, fmt("test speed not locked yet (0 - %.0f rpm)", top).c_str());
  }
  const RevStatus rs = model_->last_rev_status();
  std::string state;
  uint32_t sc = kMuted;
  if (!lv.tach_ok) {
    state = "No tach signal";
    sc = kAmber;
  } else {
    switch (rs) {
      case RevStatus::ACCEPTED:
        state = "In window: collecting";
        sc = kGreen;
        break;
      case RevStatus::ABOVE_WINDOW:
        state = "Above window: let it coast";
        sc = kAmber;
        break;
      case RevStatus::BELOW_WINDOW:
        state = "Below window: spin it up again";
        break;
      case RevStatus::SPEED_UNSTABLE:
        state = "Speed changing too fast";
        sc = kAmber;
        break;
      case RevStatus::NOT_LOCKED:
        state = "Tap LOCK SPEED at the test speed";
        sc = kChA;
        break;
      case RevStatus::FULL:
        state = "Run full";
        break;
    }
  }
  if (st.ready) {
    state = st.best_effort ? "Enough revolutions: ACCEPT, or keep going" : "Ready: tap ACCEPT";
    sc = st.best_effort ? kAmber : kGreen;
  }
  lv_label_set_text(m_state_, state.c_str());
  lv_obj_set_style_text_color(m_state_, col(sc), 0);
  lv_label_set_text(m_revs_, fmt("Revolutions  %d / %d", st.revs, set.target_revs).c_str());
  lv_bar_set_value(m_revs_bar_, std::min(100, st.revs * 100 / std::max(1, set.target_revs)), LV_ANIM_OFF);
  const bool run0 = job.step() == JobStep::RUN0;
  set_visible(m_lock_, run0 && T <= 0);
  set_visible(m_setspeed_, run0);
  set_visible(m_spin_, model_->simulate());

  const char *unit = unit_of(model_);
  for (int c = 0; c < 2; c++) {
    const ChannelStats &cs = st.ch[c];
    polar_[c].set(model_->run_dots(c), cs.mean, st.revs > 0, unit);
    if (st.revs > 0) {
      lv_label_set_text(m_read_[c], fmt("%.*f %s @ %.0f°\n+/- %.1f°   %s", decimals_for(cs.amp), cs.amp, unit,
                                        cs.phase_deg, cs.phase_unc_deg, cs.settled ? "settled" : "settling")
                                        .c_str());
      lv_obj_set_style_text_color(m_read_[c], col(cs.settled ? kText : kMuted), 0);
    } else {
      lv_label_set_text(m_read_[c], "waiting for revolutions\nin the window");
      lv_obj_set_style_text_color(m_read_[c], col(kMuted), 0);
    }
  }
  set_enabled(m_accept_, st.ready);
}

void BalancerUi::update_result_() {
  BalanceJob &job = model_->job();
  const JobConfig &cfg = job.config();
  const JobResult &r = job.result();
  const bool verified = job.step() == JobStep::VERIFIED;
  lv_label_set_text(r_instr_, job.instruction().c_str());
  lv_label_set_text(bar_[RESULT].title, verified ? "Verify result" : "Correction");
  for (int p = 0; p < 2; p++) {
    const bool used = p < cfg.planes;
    set_visible(lv_obj_get_parent(r_big_[p]), used);
    if (!used) continue;
    const PlaneResult &pr = r.plane[p];
    const PlaneSetup &ps = cfg.plane[p];
    const bool show = r.ok && (!verified || r.have_residual);
    dial_[p].set(show, pr.mass.angle_deg, pr.mass.remove, ps.positions, ps.positions_offset_deg,
                 pr.use_split ? pr.split.pos_a : -1, pr.use_split ? pr.split.pos_b : -1, !cfg.angles_with_rotation);
    if (!show) {
      lv_label_set_text(r_big_[p], "--");
      lv_label_set_text(r_detail_[p], r.error.c_str());
      continue;
    }
    const char *verb = pr.mass.remove ? "DRILL" : "ADD";
    lv_label_set_text(r_big_[p], fmt("%s %.2f g\nat %.0f°", verb, pr.mass.mass_g, pr.mass.angle_deg).c_str());
    lv_obj_set_style_text_color(r_big_[p], col(pr.mass.remove ? kRed : kAmber), 0);
    std::string d = fmt("at %.0f mm radius", ps.corr_radius_mm);
    if (pr.use_split)
      d += fmt("\nor %.2f g at #%d (%.0f°) + %.2f g at #%d (%.0f°)", pr.split.mass_a, pr.split.pos_a + 1,
               pr.split.angle_a, pr.split.mass_b, pr.split.pos_b + 1, pr.split.angle_b);
    if (pr.mass.remove)
      d += fmt("\n%.1f mm drill, %.1f mm deep (to the tip)", cfg.drill_diameter_mm, pr.drill_depth_mm);
    if (verified) d = "Trim (optional): " + d;
    lv_label_set_text(r_detail_[p], d.c_str());
  }
  std::string info;
  info += fmt("Test speed %.0f rpm\n", job.test_rpm());
  if (r.ok && !verified) {
    info += fmt("Condition number %.1f%s\n", r.condition, r.condition > 10 ? "  (planes interact: results less certain)" : "");
    for (int p = 0; p < cfg.planes; p++) {
      const TrialCheck &tc = r.trial_check[p];
      info += fmt("Trial %d effect: %.0f%% / %.0f°%s\n", p + 1, tc.amp_change_frac * 100, tc.phase_change_deg,
                  tc.ok ? "" : "  TOO SMALL: use a bigger trial weight");
    }
  }
  for (int p = 0; p < cfg.planes; p++) {
    const double lim = r.plane[p].iso_limit_gmm;
    info += fmt("Plane %d ISO G%g limit %.1f g*mm (%.2f g at %.0f mm)\n", p + 1, cfg.grade, lim,
                lim / cfg.plane[p].corr_radius_mm, cfg.plane[p].corr_radius_mm);
  }
  if (verified && r.have_residual) {
    info += "\n";
    for (int p = 0; p < cfg.planes; p++)
      info += fmt("Plane %d residual %.1f g*mm  %s\n", p + 1, r.residual_gmm[p], r.within_iso[p] ? "PASS" : "OVER LIMIT");
    info += fmt("Vibration reduced: A %.0f%%, B %.0f%%\n", r.reduction_pct[0], r.reduction_pct[1]);
  }
  lv_label_set_text(r_info_, info.c_str());
  lv_obj_t *vl = lv_obj_get_child(r_verify_, 0);
  lv_label_set_text(vl, verified ? "TRIM + VERIFY  " LV_SYMBOL_RIGHT : "VERIFY RUN  " LV_SYMBOL_RIGHT);
  set_enabled(r_verify_, r.ok || job.model().valid);
}

void BalancerUi::update_live_() {
  const LiveData &lv = model_->live();
  lv_label_set_text(l_rpm_, lv.rpm > 0 ? fmt("%.0f", lv.rpm).c_str() : "--");
  const char *unit = unit_of(model_);
  for (int c = 0; c < 2; c++) {
    const double a = std::abs(lv.v[c]);
    lv_label_set_text(l_ch_[c], fmt("1x  %.*f %s @ %.0f°\nRMS %.4f g   peak %.0f%% FS", decimals_for(a), a, unit,
                                    arg_deg(lv.v[c]), lv.rms_g[c], lv.peak_frac[c] * 100)
                                    .c_str());
  }
  lv_label_set_text(l_diag_, fmt("%s\nsample rate %.1f Hz %s\ntach pulses %u, glitches %u\nrestarts %u, missed readings %u\n"
                                 "average of last %d revolutions",
                                 lv.acq_ok ? "acquisition running"
                                           : (model_->simulate() ? "demo mode (no sensors)" : "SENSORS NOT RUNNING"),
                                 lv.fs, lv.aligned ? "(locked)" : "(not locked)", lv.tach_pulses, lv.glitches,
                                 lv.resyncs, lv.overflows, lv.revs)
                              .c_str());
  // waveform: box 652x226, drawing area x 16..636, y 40..210
  float mx = 1e-6f;
  for (int c = 0; c < 2; c++)
    for (int i = 0; i < kWaveBins; i++) mx = std::max(mx, std::fabs(lv.wave[c][i]));
  for (int c = 0; c < 2; c++) {
    for (int i = 0; i <= kWaveBins; i++) {
      const float v = lv.wave[c][i % kWaveBins];
      l_wave_pts_[c][i] = {static_cast<lv_value_precise_t>(16 + i * 620 / kWaveBins),
                           static_cast<lv_value_precise_t>(125 - v / mx * 80)};
    }
    lv_line_set_points(l_wave_[c], l_wave_pts_[c], kWaveBins + 1);
  }
  double omx = 1e-9;
  for (int c = 0; c < 2; c++)
    for (int h = 0; h < kMaxOrder; h++) omx = std::max(omx, lv.order_amp[c][h]);
  for (int c = 0; c < 2; c++)
    for (int h = 0; h < kMaxOrder; h++)
      lv_bar_set_value(l_bars_[c][h], static_cast<int32_t>(lv.order_amp[c][h] / omx * 1000), LV_ANIM_OFF);
  set_visible(l_spin_, model_->simulate());

  // Raw readings: box 652x226, drawing area x 16..636, y 40..210. Each trace
  // has its own mean removed (gravity differs with tilt) and both share one
  // scale, so their sizes compare directly.
  set_visible(l_orders_box_, !l_show_raw_);
  set_visible(l_raw_box_, l_show_raw_);
  lv_label_set_text(lv_obj_get_child(l_view_btn_, 0), l_show_raw_ ? "Orders" : "Raw readings");
  set_visible(l_pclog_btn_, !model_->simulate());
  lv_label_set_text(lv_obj_get_child(l_pclog_btn_, 0), model_->pc_log() ? "PC log: ON" : "PC log: off");
  lv_obj_set_style_bg_color(l_pclog_btn_, col(model_->pc_log() ? 0x15803d : kPanel2), 0);
  if (!l_show_raw_) return;
  const int n = std::max(lv.raw_n[0], lv.raw_n[1]);
  if (n < 2) {
    lv_label_set_text(l_raw_info_, model_->simulate() ? "no raw readings in demo mode" : "no readings yet");
    return;
  }
  float mean[2] = {0, 0}, rmx = 1e-4f;
  for (int c = 0; c < 2; c++) {
    if (lv.raw_n[c] == 0) continue;
    for (int i = 0; i < lv.raw_n[c]; i++) mean[c] += lv.raw[c][i];
    mean[c] /= lv.raw_n[c];
    for (int i = 0; i < lv.raw_n[c]; i++) rmx = std::max(rmx, std::fabs(lv.raw[c][i] - mean[c]));
  }
  const double span_s = lv.fs > 1 ? n / lv.fs : 0;
  lv_label_set_text(l_raw_info_, fmt("+/-%.1f mg, %.1f s   A %+.3f g  B %+.3f g", rmx * 1000, span_s, mean[0], mean[1])
                                     .c_str());
  for (int c = 0; c < 2; c++) {
    const int m = lv.raw_n[c];
    for (int i = 0; i < LiveData::kRawPts; i++) {
      // Right-aligned: the newest reading is at the right edge.
      const int k = i - (LiveData::kRawPts - m);
      const float v = (m > 0 && k >= 0) ? lv.raw[c][k] - mean[c] : 0.0f;
      l_raw_pts_[c][i] = {static_cast<lv_value_precise_t>(16 + i * 620 / (LiveData::kRawPts - 1)),
                          static_cast<lv_value_precise_t>(125 - v / rmx * 80)};
    }
    lv_line_set_points(l_raw_[c], l_raw_pts_[c], LiveData::kRawPts);
  }
}

}  // namespace bal
