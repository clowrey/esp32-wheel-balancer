// The screens. Each one runs top to bottom every frame: it draws from the model and reacts to
// taps on what it draws (ui_kit.h). Layout is in screen pixels, 1024x600.
#include <algorithm>
#include <cmath>
#include <vector>

#include "ui.h"

namespace bal {

namespace {

constexpr int kBtnY = 536, kBtnH = 52;  // the row of buttons along the bottom

// A point at `deg` (clockwise from 12 o'clock) and radius r around (cx, cy)
float px(float cx, double r, double deg) { return static_cast<float>(cx + r * std::sin(deg / kDegPerRad)); }
float py(float cy, double r, double deg) { return static_cast<float>(cy - r * std::cos(deg / kDegPerRad)); }

double nice_scale(double v) {
  if (!(v > 0)) return 1.0;
  const double e = std::pow(10.0, std::floor(std::log10(v)));
  for (double m : {1.0, 1.5, 2.0, 3.0, 5.0, 7.5, 10.0})
    if (v <= m * e) return m * e;
  return 10 * e;
}
int decimals_for(double v) { return v < 0.1 ? 4 : (v < 1 ? 3 : (v < 10 ? 2 : 1)); }

// Polar plot of the 1x vector: recent revolutions as dots, their mean as a needle. Returns
// the value the outer ring stands for.
double polar_plot(Canvas &c, int x, int y, int size, Rgb color, const std::vector<cplx> &dots, cplx mean,
                  bool have_mean) {
  double mx = have_mean ? std::abs(mean) : 0;
  for (const auto &d : dots) mx = std::max(mx, std::abs(d));
  const double scale = nice_scale(mx * 1.1);
  const float cx = x + size / 2.0f, cy = y + size / 2.0f;
  const float r = size / 2.0f - 4;
  c.ring(cx, cy, r - 1, r + 1, 0, 360, kMuted);
  c.ring(cx, cy, r / 2 - 0.5f, r / 2 + 0.5f, 0, 360, kGrid);
  c.line(cx, cy - r, cx, cy + r, 1, kGrid);
  c.line(cx - r, cy, cx + r, cy, 1, kGrid);
  c.text(FONT_R14, static_cast<int>(cx) + 6, y + 2, "0", kMuted);
  const int n = std::min<int>(40, static_cast<int>(dots.size()));
  for (int i = static_cast<int>(dots.size()) - n; i < static_cast<int>(dots.size()); i++) {
    const double rr = std::min(1.0, std::abs(dots[i]) / scale) * r;
    const double a = arg_deg(dots[i]);
    c.disc(px(cx, rr, a), py(cy, rr, a), 4, color, 128);  // half opacity: clusters show brighter
  }
  if (have_mean) {
    const double rr = std::min(1.0, std::abs(mean) / scale) * r;
    const double a = arg_deg(mean);
    c.line(cx, cy, px(cx, rr, a), py(cy, rr, a), 4, kText);
    c.disc(px(cx, rr, a), py(cy, rr, a), 7, kText);
  }
  return scale;
}

// Rotor dial: tach mark at the top, angles growing clockwise on screen, the way the rotor turns,
// the fixed positions and where the correction weight goes.
void dial(Canvas &c, int x, int y, int size, bool visible, double angle, bool remove, int positions, double offset,
          int split_a, int split_b, bool rotation_ccw) {
  const float cx = x + size / 2.0f, cy = y + size / 2.0f;
  const float r = size / 2.0f - 22;
  c.disc(cx, cy, r, kPanel2);
  c.ring(cx, cy, r - 3, r, 0, 360, kMuted);
  for (int k = 0; k < 12; k++) {
    const double a = k * 30.0;
    const double r_in = (k % 3 == 0) ? r - 16 : r - 9;
    c.line(px(cx, r_in, a), py(cy, r_in, a), px(cx, r - 2, a), py(cy, r - 2, a), k % 3 == 0 ? 3 : 2, kMuted);
  }
  static const char *const kNames[4] = {"0", "90", "180", "270"};
  for (int k = 0; k < 4; k++) {
    const double a = k * 90.0;
    text_mid(c, FONT_R14, static_cast<int>(px(cx, r + 12, a)) + (k == 0 ? 18 : 0), static_cast<int>(py(cy, r + 12, a)),
             kNames[k], kMuted, CENTER);
  }
  // Tach mark (the reflective tape) at 0 degrees
  c.round_rect(static_cast<int>(cx) - 4, static_cast<int>(cy - r) - 6, 8, 14, 2, kAccent);
  // Which way the rotor turns: an arc outside the rim between 205 and 245 degrees, the head at
  // the end it turns towards
  const double a0 = rotation_ccw ? 245.0 : 205.0, a1 = rotation_ccw ? 205.0 : 245.0;
  const double ra = r + 9;
  float ax[8], ay[8];
  for (int i = 0; i < 8; i++) {
    const double a = a0 + (a1 - a0) * i / 7.0;
    ax[i] = px(cx, ra, a);
    ay[i] = py(cy, ra, a);
  }
  polyline(c, ax, ay, 8, 3, kAccent);
  const double ah = a1 - (rotation_ccw ? -1.0 : 1.0) * 7.0;
  c.line(px(cx, ra + 6, ah), py(cy, ra + 6, ah), ax[7], ay[7], 3, kAccent);
  c.line(ax[7], ay[7], px(cx, ra - 6, ah), py(cy, ra - 6, ah), 3, kAccent);

  const Rgb mcol = remove ? kRed : kAmber;
  if (positions >= 3) {
    const int n = std::min(positions, 16);
    for (int i = 0; i < n; i++) {
      const double a = offset + i * 360.0 / positions;
      const bool used = visible && (i == split_a || i == split_b);
      c.disc(px(cx, r * 0.9, a), py(cy, r * 0.9, a), 6, used ? mcol : kMuted);
      if (used)
        text_mid(c, FONT_R14, static_cast<int>(px(cx, r * 0.62, a)), static_cast<int>(py(cy, r * 0.62, a)),
                 fmt("#%d", i + 1), kText, CENTER);
    }
  }
  if (visible) {
    const float mx = px(cx, r * 0.72, angle), my = py(cy, r * 0.72, angle);
    c.line(cx, cy, mx, my, 4, mcol);
    c.disc(mx, my, 13, kText);
    c.disc(mx, my, 11, mcol);
  }
}

}  // namespace

// ============================================================================
// Home

void BalancerUi::home_(Frame &f) {
  Canvas &c = f.c;
  bar_(f, "Dynamic balancer");
  if (tile(f, 24, 80, 476, 210, "New balance", "Two-plane or static, trial weights", kAccent)) show(SETUP);
  if (tile(f, 524, 80, 476, 210, "Trim run", "One run with the saved machine", 0x1f6f4a, model_->has_machine()) &&
      model_->start_trim()) {
    model_->start_run();
    m_prev_test_rpm_ = 0;
    show(MEASURE);
  }
  if (tile(f, 24, 310, 476, 210, "Live vibration", "Speed, 1x, orders, waveform", kPanel2)) show(LIVE);
  if (tile(f, 524, 310, 476, 210, "Settings", "Sensors, units, speed window", kPanel2)) show(SETTINGS);
  text_mid(c, FONT_R20, 24, 548, model_->machine_summary().c_str(), kMuted);
  if (model_->simulate())
    text_mid(c, FONT_R16, 24, 578,
             "DEMO MODE: simulated wheel, no sensors. Use SPIN to spin it. Set simulate: false for real sensors.",
             0xa78bfa);
}

// ============================================================================
// Rotor setup

void BalancerUi::setup_(Frame &f) {
  Canvas &c = f.c;
  bar_(f, "Rotor setup");
  JobConfig &cfg = model_->job().mutable_config();
  const int step = ROW_H + 5;

  panel(c, 16, 66, 488, 458);
  c.text(FONT_R16, 32, 74, "ROTOR", kMuted);
  int x = 32, y = 100, w = 456;
  if (setting_row(f, x, y, w, "Planes", cfg.planes == 2 ? "2 (dynamic)" : "1 (static)")) {
    cfg.planes = cfg.planes == 2 ? 1 : 2;
    if (cfg.planes == 1) plane_sel_ = 0;
  }
  y += step;
  number_row_(f, x, y, w, "Rotor mass", "kg", 2, cfg.rotor_mass_kg, 0.01, 10000);
  y += step;
  number_row_(f, x, y, w, "Service speed", "rpm", 0, cfg.service_rpm, 10, 100000);
  y += step;
  if (setting_row(f, x, y, w, "Balance grade", fmt("G%g", cfg.grade))) {
    const double grades[] = {1.0, 2.5, 6.3, 16.0, 40.0};
    int i = 0;
    while (i < 5 && grades[i] != cfg.grade) i++;
    cfg.grade = grades[(i + 1) % 5];
  }
  y += step;
  if (setting_row(f, x, y, w, "Angles measured", cfg.angles_with_rotation ? "with rotation" : "against rotation"))
    cfg.angles_with_rotation = !cfg.angles_with_rotation;
  y += step;
  if (setting_row(f, x, y, w, "Last trial weight", cfg.keep_last_trial ? "leave on" : "remove"))
    cfg.keep_last_trial = !cfg.keep_last_trial;
  y += step;
  number_row_(f, x, y, w, "Drill diameter", "mm", 1, cfg.drill_diameter_mm, 0.5, 50);
  y += step;
  number_row_(f, x, y, w, "Material density", "g/cm3", 2, cfg.density_g_cm3, 0.5, 25);

  panel(c, 520, 66, 488, 458);
  c.text(FONT_R16, 536, 74, "PLANES", kMuted);
  if (cfg.planes < 2) plane_sel_ = 0;
  for (int p = 0; p < 2; p++) {
    if (button(f, 536 + p * 232, 98, 224, 40, p == 0 ? "Plane 1" : "Plane 2", p == plane_sel_ ? kAccent : kPanel2,
               p == 0 || cfg.planes == 2))
      plane_sel_ = p;
  }
  PlaneSetup &ps = cfg.plane[plane_sel_];
  PlaneSetup *pp = &ps;
  x = 536;
  y = 146;
  const bool oz = model_->settings().mass_oz;
  if (setting_row(f, x, y, w, "Trial mass", mass_(ps.trial_mass_g).c_str())) {
    ask_number_(oz ? "Trial mass (oz)" : "Trial mass (g)", oz ? ps.trial_mass_g / 28.349523125 : ps.trial_mass_g, 2,
                [pp, oz](double v) { pp->trial_mass_g = std::min(10000.0, std::max(0.001, oz ? v * 28.349523125 : v)); });
  }
  y += step;
  number_row_(f, x, y, w, "Trial angle", "°", 0, ps.trial_angle_deg, -360, 720);
  y += step;
  number_row_(f, x, y, w, "Trial radius", "mm", 1, ps.trial_radius_mm, 1, 5000);
  y += step;
  number_row_(f, x, y, w, "Correction radius", "mm", 1, ps.corr_radius_mm, 1, 5000);
  y += step;
  if (setting_row(f, x, y, w, "Correction", ps.remove_mode ? "drill out" : "add weight")) ps.remove_mode = !ps.remove_mode;
  y += step;
  if (setting_row(f, x, y, w, "Fixed positions", ps.positions >= 3 ? fmt("%d spots", ps.positions) : "any angle")) {
    ask_number_("Fixed positions (0 = any angle)", ps.positions, 0, [pp](double v) {
      const int n = static_cast<int>(std::lround(v));
      pp->positions = (n >= 3 && n <= 16) ? n : 0;
    });
  }
  y += step;
  number_row_(f, x, y, w, "Spot #1 angle", "°", 0, ps.positions_offset_deg, -360, 720);

  // What trial weight to use
  const double g = model_->job().suggested_trial_g(plane_sel_);
  const double rpm = model_->job().test_rpm() > 0 ? model_->job().test_rpm() : cfg.service_rpm;
  const std::string why = model_->job().suggested_trial_capped(plane_sel_)
                              ? std::string("(10x the ISO limit for this plane)")
                              : std::string(fmt("(10%% of rotor weight at %.0f rpm)", rpm));
  const std::string pieces = mass_pieces_(g);
  text_block(c, FONT_R16, 536, 470, w,
             fmt("Suggested trial for plane %d: %s at %.0f mm%s%s\n%s", plane_sel_ + 1, mass_(g).c_str(),
                 ps.trial_radius_mm, pieces.empty() ? "" : ", about ", pieces.c_str(), why.c_str()),
             kMuted, 3);

  if (button(f, 16, kBtnY, 200, kBtnH, "Home")) show(HOME);
  if (button(f, 232, kBtnY, 240, kBtnH, "Save as default")) {
    model_->save_job_defaults();
    toast_msg_("Rotor setup saved as the default.");
  }
  if (button_next(f, 788, kBtnY, 220, kBtnH, "START", kGreen, true, FONT_M28)) on_start_();
}

// ============================================================================
// Measure

void BalancerUi::measure_(Frame &f) {
  Canvas &c = f.c;
  bar_(f, "Measure");
  BalanceJob &job = model_->job();
  const LiveData &lv = model_->live();
  const MeasureStats &st = model_->run_stats();
  const UiSettings &set = model_->settings();

  panel(c, 16, 66, 992, 86);
  c.text(FONT_R20, 32, 72, job_step_name(job.step()), kChA);
  text_block(c, FONT_R20, 32, 98, 960, job.instruction().c_str(), kText, 2);

  // Speed
  panel(c, 16, 162, 330, 366);
  c.text(FONT_R16, 32, 170, "SPEED", kMuted);
  c.text(FONT_M56, 30, 186, lv.rpm > 0 ? fmt("%.0f", lv.rpm) : "--", kText);
  const double T = job.test_rpm();
  const int gx = 32, gy = 258, gw = 298, gh = 26;
  const char *window;
  if (T > 0) {
    const double lo = T * 0.85, hi = T * 1.15;
    hbar(c, gx, gy, gw, gh, static_cast<float>((lv.rpm - lo) / (hi - lo)), kMuted, kPanel2, 4);
    const double w0 = (T * (1 - set.window) - lo) / (hi - lo), w1 = (T * (1 + set.window) - lo) / (hi - lo);
    const int bx = gx + static_cast<int>(w0 * gw), bw = std::max(6, static_cast<int>((w1 - w0) * gw));
    c.fill_blend(bx, gy - 4, bw, gh + 8, kGreen, 102);
    c.round_frame(bx, gy - 4, bw, gh + 8, 3, 2, kGreen);
    window = fmt("window %.0f - %.0f rpm  (+/- %.1f%%)", T * (1 - set.window), T * (1 + set.window), set.window * 100);
  } else {
    const double top = std::max(100.0, 2.0 * job.config().service_rpm);
    hbar(c, gx, gy, gw, gh, static_cast<float>(lv.rpm / top), kMuted, kPanel2, 4);
    window = fmt("test speed not locked yet (0 - %.0f rpm)", top);
  }
  c.text(FONT_R16, gx, 292, window, kMuted);

  const RevStatus rs = model_->last_rev_status();
  std::string state;
  Rgb sc = kMuted;
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
        state = "Above window: slow down or let it coast";
        sc = kAmber;
        break;
      case RevStatus::BELOW_WINDOW:
        state = "Below window: speed up";
        break;
      case RevStatus::SPEED_UNSTABLE:
        state = "Speed changing too fast";
        sc = kAmber;
        break;
      case RevStatus::NOT_LOCKED:
        state = model_->auto_lock_hi() > 0
                    ? fmt("Hold %.0f-%.0f rpm steady: locks by itself", model_->auto_lock_lo(), model_->auto_lock_hi())
                    : "Tap LOCK SPEED at the test speed";
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
  text_block(c, FONT_R20, gx, 314, gw, state.c_str(), sc, 2);
  c.text(FONT_R20, gx, 364, fmt("Revolutions  %d / %d", st.revs, set.target_revs), kText);
  hbar(c, gx, 394, gw, 12, static_cast<float>(st.revs) / std::max(1, set.target_revs), kGreen, kPanel2, 6);
  const bool run0 = job.step() == JobStep::RUN0;
  if (run0 && T <= 0 && button(f, gx, 418, gw, 48, "LOCK SPEED", kAccent)) model_->lock_speed(0);
  if (run0 && button(f, gx, 472, 145, 44, "Set speed")) {
    UiModel *m = model_;
    ask_number_("Test speed (rpm)", T > 0 ? T : lv.rpm, 0, [m](double v) {
      if (v > 0) m->lock_speed(v);
    });
  }
  if (model_->simulate() && button(f, gx + 153, 472, 145, 44, "SPIN", kPurple)) model_->sim_spin();

  // One polar plot per sensor
  const char *unit = quantity_unit(set.quantity);
  for (int ch = 0; ch < 2; ch++) {
    const int x0 = 356 + ch * 332, y0 = 162;
    const Rgb color = ch == 0 ? kChA : kChB;
    panel(c, x0, y0, 320, 366);
    c.text(FONT_R16, x0 + 16, y0 + 8, ch == 0 ? "SENSOR A (plane 1)" : "SENSOR B (plane 2)", color);
    const ChannelStats &cs = st.ch[ch];
    const double scale = polar_plot(c, x0 + 25, y0 + 32, 270, color, model_->run_dots(ch), cs.mean, st.revs > 0);
    c.text(FONT_R16, x0 + 308, y0 + 8, fmt("ring %.*f %s", decimals_for(scale), scale, unit), kMuted, RIGHT);
    if (st.revs > 0) {
      text_block(c, FONT_R20, x0 + 16, y0 + 308, 0,
                 fmt("%.*f %s @ %.0f°\n+/- %.1f°   %s", decimals_for(cs.amp), cs.amp, unit, cs.phase_deg,
                     cs.phase_unc_deg, cs.settled ? "settled" : "settling"),
                 cs.settled ? kText : kMuted, 2);
    } else {
      text_block(c, FONT_R20, x0 + 16, y0 + 308, 0, "waiting for revolutions\nin the window", kMuted, 2);
    }
  }

  if (button(f, 16, kBtnY, 200, kBtnH, "Cancel")) {
    model_->cancel_run();
    show(HOME);
  }
  if (button(f, 232, kBtnY, 220, kBtnH, "Restart run")) model_->start_run();
  if (button(f, 788, kBtnY, 220, kBtnH, "ACCEPT", kGreen, st.ready, FONT_M28)) on_accept_();
}

// ============================================================================
// Correction

void BalancerUi::result_(Frame &f) {
  Canvas &c = f.c;
  BalanceJob &job = model_->job();
  const JobConfig &cfg = job.config();
  const JobResult &r = job.result();
  const bool verified = job.step() == JobStep::VERIFIED;
  bar_(f, verified ? "Verify result" : "Correction");

  panel(c, 16, 66, 992, 60);
  text_block(c, FONT_R20, 32, 72, 960, job.instruction().c_str(), kText, 2);
  for (int p = 0; p < 2 && p < cfg.planes; p++) {
    const int x0 = 16 + p * 338, y0 = 136;
    panel(c, x0, y0, 330, 392);
    c.text(FONT_R16, x0 + 16, y0 + 8, p == 0 ? "PLANE 1" : "PLANE 2", p == 0 ? kChA : kChB);
    const PlaneResult &pr = r.plane[p];
    const PlaneSetup &ps = cfg.plane[p];
    const bool show = r.ok && (!verified || r.have_residual);
    dial(c, x0 + 57, y0 + 24, 216, show, pr.mass.angle_deg, pr.mass.remove, ps.positions, ps.positions_offset_deg,
         pr.use_split ? pr.split.pos_a : -1, pr.use_split ? pr.split.pos_b : -1, !cfg.angles_with_rotation);
    if (!show) {
      c.text(FONT_M28, x0 + 16, y0 + 244, "--", kText);
      text_block(c, FONT_R14, x0 + 16, y0 + 314, 298, r.error.c_str(), kMuted, 4);
      continue;
    }
    const Rgb big = pr.mass.remove ? kRed : kAmber;
    text_block(c, FONT_M28, x0 + 16, y0 + 244, 0,
               fmt("%s %s\nat %.0f°", pr.mass.remove ? "DRILL" : "ADD", mass_(pr.mass.mass_g).c_str(), pr.mass.angle_deg),
               big, 2);
    std::string d = fmt("at %.0f mm radius", ps.corr_radius_mm);
    const std::string pieces = mass_pieces_(pr.mass.mass_g);
    if (!pr.mass.remove && !pieces.empty()) d += "\nstick on: " + pieces;
    if (pr.use_split)
      d += fmt("\nor %s at #%d (%.0f°) + %s at #%d (%.0f°)", mass_(pr.split.mass_a).c_str(), pr.split.pos_a + 1,
               pr.split.angle_a, mass_(pr.split.mass_b).c_str(), pr.split.pos_b + 1, pr.split.angle_b);
    if (pr.mass.remove) d += fmt("\n%.1f mm drill, %.1f mm deep (to the tip)", cfg.drill_diameter_mm, pr.drill_depth_mm);
    if (verified) d = "Trim (optional): " + d;
    text_block(c, FONT_R14, x0 + 16, y0 + 314, 298, d.c_str(), kMuted, 4);
  }

  panel(c, 692, 136, 316, 392);
  c.text(FONT_R16, 708, 144, "DETAILS", kMuted);
  std::string info = fmt("Test speed %.0f rpm\n", job.test_rpm());
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
    info += fmt("Plane %d ISO G%g limit %.1f g*mm (%s at %.0f mm)\n", p + 1, cfg.grade, lim,
                mass_(lim / cfg.plane[p].corr_radius_mm).c_str(), cfg.plane[p].corr_radius_mm);
  }
  if (verified && r.have_residual) {
    info += "\n";
    for (int p = 0; p < cfg.planes; p++)
      info += fmt("Plane %d residual %.1f g*mm  %s\n", p + 1, r.residual_gmm[p], r.within_iso[p] ? "PASS" : "OVER LIMIT");
    info += fmt("Vibration reduced: A %.0f%%, B %.0f%%\n", r.reduction_pct[0], r.reduction_pct[1]);
  }
  text_block(c, FONT_R14, 708, 170, 286, info.c_str(), kText, 20);

  if (button(f, 16, kBtnY, 200, kBtnH, "Home")) show(HOME);
  if (button(f, 232, kBtnY, 250, kBtnH, "Save machine")) {
    model_->save_machine();
    toast_msg_("Influence coefficients saved. Use Trim run next time on this machine.");
  }
  if (button_next(f, 768, kBtnY, 240, kBtnH, verified ? "TRIM + VERIFY" : "VERIFY RUN", kGreen, r.ok || job.model().valid))
    on_verify_();
}

// ============================================================================
// Live vibration

void BalancerUi::live_(Frame &f) {
  Canvas &c = f.c;
  bar_(f, "Live vibration");
  const LiveData &lv = model_->live();
  const char *unit = quantity_unit(model_->settings().quantity);
  static const Rgb chc[2] = {kChA, kChB};

  panel(c, 16, 66, 330, 462);
  c.text(FONT_R16, 32, 74, "SPEED", kMuted);
  c.text(FONT_M56, 30, 90, lv.rpm > 0 ? fmt("%.0f", lv.rpm) : "--", kText);
  for (int ch = 0; ch < 2; ch++) {
    const double a = std::abs(lv.v[ch]);
    c.text(FONT_R16, 32, 166 + ch * 110, ch == 0 ? "SENSOR A" : "SENSOR B", chc[ch]);
    text_block(c, FONT_R20, 32, 186 + ch * 110, 0,
               fmt("1x  %.*f %s @ %.0f°\nRMS %.4f g   peak %.0f%% FS\nfailed reads %u, missed %u", decimals_for(a), a,
                   unit, arg_deg(lv.v[ch]), lv.rms_g[ch], lv.peak_frac[ch] * 100, lv.read_errors[ch], lv.missed[ch]),
               kText, 3);
  }
  text_block(c, FONT_R16, 32, 400, 298,
             fmt("%s\nsample rate %.1f Hz %s\ntach pulses %u, tach glitches %u\nrestarts %u, missed + failed %u\n"
                 "average of last %d revolutions",
                 lv.acq_ok ? "acquisition running" : (model_->simulate() ? "demo mode (no sensors)" : "SENSORS NOT RUNNING"),
                 lv.fs, lv.aligned ? "(locked)" : "(not locked)", lv.tach_pulses, lv.glitches, lv.resyncs, lv.overflows,
                 lv.revs),
             kMuted, 6);

  // The averaged revolution: drawing area x 372..992, centre line y 191, +/-80 px
  panel(c, 356, 66, 652, 226);
  c.text(FONT_R16, 372, 74, "AVERAGED REVOLUTION  (tach mark at left edge)", kMuted);
  float mx = 1e-6f;
  for (int ch = 0; ch < 2; ch++)
    for (int i = 0; i < kWaveBins; i++) mx = std::max(mx, std::fabs(lv.wave[ch][i]));
  for (int ch = 0; ch < 2; ch++) {
    float x0 = 372.0f, y0 = 191.0f - lv.wave[ch][0] / mx * 80.0f;
    for (int i = 1; i <= kWaveBins; i++) {
      const float x1 = 372.0f + i * 620.0f / kWaveBins, y1 = 191.0f - lv.wave[ch][i % kWaveBins] / mx * 80.0f;
      c.line(x0, y0, x1, y1, 2, chc[ch]);
      x0 = x1;
      y0 = y1;
    }
  }

  // Bottom right: the orders, or the raw readings
  const int bx = 356, by = 302;
  panel(c, bx, by, 652, 226);
  if (!l_show_raw_) {
    c.text(FONT_R16, bx + 16, by + 8, "ORDERS  1x - 10x", kMuted);
    double omx = 1e-9;
    for (int ch = 0; ch < 2; ch++)
      for (int h = 0; h < kMaxOrder; h++) omx = std::max(omx, lv.order_amp[ch][h]);
    for (int h = 0; h < kMaxOrder; h++) {
      for (int ch = 0; ch < 2; ch++)
        vbar(c, bx + 34 + h * 61 + ch * 22, by + 36, 18, 150, static_cast<float>(lv.order_amp[ch][h] / omx), chc[ch]);
      c.text(FONT_R14, bx + 54 + h * 61, by + 194, fmt("%dx", h + 1), kMuted, CENTER);
    }
  } else {
    // Each trace has its own mean removed (gravity differs with tilt); both share one scale,
    // so their sizes compare directly. The newest reading is at the right edge.
    c.text(FONT_R16, bx + 16, by + 8, "RAW  (mean removed)", kMuted);
    c.fill(bx + 16, by + 125, 620, 1, kPanel2);
    const int n = std::max(lv.raw_n[0], lv.raw_n[1]);
    if (n < 2) {
      c.text(FONT_R16, bx + 200, by + 8, model_->simulate() ? "no raw readings in demo mode" : "no readings yet", kMuted);
    } else {
      float mean[2] = {0, 0}, rmx = 1e-4f;
      for (int ch = 0; ch < 2; ch++) {
        if (lv.raw_n[ch] == 0) continue;
        for (int i = 0; i < lv.raw_n[ch]; i++) mean[ch] += lv.raw[ch][i];
        mean[ch] /= lv.raw_n[ch];
        for (int i = 0; i < lv.raw_n[ch]; i++) rmx = std::max(rmx, std::fabs(lv.raw[ch][i] - mean[ch]));
      }
      const double span_s = lv.fs > 1 ? n / lv.fs : 0;
      c.text(FONT_R16, bx + 200, by + 8,
             fmt("+/-%.1f mg, %.1f s   A %+.3f g  B %+.3f g", rmx * 1000, span_s, mean[0], mean[1]), kMuted);
      for (int ch = 0; ch < 2; ch++) {
        const int m = lv.raw_n[ch];
        if (m < 2) continue;
        const int first = LiveData::kRawPts - m;
        auto pt_x = [&](int i) { return bx + 16.0f + (first + i) * 620.0f / (LiveData::kRawPts - 1); };
        auto pt_y = [&](int i) { return by + 125.0f - (lv.raw[ch][i] - mean[ch]) / rmx * 80.0f; };
        for (int i = 1; i < m; i++) c.line(pt_x(i - 1), pt_y(i - 1), pt_x(i), pt_y(i), 1.2f, chc[ch]);
      }
    }
  }

  if (button(f, 16, kBtnY, 200, kBtnH, "Home")) show(HOME);
  if (model_->simulate() && button(f, 232, kBtnY, 200, kBtnH, "SPIN", kPurple)) model_->sim_spin();
  if (button(f, 592, kBtnY, 200, kBtnH, l_show_raw_ ? "Orders" : "Raw readings")) l_show_raw_ = !l_show_raw_;
  if (!model_->simulate()) {
    const bool on = model_->pc_log();
    if (button(f, 808, kBtnY, 200, kBtnH, on ? "PC log: ON" : "PC log: off", on ? 0x15803d : kPanel2))
      model_->set_pc_log(!on);
  }
}

// ============================================================================
// Settings

void BalancerUi::settings_(Frame &f) {
  Canvas &c = f.c;
  bar_(f, "Settings");
  const bool was_active = f.active;
  if (det_ch_ >= 0) f.active = false;  // the axis finder is open over it
  UiSettings &st = model_->settings();
  const LiveData &lv = model_->live();
  UiModel *m = model_;
  UiSettings *sp = &st;
  // Every change is saved to flash straight away, so it survives a reboot or a firmware update.
  const int x = 32, w = 528, step = ROW_H + 4;
  int y = 78;

  panel(c, 16, 66, 560, 458);
  for (int ch = 0; ch < 2; ch++) {
    if (setting_row(f, x, y, w, ch == 0 ? "Sensor A polarity" : "Sensor B polarity", st.invert[ch] ? "inverted" : "normal")) {
      st.invert[ch] = !st.invert[ch];
      model_->apply_settings(true);
    }
    y += step;
  }
  if (setting_row(f, x, y, w, "Weight units", st.mass_oz ? "oz" : "g")) {
    st.mass_oz = !st.mass_oz;
    model_->apply_settings(true);
  }
  y += step;
  if (setting_row(f, x, y, w, "Units", quantity_unit(st.quantity))) {
    st.quantity = static_cast<Quantity>((static_cast<int>(st.quantity) + 1) % 3);
    model_->apply_settings(true);
  }
  y += step;
  if (setting_row(f, x, y, w, "Speed window", fmt("+/- %.1f %%", st.window * 100))) {
    ask_number_("Speed window (+/- %)", st.window * 100, 1, [m, sp](double v) {
      sp->window = std::min(10.0, std::max(0.5, v)) / 100.0;
      m->apply_settings(true);
    });
  }
  y += step;
  if (setting_row(f, x, y, w, "Revolutions per run", fmt("%d", st.target_revs))) {
    ask_number_("Minimum revolutions per run", st.target_revs, 0, [m, sp](double v) {
      sp->target_revs = static_cast<int>(std::min(1000.0, std::max(5.0, v)));
      m->apply_settings(true);
    });
  }
  y += step;
  if (setting_row(f, x, y, w, "Max phase uncertainty", fmt("%.1f °", st.max_phase_unc_deg))) {
    ask_number_("Max phase uncertainty (°)", st.max_phase_unc_deg, 1, [m, sp](double v) {
      sp->max_phase_unc_deg = std::min(20.0, std::max(0.2, v));
      m->apply_settings(true);
    });
  }
  text_block(c, FONT_R14, 32, 444, 528,
             "Axes: tap FIND AXES and push the tyre the way the arrow shows, or pick X / Y / Z. Gravity shows which "
             "axis is vertical. Changing an axis means starting the job again. Changes are saved straight away.",
             kMuted, 4);

  // Right: which axis each sensor measures, and the network
  panel(c, 592, 66, 416, 458);
  c.text(FONT_R16, 604, 72, "SENSOR AXES", kMuted);
  static const char kAx[] = "XYZ";
  for (int ch = 0; ch < 2; ch++) {
    const int y0 = 98 + ch * 112;
    const Rgb color = ch == 0 ? kChA : kChB;
    text_mid(c, FONT_M28, 608, y0 + 22, ch == 0 ? "A" : "B", color);
    for (int k = 0; k < 3; k++) {
      const char label[2] = {kAx[k], 0};
      if (button(f, 642 + k * 72, y0, 64, 44, label, st.axis[ch] == k ? color : kPanel2, true, FONT_M20,
                 fmt("%c axis %c", 'A' + ch, kAx[k])) &&
          st.axis[ch] != k) {
        st.axis[ch] = k;
        model_->apply_settings(true);
        toast_msg_(fmt("Sensor %c now reads %c. Runs measured before this no longer match: start the job again.",
                       'A' + ch, kAx[k]));
      }
    }
    text_mid(c, FONT_R14, 866, y0 + 22, fmt("measuring %c", kAx[st.axis[ch]]), kText);
    const float *mn = lv.axis_mean[ch], *r = lv.axis_rms[ch];
    text_block(c, FONT_R14, 606, y0 + 52, 0,
               fmt("gravity  X %+.2f  Y %+.2f  Z %+.2f g\nshake    X %.1f  Y %.1f  Z %.1f mg rms", mn[0], mn[1], mn[2],
                   r[0] * 1000, r[1] * 1000, r[2] * 1000),
               kMuted, 2);
  }
  if (button(f, 604, 330, 392, 50, "FIND AXES (guided)", kAccent)) open_detect_();
  c.text(FONT_R16, 604, 398, "NETWORK", kMuted);
  const std::string net = model_->network_status();
  text_block(c, FONT_R14, 604, 422, 392, net.empty() ? "not available here" : net.c_str(), kText, 5);

  if (button(f, 16, kBtnY, 200, kBtnH, "Home")) show(HOME);
  // Press and hold: the sensor's teach input stays on for as long as the button is held,
  // exactly like holding the key on the sensor.
  const int tx = 232, tw = 300;
  if (model_->has_tach_teach()) {
    const bool hold = f.holding(tx, kBtnY, tw, kBtnH);
    if (hold != model_->tach_teach()) model_->set_tach_teach(hold);
    button(f, tx, kBtnY, tw, kBtnH, model_->tach_teach() ? "TEACHING..." : "TEACH TACH (hold)",
           model_->tach_teach() ? kAmber : kPanel2, true, FONT_M20, "TEACH TACH (hold)");
  } else if (button(f, tx, kBtnY, tw, kBtnH, "TEACH TACH (hold)")) {
    toast_msg_("No teach pin: set 'teach_pin' under 'tach' in the YAML.");
  }
  if (button(f, 788, kBtnY, 220, kBtnH, "SAVE", kGreen, true, FONT_M28)) {
    model_->apply_settings(true);
    toast_msg_("Settings saved.");
  }

  f.active = was_active;
  if (det_ch_ >= 0) {
    c.fill_blend(0, 0, SCREEN_W, SCREEN_H, 0x000000, 178);
    detect_(f);
  }
}

// ============================================================================
// Guided axis finder. The person pushes the wheel the way one sensor should measure; the axis
// with clearly the most AC motion (gravity removed) wins. Sensor A: forward and back (the way
// the car drives). Sensor B: in and out along the axle. Seen from above, as drawn.

static constexpr uint32_t kDetectMs = 4000;

void BalancerUi::open_detect_() {
  det_ch_ = 0;
  det_running_ = false;
  det_done_ = false;
  det_result_.clear();
  for (float &p : det_peak_) p = 0;
}

// The right-hand button: START a sensor, or move on after a result
void BalancerUi::detect_next_() {
  if (det_running_) return;
  if (det_done_) {
    const bool two = model_->live().raw_n[1] > 0 || model_->simulate();
    if (det_ch_ == 0 && two) {
      det_ch_ = 1;
      det_done_ = false;
      det_result_.clear();
      for (float &p : det_peak_) p = 0;
    } else {
      det_ch_ = -1;
    }
  } else {
    det_running_ = true;
    det_end_ms_ = now_ms_ + kDetectMs;
    for (float &p : det_peak_) p = 0;
  }
}

void BalancerUi::finish_detect_() {
  const int ch = det_ch_;
  static const char kAx[] = "XYZ";
  det_running_ = false;
  det_done_ = true;
  int best = 0;
  for (int k = 1; k < 3; k++)
    if (det_peak_[k] > det_peak_[best]) best = k;
  float second = 0;
  for (int k = 0; k < 3; k++)
    if (k != best) second = std::max(second, det_peak_[k]);
  if (det_peak_[best] < 0.005f) {
    det_result_ = fmt("Sensor %c hardly moved (%.1f mg). Push harder and try again.", 'A' + ch, det_peak_[best] * 1000);
    det_done_ = false;  // same sensor again
  } else if (det_peak_[best] < 1.5f * second) {
    det_result_ = fmt("No clear axis for sensor %c: %c and %c moved about the same. The board is probably at an "
                      "angle to the push. Try again pushing straight, or pick by hand.",
                      'A' + ch, kAx[best], kAx[det_peak_[0] == second ? 0 : (det_peak_[1] == second ? 1 : 2)]);
    det_done_ = false;
  } else {
    UiSettings &st = model_->settings();
    const bool changed = st.axis[ch] != best;
    st.axis[ch] = best;
    model_->apply_settings(true);
    det_result_ = fmt("Sensor %c: %c axis (%.0f mg, next best %.0f mg).%s", 'A' + ch, kAx[best], det_peak_[best] * 1000,
                      second * 1000, changed ? " Changed and saved: start the balancing job again." : " Already set.");
  }
}

void BalancerUi::detect_(Frame &f) {
  Canvas &c = f.c;
  const int ch = det_ch_;
  const LiveData &lv = model_->live();
  const Rgb ca = ch == 0 ? kChA : kChB;
  const int ox = 24, oy = 64;
  c.round_rect(ox, oy, 976, 468, 10, 0x111827);
  c.round_frame(ox, oy, 976, 468, 10, 2, kAccent);
  text_mid(c, FONT_M28, ox + 20, oy + 30, fmt("Find the axis of sensor %c", 'A' + ch), kText);

  // Top view: the tyre on the left, the axle to the knuckle on the right
  const int px0 = ox + 20, py0 = oy + 60;
  panel(c, px0, py0, 420, 330, kPanel);
  c.text(FONT_R14, px0 + 10, py0 + 8, "VIEW FROM ABOVE", kMuted);
  c.text(FONT_R14, px0 + 10, py0 + 30, "car front", kMuted);
  c.round_rect(px0 + 60, py0 + 45, 70, 240, 18, 0x374151);
  c.text(FONT_R14, px0 + 72, py0 + 290, "tyre", kMuted);
  c.line(px0 + 130, py0 + 165, px0 + 300, py0 + 165, 8, kMuted);
  c.round_rect(px0 + 300, py0 + 115, 34, 100, 6, 0x4b5563);
  c.text(FONT_R14, px0 + 286, py0 + 220, "knuckle", kMuted);
  c.text(FONT_R14, px0 + 196, py0 + 140, "axle", kMuted);
  // Arrow A: forward / back (up and down in this view), beside the tyre
  const Rgb col_a = ch == 0 ? kChA : 0x4b5563, col_b = ch == 1 ? kChB : 0x4b5563;
  auto seg = [&](int x0, int y0, int x1, int y1, Rgb col) { c.line(px0 + x0, py0 + y0, px0 + x1, py0 + y1, 6, col); };
  seg(30, 60, 30, 270, col_a);
  seg(18, 80, 30, 60, col_a);
  seg(30, 60, 42, 80, col_a);
  seg(18, 250, 30, 270, col_a);
  seg(30, 270, 42, 250, col_a);
  // Arrow B: in / out along the axle, under it
  seg(150, 250, 390, 250, col_b);
  seg(170, 238, 150, 250, col_b);
  seg(150, 250, 170, 262, col_b);
  seg(370, 238, 390, 250, col_b);
  seg(390, 250, 370, 262, col_b);

  std::string msg;
  if (det_running_) {
    msg = ch == 0 ? "Keep pushing the tyre FORWARD and BACK (the way the car drives), about once a second..."
                  : "Keep pushing the tyre IN and OUT along the axle, about once a second...";
  } else if (!det_result_.empty() && (det_done_ || det_peak_[0] + det_peak_[1] + det_peak_[2] > 0)) {
    msg = det_result_;
  } else {
    msg = ch == 0 ? "Sensor A measures FORWARD / BACK, the way the car drives (blue arrow).\n\nTap START, then push "
                    "the tyre forward and back by hand, about once a second, for 4 seconds."
                  : "Sensor B measures IN / OUT along the axle (orange arrow).\n\nTap START, then push the tyre "
                    "in and out along the axle, about once a second, for 4 seconds.";
  }
  text_block(c, FONT_R20, ox + 460, oy + 64, 496, msg.c_str(), kText, 5);

  // Live bars: this sensor's shake on each axis now, scaled to the largest
  const float *r = lv.axis_rms[ch];
  const float mx = std::max(0.002f, std::max(r[0], std::max(r[1], r[2])));
  const int cur = model_->settings().axis[ch];
  static const char *const kAxN[3] = {"X", "Y", "Z"};
  for (int k = 0; k < 3; k++) {
    const int yk = oy + 206 + k * 50;
    text_mid(c, FONT_M28, ox + 460, yk + 13, kAxN[k], kText);
    hbar(c, ox + 500, yk, 330, 26, r[k] / mx, k == cur ? ca : kMuted, kPanel2, 4);
    text_mid(c, FONT_R20, ox + 846, yk + 13, fmt("%.0f mg", r[k] * 1000), kText);
  }
  float prog = det_done_ ? 1.0f : 0.0f;
  if (det_running_) {
    const int32_t left = static_cast<int32_t>(det_end_ms_ - now_ms_);
    prog = 1.0f - std::max<int32_t>(0, left) / static_cast<float>(kDetectMs);
  }
  hbar(c, ox + 460, oy + 360, 496, 10, prog, kAmber, kPanel2, 5);

  if (button(f, ox + 20, oy + 402, 200, kBtnH, "Close")) {
    det_running_ = false;
    det_ch_ = -1;
  }
  const char *rl = det_running_ ? "..."
                   : det_done_  ? ((ch == 0 && (lv.raw_n[1] > 0 || model_->simulate())) ? "Sensor B" : "Done")
                                : "START";
  if (button(f, ox + 756, oy + 402, 200, kBtnH, rl, kGreen)) detect_next_();
}

}  // namespace bal
