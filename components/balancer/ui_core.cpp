#include "ui_core.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace bal {

ModelCore::ModelCore() {
  for (int p = 0; p < 2; p++) sim_fitted_[p] = sim_correction_total_[p] = 0;
  load_defaults();
}

void ModelCore::configure_measurement_() {
  MeasureConfig mc = meas_.config();
  mc.window = settings_.window;
  mc.target_revs = settings_.target_revs;
  mc.max_phase_unc_deg = settings_.max_phase_unc_deg;
  mc.quantity = settings_.quantity;
  mc.amp_floor = default_amp_floor(settings_.quantity);
  if (amp_floor_um_ > 0) {
    // Given as a displacement; express it in the unit the run is measured in.
    const double w = job_.test_rpm() * kTwoPi / 60.0;
    if (settings_.quantity == Quantity::DISPLACEMENT_UM)
      mc.amp_floor = amp_floor_um_;
    else if (w > 0 && settings_.quantity == Quantity::VELOCITY_MM_S)
      mc.amp_floor = amp_floor_um_ * w * 1e-3;
    else if (w > 0)
      mc.amp_floor = amp_floor_um_ * 1e-6 * w * w / kStandardGravity;
  }
  mc.target_rpm = job_.test_rpm();
  mc.max_period_change = max_period_change_;
  mc.best_effort_factor = best_effort_factor_;
  meas_.configure(mc);
  job_.mutable_config().window = settings_.window;
}

void ModelCore::load_defaults() {
  // A car wheel: about 20 kg, balanced to 5 g or better per rim edge.
  // G6.3 at 800 rpm (roughly 100 km/h) allows 750 g*mm per plane, which is
  // 3.8 g at a 200 mm rim radius.
  JobConfig cfg;
  cfg.rotor_mass_kg = 20.0;
  cfg.service_rpm = 800.0;
  cfg.grade = 6.3;
  for (auto &pl : cfg.plane) {
    pl.trial_mass_g = 30.0;
    pl.trial_radius_mm = 200.0;
    pl.corr_radius_mm = 200.0;
    pl.remove_mode = false;  // stick-on or clip-on weights
  }
  // Hand-spun and coasting at a few revolutions per second: few
  // revolutions fit in a narrow window, and each one slows noticeably.
  // On a soft rig the displacement barely depends on speed, so a wide
  // window in displacement units is honest.
  UiSettings st;
  for (int c = 0; c < kChannels; c++) {  // set from YAML
    st.invert[c] = settings_.invert[c];
    st.axis[c] = settings_.axis[c];
  }
  st.quantity = Quantity::DISPLACEMENT_UM;
  st.window = 0.10;
  st.target_revs = 20;
  st.max_phase_unc_deg = 3.0;
  max_period_change_ = 0.03;
  // MEMS noise: a reading known to 2.5 um is as good as it gets in one or
  // two spins, and is a few percent of what the tolerance looks like.
  amp_floor_um_ = 2.5;
  best_effort_factor_ = 4;
  settings_ = st;
  job_.reset(cfg);  // keeps the sensor count limit
  sim_ = RevSim();  // demo wheel to match
  sim_.set_tire();
  configure_measurement_();
}

void ModelCore::apply_settings(bool persist) {
  const bool was = measuring_;
  configure_measurement_();  // note: clears the current run
  measuring_ = was;
  stats_ = meas_.stats();
  for (auto &d : dots_) d.clear();
  on_settings_changed();
  if (persist) persist_settings();
}

void ModelCore::start_run() {
  configure_measurement_();
  stats_ = meas_.stats();
  for (auto &d : dots_) d.clear();
  last_status_ = job_.test_rpm() > 0 ? RevStatus::BELOW_WINDOW : RevStatus::NOT_LOCKED;
  measuring_ = true;
  set_sim_fitted_for_step_();
}

bool ModelCore::accept_run(std::string *error) {
  if (!measuring_) return false;
  stats_ = meas_.stats();
  if (!stats_.ready) {
    if (error) *error = "Not enough revolutions in the window yet.";
    return false;
  }
  measuring_ = false;
  const JobStep before = job_.step();
  const bool ok = job_.record(stats_, error);
  // A verify run produces a new (trim) result; the next verify fits it.
  if (before == JobStep::VERIFY) sim_verify_applied_ = false;
  return ok;
}

void ModelCore::lock_speed(double rpm) {
  if (rpm <= 0) rpm = last_rpm_;
  if (rpm <= 0) return;
  job_.set_test_rpm(rpm);
  meas_.set_target_rpm(rpm);
  meas_.reset();
  for (auto &d : dots_) d.clear();
  stats_ = meas_.stats();
}

void ModelCore::save_machine() {
  if (!job_.model().valid) return;
  machine_ = job_.model();
  machine_rpm_ = job_.test_rpm();
  machine_cfg_ = job_.config();
  has_machine_ = true;
  persist_machine();
}

bool ModelCore::start_trim() {
  if (!has_machine()) return false;
  job_.reset(machine_cfg_);
  job_.load_model(machine_, machine_rpm_);
  return true;
}

std::string ModelCore::machine_summary() const {
  if (!has_machine_) return "No saved machine yet: finish a balance and tap Save machine.";
  char buf[160];
  snprintf(buf, sizeof(buf), "Saved machine: %d-plane, test speed %.0f rpm, %.2f kg rotor", machine_.planes,
           machine_rpm_, machine_cfg_.rotor_mass_kg);
  return buf;
}

void ModelCore::sim_spin() {
  // A hand spin: a quarter above the test speed, then a long coast.
  const double t = job_.test_rpm() > 0 ? job_.test_rpm() : 240.0;
  sim_.blast(t * 1.25, 30.0);
}

// What the operator has fitted for the current step (demo wheel only).
void ModelCore::set_sim_fitted_for_step_() {
  const JobConfig &cfg = job_.config();
  auto fitted_from_result = [&](int p) -> cplx {
    const PlaneResult &pr = job_.result().plane[p];
    double ang = job_.to_internal_angle(pr.mass.angle_deg) + (pr.mass.remove ? 180.0 : 0.0);
    return unbalance(pr.mass.mass_g, cfg.plane[p].corr_radius_mm, ang);
  };
  switch (job_.step()) {
    case JobStep::RUN0:
      sim_fitted_[0] = sim_fitted_[1] = 0;
      sim_correction_total_[0] = sim_correction_total_[1] = 0;
      sim_verify_applied_ = false;
      break;
    case JobStep::TRIAL1:
      sim_fitted_[0] = job_.trial_vector(0);
      sim_fitted_[1] = 0;
      break;
    case JobStep::TRIAL2:
      sim_fitted_[0] = 0;
      sim_fitted_[1] = job_.trial_vector(1);
      break;
    case JobStep::VERIFY: {
      // Fit what the result screen showed (first verify after a result, or a
      // trim after a verify), on top of anything fitted before.
      if (job_.result().ok && !sim_verify_applied_) {
        for (int p = 0; p < cfg.planes; p++) sim_correction_total_[p] += fitted_from_result(p);
        sim_verify_applied_ = true;
      }
      const int last = cfg.planes - 1;
      for (int p = 0; p < 2; p++) sim_fitted_[p] = sim_correction_total_[p];
      if (cfg.keep_last_trial) sim_fitted_[last] += job_.trial_vector(last);
      break;
    }
    default:
      break;
  }
  sim_.set_fitted(sim_fitted_[0], sim_fitted_[1]);
}

void ModelCore::feed_rev(const RevResult &r, uint32_t now_ms) {
  last_rpm_ = r.rpm;
  last_rev_ms_ = now_ms;
  LiveRev lr;
  lr.omega = r.omega;
  lr.rpm = r.rpm;
  for (int c = 0; c < kChannels; c++) {
    for (int h = 0; h < kMaxOrder; h++) lr.o[c][h] = r.order[c][h];
    for (int b = 0; b < kWaveBins; b++) lr.wave[c][b] = r.wave[c][b];
  }
  live_revs_.push_back(lr);
  while (live_revs_.size() > kLiveRevs) live_revs_.pop_front();

  if (measuring_) {
    last_status_ = meas_.add(r);
    if (last_status_ == RevStatus::ACCEPTED) {
      for (int c = 0; c < kChannels; c++) {
        dots_[c].push_back(convert_accel(r.order[c][0], settings_.quantity, 1, r.omega));
        if (dots_[c].size() > 60) dots_[c].erase(dots_[c].begin());
      }
    }
  }
}

void ModelCore::tick(uint32_t now_ms) {
  if (simulate_) {
    const double dt = last_tick_ms_ ? std::min<uint32_t>(200, now_ms - last_tick_ms_) / 1000.0 : 0.0;
    if (sim_buf_.empty()) sim_buf_.resize(32);
    const int n = sim_.step(dt, sim_buf_.data(), static_cast<int>(sim_buf_.size()));
    for (int i = 0; i < n; i++) feed_rev(sim_buf_[i], now_ms);
    live_.acq_ok = false;
    live_.aligned = true;
    live_.fs = 250;
    for (int c = 0; c < 2; c++) {
      live_.rms_g[c] = live_revs_.empty() ? 0.0f : static_cast<float>(std::abs(live_revs_.back().o[c][0]) / std::sqrt(2.0));
      live_.peak_frac[c] = 0.02f;
    }
  }
  last_tick_ms_ = now_ms;

  if (now_ms - last_stats_ms_ < 200) return;
  last_stats_ms_ = now_ms;
  live_.tach_ok = last_rev_ms_ != 0 && now_ms - last_rev_ms_ < 2000;
  live_.rpm = live_.tach_ok ? last_rpm_ : 0;
  if (measuring_) stats_ = meas_.stats();

  // Live averages over the last few revolutions (no speed window).
  const int n = static_cast<int>(live_revs_.size());
  live_.revs = n;
  for (int c = 0; c < kChannels; c++) {
    cplx v{0, 0};
    double oa[kMaxOrder] = {};
    double wave[kWaveBins] = {};
    for (const auto &lr : live_revs_) {
      v += convert_accel(lr.o[c][0], settings_.quantity, 1, lr.omega);
      for (int h = 0; h < kMaxOrder; h++) oa[h] += std::abs(convert_accel(lr.o[c][h], settings_.quantity, h + 1, lr.omega));
      for (int b = 0; b < kWaveBins; b++) wave[b] += lr.wave[c][b];
    }
    const double k = n > 0 ? 1.0 / n : 0.0;
    live_.v[c] = v * k;
    for (int h = 0; h < kMaxOrder; h++) live_.order_amp[c][h] = oa[h] * k;
    for (int b = 0; b < kWaveBins; b++) live_.wave[c][b] = static_cast<float>(wave[b] * k);
  }
  if (!live_.tach_ok) live_revs_.clear();
}

}  // namespace bal
