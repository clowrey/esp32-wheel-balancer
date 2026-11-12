#include "dsp_job.h"

#include <cmath>
#include <cstdarg>
#include <cstdio>

namespace bal {

namespace {
std::string fmt(const char *f, ...) __attribute__((format(printf, 1, 2)));
std::string fmt(const char *f, ...) {
  char buf[320];
  va_list ap;
  va_start(ap, f);
  vsnprintf(buf, sizeof(buf), f, ap);
  va_end(ap);
  return buf;
}
}  // namespace

const char *job_step_name(JobStep s) {
  switch (s) {
    case JobStep::SETUP:
      return "Setup";
    case JobStep::RUN0:
      return "Run 0: as found";
    case JobStep::TRIAL1:
      return "Run 1: trial weight, plane 1";
    case JobStep::TRIAL2:
      return "Run 2: trial weight, plane 2";
    case JobStep::RESULT:
      return "Correction";
    case JobStep::VERIFY:
      return "Verify run";
    case JobStep::VERIFIED:
      return "Verify result";
  }
  return "?";
}

void BalanceJob::reset(const JobConfig &cfg) {
  cfg_ = cfg;
  if (cfg_.planes < 1) cfg_.planes = 1;
  if (cfg_.planes > max_planes_) cfg_.planes = max_planes_;
  step_ = JobStep::SETUP;
  test_rpm_ = 0;
  for (int s = 0; s < kMaxSensors; s++) {
    v0_[s] = vver_[s] = 0;
    vt_[0][s] = vt_[1][s] = 0;
  }
  model_ = InfluenceModel();
  result_ = JobResult();
}

void BalanceJob::start() {
  if (cfg_.planes > max_planes_) cfg_.planes = max_planes_;
  step_ = JobStep::RUN0;
}

double BalanceJob::to_user_angle(double internal_deg) const {
  return cfg_.angles_with_rotation ? wrap360(-internal_deg) : wrap360(internal_deg);
}
double BalanceJob::to_internal_angle(double user_deg) const {
  return cfg_.angles_with_rotation ? wrap360(-user_deg) : wrap360(user_deg);
}

cplx BalanceJob::trial_vector(int p) const {
  const PlaneSetup &ps = cfg_.plane[p];
  return unbalance(ps.trial_mass_g, ps.trial_radius_mm, to_internal_angle(ps.trial_angle_deg));
}

double BalanceJob::plane_iso_limit_gmm_(int p) const {
  const double total = iso_uper_gmm(cfg_.grade, cfg_.rotor_mass_kg, cfg_.service_rpm);
  const double share = cfg_.planes == 1 ? 1.0 : (p == 0 ? cfg_.plane1_load_fraction : 1.0 - cfg_.plane1_load_fraction);
  return total * share;
}

// The 10%-of-rotor-weight rule grows without limit as the test speed drops
// (a tire at 240 rpm would get 150 g), so it is capped at ten times the ISO
// limit for the plane: enough to move the reading clearly, never silly.
double BalanceJob::suggested_trial_g(int p) const {
  const double rpm = test_rpm_ > 0 ? test_rpm_ : cfg_.service_rpm;
  const double r = cfg_.plane[p].trial_radius_mm;
  if (r <= 0) return 0;
  const double by_weight = suggest_trial_gmm(cfg_.rotor_mass_kg, rpm, 0.1);
  const double cap = kTrialIsoMultiple * plane_iso_limit_gmm_(p);
  return (cap > 0 && cap < by_weight ? cap : by_weight) / r;
}

bool BalanceJob::suggested_trial_capped(int p) const {
  const double rpm = test_rpm_ > 0 ? test_rpm_ : cfg_.service_rpm;
  const double cap = kTrialIsoMultiple * plane_iso_limit_gmm_(p);
  return cap > 0 && cap < suggest_trial_gmm(cfg_.rotor_mass_kg, rpm, 0.1);
}

std::string BalanceJob::instruction() const {
  const double lo = test_rpm_ * (1.0 - cfg_.window), hi = test_rpm_ * (1.0 + cfg_.window);
  std::string coast;
  if (test_rpm_ > 0)
    coast = air_drive_
                ? fmt("Spin above %.0f rpm, cut the air, and let it coast through %.0f-%.0f rpm.", hi * 1.05, lo, hi)
                : fmt("Spin it above %.0f rpm and let it coast through %.0f-%.0f rpm.", hi * 1.05, lo, hi);
  else
    coast = air_drive_ ? "Spin it up with air and tap LOCK SPEED at the test speed you want, then let it coast."
                       : "Spin it up and tap LOCK SPEED at the test speed you want, then let it coast.";

  auto trial_txt = [&](int p) {
    const PlaneSetup &ps = cfg_.plane[p];
    return fmt("Fit %.2f g at %.0f° on plane %d (radius %.0f mm).", ps.trial_mass_g,
               wrap360(ps.trial_angle_deg), p + 1, ps.trial_radius_mm);
  };

  switch (step_) {
    case JobStep::SETUP:
      return "Check the rotor details, then tap START.";
    case JobStep::RUN0:
      return "No trial weights fitted. " + coast;
    case JobStep::TRIAL1:
      return trial_txt(0) + " " + coast;
    case JobStep::TRIAL2:
      return "Remove the plane 1 trial weight. " + trial_txt(1) + " " + coast;
    case JobStep::RESULT: {
      if (!result_.ok) return "Could not compute a correction: " + result_.error;
      std::string s;
      const int last = cfg_.planes - 1;
      if (!cfg_.keep_last_trial) s = fmt("Remove the plane %d trial weight. ", last + 1);
      s += "Fit the corrections shown, then tap VERIFY.";
      return s;
    }
    case JobStep::VERIFY:
      return "Corrections fitted. " + coast;
    case JobStep::VERIFIED: {
      if (!result_.have_residual) return "Verify run failed.";
      bool ok = result_.within_iso[0] && (cfg_.planes < 2 || result_.within_iso[1]);
      return ok ? "Within the ISO limit. Done, or TRIM to improve further."
                : "Above the ISO limit. Fit the trim corrections shown, then VERIFY again.";
    }
  }
  return "";
}

void BalanceJob::load_model(const InfluenceModel &m, double test_rpm) {
  model_ = m;
  cfg_.planes = m.planes;
  test_rpm_ = test_rpm;
  result_ = JobResult();
  result_.ok = m.valid;
  step_ = JobStep::VERIFY;
}

bool BalanceJob::record(const MeasureStats &st, std::string *error) {
  cplx v[kMaxSensors];
  for (int s = 0; s < kChannels; s++) v[s] = st.ch[s].mean;
  switch (step_) {
    case JobStep::RUN0:
      for (int s = 0; s < kChannels; s++) v0_[s] = v[s];
      if (test_rpm_ <= 0) test_rpm_ = st.rpm_mean;
      step_ = JobStep::TRIAL1;
      return true;
    case JobStep::TRIAL1:
      for (int s = 0; s < kChannels; s++) vt_[0][s] = v[s];
      result_.trial_check[0] = check_trial(v0_, vt_[0], kChannels);
      if (cfg_.planes == 1) {
        const bool ok = compute_(error);
        step_ = JobStep::RESULT;
        return ok;
      }
      step_ = JobStep::TRIAL2;
      return true;
    case JobStep::TRIAL2: {
      for (int s = 0; s < kChannels; s++) vt_[1][s] = v[s];
      result_.trial_check[1] = check_trial(v0_, vt_[1], kChannels);
      const bool ok = compute_(error);
      step_ = JobStep::RESULT;
      return ok;
    }
    case JobStep::VERIFY: {
      for (int s = 0; s < kChannels; s++) vver_[s] = v[s];
      std::string err;
      Correction c = solve_correction(model_, vver_, &err);
      if (!c.ok) {
        if (error) *error = err;
        result_.have_residual = false;
        step_ = JobStep::VERIFIED;
        return false;
      }
      result_.have_residual = true;
      const double total = iso_uper_gmm(cfg_.grade, cfg_.rotor_mass_kg, cfg_.service_rpm);
      for (int p = 0; p < cfg_.planes; p++) {
        result_.residual_u[p] = -c.u[p];
        result_.residual_gmm[p] = std::abs(c.u[p]);
        // The trim correction is what's still needed: show it as the plane result.
        fill_plane_(p, c.u[p]);
        const double share = cfg_.planes == 1 ? 1.0
                             : (p == 0 ? cfg_.plane1_load_fraction : 1.0 - cfg_.plane1_load_fraction);
        result_.plane[p].iso_limit_gmm = total * share;
        result_.within_iso[p] = result_.residual_gmm[p] <= result_.plane[p].iso_limit_gmm;
      }
      for (int s = 0; s < kChannels; s++) {
        const double a0 = std::abs(v0_[s]);
        result_.reduction_pct[s] = a0 > 0 ? (1.0 - std::abs(vver_[s]) / a0) * 100.0 : 0.0;
      }
      step_ = JobStep::VERIFIED;
      return true;
    }
    default:
      if (error) *error = "not measuring in this step";
      return false;
  }
}

void BalanceJob::fill_plane_(int p, cplx u) {
  PlaneResult &pr = result_.plane[p];
  const PlaneSetup &ps = cfg_.plane[p];
  pr.u = u;
  pr.mass = to_mass(u, ps.corr_radius_mm, ps.remove_mode);
  pr.mass.angle_deg = to_user_angle(pr.mass.angle_deg);
  pr.use_split = ps.positions >= 3;
  if (pr.use_split)
    pr.split = split_to_positions(pr.mass.mass_g, pr.mass.angle_deg, ps.positions, ps.positions_offset_deg);
  pr.drill_depth_mm = ps.remove_mode ? drill_depth_mm(pr.mass.mass_g, cfg_.drill_diameter_mm, cfg_.density_g_cm3) : 0;
}

bool BalanceJob::compute_(std::string *error) {
  const int planes = cfg_.planes;
  cplx t[2] = {trial_vector(0), trial_vector(1)};
  std::string err;
  if (!build_influence(v0_, vt_, t, kChannels, planes, model_, &err)) {
    result_.ok = false;
    result_.error = err;
    if (error) *error = err;
    return false;
  }
  model_.rpm = test_rpm_;
  Correction c = solve_correction(model_, v0_, &err);
  if (!c.ok) {
    result_.ok = false;
    result_.error = err;
    if (error) *error = err;
    return false;
  }
  result_.condition = c.condition;
  const double total = iso_uper_gmm(cfg_.grade, cfg_.rotor_mass_kg, cfg_.service_rpm);
  for (int p = 0; p < planes; p++) {
    cplx u = c.u[p];
    if (cfg_.keep_last_trial && p == planes - 1) u -= t[p];  // that weight is already on
    fill_plane_(p, u);
    const double share = planes == 1 ? 1.0 : (p == 0 ? cfg_.plane1_load_fraction : 1.0 - cfg_.plane1_load_fraction);
    result_.plane[p].iso_limit_gmm = total * share;
  }
  result_.ok = true;
  result_.error.clear();
  return true;
}

}  // namespace bal
