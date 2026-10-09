#pragma once
// One balancing job: the wizard's state and arithmetic, with no UI or
// hardware in it (PLAN §6, §7). The UI asks it what to tell the operator and
// hands it finished measurements; it hands back the correction per plane.

#include <cstdint>
#include <string>

#include "dsp_balance.h"
#include "dsp_measurement.h"
#include "dsp_types.h"

namespace balancer {

struct PlaneSetup {
  double trial_mass_g = 3.5;
  double trial_angle_deg = 0;   // user convention (see JobConfig::angles_with_rotation)
  double trial_radius_mm = 40;
  double corr_radius_mm = 40;
  bool remove_mode = true;      // drill out instead of adding
  int positions = 0;            // 0 = any angle; N = only N equally spaced spots
  double positions_offset_deg = 0;  // angle of spot #1 (user convention)
};

struct JobConfig {
  int planes = 2;                    // 1 (static) or 2 (dynamic)
  double rotor_mass_kg = 3.5;
  double service_rpm = 1750;
  double grade = 2.5;                // ISO 21940-11 G number, mm/s
  double plane1_load_fraction = 0.5; // share of the ISO limit given to plane 1
  bool angles_with_rotation = false; // false: angles measured against rotation
  bool keep_last_trial = false;      // last trial weight stays fitted
  double drill_diameter_mm = 6.0;
  double density_g_cm3 = 2.7;        // aluminium end rings
  double window = 0.03;              // speed window used for every run (+/- fraction)
  PlaneSetup plane[2];
};

enum class JobStep : uint8_t {
  SETUP = 0,
  RUN0,     // as found
  TRIAL1,   // trial weight on plane 1
  TRIAL2,   // trial weight on plane 2 (dynamic only)
  RESULT,   // correction computed
  VERIFY,   // measuring after the correction is fitted
  VERIFIED, // verify result available; trim possible
};

const char *job_step_name(JobStep s);

struct PlaneResult {
  cplx u{0, 0};          // g*mm to add (internal convention)
  MassAngle mass;        // at the correction radius, user angle convention
  SplitResult split;     // if the plane has fixed positions (user convention)
  bool use_split = false;
  double drill_depth_mm = 0;  // remove mode only
  double iso_limit_gmm = 0;   // permissible residual for this plane
};

struct JobResult {
  bool ok = false;
  std::string error;
  PlaneResult plane[2];
  double condition = 0;
  TrialCheck trial_check[2];
  // after a verify run:
  bool have_residual = false;
  cplx residual_u[2];          // estimated remaining unbalance per plane, g*mm
  double residual_gmm[2] = {0, 0};
  bool within_iso[2] = {false, false};
  double reduction_pct[2] = {0, 0};  // vibration reduction per sensor vs as-found
};

class BalanceJob {
 public:
  void reset(const JobConfig &cfg);
  const JobConfig &config() const { return cfg_; }
  JobConfig &mutable_config() { return cfg_; }

  JobStep step() const { return step_; }
  void set_step(JobStep s) { step_ = s; }
  // Advance from SETUP to RUN0.
  void start();

  // Test speed (rpm), locked at RUN0 and used for every later run.
  double test_rpm() const { return test_rpm_; }
  void set_test_rpm(double rpm) { test_rpm_ = rpm; }

  // Operator instruction for the current step.
  std::string instruction() const;
  // Suggested trial mass (g) for a plane at the locked or service speed.
  double suggested_trial_g(int plane) const;

  // Store a finished measurement for the current step and advance.
  // Returns false (with error) if the result could not be computed.
  bool record(const MeasureStats &stats, std::string *error = nullptr);
  // Start a verify/trim run after the correction is fitted.
  void begin_verify() { step_ = JobStep::VERIFY; }

  const JobResult &result() const { return result_; }
  const InfluenceModel &model() const { return model_; }
  // Load a saved model (from a previous job on the same machine) so a
  // single verify run gives a trim correction.
  void load_model(const InfluenceModel &m, double test_rpm);

  // Angle conversion between internal (against rotation) and user convention.
  double to_user_angle(double internal_deg) const;
  double to_internal_angle(double user_deg) const;
  cplx trial_vector(int plane) const;  // g*mm, internal convention

  const cplx *v0() const { return v0_; }
  const cplx *vt(int plane) const { return vt_[plane]; }
  const cplx *vverify() const { return vver_; }

 private:
  bool compute_(std::string *error);
  void fill_plane_(int p, cplx u);

  JobConfig cfg_;
  JobStep step_ = JobStep::SETUP;
  double test_rpm_ = 0;
  cplx v0_[kMaxSensors];
  cplx vt_[2][kMaxSensors];
  cplx vver_[kMaxSensors];
  InfluenceModel model_;
  JobResult result_;
};

}  // namespace balancer
