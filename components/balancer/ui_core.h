#pragma once
// Shared implementation of UiModel: job, runs, live averages, demo rotor.
// The ESPHome component derives from it and adds acquisition + persistence;
// tools/ui_preview uses it directly.

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "dsp_sim.h"
#include "ui_model.h"

namespace bal {

class ModelCore : public UiModel {
 public:
  ModelCore();

  // Feed one analysed revolution (from acquisition or the demo rotor).
  void feed_rev(const RevResult &r, uint32_t now_ms);
  // Call often (every loop); recomputes cached statistics at ~5 Hz and steps
  // the demo rotor.
  void tick(uint32_t now_ms);

  void set_simulate(bool on) { simulate_ = on; }
  // Load the starting values for a kind of work (job, measurement settings,
  // wording). Call before restoring anything saved.
  void apply_preset(Preset p);
  Preset preset() const { return preset_; }
  void set_digital_sensor(bool on) { digital_sensor_ = on; }
  // Largest revolution-to-revolution speed change a revolution may have and
  // still count (fraction of the period).
  void set_max_period_change(double f) { max_period_change_ = f; }
  void set_best_effort_factor(int n) { best_effort_factor_ = n; }
  // How well a reading has to be known before it counts as settled even
  // though its phase is not (a balanced rotor has no phase), as a
  // displacement in um. 0 keeps the per-unit defaults.
  void set_amp_floor_um(double um) { amp_floor_um_ = um; }
  LiveData &live_mut() { return live_; }
  void load_machine(const InfluenceModel &m, double rpm, const JobConfig &cfg) {
    machine_ = m;
    machine_rpm_ = rpm;
    machine_cfg_ = cfg;
    has_machine_ = m.valid;
  }
  const InfluenceModel &machine() const { return machine_; }
  double machine_rpm() const { return machine_rpm_; }
  const JobConfig &machine_cfg() const { return machine_cfg_; }

  // --- UiModel ---
  BalanceJob &job() override { return job_; }
  UiSettings &settings() override { return settings_; }
  void apply_settings(bool persist) override;
  void save_job_defaults() override { persist_job_defaults(); }
  const LiveData &live() override { return live_; }
  void start_run() override;
  void cancel_run() override { measuring_ = false; }
  bool accept_run(std::string *error) override;
  bool measuring() const override { return measuring_; }
  const MeasureStats &run_stats() override { return stats_; }
  RevStatus last_rev_status() const override { return last_status_; }
  const std::vector<cplx> &run_dots(int ch) override { return dots_[ch]; }
  void lock_speed(double rpm) override;
  bool has_machine() const override { return has_machine_; }
  void save_machine() override;
  bool start_trim() override;
  std::string machine_summary() const override;
  bool digital_sensor() const override { return digital_sensor_; }
  bool simulate() const override { return simulate_; }
  void sim_blast() override;

 protected:
  // Hooks for the device.
  virtual void persist_settings() {}
  virtual void persist_job_defaults() {}
  virtual void persist_machine() {}
  virtual void on_settings_changed() {}

  void configure_measurement_();
  void set_sim_fitted_for_step_();

  BalanceJob job_;
  UiSettings settings_;
  Measurement meas_;
  MeasureStats stats_;
  bool measuring_ = false;
  RevStatus last_status_ = RevStatus::NOT_LOCKED;
  std::vector<cplx> dots_[kChannels];
  LiveData live_;
  double last_rpm_ = 0;
  uint32_t last_rev_ms_ = 0;
  uint32_t last_stats_ms_ = 0;
  uint32_t last_tick_ms_ = 0;

  struct LiveRev {
    double omega;
    double rpm;
    cplx o[kChannels][kMaxOrder];
    float wave[kChannels][kWaveBins];
  };
  std::deque<LiveRev> live_revs_;
  static constexpr size_t kLiveRevs = 16;

  bool has_machine_ = false;
  InfluenceModel machine_;
  double machine_rpm_ = 0;
  JobConfig machine_cfg_;

  Preset preset_ = Preset::MOTOR_ROTOR;
  bool digital_sensor_ = false;
  double max_period_change_ = 0.01;
  int best_effort_factor_ = 0;
  double amp_floor_um_ = 0;

  bool simulate_ = false;
  RevSim sim_;
  cplx sim_fitted_[2];
  cplx sim_correction_total_[2];
  bool sim_verify_applied_ = false;  // corrections for this result already fitted
};

}  // namespace bal
