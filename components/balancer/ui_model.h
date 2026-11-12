#pragma once
// What the touchscreen UI needs from the rest of the firmware. The ESPHome
// component implements this on the device; tools/ui_preview implements it
// on a PC with the simulated wheel so screens can be rendered to images.

#include <string>
#include <vector>

#include "dsp_job.h"
#include "dsp_measurement.h"
#include "dsp_types.h"

namespace bal {

struct LiveData {
  double rpm = 0;              // 0 when no tach for a while
  bool tach_ok = false;
  int revs = 0;                // revolutions in the live average
  cplx v[kChannels];           // 1x, configured quantity, lag-complex
  double order_amp[kChannels][kMaxOrder] = {};
  float wave[kChannels][kWaveBins] = {};  // g
  float rms_g[kChannels] = {0, 0};
  float peak_frac[kChannels] = {0, 0};   // of the sensor's full-scale range
  float fs = 0;                // measured reading rate
  bool acq_ok = false;         // acquisition running
  bool aligned = false;        // timebase locked
  unsigned resyncs = 0, overflows = 0, glitches = 0, tach_pulses = 0;
};

struct UiSettings {
  bool invert[kChannels] = {false, false};
  Quantity quantity = Quantity::VELOCITY_MM_S;
  double window = 0.03;
  int target_revs = 30;
  double max_phase_unc_deg = 2.0;
};

class UiModel {
 public:
  virtual ~UiModel() = default;

  virtual BalanceJob &job() = 0;
  virtual UiSettings &settings() = 0;
  virtual void apply_settings(bool persist) = 0;
  virtual void save_job_defaults() = 0;

  virtual const LiveData &live() = 0;

  // One measurement run for the job's current step.
  virtual void start_run() = 0;
  virtual void cancel_run() = 0;
  virtual bool accept_run(std::string *error) = 0;
  virtual bool measuring() const = 0;
  virtual const MeasureStats &run_stats() = 0;
  virtual RevStatus last_rev_status() const = 0;
  virtual const std::vector<cplx> &run_dots(int channel) = 0;  // recent accepted revolutions
  // Lock the test speed (job + current run). rpm <= 0 means "current speed".
  virtual void lock_speed(double rpm) = 0;

  // Saved machine (influence model) for single-run trim balancing.
  virtual bool has_machine() const = 0;
  virtual void save_machine() = 0;
  virtual bool start_trim() = 0;
  virtual std::string machine_summary() const = 0;

  // Demo mode with the simulated wheel.
  virtual bool simulate() const = 0;
  virtual void sim_spin() = 0;
};

}  // namespace bal
