#pragma once
// Speed-window accumulator for one measurement run (PLAN §5, §5.1).
//
// Revolutions arrive one at a time from RevolutionAnalyzer. Only those whose
// speed is inside the target window and whose speed did not change too fast
// are kept. The run is "ready" once enough revolutions are in and the 1x
// phase is known well enough on every channel. Several coast-down passes
// simply keep adding revolutions until that happens.

#include <cstdint>
#include <vector>

#include "dsp_revolution.h"
#include "dsp_types.h"

namespace balancer {

struct MeasureConfig {
  double target_rpm = 0;            // 0 = not locked yet (nothing is accepted)
  double window = 0.03;             // +/- fraction around target_rpm
  int target_revs = 30;             // revolutions needed before "ready"
  int max_revs = 4000;              // stop accepting after this many
  double max_period_change = 0.01;  // reject revs whose period jumped more than this
  double max_phase_unc_deg = 2.0;   // 1x phase uncertainty needed for "ready"
  double amp_floor = 0.02;          // channel counts as settled if its 1x is below this (units of quantity)
  int block_revs = 10;              // block size for the uncertainty estimate
  Quantity quantity = Quantity::VELOCITY_MM_S;
};

enum class RevStatus : uint8_t {
  ACCEPTED = 0,
  NOT_LOCKED,      // no target speed yet
  BELOW_WINDOW,
  ABOVE_WINDOW,
  SPEED_UNSTABLE,  // period changed too much since the previous rev
  FULL,            // max_revs reached
};

const char *rev_status_str(RevStatus s);

struct ChannelStats {
  cplx mean{0, 0};          // 1x, lag-complex, in the configured quantity
  double amp = 0;           // |mean|
  double phase_deg = 0;     // lag, 0..360
  double amp_sd = 0;        // per-revolution scatter of the 1x amplitude
  double phase_sd_deg = 0;  // per-revolution scatter of the 1x phase
  double std_error = 0;     // standard error of the mean vector (same units)
  double phase_unc_deg = 180;  // atan(std_error / amp)
  double phase_slope = 0;   // d(phase)/d(speed), degrees per 1% speed change (NaN if unknown)
  bool settled = false;     // phase_unc ok, or amplitude below the floor
  double order_amp[kMaxOrder] = {0};  // mean amplitude per order (for the spectrum)
};

struct MeasureStats {
  int revs = 0;
  double rpm_mean = 0, rpm_min = 0, rpm_max = 0;
  ChannelStats ch[kChannels];
  bool ready = false;
  uint32_t rejected[6] = {0};  // count by RevStatus
};

class Measurement {
 public:
  explicit Measurement(const MeasureConfig &cfg = MeasureConfig()) { configure(cfg); }

  void configure(const MeasureConfig &cfg);
  const MeasureConfig &config() const { return cfg_; }
  void reset();  // keeps the config (and target speed)
  void set_target_rpm(double rpm) { cfg_.target_rpm = rpm; }

  RevStatus add(const RevResult &rev);

  MeasureStats stats() const;
  int revs() const { return static_cast<int>(revs_.size()); }
  // Synchronously averaged waveform of the accepted revolutions, in g.
  void waveform(int channel, float *out_bins) const;

 private:
  struct Rev {
    double rpm;
    cplx v[kChannels];
  };
  MeasureConfig cfg_;
  std::vector<Rev> revs_;
  double order_sum_[kChannels][kMaxOrder];
  double wave_sum_[kChannels][kWaveBins];
  uint32_t rejected_[6];
};

}  // namespace balancer
