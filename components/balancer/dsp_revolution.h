#pragma once
// Per-revolution analysis: takes a continuous two-channel sample stream and
// tach events expressed as fractional sample indices on the same counter,
// and produces one RevResult per complete revolution: the order vectors
// (1x..10x) and a 128-bin waveform for each channel.
//
// The orders are computed by a direct DFT over the raw samples of that
// revolution (no resampling, so no aliasing of high orders into 1x).

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

#include "dsp_types.h"

namespace bal {

// Orders whose frequency is above this fraction of the sample rate are
// reported as zero.
constexpr double kMaxOrderBand = 0.45;

struct RevolutionConfig {
  double fs = 16000.0;        // sample rate, Hz
  double min_rpm = 60.0;      // slower revolutions are discarded (gap)
  double max_rpm = 12000.0;   // tach pulses closer than this are glitches
  double delay_s = 17.4 / 16000.0;  // fixed signal-chain delay to remove (PCM1808 group delay)
  size_t ring_size = 1u << 15;      // samples kept per channel (power of two)
};

struct RevResult {
  uint32_t index = 0;        // running revolution counter
  double t_start = 0;        // sample index of the tach pulse that starts it
  double period = 0;         // samples
  double rpm = 0;
  double omega = 0;          // rad/s
  double period_change = 0;  // (P - P_prev) / P_prev; 0 for the first rev after a gap
  cplx order[kChannels][kMaxOrder];   // acceleration in g, lag-complex, order h at [h-1]
  float wave[kChannels][kWaveBins];   // g, mean-removed, bin k = angle k*360/kWaveBins
};

class RevolutionAnalyzer {
 public:
  explicit RevolutionAnalyzer(const RevolutionConfig &cfg = RevolutionConfig());

  void configure(const RevolutionConfig &cfg);
  void reset();

  // Append n consecutive samples per channel (already scaled to g).
  void push_samples(const float *ch0, const float *ch1, size_t n);
  // Interleaved stereo variant: [ch0, ch1, ch0, ch1, ...], n frames.
  void push_interleaved(const float *lr, size_t n);
  // A tach pulse at a fractional sample index (same counter as the samples).
  void push_tach(double sample_index);

  // Pops the next finished revolution, if one is ready.
  bool pop(RevResult &out);

  uint64_t samples_seen() const { return seen_; }
  uint32_t glitches() const { return glitches_; }
  uint32_t gaps() const { return gaps_; }
  const RevolutionConfig &config() const { return cfg_; }

 private:
  bool analyze_(double t0, double t1, RevResult &out);

  RevolutionConfig cfg_;
  std::vector<float> ring_[kChannels];
  size_t mask_ = 0;
  uint64_t seen_ = 0;
  std::deque<double> tach_;
  double last_period_ = 0;
  uint32_t rev_index_ = 0;
  uint32_t glitches_ = 0;
  uint32_t gaps_ = 0;
};

}  // namespace bal
