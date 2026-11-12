#include "dsp_revolution.h"

#include <algorithm>
#include <cmath>

namespace bal {

RevolutionAnalyzer::RevolutionAnalyzer(const RevolutionConfig &cfg) { configure(cfg); }

void RevolutionAnalyzer::configure(const RevolutionConfig &cfg) {
  cfg_ = cfg;
  size_t n = 1;
  while (n < cfg_.ring_size) n <<= 1;
  cfg_.ring_size = n;
  mask_ = n - 1;
  for (auto &r : ring_) std::vector<float>(n, 0.0f).swap(r);  // swap, so a smaller ring frees the old one
  reset();
}

void RevolutionAnalyzer::reset() {
  seen_ = 0;
  tach_.clear();
  last_period_ = 0;
  rev_index_ = 0;
  glitches_ = 0;
  gaps_ = 0;
}

void RevolutionAnalyzer::push_samples(const float *ch0, const float *ch1, size_t n) {
  for (size_t i = 0; i < n; i++) {
    const size_t k = (seen_ + i) & mask_;
    ring_[0][k] = ch0[i];
    ring_[1][k] = ch1[i];
  }
  seen_ += n;
}

void RevolutionAnalyzer::push_interleaved(const float *lr, size_t n) {
  for (size_t i = 0; i < n; i++) {
    const size_t k = (seen_ + i) & mask_;
    ring_[0][k] = lr[2 * i];
    ring_[1][k] = lr[2 * i + 1];
  }
  seen_ += n;
}

void RevolutionAnalyzer::push_tach(double t) {
  const double min_period = cfg_.fs * 60.0 / cfg_.max_rpm;
  if (!tach_.empty()) {
    const double dt = t - tach_.back();
    if (dt < min_period) {  // double trigger / noise: keep the first edge
      glitches_++;
      return;
    }
  }
  tach_.push_back(t);
  while (tach_.size() > 64) tach_.pop_front();  // never let a stalled consumer grow this
}

bool RevolutionAnalyzer::pop(RevResult &out) {
  const double max_period =
      std::min(cfg_.fs * 60.0 / cfg_.min_rpm, static_cast<double>(cfg_.ring_size) - 8.0);
  while (tach_.size() >= 2) {
    const double t0 = tach_[0];
    const double t1 = tach_[1];
    if (static_cast<double>(seen_) < t1 + 2.0) return false;  // samples not here yet
    const double period = t1 - t0;
    if (period > max_period) {  // rotor stopped / pulses lost: start over
      tach_.pop_front();
      last_period_ = 0;
      gaps_++;
      continue;
    }
    const bool ok = analyze_(t0, t1, out);
    tach_.pop_front();
    if (ok) return true;
  }
  return false;
}

bool RevolutionAnalyzer::analyze_(double t0, double t1, RevResult &out) {
  const double period = t1 - t0;
  const int64_t n_first = static_cast<int64_t>(std::ceil(t0));
  const int64_t n_last = static_cast<int64_t>(std::ceil(t1)) - 1;
  if (n_last < n_first + 8) return false;
  if (static_cast<uint64_t>(n_last) >= seen_) return false;
  if (seen_ > cfg_.ring_size && static_cast<uint64_t>(n_first) < seen_ - cfg_.ring_size) {
    last_period_ = 0;  // data already overwritten; consumer fell behind
    gaps_++;
    return false;
  }
  const int64_t count = n_last - n_first + 1;

  // Pass 1: mean per channel (removes any DC the AC coupling let through).
  double mean[kChannels] = {0, 0};
  for (int64_t n = n_first; n <= n_last; n++) {
    const size_t k = static_cast<size_t>(n) & mask_;
    mean[0] += ring_[0][k];
    mean[1] += ring_[1][k];
  }
  mean[0] /= count;
  mean[1] /= count;

  // Pass 2: DFT of orders 1..kMaxOrder against the tach-referenced angle,
  // plus box-car binning for the waveform. Phasor is advanced by rotation and
  // renormalised periodically, so there is no sin/cos per sample.
  cplxf acc[kChannels][kMaxOrder];
  float wsum[kChannels][kWaveBins];
  uint16_t wcnt[kWaveBins];
  for (int c = 0; c < kChannels; c++) {
    for (int h = 0; h < kMaxOrder; h++) acc[c][h] = 0.0f;
    for (int b = 0; b < kWaveBins; b++) wsum[c][b] = 0.0f;
  }
  for (int b = 0; b < kWaveBins; b++) wcnt[b] = 0;

  const double dtheta = kTwoPi / period;
  const double theta0 = (static_cast<double>(n_first) - t0) * dtheta;
  cplxf p = std::polar(1.0f, static_cast<float>(-theta0));
  const cplxf dp = std::polar(1.0f, static_cast<float>(-dtheta));
  const float m0 = static_cast<float>(mean[0]);
  const float m1 = static_cast<float>(mean[1]);
  const double bins_per_sample = kWaveBins / period;

  int64_t i = 0;
  for (int64_t n = n_first; n <= n_last; n++, i++) {
    const size_t k = static_cast<size_t>(n) & mask_;
    const float x0 = ring_[0][k] - m0;
    const float x1 = ring_[1][k] - m1;
    cplxf q = p;
    for (int h = 0; h < kMaxOrder; h++) {
      acc[0][h] += x0 * q;
      acc[1][h] += x1 * q;
      q *= p;
    }
    int bin = static_cast<int>((static_cast<double>(n) - t0) * bins_per_sample);
    bin = std::min(std::max(bin, 0), kWaveBins - 1);
    wsum[0][bin] += x0;
    wsum[1][bin] += x1;
    wcnt[bin]++;

    p *= dp;
    if ((i & 63) == 63) {
      // Re-anchor exactly to avoid float drift over long revolutions.
      p = std::polar(1.0f, static_cast<float>(-(theta0 + (i + 1) * dtheta)));
    }
  }

  out.index = rev_index_++;
  out.t_start = t0;
  out.period = period;
  out.rpm = cfg_.fs * 60.0 / period;
  out.omega = kTwoPi * cfg_.fs / period;
  out.period_change = last_period_ > 0 ? (period - last_period_) / last_period_ : 0.0;
  last_period_ = period;

  const double scale = 2.0 / static_cast<double>(count);
  for (int c = 0; c < kChannels; c++) {
    for (int h = 0; h < kMaxOrder; h++) {
      // conj(): the DFT gives A*e^{-j*lag}; we store A*e^{+j*lag}.
      cplx v = std::conj(cplx(acc[c][h].real(), acc[c][h].imag())) * scale;
      // Remove the fixed chain delay: it shows up as extra lag of h*omega*tau.
      v *= std::polar(1.0, -(h + 1) * out.omega * cfg_.delay_s);
      // An order above 0.45*fs would be an alias, not a measurement. Only
      // slow-sampling digital sensors ever get there.
      if (static_cast<double>(h + 1) > kMaxOrderBand * period) v = 0;
      out.order[c][h] = v;
    }
    for (int b = 0; b < kWaveBins; b++)
      out.wave[c][b] = wcnt[b] ? wsum[c][b] / wcnt[b] : 0.0f;
  }
  return true;
}

}  // namespace bal
