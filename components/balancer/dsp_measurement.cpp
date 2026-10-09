#include "dsp_measurement.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace bal {

double default_amp_floor(Quantity q) {
  switch (q) {
    case Quantity::ACCEL_G:
      return 0.0003;
    case Quantity::VELOCITY_MM_S:
      return 0.02;
    case Quantity::DISPLACEMENT_UM:
      return 0.15;
  }
  return 0.02;
}

const char *rev_status_str(RevStatus s) {
  switch (s) {
    case RevStatus::ACCEPTED:
      return "accepted";
    case RevStatus::NOT_LOCKED:
      return "speed not locked";
    case RevStatus::BELOW_WINDOW:
      return "below window";
    case RevStatus::ABOVE_WINDOW:
      return "above window";
    case RevStatus::SPEED_UNSTABLE:
      return "speed unstable";
    case RevStatus::FULL:
      return "full";
  }
  return "?";
}

void Measurement::configure(const MeasureConfig &cfg) {
  cfg_ = cfg;
  reset();
}

void Measurement::reset() {
  revs_.clear();
  revs_.reserve(std::min(cfg_.max_revs, 4096));
  for (int c = 0; c < kChannels; c++) {
    for (int h = 0; h < kMaxOrder; h++) order_sum_[c][h] = 0;
    for (int b = 0; b < kWaveBins; b++) wave_sum_[c][b] = 0;
  }
  for (auto &r : rejected_) r = 0;
}

RevStatus Measurement::add(const RevResult &rev) {
  RevStatus st = RevStatus::ACCEPTED;
  if (cfg_.target_rpm <= 0) {
    st = RevStatus::NOT_LOCKED;
  } else if (rev.rpm < cfg_.target_rpm * (1.0 - cfg_.window)) {
    st = RevStatus::BELOW_WINDOW;
  } else if (rev.rpm > cfg_.target_rpm * (1.0 + cfg_.window)) {
    st = RevStatus::ABOVE_WINDOW;
  } else if (std::fabs(rev.period_change) > cfg_.max_period_change) {
    st = RevStatus::SPEED_UNSTABLE;
  } else if (static_cast<int>(revs_.size()) >= cfg_.max_revs) {
    st = RevStatus::FULL;
  }
  if (st != RevStatus::ACCEPTED) {
    rejected_[static_cast<int>(st)]++;
    return st;
  }

  Rev r;
  r.rpm = rev.rpm;
  for (int c = 0; c < kChannels; c++) {
    r.v[c] = convert_accel(rev.order[c][0], cfg_.quantity, 1, rev.omega);
    for (int h = 0; h < kMaxOrder; h++)
      order_sum_[c][h] += std::abs(convert_accel(rev.order[c][h], cfg_.quantity, h + 1, rev.omega));
    for (int b = 0; b < kWaveBins; b++) wave_sum_[c][b] += rev.wave[c][b];
  }
  revs_.push_back(r);
  return RevStatus::ACCEPTED;
}

void Measurement::waveform(int channel, float *out) const {
  const double n = revs_.empty() ? 1.0 : static_cast<double>(revs_.size());
  for (int b = 0; b < kWaveBins; b++) out[b] = static_cast<float>(wave_sum_[channel][b] / n);
}

MeasureStats Measurement::stats() const {
  MeasureStats s;
  for (int i = 0; i < 6; i++) s.rejected[i] = rejected_[i];
  const int n = static_cast<int>(revs_.size());
  s.revs = n;
  if (n == 0) return s;

  double rpm_sum = 0;
  s.rpm_min = std::numeric_limits<double>::max();
  s.rpm_max = 0;
  for (const auto &r : revs_) {
    rpm_sum += r.rpm;
    s.rpm_min = std::min(s.rpm_min, r.rpm);
    s.rpm_max = std::max(s.rpm_max, r.rpm);
  }
  s.rpm_mean = rpm_sum / n;

  bool all_settled = true;
  for (int c = 0; c < kChannels; c++) {
    ChannelStats &cs = s.ch[c];
    cplx sum{0, 0};
    for (const auto &r : revs_) sum += r.v[c];
    cs.mean = sum / static_cast<double>(n);
    cs.amp = std::abs(cs.mean);
    cs.phase_deg = arg_deg(cs.mean);
    for (int h = 0; h < kMaxOrder; h++) cs.order_amp[h] = order_sum_[c][h] / n;

    // Per-revolution scatter.
    double var_amp = 0, var_ph = 0, var_vec = 0;
    for (const auto &r : revs_) {
      const double da = std::abs(r.v[c]) - cs.amp;
      const double dp = wrap180(arg_deg(r.v[c]) - cs.phase_deg);
      var_amp += da * da;
      var_ph += dp * dp;
      var_vec += std::norm(r.v[c] - cs.mean);
    }
    if (n > 1) {
      cs.amp_sd = std::sqrt(var_amp / (n - 1));
      cs.phase_sd_deg = std::sqrt(var_ph / (n - 1));
    }

    // Standard error of the mean vector. Uses block means when there are
    // enough blocks, which stays honest if neighbouring revolutions are
    // correlated (slow wander), otherwise the per-revolution estimate.
    const int bs = std::max(1, cfg_.block_revs);
    const int nb = n / bs;
    if (nb >= 3) {
      double var_b = 0;
      std::vector<cplx> bm(nb);
      for (int b = 0; b < nb; b++) {
        cplx t{0, 0};
        for (int i = 0; i < bs; i++) t += revs_[b * bs + i].v[c];
        bm[b] = t / static_cast<double>(bs);
      }
      cplx bmean{0, 0};
      for (auto &v : bm) bmean += v;
      bmean /= static_cast<double>(nb);
      for (auto &v : bm) var_b += std::norm(v - bmean);
      cs.std_error = std::sqrt(var_b / (nb - 1) / nb);
    } else if (n > 1) {
      cs.std_error = std::sqrt(var_vec / (n - 1) / n);
    } else {
      cs.std_error = std::numeric_limits<double>::infinity();
    }
    cs.phase_unc_deg = cs.amp > 0 ? std::atan2(cs.std_error, cs.amp) * kDegPerRad : 180.0;
    cs.settled = (cs.phase_unc_deg <= cfg_.max_phase_unc_deg) || (cs.std_error <= cfg_.amp_floor);
    all_settled = all_settled && cs.settled;

    // Phase-vs-speed slope (deg per 1% speed). Only meaningful when the
    // revolutions span some speed range.
    const double spread_pct = (s.rpm_max - s.rpm_min) / s.rpm_mean * 100.0;
    if (n >= 10 && spread_pct >= 0.5) {
      double sx = 0, sy = 0, sxx = 0, sxy = 0;
      for (const auto &r : revs_) {
        const double x = (r.rpm / s.rpm_mean - 1.0) * 100.0;
        const double y = wrap180(arg_deg(r.v[c]) - cs.phase_deg);
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
      }
      const double den = n * sxx - sx * sx;
      cs.phase_slope = den > 0 ? (n * sxy - sx * sy) / den : std::numeric_limits<double>::quiet_NaN();
    } else {
      cs.phase_slope = std::numeric_limits<double>::quiet_NaN();
    }
  }
  s.ready = n >= cfg_.target_revs && all_settled;
  return s;
}

}  // namespace bal
