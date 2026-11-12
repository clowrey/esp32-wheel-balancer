#include "dsp_resample.h"

#include <cmath>

namespace bal {

namespace {
// A sensor that stays quiet longer than this many output samples restarts
// the grid; shorter holes are bridged by interpolation.
constexpr double kMaxGapSamples = 50.0;
// One channel running this far ahead of the other means the other sensor
// has stopped answering.
constexpr size_t kMaxBacklog = 512;
}  // namespace

void StreamResampler::configure(double timer_hz, double out_fs, int channels) {
  timer_hz_ = timer_hz;
  fs_ = out_fs;
  ticks_per_sample_ = timer_hz / out_fs;
  channels_ = channels < 2 ? 1 : 2;
  restarts_ = 0;
  reset();
}

void StreamResampler::reset() {
  for (auto &c : ch_) c = Chan();
  started_ = false;
  have_last_ = false;
  tach_.clear();
  frames_out_ = 0;
  restart_flag_ = false;
}

int64_t StreamResampler::unwrap_(uint32_t ticks) {
  // Events arrive within milliseconds of each other, far inside half the
  // 32-bit range, so the nearest 64-bit value to the previous event is right.
  int64_t t = static_cast<int64_t>(ticks);
  if (have_last_) t = last64_ + static_cast<int32_t>(ticks - static_cast<uint32_t>(last64_));
  last64_ = t;
  have_last_ = true;
  return t;
}

void StreamResampler::restart_() {
  for (auto &c : ch_) {
    const double rate = c.rate;
    c = Chan();
    c.rate = rate;
  }
  started_ = false;
  tach_.clear();
  frames_out_ = 0;
  restarts_++;
  restart_flag_ = true;
}

bool StreamResampler::take_restart() {
  const bool r = restart_flag_;
  restart_flag_ = false;
  return r;
}

void StreamResampler::push(int ch, uint32_t ticks, float value) {
  if (ch < 0 || ch >= channels_) return;
  const int64_t t = unwrap_(ticks);
  Chan &c = ch_[ch];

  if (c.have) {
    const int64_t dt = t - c.t;
    if (dt <= 0) return;  // duplicate or out of order: ignore
    // Either this sensor was silent for too long, or the other one is: start
    // a new grid with this reading as its first.
    if (static_cast<double>(dt) > kMaxGapSamples * ticks_per_sample_ || c.out.size() > kMaxBacklog) restart_();
  }
  if (!started_) {
    origin_ = t;
    started_ = true;
  }
  if (!c.have) {
    // First reading of this channel: hold its value back to the origin so
    // every channel covers the same grid indices.
    const int64_t k_now = static_cast<int64_t>(std::floor(static_cast<double>(t - origin_) / ticks_per_sample_));
    for (int64_t k = c.next_k; k <= k_now; k++) c.out.push_back(value);
    c.next_k = k_now + 1;
    c.have = true;
    c.t = t;
    c.v = value;
    return;
  }

  const double span = static_cast<double>(t - c.t);
  // The sensor's own rate, smoothed; an interval with lost readings is an
  // outlier and is left out unless it keeps happening.
  const double inst = timer_hz_ / span;
  if (c.rate <= 0 || c.rate_outliers > 8) {
    c.rate = inst;
    c.rate_outliers = 0;
  } else if (std::fabs(inst - c.rate) < 0.2 * c.rate) {
    c.rate += 0.05 * (inst - c.rate);
    c.rate_outliers = 0;
  } else {
    c.rate_outliers++;
  }

  // Emit every grid point in (previous reading, this reading].
  for (;;) {
    const double tk = static_cast<double>(origin_) + static_cast<double>(c.next_k) * ticks_per_sample_;
    if (tk > static_cast<double>(t)) break;
    const double f = (tk - static_cast<double>(c.t)) / span;
    c.out.push_back(c.v + static_cast<float>(f) * (value - c.v));
    c.next_k++;
  }
  c.t = t;
  c.v = value;
}

void StreamResampler::push_tach(uint32_t ticks) {
  const int64_t t = unwrap_(ticks);
  if (!started_) return;
  const double idx = static_cast<double>(t - origin_) / ticks_per_sample_;
  if (idx < 0) return;
  tach_.push_back(idx);
  while (tach_.size() > 64) tach_.pop_front();
}

size_t StreamResampler::pop_frames(float *ch0, float *ch1, size_t max) {
  size_t n = ch_[0].out.size();
  if (channels_ == 2 && ch_[1].out.size() < n) n = ch_[1].out.size();
  if (n > max) n = max;
  for (size_t i = 0; i < n; i++) {
    ch0[i] = ch_[0].out.front();
    ch_[0].out.pop_front();
    if (channels_ == 2) {
      ch1[i] = ch_[1].out.front();
      ch_[1].out.pop_front();
    } else {
      ch1[i] = 0.0f;
    }
  }
  frames_out_ += n;
  return n;
}

bool StreamResampler::pop_tach(double &grid_index) {
  if (tach_.empty()) return false;
  grid_index = tach_.front();
  tach_.pop_front();
  return true;
}

}  // namespace bal
