#include "dsp_timebase.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace balancer {

void TachTimebase::configure(double timer_hz, double sample_rate, uint32_t prescale) {
  timer_hz_ = timer_hz;
  fs_ = sample_rate;
  prescale_ = prescale;
  reset();
}

void TachTimebase::reset() {
  n_ = head_ = 0;
  have_ = aligned_ = false;
  last64_ = 0;
  pending_n_ = ready_n_ = 0;
  measured_fs_ = 0;
}

int64_t TachTimebase::unwrap_(uint32_t ticks) const {
  if (!have_) return static_cast<int64_t>(ticks);
  // Nearest 64-bit value to the latest LRCK capture (they arrive every
  // ~16 ms, far inside half the 32-bit range).
  return last64_ + static_cast<int32_t>(ticks - static_cast<uint32_t>(last64_));
}

void TachTimebase::on_lrck(uint32_t ticks) {
  const int64_t t = unwrap_(ticks);
  const double nominal = timer_hz_ * prescale_ / fs_;
  if (n_ > 0) {
    const int prev = (head_ + kHist - 1) % kHist;
    const int64_t dt = t - lrck_ticks_[prev];
    const int64_t groups = std::llround(static_cast<double>(dt) / nominal);
    if (groups < 1 || groups > 8) {
      // Nonsense interval (timer restarted, long stall): start over.
      n_ = 0;
      aligned_ = false;
      realigns_++;
    } else {
      lrck_frame_[head_] = aligned_ ? lrck_frame_[prev] + groups * static_cast<int64_t>(prescale_) : -1;
      measured_fs_ = timer_hz_ * static_cast<double>(groups * prescale_) / static_cast<double>(dt);
    }
  }
  if (n_ == 0) lrck_frame_[head_] = -1;
  lrck_ticks_[head_] = t;
  head_ = (head_ + 1) % kHist;
  if (n_ < kHist) n_++;
  last64_ = t;
  have_ = true;
  place_pending_();
}

void TachTimebase::on_soft(uint32_t ticks, uint32_t frames_done) {
  if (n_ < 2) return;
  const int64_t s = unwrap_(ticks);
  const int last = (head_ + kHist - 1) % kHist;
  const int prev = (head_ + kHist - 2) % kHist;
  const int64_t span = lrck_ticks_[last] - lrck_ticks_[prev];
  const double nominal = timer_hz_ * prescale_ / fs_;
  const int64_t groups = std::max<int64_t>(1, std::llround(static_cast<double>(span) / nominal));
  const double ticks_per_frame = static_cast<double>(span) / static_cast<double>(groups * prescale_);
  // The LRCK rising edge sits half a frame into frame n. The DMA-complete
  // instant is the start of frame `frames_done` plus a small latency, so
  //   x = frames_done - (s - edge)/T = n + 0.5 - latency/T
  const double x = static_cast<double>(frames_done) - static_cast<double>(s - lrck_ticks_[last]) / ticks_per_frame;
  const int64_t n = std::llround(x - 0.4);
  if (!aligned_) {
    int idx = last;
    int64_t f = n;
    for (int i = 0; i < n_; i++) {
      lrck_frame_[idx] = f;
      idx = (idx + kHist - 1) % kHist;
      f -= prescale_;
    }
    aligned_ = true;
    place_pending_();
  } else if (std::llabs(n - lrck_frame_[last]) > 1) {
    // The two clocks disagree by more than a frame: something was lost.
    aligned_ = false;
    for (int i = 0; i < kHist; i++) lrck_frame_[i] = -1;
    realigns_++;
  }
}

void TachTimebase::on_tach(uint32_t ticks) {
  if (pending_n_ >= kPending) {
    dropped_tach_++;
    return;
  }
  pending_[pending_n_++] = unwrap_(ticks);
  place_pending_();
}

void TachTimebase::place_pending_() {
  if (!aligned_ || n_ < 2) return;
  int keep = 0;
  const int newest = (head_ + kHist - 1) % kHist;
  for (int i = 0; i < pending_n_; i++) {
    const int64_t t = pending_[i];
    if (t >= lrck_ticks_[newest]) {  // wait for a later LRCK edge
      pending_[keep++] = t;
      continue;
    }
    bool placed = false;
    int b = newest;
    for (int k = 1; k < n_; k++) {
      const int a = (b + kHist - 1) % kHist;
      if (t >= lrck_ticks_[a] && t < lrck_ticks_[b] && lrck_frame_[a] >= 0 && lrck_frame_[b] >= 0) {
        const double frac = static_cast<double>(t - lrck_ticks_[a]) / static_cast<double>(lrck_ticks_[b] - lrck_ticks_[a]);
        const double f = static_cast<double>(lrck_frame_[a]) + 0.5 +
                         frac * static_cast<double>(lrck_frame_[b] - lrck_frame_[a]);
        if (ready_n_ < kPending) ready_[ready_n_++] = f;
        placed = true;
        break;
      }
      b = a;
    }
    if (!placed) dropped_tach_++;  // older than the history
  }
  pending_n_ = keep;
}

bool TachTimebase::pop_tach(double &frame_index) {
  if (ready_n_ == 0) return false;
  frame_index = ready_[0];
  for (int i = 1; i < ready_n_; i++) ready_[i - 1] = ready_[i];
  ready_n_--;
  return true;
}

}  // namespace balancer
