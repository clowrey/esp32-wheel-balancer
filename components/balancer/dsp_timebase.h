#pragma once
// Tach-to-sample timebase (PLAN §4). Pure C++ so it can be tested on the host.
//
// Inputs, all from one free-running 32-bit capture timer:
//   * on_lrck(ticks):        every `prescale`-th LRCK rising edge
//   * on_soft(ticks, frames): DMA-buffer-complete instant, with the number of
//                             frames completed so far
//   * on_tach(ticks):        tach edges
// Output: tach edges as fractional frame indices (frame 0 = first frame the
// I2S produced), ready to hand to RevolutionAnalyzer after subtracting the
// analyzer's origin.

#include <cstdint>

namespace bal {

class TachTimebase {
 public:
  void configure(double timer_hz, double sample_rate, uint32_t prescale);
  void reset();

  void on_lrck(uint32_t ticks);
  void on_soft(uint32_t ticks, uint32_t frames_done);
  void on_tach(uint32_t ticks);

  // Next tach edge that can be placed (bracketed by two LRCK captures).
  bool pop_tach(double &frame_index);

  bool aligned() const { return aligned_; }
  double measured_fs() const { return measured_fs_; }
  uint32_t realigns() const { return realigns_; }
  uint32_t dropped_tach() const { return dropped_tach_; }

 private:
  int64_t unwrap_(uint32_t ticks) const;
  void place_pending_();

  double timer_hz_ = 80e6;
  double fs_ = 16000;
  uint32_t prescale_ = 256;

  static constexpr int kHist = 16;
  int64_t lrck_ticks_[kHist];
  int64_t lrck_frame_[kHist];
  int n_ = 0;
  int head_ = 0;  // next write slot
  int64_t last64_ = 0;
  bool have_ = false;
  bool aligned_ = false;
  double measured_fs_ = 0;
  uint32_t realigns_ = 0;

  static constexpr int kPending = 32;
  int64_t pending_[kPending];
  int pending_n_ = 0;
  double ready_[kPending];
  int ready_n_ = 0;
  uint32_t dropped_tach_ = 0;
};

}  // namespace bal
