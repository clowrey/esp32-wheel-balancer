#pragma once
// Uniform sample grid for digital sensors (MPU-6050 and friends). Pure C++.
//
// A digital accelerometer samples on its own oscillator, and two of them
// never agree. Each reading therefore arrives with the capture-timer value
// latched on its data-ready edge. This class interpolates every sensor onto
// one uniform grid that is defined by the capture timer itself:
//
//   grid sample k is at  origin + k * timer_hz / out_fs  ticks
//
// Tach edges are latched by the same timer, so their place on the grid is a
// plain division. The output (frames + tach indices) is exactly what
// RevolutionAnalyzer wants, and everything stays in sample-index units.

#include <cstddef>
#include <cstdint>
#include <deque>

namespace bal {

class StreamResampler {
 public:
  // channels: 1 (channel B reads zero) or 2.
  void configure(double timer_hz, double out_fs, int channels);
  void reset();

  // A reading from sensor `ch` whose data-ready edge was latched at `ticks`
  // (32-bit capture value, wraps). Readings of one sensor must be in time
  // order; missing readings are bridged by interpolation.
  void push(int ch, uint32_t ticks, float value);
  // A tach edge latched by the same timer.
  void push_tach(uint32_t ticks);

  // Uniform frames that are ready on every channel. Returns how many were
  // written (at most max).
  size_t pop_frames(float *ch0, float *ch1, size_t max);
  // Tach edges as fractional grid indices (same counter as the frames).
  bool pop_tach(double &grid_index);

  // True once after the grid had to start over (a sensor went quiet for too
  // long). The consumer must reset its analyzer: frame 0 is a new origin.
  bool take_restart();

  bool started() const { return started_; }
  double out_fs() const { return fs_; }
  uint64_t frames_out() const { return frames_out_; }
  uint32_t restarts() const { return restarts_; }
  double measured_rate(int ch) const { return ch_[ch].rate; }  // the sensor's own output rate, Hz

 private:
  struct Chan {
    bool have = false;
    int64_t t = 0;     // ticks of the latest reading
    float v = 0;
    int64_t next_k = 0;  // next grid index this channel will produce
    std::deque<float> out;
    double rate = 0;
    int rate_outliers = 0;
  };
  int64_t unwrap_(uint32_t ticks);
  void restart_();

  double timer_hz_ = 80e6;
  double fs_ = 250;
  double ticks_per_sample_ = 320000;
  int channels_ = 2;
  Chan ch_[2];
  bool started_ = false;
  int64_t origin_ = 0;
  int64_t last64_ = 0;
  bool have_last_ = false;
  std::deque<double> tach_;
  uint64_t frames_out_ = 0;
  uint32_t restarts_ = 0;
  bool restart_flag_ = false;
};

}  // namespace bal
