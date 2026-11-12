#pragma once
// Synthetic tire rig with digital accelerometers, for host tests.
//
// A wheel on a soft (leaf spring) support, spun by hand and left to coast.
// Two MPU-6050-class sensors, each on its own oscillator, report a reading
// on every data-ready edge; the edge is timestamped by the same free-running
// 32-bit capture timer that timestamps the tach. The physics is the same
// linear model as sim_rotor.h: accel(g) = (omega/omega_ref)^2 * A_true * U,
// which on a soft rig means a displacement that does not depend on speed.
//
// Each reading carries the sensor's low-pass group delay, a tilt offset,
// white noise, 16-bit quantisation, timestamp jitter, and now and then a
// reading is lost.

#include <cmath>
#include <cstdint>
#include <functional>
#include <random>

#include "dsp_types.h"

namespace sim {

using bal::cplx;

struct MpuRig {
  double timer_hz = 80e6;
  double rate[2] = {249.2, 251.3};  // each sensor's own output rate, Hz
  double start[2] = {0.0031, 0.0007};  // first reading of each sensor, s
  double dlpf_delay_s = 0.0049;     // 44 Hz filter
  double noise_g_rms = 0.0033;      // per reading (400 ug/rtHz in a 44 Hz band)
  double lsb_g = 1.0 / 16384.0;     // +/-2 g range
  double offset_g[2] = {0.03, -0.05};  // mounting tilt
  double ts_jitter_s = 3e-6;
  double drop_prob = 0.002;         // readings lost (task late, I2C retry)
  double rpm_ref = 240.0;
  cplx a_true[2][2];                // g per g*mm at rpm_ref, lag-complex [sensor][plane]
  cplx u0[2];                       // initial unbalance per plane, g*mm
  double second_order_frac = 0.2;
  double nonsync_hz = 1.3, nonsync_g = 0.002;  // the rig swaying at its own resonance
  double tach_jitter_s = 30e-6;
  uint32_t tick_offset = 0xFFFFFFFFu - 240000000u;  // the timer wraps 3 s in
  double time_origin_s = 0;         // capture-timer time at which this call starts (keeps time moving
                                    // forward when the wheel is spun again)
  uint32_t seed = 1;
};

// `passes` hand spins from rpm_hi coasting down to rpm_lo with time constant
// tau_s, back to back. Calls on_reading(sensor, ticks, g) and on_tach(ticks)
// in time order, the way the acquisition task sees them. Returns the
// simulated duration in seconds.
inline double run_hand_spins(const MpuRig &m, const cplx *u_extra, int passes, double rpm_hi, double rpm_lo,
                             double tau_s, const std::function<void(int, uint32_t, float)> &on_reading,
                             const std::function<void(uint32_t)> &on_tach) {
  std::mt19937 rng(m.seed);
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  const double w_ref = m.rpm_ref * bal::kTwoPi / 60.0;
  const double w_hi = rpm_hi * bal::kTwoPi / 60.0;
  cplx resp[2];
  for (int s = 0; s < 2; s++)
    resp[s] = m.a_true[s][0] * (m.u0[0] + u_extra[0]) + m.a_true[s][1] * (m.u0[1] + u_extra[1]);
  const double ph2[2] = {37.0, 211.0};
  const double t_pass = -tau_s * std::log(rpm_lo / rpm_hi);
  auto ticks_at = [&](double t) {
    return static_cast<uint32_t>(m.tick_offset +
                                 static_cast<uint64_t>(std::llround((m.time_origin_s + t) * m.timer_hz)));
  };

  double pass_start = 0;
  double theta_base = 0;
  int pass = 0;
  auto theta_at = [&](double t) { return theta_base + w_hi * tau_s * (1.0 - std::exp(-(t - pass_start) / tau_s)); };
  auto omega_at = [&](double t) { return w_hi * std::exp(-(t - pass_start) / tau_s); };
  // Time at which the rotor reaches angle th in the current pass.
  auto time_of = [&](double th) {
    const double left = 1.0 - (th - theta_base) / (w_hi * tau_s);
    return left > 0 ? pass_start - tau_s * std::log(left) : 1e300;  // never reached in this pass
  };

  double next_reading[2] = {m.start[0], m.start[1]};
  double next_tach_theta = 0;
  for (;;) {
    const double pass_end = pass_start + t_pass;
    const double t_tach = time_of(next_tach_theta);
    int which = 2;  // 0/1: sensor reading, 2: tach
    double t = t_tach;
    for (int s = 0; s < 2; s++) {
      if (next_reading[s] < t) {
        t = next_reading[s];
        which = s;
      }
    }
    if (t >= pass_end) {
      // The wheel is spun up again: restart cleanly on a whole revolution.
      if (++pass >= passes) return pass_end;
      theta_base = std::ceil(theta_at(pass_end) / bal::kTwoPi) * bal::kTwoPi;
      pass_start = pass_end;
      next_tach_theta = theta_base;
      continue;
    }
    if (which == 2) {
      on_tach(ticks_at(t + m.tach_jitter_s * gauss(rng)));
      next_tach_theta += bal::kTwoPi;
      continue;
    }
    const int s = which;
    next_reading[s] += 1.0 / m.rate[s];
    if (uni(rng) < m.drop_prob) continue;
    const double td = t - m.dlpf_delay_s;  // what the filter outputs now happened this long ago
    const double th = theta_at(td);
    const double w = omega_at(td);
    const cplx r = resp[s] * (w / w_ref) * (w / w_ref);
    double x = std::abs(r) * std::cos(th - std::arg(r));
    x += m.second_order_frac * std::abs(r) * std::cos(2 * th - ph2[s] / bal::kDegPerRad);
    x += m.nonsync_g * std::cos(bal::kTwoPi * m.nonsync_hz * t + s);
    x += m.offset_g[s] + m.noise_g_rms * gauss(rng);
    x = std::round(x / m.lsb_g) * m.lsb_g;
    on_reading(s, ticks_at(t + m.ts_jitter_s * gauss(rng)), static_cast<float>(x));
  }
}

}  // namespace sim
