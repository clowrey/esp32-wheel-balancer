#pragma once
// Synthetic sample-level rotor for host tests of the DSP (PLAN Phase 2).
//
// A rigid rotor on a stand, two correction planes, two accelerometers.
// The true response is linear: accel(g) = (omega/omega_ref)^2 * A_true * U,
// where U is the total unbalance per plane in g*mm. The rotor is spun up and
// then coasts down exponentially; tach pulses come once per revolution. Each
// channel also gets a 2x component, a non-synchronous tone (stand/blower),
// white noise, and the ADC's fixed delay.

#include <cmath>
#include <cstdint>
#include <functional>
#include <random>
#include <vector>

#include "dsp_types.h"

namespace sim {

using bal::cplx;

inline double rpm_ref_to_w(double rpm) { return rpm * bal::kTwoPi / 60.0; }

struct RotorModel {
  double fs = 16000.0;
  double rpm_ref = 1500.0;            // speed at which a_true is defined
  cplx a_true[2][2];                  // g per g*mm at rpm_ref, lag-complex [sensor][plane]
  cplx u0[2];                         // initial unbalance per plane, g*mm
  double delay_s = 17.4 / 16000.0;    // ADC group delay applied to the vibration
  double noise_g_rms = 0.001;         // white noise per channel
  double nonsync_hz = 47.3, nonsync_g = 0.01;
  double second_order_frac = 0.2;     // 2x amplitude as a fraction of 1x
  double tach_jitter_s = 2e-6;        // timing jitter on the tach pulse
  uint32_t seed = 1;
};

// Generates a run: `passes` coast-downs from rpm_hi to rpm_lo with time
// constant tau_s. Calls on_samples(ch0, ch1, n) in blocks and on_tach(index)
// for each pulse, interleaved in time order (tach after the samples that
// contain it, which is what the real acquisition does).
inline void run_coastdowns(const RotorModel &m, const cplx *u_extra, int passes, double rpm_hi,
                           double rpm_lo, double tau_s,
                           const std::function<void(const float *, const float *, size_t)> &on_samples,
                           const std::function<void(double)> &on_tach) {
  std::mt19937 rng(m.seed);
  std::normal_distribution<double> gauss(0.0, 1.0);
  const double w_ref = rpm_ref_to_w(m.rpm_ref);
  cplx u[2] = {m.u0[0] + u_extra[0], m.u0[1] + u_extra[1]};
  cplx resp[2];
  for (int s = 0; s < 2; s++) resp[s] = m.a_true[s][0] * u[0] + m.a_true[s][1] * u[1];
  // Fixed random 2x phase per sensor.
  const double ph2[2] = {37.0, 211.0};

  const double t_pass = -tau_s * std::log(rpm_lo / rpm_hi);  // time to coast hi -> lo
  const double w_hi = rpm_ref_to_w(rpm_hi);
  const size_t block = 256;
  std::vector<float> b0(block), b1(block);
  uint64_t n = 0;
  double theta_base = 0;  // angle accumulated by earlier passes
  double next_tach_theta = 0;
  for (int pass = 0; pass < passes; pass++) {
    const size_t pass_samples = static_cast<size_t>(t_pass * m.fs);
    // theta(t) within a pass = w_hi * tau * (1 - exp(-t/tau))
    auto theta_at = [&](double t) { return theta_base + w_hi * tau_s * (1.0 - std::exp(-t / tau_s)); };
    auto omega_at = [&](double t) { return w_hi * std::exp(-t / tau_s); };
    const double t0_abs = static_cast<double>(n) / m.fs;
    size_t i = 0;
    while (i < pass_samples) {
      const size_t cnt = std::min(block, pass_samples - i);
      std::vector<double> tachs;
      for (size_t k = 0; k < cnt; k++) {
        const double t = static_cast<double>(i + k) / m.fs;
        const double td = t - m.delay_s;  // vibration seen through the ADC delay
        const double th = theta_at(td);
        const double w = omega_at(td);
        const double scale = (w / w_ref) * (w / w_ref);
        const double tabs = t0_abs + t;
        for (int s = 0; s < 2; s++) {
          const cplx r = resp[s] * scale;
          // lag-complex L = A e^{j lag} -> x = A cos(theta - lag)
          double x = std::abs(r) * std::cos(th - std::arg(r));
          x += m.second_order_frac * std::abs(r) * std::cos(2 * th - ph2[s] / bal::kDegPerRad);
          x += m.nonsync_g * std::cos(bal::kTwoPi * m.nonsync_hz * tabs + s);
          x += m.noise_g_rms * gauss(rng);
          (s == 0 ? b0 : b1)[k] = static_cast<float>(x);
        }
        // Tach pulses crossing in this sample interval (true angle, no delay).
        const double th_now = theta_at(t);
        const double th_next = theta_at(t + 1.0 / m.fs);
        while (next_tach_theta < th_next) {
          if (next_tach_theta >= th_now) {
            const double frac = (next_tach_theta - th_now) / (th_next - th_now);
            const double idx = static_cast<double>(n + i + k) + frac + m.tach_jitter_s * m.fs * gauss(rng);
            tachs.push_back(idx);
          }
          next_tach_theta += bal::kTwoPi;
        }
      }
      on_samples(b0.data(), b1.data(), cnt);
      for (double tt : tachs) on_tach(tt);
      i += cnt;
    }
    n += pass_samples;
    // Between passes the rotor is "spun back up": restart the angle cleanly
    // on a whole revolution so the tach sequence stays consistent.
    theta_base = std::ceil(theta_at(t_pass) / bal::kTwoPi) * bal::kTwoPi;
    next_tach_theta = theta_base;
  }
}


}  // namespace sim
