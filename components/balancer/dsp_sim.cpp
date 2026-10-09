#include "dsp_sim.h"

#include <cmath>

namespace bal {

RevSim::RevSim() : rng_(42) {
  // Same rotor as the host simulation test (g per g*mm at 1500 rpm).
  a_true[0][0] = polar_deg(2.0e-4, 40);
  a_true[0][1] = polar_deg(0.6e-4, 205);
  a_true[1][0] = polar_deg(0.5e-4, 170);
  a_true[1][1] = polar_deg(1.8e-4, 70);
  u0[0] = polar_deg(80, 30);    // 2 g at 40 mm
  u0[1] = polar_deg(120, 250);  // 3 g at 40 mm
  fitted_[0] = fitted_[1] = 0;
}

void RevSim::blast(double rpm, double tau_s) {
  rpm_ = rpm;
  tau_ = tau_s;
  last_period_ = 0;  // speed jumped: next revolution is not continuous
}

int RevSim::step(double dt, RevResult *out, int max) {
  int n = 0;
  const double stop_rpm = 60.0;
  while (dt > 0 && rpm_ > stop_rpm && n < max) {
    const double rev_per_s = rpm_ / 60.0;
    const double to_next = (1.0 - (angle_ - std::floor(angle_))) / rev_per_s;
    const double h = std::min(dt, to_next);
    angle_ += rev_per_s * h;
    t_ += h;
    dt -= h;
    rpm_ *= std::exp(-h / tau_);
    if (h == to_next) {
      // Completed a revolution.
      angle_ = std::round(angle_);
      RevResult &r = out[n++];
      r = RevResult();
      r.index = index_++;
      r.rpm = rpm_;
      r.omega = rpm_ * kTwoPi / 60.0;
      r.period = 16000.0 * 60.0 / rpm_;
      r.period_change = last_period_ > 0 ? (r.period - last_period_) / last_period_ : 0.0;
      last_period_ = r.period;
      const double s = (rpm_ / rpm_ref) * (rpm_ / rpm_ref);
      const cplx u[2] = {u0[0] + fitted_[0], u0[1] + fitted_[1]};
      for (int c = 0; c < 2; c++) {
        cplx v = (a_true[c][0] * u[0] + a_true[c][1] * u[1]) * s;
        v += cplx(gauss_(rng_), gauss_(rng_)) * (noise_g / std::sqrt(2.0));
        r.order[c][0] = v;
        r.order[c][1] = v * 0.2 * polar_deg(1.0, c ? 211 : 37);
        for (int h2 = 2; h2 < kMaxOrder; h2++)
          r.order[c][h2] = cplx(gauss_(rng_), gauss_(rng_)) * (noise_g * 0.3);
        for (int b = 0; b < kWaveBins; b++) {
          const double th = kTwoPi * b / kWaveBins;
          r.wave[c][b] = static_cast<float>(std::abs(v) * std::cos(th - std::arg(v)) +
                                            0.2 * std::abs(v) * std::cos(2 * th - (c ? 3.68 : 0.65)));
        }
      }
    }
  }
  if (rpm_ <= stop_rpm) rpm_ = 0;
  return n;
}

}  // namespace bal
