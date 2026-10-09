#pragma once
// Revolution-level rotor simulator for the on-device demo mode (`simulate:
// true`). Produces RevResult values directly, so the whole wizard can be
// exercised on the bench with no front end. The physics matches
// tests/sim_rotor.h: linear two-plane rotor, response ~ omega^2, noise.

#include <cstdint>
#include <random>

#include "dsp_revolution.h"
#include "dsp_types.h"

namespace bal {

class RevSim {
 public:
  RevSim();
  // Air blast: jump to rpm and start coasting with time constant tau_s.
  void blast(double rpm, double tau_s = 10.0);
  // Extra unbalance currently fitted to the rotor (trial or correction), g*mm per plane.
  void set_fitted(cplx p1, cplx p2) {
    fitted_[0] = p1;
    fitted_[1] = p2;
  }
  // Advance simulated time by dt seconds; returns how many revolutions were
  // produced into out (up to max).
  int step(double dt, RevResult *out, int max);
  double rpm() const { return rpm_; }

  cplx a_true[2][2];
  cplx u0[2];
  double rpm_ref = 1500;
  double noise_g = 0.0015;  // per-revolution noise on the 1x vector

 private:
  std::mt19937 rng_;
  std::normal_distribution<double> gauss_{0.0, 1.0};
  double rpm_ = 0;
  double tau_ = 10;
  double angle_ = 0;  // revolutions, fractional
  double t_ = 0;
  uint32_t index_ = 0;
  double last_period_ = 0;
  cplx fitted_[2];
};

}  // namespace bal
