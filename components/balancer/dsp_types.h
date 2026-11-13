#pragma once
// Shared types for the balancer DSP. Pure C++17: no ESPHome / IDF.
//
// Vector convention used everywhere in dsp_*:
//   * A vibration reading is a "lag-complex" number: amplitude * e^{+j*lag},
//     where lag is how many degrees of rotation after the tach pulse the
//     1x peak occurs.
//   * A weight / unbalance is mass*radius (g*mm) * e^{+j*angle}, where angle
//     is measured from the tach mark AGAINST the direction of rotation.
//   With both conventions, adding a weight further round the rotor delays the
//   response by the same amount, so the system is linear: V = V0 + A*U.
//   Conversion to the user's preferred angle direction happens in the UI.

#include <cmath>
#include <complex>
#include <cstdint>

namespace bal {

using cplx = std::complex<double>;
using cplxf = std::complex<float>;

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kDegPerRad = 180.0 / kPi;
constexpr double kStandardGravity = 9.80665;  // m/s^2 per g

constexpr int kChannels = 2;
constexpr int kMaxOrder = 10;   // orders 1..10 are extracted per revolution
constexpr int kWaveBins = 128;  // averaged-revolution waveform resolution

inline double wrap360(double deg) {
  deg = std::fmod(deg, 360.0);
  if (deg < 0) deg += 360.0;
  return deg;
}
inline double wrap180(double deg) { return wrap360(deg + 180.0) - 180.0; }
inline cplx polar_deg(double mag, double deg) { return std::polar(mag, deg / kDegPerRad); }
inline double arg_deg(cplx c) { return wrap360(std::arg(c) * kDegPerRad); }

// What physical quantity a reading is expressed in.
enum class Quantity : uint8_t {
  ACCEL_G = 0,          // acceleration, g peak
  VELOCITY_MM_S = 1,    // velocity, mm/s peak
  DISPLACEMENT_UM = 2,  // displacement, um peak
};

inline const char *quantity_unit(Quantity q) {
  switch (q) {
    case Quantity::ACCEL_G:
      return "g";
    case Quantity::VELOCITY_MM_S:
      return "mm/s";
    case Quantity::DISPLACEMENT_UM:
      return "um";
  }
  return "";
}

// Convert a lag-complex acceleration (g) of order h at rotor angular speed
// omega (rad/s) into the requested quantity. Velocity lags acceleration by
// 90 deg, displacement by 180 deg.
inline cplx convert_accel(cplx accel_g, Quantity q, int order, double omega) {
  const double w = order * omega;
  if (q == Quantity::ACCEL_G || w <= 0) return accel_g;
  const double a_si = kStandardGravity;  // g -> m/s^2
  if (q == Quantity::VELOCITY_MM_S) return accel_g * cplx(0.0, 1.0) * (a_si * 1e3 / w);
  return -accel_g * (a_si * 1e6 / (w * w));  // displacement, um
}

}  // namespace bal
