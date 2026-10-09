#pragma once
// Influence-coefficient balancing (PLAN §6). All weights are unbalance
// vectors in g*mm (mass * radius) using the angle convention in dsp_types.h.

#include <string>

#include "dsp_types.h"

namespace balancer {

constexpr int kMaxSensors = 4;
constexpr int kMaxPlanes = 4;

// Unbalance vector (g*mm) for a mass at a radius and angle.
inline cplx unbalance(double mass_g, double radius_mm, double angle_deg) {
  return polar_deg(mass_g * radius_mm, angle_deg);
}

struct InfluenceModel {
  int sensors = 0;
  int planes = 0;
  cplx a[kMaxSensors][kMaxPlanes];  // response per g*mm: a[sensor][plane]
  double rpm = 0;                   // test speed the model was measured at
  Quantity quantity = Quantity::VELOCITY_MM_S;
  bool valid = false;
};

// Builds the influence matrix from the as-found run and one trial run per
// plane. v0[s] = reading of sensor s with no trial weight; vt[p][s] = reading
// of sensor s with only trial weight t[p] fitted in plane p.
bool build_influence(const cplx *v0, const cplx vt[][kMaxSensors], const cplx *t, int sensors,
                     int planes, InfluenceModel &out, std::string *err = nullptr);

struct Correction {
  cplx u[kMaxPlanes];         // unbalance to ADD per plane, g*mm
  cplx residual[kMaxSensors];  // predicted reading after correction
  double condition = 0;       // condition number of the influence matrix
  bool ok = false;
};

// Least-squares correction: minimises |v + A*u| (exact when sensors == planes).
Correction solve_correction(const InfluenceModel &m, const cplx *v, std::string *err = nullptr);

// Predicted reading on sensor s after adding u (g*mm per plane) to a rotor reading v.
cplx predict(const InfluenceModel &m, const cplx *v, const cplx *u, int s);

// Rule-of-thumb check that a trial weight changed the reading enough:
// at least min_amp_frac relative amplitude change OR min_phase_deg phase shift
// on at least one sensor.
struct TrialCheck {
  double amp_change_frac = 0;  // largest |v1 - v0| / |v0| over sensors
  double phase_change_deg = 0; // largest |phase change| over sensors
  bool ok = false;
};
TrialCheck check_trial(const cplx *v0, const cplx *v1, int sensors, double min_amp_frac = 0.3,
                       double min_phase_deg = 30.0);

// Express a correction (g*mm vector) as a mass at a radius. In remove mode
// the mass is drilled out 180 deg away.
struct MassAngle {
  double mass_g = 0;
  double angle_deg = 0;
  bool remove = false;
};
MassAngle to_mass(cplx u, double radius_mm, bool remove_mode);

// Split a mass at an angle onto the two neighbouring of N equally spaced
// positions (holes, blades, lugs). Position k is at offset + k*360/N.
struct SplitResult {
  int pos_a = 0, pos_b = 0;
  double angle_a = 0, angle_b = 0;
  double mass_a = 0, mass_b = 0;
};
SplitResult split_to_positions(double mass_g, double angle_deg, int positions, double offset_deg);

// Depth (mm, full diameter, measured to the drill point tip) to remove
// mass_g with a drill of diameter_mm in material of density_g_cm3, allowing
// for the conical point (118 deg by default).
double drill_depth_mm(double mass_g, double diameter_mm, double density_g_cm3,
                      double point_angle_deg = 118.0);

// ISO 21940-11 permissible residual unbalance, g*mm (whole rotor).
// grade = G number in mm/s, rotor mass in kg, maximum service speed in rpm.
double iso_uper_gmm(double grade_mm_s, double rotor_mass_kg, double service_rpm);

// Suggested trial unbalance (g*mm) whose centrifugal force at test_rpm is
// `fraction` of the rotor weight. Divide by the trial radius for grams.
double suggest_trial_gmm(double rotor_mass_kg, double test_rpm, double fraction = 0.1);

}  // namespace balancer
