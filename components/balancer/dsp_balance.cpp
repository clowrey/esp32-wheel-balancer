#include "dsp_balance.h"

#include <algorithm>
#include <cmath>

namespace bal {

namespace {

// Solves the n x n complex system M x = b in place (partial pivoting).
bool solve_linear(cplx m[kMaxPlanes][kMaxPlanes], cplx *b, int n) {
  for (int col = 0; col < n; col++) {
    int piv = col;
    for (int r = col + 1; r < n; r++)
      if (std::abs(m[r][col]) > std::abs(m[piv][col])) piv = r;
    if (std::abs(m[piv][col]) < 1e-300) return false;
    if (piv != col) {
      for (int k = 0; k < n; k++) std::swap(m[piv][k], m[col][k]);
      std::swap(b[piv], b[col]);
    }
    for (int r = col + 1; r < n; r++) {
      const cplx f = m[r][col] / m[col][col];
      for (int k = col; k < n; k++) m[r][k] -= f * m[col][k];
      b[r] -= f * b[col];
    }
  }
  for (int r = n - 1; r >= 0; r--) {
    cplx s = b[r];
    for (int k = r + 1; k < n; k++) s -= m[r][k] * b[k];
    b[r] = s / m[r][r];
  }
  return true;
}

// Eigenvalues of a Hermitian positive semidefinite matrix by Jacobi
// rotations (n <= 4); used only for the condition number.
void hermitian_eigs(cplx h[kMaxPlanes][kMaxPlanes], int n, double *eig) {
  // Convert to a real symmetric 2n x 2n matrix [[Re, -Im], [Im, Re]], whose
  // eigenvalues are those of h, each repeated twice.
  const int N = 2 * n;
  double a[2 * kMaxPlanes][2 * kMaxPlanes];
  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++) {
      a[i][j] = h[i][j].real();
      a[i][j + n] = -h[i][j].imag();
      a[i + n][j] = h[i][j].imag();
      a[i + n][j + n] = h[i][j].real();
    }
  for (int sweep = 0; sweep < 60; sweep++) {
    double off = 0;
    for (int i = 0; i < N; i++)
      for (int j = i + 1; j < N; j++) off += a[i][j] * a[i][j];
    if (off < 1e-30) break;
    for (int p = 0; p < N; p++)
      for (int q = p + 1; q < N; q++) {
        if (std::fabs(a[p][q]) < 1e-300) continue;
        const double theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
        const double t = (theta >= 0 ? 1.0 : -1.0) / (std::fabs(theta) + std::sqrt(theta * theta + 1));
        const double c = 1 / std::sqrt(t * t + 1), s = t * c;
        for (int k = 0; k < N; k++) {
          const double akp = a[k][p], akq = a[k][q];
          a[k][p] = c * akp - s * akq;
          a[k][q] = s * akp + c * akq;
        }
        for (int k = 0; k < N; k++) {
          const double apk = a[p][k], aqk = a[q][k];
          a[p][k] = c * apk - s * aqk;
          a[q][k] = s * apk + c * aqk;
        }
      }
  }
  double d[2 * kMaxPlanes];
  for (int i = 0; i < N; i++) d[i] = a[i][i];
  for (int i = 1; i < N; i++) {  // insertion sort (N <= 8)
    const double v = d[i];
    int j = i - 1;
    while (j >= 0 && d[j] > v) {
      d[j + 1] = d[j];
      j--;
    }
    d[j + 1] = v;
  }
  for (int i = 0; i < n; i++) eig[i] = d[2 * i];  // each eigenvalue appears twice
}

}  // namespace

bool build_influence(const cplx *v0, const cplx vt[][kMaxSensors], const cplx *t, int sensors,
                     int planes, InfluenceModel &out, std::string *err) {
  out = InfluenceModel();
  if (sensors < 1 || sensors > kMaxSensors || planes < 1 || planes > kMaxPlanes) {
    if (err) *err = "bad sensor/plane count";
    return false;
  }
  if (sensors < planes) {
    if (err) *err = "need at least as many sensors as planes";
    return false;
  }
  for (int p = 0; p < planes; p++) {
    if (std::abs(t[p]) <= 0) {
      if (err) *err = "trial weight is zero";
      return false;
    }
    for (int s = 0; s < sensors; s++) out.a[s][p] = (vt[p][s] - v0[s]) / t[p];
  }
  out.sensors = sensors;
  out.planes = planes;
  out.valid = true;
  return true;
}

Correction solve_correction(const InfluenceModel &m, const cplx *v, std::string *err) {
  Correction c;
  if (!m.valid) {
    if (err) *err = "no influence model";
    return c;
  }
  const int S = m.sensors, P = m.planes;
  // Normal equations: (A^H A) u = -A^H v
  cplx n[kMaxPlanes][kMaxPlanes];
  cplx rhs[kMaxPlanes];
  for (int i = 0; i < P; i++) {
    rhs[i] = 0;
    for (int s = 0; s < S; s++) rhs[i] -= std::conj(m.a[s][i]) * v[s];
    for (int j = 0; j < P; j++) {
      n[i][j] = 0;
      for (int s = 0; s < S; s++) n[i][j] += std::conj(m.a[s][i]) * m.a[s][j];
    }
  }
  // Condition number of A = sqrt(cond(A^H A)).
  cplx ncopy[kMaxPlanes][kMaxPlanes];
  for (int i = 0; i < P; i++)
    for (int j = 0; j < P; j++) ncopy[i][j] = n[i][j];
  double eig[kMaxPlanes] = {0, 0, 0, 0};
  hermitian_eigs(ncopy, P, eig);
  c.condition = eig[0] > 0 ? std::sqrt(eig[P - 1] / eig[0]) : INFINITY;

  if (!solve_linear(n, rhs, P)) {
    if (err) *err = "influence matrix is singular (trial weights had no independent effect)";
    return c;
  }
  for (int p = 0; p < P; p++) c.u[p] = rhs[p];
  for (int s = 0; s < S; s++) c.residual[s] = predict(m, v, c.u, s);
  c.ok = true;
  return c;
}

cplx predict(const InfluenceModel &m, const cplx *v, const cplx *u, int s) {
  cplx r = v[s];
  for (int p = 0; p < m.planes; p++) r += m.a[s][p] * u[p];
  return r;
}

TrialCheck check_trial(const cplx *v0, const cplx *v1, int sensors, double min_amp_frac,
                       double min_phase_deg) {
  TrialCheck t;
  for (int s = 0; s < sensors; s++) {
    const double a0 = std::abs(v0[s]);
    const double frac = a0 > 0 ? std::abs(v1[s] - v0[s]) / a0 : INFINITY;
    const double dph = std::fabs(wrap180(arg_deg(v1[s]) - arg_deg(v0[s])));
    t.amp_change_frac = std::max(t.amp_change_frac, frac);
    t.phase_change_deg = std::max(t.phase_change_deg, dph);
  }
  t.ok = t.amp_change_frac >= min_amp_frac || t.phase_change_deg >= min_phase_deg;
  return t;
}

MassAngle to_mass(cplx u, double radius_mm, bool remove_mode) {
  MassAngle m;
  m.mass_g = radius_mm > 0 ? std::abs(u) / radius_mm : 0;
  m.angle_deg = arg_deg(u);
  m.remove = remove_mode;
  if (remove_mode) m.angle_deg = wrap360(m.angle_deg + 180.0);
  return m;
}

SplitResult split_to_positions(double mass_g, double angle_deg, int positions, double offset_deg) {
  SplitResult r;
  if (positions < 3) {  // with 2 positions only angles on that line are reachable
    r.mass_a = mass_g;
    r.angle_a = angle_deg;
    return r;
  }
  const double pitch = 360.0 / positions;
  const double rel = wrap360(angle_deg - offset_deg);
  int a = static_cast<int>(std::floor(rel / pitch)) % positions;
  int b = (a + 1) % positions;
  r.pos_a = a;
  r.pos_b = b;
  r.angle_a = wrap360(offset_deg + a * pitch);
  r.angle_b = wrap360(offset_deg + b * pitch);
  // m_a e^{j a} + m_b e^{j b} = m e^{j theta}
  const double da = (rel - a * pitch) / kDegPerRad;  // angle from a to theta
  const double db = pitch / kDegPerRad - da;         // angle from theta to b
  const double sp = std::sin(pitch / kDegPerRad);
  r.mass_a = mass_g * std::sin(db) / sp;
  r.mass_b = mass_g * std::sin(da) / sp;
  return r;
}

double drill_depth_mm(double mass_g, double diameter_mm, double density_g_cm3,
                      double point_angle_deg) {
  if (mass_g <= 0 || diameter_mm <= 0 || density_g_cm3 <= 0) return 0;
  const double vol_mm3 = mass_g / (density_g_cm3 * 1e-3);
  const double area = kPi * diameter_mm * diameter_mm / 4.0;
  const double cone_h = (diameter_mm / 2.0) / std::tan(point_angle_deg / 2.0 / kDegPerRad);
  const double cone_vol = area * cone_h / 3.0;
  if (vol_mm3 <= cone_vol) {
    // Only the point is in: volume of a cone of height h scales with h^3.
    return cone_h * std::cbrt(vol_mm3 / cone_vol);
  }
  return cone_h + (vol_mm3 - cone_vol) / area;
}

double iso_uper_gmm(double grade_mm_s, double rotor_mass_kg, double service_rpm) {
  if (service_rpm <= 0) return 0;
  // U = 1000 * G * m / omega with G in mm/s, m in kg, omega in rad/s -> g*mm
  // (= 9549 * G * m / n)
  return 1000.0 * grade_mm_s * rotor_mass_kg / (service_rpm * kTwoPi / 60.0);
}

double suggest_trial_gmm(double rotor_mass_kg, double test_rpm, double fraction) {
  if (test_rpm <= 0) return 0;
  const double w = test_rpm * kTwoPi / 60.0;
  const double u_kg_m = fraction * rotor_mass_kg * kStandardGravity / (w * w);
  return u_kg_m * 1e6;  // kg*m -> g*mm
}

}  // namespace bal
