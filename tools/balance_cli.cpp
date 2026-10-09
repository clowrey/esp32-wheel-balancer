// Two-plane solver from the command line, for cross-checking against other
// tools (tools/crosscheck_pyprb.py).
//
// usage: balance_cli a10 p10 a20 p20  a11 p11 a21 p21  a12 p12 a22 p22  m1 ang1 m2 ang2
//   aXY/pXY: amplitude and phase (deg) of sensor X in run Y (0 = as found,
//            1 = trial on plane 1, 2 = trial on plane 2)
//   m/ang:   trial masses (g) and angles (deg); radius taken as 1 so the
//            output is directly in grams.
// prints: mass1 angle1 mass2 angle2
#include <cstdio>
#include <cstdlib>

#include "dsp_balance.h"

using namespace bal;

int main(int argc, char **argv) {
  if (argc != 17) {
    std::fprintf(stderr, "need 16 numbers\n");
    return 2;
  }
  double x[16];
  for (int i = 0; i < 16; i++) x[i] = std::atof(argv[i + 1]);
  cplx v0[2] = {polar_deg(x[0], x[1]), polar_deg(x[2], x[3])};
  cplx vt[2][kMaxSensors] = {{polar_deg(x[4], x[5]), polar_deg(x[6], x[7])},
                             {polar_deg(x[8], x[9]), polar_deg(x[10], x[11])}};
  cplx t[2] = {polar_deg(x[12], x[13]), polar_deg(x[14], x[15])};
  InfluenceModel m;
  std::string err;
  if (!build_influence(v0, vt, t, 2, 2, m, &err)) {
    std::fprintf(stderr, "%s\n", err.c_str());
    return 1;
  }
  auto c = solve_correction(m, v0, &err);
  if (!c.ok) {
    std::fprintf(stderr, "%s\n", err.c_str());
    return 1;
  }
  auto a = to_mass(c.u[0], 1.0, false), b = to_mass(c.u[1], 1.0, false);
  std::printf("%.9f %.9f %.9f %.9f\n", a.mass_g, a.angle_deg, b.mass_g, b.angle_deg);
  return 0;
}
