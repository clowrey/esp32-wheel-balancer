// Host tests for the balancer DSP (PLAN Phase 2).
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "third_party/doctest.h"

#include <cmath>
#include <cstdio>
#include <vector>

#include "dsp_balance.h"
#include "dsp_measurement.h"
#include "dsp_revolution.h"
#include "sim_rotor.h"

using namespace bal;

namespace {

// Feeds a constant-speed tone into the analyzer and returns the first N revs.
std::vector<RevResult> tone_revs(double rpm, double amp_g, double lag_deg, double delay_s,
                                 double cfg_delay_s, int revs, double tach_offset_samples = 0.37) {
  RevolutionConfig cfg;
  cfg.delay_s = cfg_delay_s;
  RevolutionAnalyzer an(cfg);
  const double fs = cfg.fs;
  const double w = rpm * kTwoPi / 60.0;
  const double period = fs * 60.0 / rpm;
  const int total = static_cast<int>(period * (revs + 2));
  std::vector<float> a(total), b(total);
  for (int n = 0; n < total; n++) {
    const double t = (n - tach_offset_samples) / fs - delay_s;
    a[n] = static_cast<float>(amp_g * std::cos(w * t - lag_deg / kDegPerRad));
    b[n] = static_cast<float>(0.5 * amp_g * std::cos(w * t - (lag_deg + 90) / kDegPerRad) +
                              0.3 * amp_g * std::cos(2 * w * t));
  }
  an.push_samples(a.data(), b.data(), total);
  for (int k = 0; k <= revs + 1; k++) an.push_tach(tach_offset_samples + k * period);
  std::vector<RevResult> out;
  RevResult r;
  while (an.pop(r)) out.push_back(r);
  return out;
}

}  // namespace

TEST_CASE("revolution analyzer recovers amplitude and phase of a pure tone") {
  auto revs = tone_revs(1517.0, 0.05, 63.0, 0, 0, 5);
  REQUIRE(revs.size() >= 5);
  for (auto &r : revs) {
    CHECK(r.rpm == doctest::Approx(1517.0).epsilon(1e-6));
    CHECK(std::abs(r.order[0][0]) == doctest::Approx(0.05).epsilon(0.005));
    CHECK(wrap180(arg_deg(r.order[0][0]) - 63.0) == doctest::Approx(0).epsilon(0.2).scale(1));
    CHECK(std::abs(r.order[1][0]) == doctest::Approx(0.025).epsilon(0.005));
    CHECK(wrap180(arg_deg(r.order[1][0]) - 153.0) == doctest::Approx(0).epsilon(0.2).scale(1));
    // 2x on channel 1, and no 2x leaking into channel 0
    CHECK(std::abs(r.order[1][1]) == doctest::Approx(0.015).epsilon(0.01));
    CHECK(std::abs(r.order[0][1]) < 0.0005);
  }
}

TEST_CASE("fixed chain delay is removed from the phase") {
  const double tau = 17.4 / 16000.0;
  auto raw = tone_revs(1500.0, 0.05, 20.0, tau, 0, 3);
  auto comp = tone_revs(1500.0, 0.05, 20.0, tau, tau, 3);
  const double extra = 360.0 * 25.0 * tau;  // degrees of lag the delay adds at 25 Hz
  CHECK(wrap180(arg_deg(raw[1].order[0][0]) - 20.0 - extra) == doctest::Approx(0).epsilon(0.3).scale(1));
  CHECK(wrap180(arg_deg(comp[1].order[0][0]) - 20.0) == doctest::Approx(0).epsilon(0.3).scale(1));
}

TEST_CASE("tach glitches are ignored and long gaps restart") {
  RevolutionAnalyzer an;
  std::vector<float> z(24000, 0.0f);
  an.push_samples(z.data(), z.data(), 2000);
  an.push_tach(100.0);
  an.push_tach(105.0);  // 5 samples later = 192000 rpm -> glitch
  an.push_tach(740.0);
  an.push_tach(1380.0);
  RevResult r;
  int n = 0;
  while (an.pop(r)) n++;
  CHECK(an.glitches() == 1);
  CHECK(n == 2);
  CHECK(an.gaps() == 0);
  // Next pulse 20000 samples later (1.25 s = 48 rpm, below min_rpm 60): a gap.
  an.push_tach(1380.0 + 20000.0);
  CHECK_FALSE(an.pop(r));  // waits for samples first
  an.push_samples(z.data(), z.data(), 24000);
  CHECK_FALSE(an.pop(r));
  CHECK(an.gaps() == 1);
}

TEST_CASE("measurement only keeps revolutions inside the speed window") {
  MeasureConfig mc;
  mc.target_rpm = 1500;
  mc.window = 0.03;
  mc.quantity = Quantity::ACCEL_G;
  mc.target_revs = 3;
  Measurement m(mc);
  RevResult r{};
  r.order[0][0] = polar_deg(0.1, 30);
  r.order[1][0] = polar_deg(0.2, 60);
  r.rpm = 1600;
  CHECK(m.add(r) == RevStatus::ABOVE_WINDOW);
  r.rpm = 1400;
  CHECK(m.add(r) == RevStatus::BELOW_WINDOW);
  r.rpm = 1500;
  r.period_change = 0.05;
  CHECK(m.add(r) == RevStatus::SPEED_UNSTABLE);
  r.period_change = 0.001;
  for (int i = 0; i < 3; i++) CHECK(m.add(r) == RevStatus::ACCEPTED);
  auto s = m.stats();
  CHECK(s.revs == 3);
  CHECK(s.ch[0].amp == doctest::Approx(0.1));
  CHECK(s.ch[1].phase_deg == doctest::Approx(60));
  CHECK(s.ready);
  CHECK(s.rejected[static_cast<int>(RevStatus::ABOVE_WINDOW)] == 1);
}

TEST_CASE("velocity and displacement conversion") {
  const double w = 1500 * kTwoPi / 60;
  // 1 g at 25 Hz -> 9806.65/(2*pi*25) = 62.43 mm/s, lagging by 90 deg
  cplx v = convert_accel(polar_deg(1.0, 10), Quantity::VELOCITY_MM_S, 1, w);
  CHECK(std::abs(v) == doctest::Approx(62.43).epsilon(0.001));
  CHECK(arg_deg(v) == doctest::Approx(100));
  cplx d = convert_accel(polar_deg(1.0, 10), Quantity::DISPLACEMENT_UM, 1, w);
  CHECK(std::abs(d) == doctest::Approx(397.4).epsilon(0.001));
  CHECK(arg_deg(d) == doctest::Approx(190));
}

TEST_CASE("single-plane solution is exact for a linear rotor") {
  const cplx a = polar_deg(0.02, 75);  // mm/s per g*mm
  const cplx u0 = unbalance(2.0, 40, 123);
  const cplx t = unbalance(3.5, 40, 0);
  const cplx v0 = a * u0, v1 = a * (u0 + t);
  cplx vt[1][kMaxSensors] = {{v1}};
  InfluenceModel m;
  REQUIRE(build_influence(&v0, vt, &t, 1, 1, m));
  auto c = solve_correction(m, &v0);
  REQUIRE(c.ok);
  CHECK(std::abs(c.u[0] + u0) < 1e-9);
  auto ma = to_mass(c.u[0], 40, false);
  CHECK(ma.mass_g == doctest::Approx(2.0));
  CHECK(ma.angle_deg == doctest::Approx(303));
  auto mr = to_mass(c.u[0], 40, true);  // drill out opposite: where the heavy spot is
  CHECK(mr.angle_deg == doctest::Approx(123));
  CHECK(std::abs(c.residual[0]) < 1e-12);
}

TEST_CASE("two-plane solution and least squares") {
  cplx A[2][2] = {{polar_deg(0.020, 40), polar_deg(0.006, 200)},
                  {polar_deg(0.005, 170), polar_deg(0.018, 65)}};
  cplx u0[2] = {unbalance(1.5, 40, 30), unbalance(2.5, 40, 250)};
  cplx t[2] = {unbalance(3.5, 40, 0), unbalance(3.5, 40, 0)};
  cplx v0[2], vt[2][kMaxSensors];
  for (int s = 0; s < 2; s++) v0[s] = A[s][0] * u0[0] + A[s][1] * u0[1];
  for (int p = 0; p < 2; p++)
    for (int s = 0; s < 2; s++)
      vt[p][s] = v0[s] + A[s][p] * t[p];
  InfluenceModel m;
  REQUIRE(build_influence(v0, vt, t, 2, 2, m));
  auto c = solve_correction(m, v0);
  REQUIRE(c.ok);
  CHECK(std::abs(c.u[0] + u0[0]) < 1e-9);
  CHECK(std::abs(c.u[1] + u0[1]) < 1e-9);
  CHECK(c.condition > 1.0);
  CHECK(c.condition < 10.0);

  // Static (1 plane) with 2 sensors: least squares.
  cplx vt1[1][kMaxSensors] = {{vt[0][0], vt[0][1]}};
  InfluenceModel m1;
  REQUIRE(build_influence(v0, vt1, t, 2, 1, m1));
  auto c1 = solve_correction(m1, v0);
  REQUIRE(c1.ok);
  // residual must be orthogonal to the plane-1 influence column
  cplx dot = std::conj(m1.a[0][0]) * c1.residual[0] + std::conj(m1.a[1][0]) * c1.residual[1];
  CHECK(std::abs(dot) < 1e-12);
}

TEST_CASE("condition number of a well separated identity-like matrix is 1") {
  InfluenceModel m;
  m.sensors = m.planes = 2;
  m.valid = true;
  m.a[0][0] = 1;
  m.a[1][1] = cplx(0, 1);
  m.a[0][1] = m.a[1][0] = 0;
  cplx v[2] = {1, 1};
  auto c = solve_correction(m, v);
  CHECK(c.condition == doctest::Approx(1.0));
}

TEST_CASE("trial-weight check") {
  cplx v0[2] = {polar_deg(1.0, 0), polar_deg(1.0, 0)};
  cplx small[2] = {polar_deg(1.1, 5), polar_deg(1.05, 3)};
  cplx big[2] = {polar_deg(1.2, 40), polar_deg(1.0, 0)};
  CHECK_FALSE(check_trial(v0, small, 2).ok);
  CHECK(check_trial(v0, big, 2).ok);
}

TEST_CASE("splitting onto fixed positions reproduces the vector") {
  for (double ang : {0.0, 17.0, 44.9, 45.0, 200.0, 359.0}) {
    auto s = split_to_positions(3.0, ang, 8, 0.0);
    const cplx sum = polar_deg(s.mass_a, s.angle_a) + polar_deg(s.mass_b, s.angle_b);
    CHECK(std::abs(sum - polar_deg(3.0, ang)) < 1e-9);
    CHECK(s.mass_a >= -1e-12);
    CHECK(s.mass_b >= -1e-12);
  }
  auto s = split_to_positions(1.0, 100.0, 6, 10.0);  // positions at 10, 70, 130...
  CHECK(s.pos_a == 1);
  CHECK(s.pos_b == 2);
}

TEST_CASE("ISO limit, trial-weight helper and drill depth match PLAN §6.1") {
  CHECK(iso_uper_gmm(2.5, 3.5, 1750) == doctest::Approx(47.75).epsilon(0.001));
  const double trial = suggest_trial_gmm(3.5, 1500, 0.1);
  CHECK(trial == doctest::Approx(139.2).epsilon(0.002));
  CHECK(trial / 40.0 == doctest::Approx(3.48).epsilon(0.002));
  // 0.6 g of aluminium with a 6 mm drill: cylinder volume 222 mm^3
  const double d = drill_depth_mm(0.6, 6.0, 2.7);
  const double area = kPi * 9.0;
  const double cone_h = 3.0 / std::tan(59.0 / kDegPerRad);
  const double vol = area * (d - cone_h) + area * cone_h / 3.0;
  CHECK(vol * 2.7e-3 == doctest::Approx(0.6).epsilon(1e-9));
}

// ---------------------------------------------------------------------------
// End-to-end: simulated air-driven coast-downs -> DSP -> solver.
// Exit test from PLAN Phase 2: injected unbalance recovered within 2% / 1 deg
// at 20 dB SNR.

namespace {

struct RunOut {
  MeasureStats stats;
};

RunOut measure_run(const sim::RotorModel &model, const cplx *extra, double target_rpm,
                   int passes) {
  RevolutionAnalyzer an;
  MeasureConfig mc;
  mc.target_rpm = target_rpm;
  mc.target_revs = 30;
  Measurement meas(mc);
  RevResult r;
  sim::run_coastdowns(
      model, extra, passes, target_rpm * 1.15, target_rpm * 0.9, 10.0,
      [&](const float *a, const float *b, size_t n) {
        an.push_samples(a, b, n);
        while (an.pop(r)) meas.add(r);
      },
      [&](double t) { an.push_tach(t); });
  while (an.pop(r)) meas.add(r);
  return {meas.stats()};
}

}  // namespace

TEST_CASE("simulated two-plane balance on an air-driven 1 hp rotor") {
  sim::RotorModel model;
  model.rpm_ref = 1500;
  // Response in g per g*mm at 1500 rpm. Chosen so the as-found 1x is ~0.03 g.
  model.a_true[0][0] = polar_deg(2.0e-4, 40);
  model.a_true[0][1] = polar_deg(0.6e-4, 205);
  model.a_true[1][0] = polar_deg(0.5e-4, 170);
  model.a_true[1][1] = polar_deg(1.8e-4, 70);
  model.u0[0] = unbalance(2.0, 40, 30);   // 80 g*mm
  model.u0[1] = unbalance(3.0, 40, 250);  // 120 g*mm
  // 20 dB SNR against the as-found 1x on sensor 0: noise rms = 1x rms / 10
  const cplx v0_true = model.a_true[0][0] * model.u0[0] + model.a_true[0][1] * model.u0[1];
  model.noise_g_rms = std::abs(v0_true) / std::sqrt(2.0) / 10.0;
  model.nonsync_g = std::abs(v0_true) * 0.5;

  const double rpm = 1500;
  const cplx none[2] = {0, 0};
  const cplx t1 = unbalance(3.5, 40, 0), t2 = unbalance(3.5, 40, 0);
  const cplx trial1[2] = {t1, 0}, trial2[2] = {0, t2};

  model.seed = 11;
  auto r0 = measure_run(model, none, rpm, 3);
  model.seed = 12;
  auto r1 = measure_run(model, trial1, rpm, 3);
  model.seed = 13;
  auto r2 = measure_run(model, trial2, rpm, 3);
  for (auto *r : {&r0, &r1, &r2}) {
    INFO("revs " << r->stats.revs << " unc " << r->stats.ch[0].phase_unc_deg);
    CHECK(r->stats.ready);
    CHECK(r->stats.revs >= 30);
  }

  cplx v0[2] = {r0.stats.ch[0].mean, r0.stats.ch[1].mean};
  cplx vt[2][kMaxSensors] = {{r1.stats.ch[0].mean, r1.stats.ch[1].mean},
                             {r2.stats.ch[0].mean, r2.stats.ch[1].mean}};
  cplx t[2] = {t1, t2};
  CHECK(check_trial(v0, vt[0], 2).ok);
  CHECK(check_trial(v0, vt[1], 2).ok);

  InfluenceModel m;
  REQUIRE(build_influence(v0, vt, t, 2, 2, m));
  auto c = solve_correction(m, v0);
  REQUIRE(c.ok);
  for (int p = 0; p < 2; p++) {
    const cplx want = -model.u0[p];
    const double amp_err = std::abs(std::abs(c.u[p]) / std::abs(want) - 1.0);
    const double ph_err = std::fabs(wrap180(arg_deg(c.u[p]) - arg_deg(want)));
    std::printf("plane %d: got %.2f g*mm @ %.2f deg, want %.2f @ %.2f  (err %.2f%%, %.2f deg)\n",
                p + 1, std::abs(c.u[p]), arg_deg(c.u[p]), std::abs(want), arg_deg(want),
                amp_err * 100, ph_err);
    CHECK(amp_err < 0.02);
    CHECK(ph_err < 1.0);
  }

  // Verify run with the correction fitted: residual should be tiny.
  const cplx fitted[2] = {c.u[0], c.u[1]};
  model.seed = 14;
  auto rv = measure_run(model, fitted, rpm, 3);
  std::printf("as found %.3f / %.3f mm/s, after %.3f / %.3f mm/s\n", r0.stats.ch[0].amp,
              r0.stats.ch[1].amp, rv.stats.ch[0].amp, rv.stats.ch[1].amp);
  CHECK(rv.stats.ch[0].amp < 0.05 * r0.stats.ch[0].amp);
  CHECK(rv.stats.ch[1].amp < 0.05 * r0.stats.ch[1].amp);
}

// ---------------------------------------------------------------------------
// Job / wizard logic

#include "dsp_job.h"

namespace {
MeasureStats stats_of(const cplx *v, double rpm) {
  MeasureStats s;
  s.revs = 40;
  s.rpm_mean = rpm;
  s.ready = true;
  for (int c = 0; c < kChannels; c++) s.ch[c].mean = v[c];
  return s;
}
}  // namespace

TEST_CASE("job walks the two-plane procedure and the verify run passes ISO") {
  for (bool with_rot : {false, true}) {
    for (bool remove : {true, false}) {
      CAPTURE(with_rot);
      CAPTURE(remove);
      cplx A[2][2] = {{polar_deg(0.020, 40), polar_deg(0.006, 200)},
                      {polar_deg(0.005, 170), polar_deg(0.018, 65)}};
      cplx u0[2] = {unbalance(2.0, 40, 30), unbalance(3.0, 40, 250)};
      auto reading = [&](const cplx *extra) {
        static cplx v[2];
        for (int s = 0; s < 2; s++)
          v[s] = A[s][0] * (u0[0] + extra[0]) + A[s][1] * (u0[1] + extra[1]);
        return v;
      };
      JobConfig cfg;
      cfg.angles_with_rotation = with_rot;
      cfg.plane[0].remove_mode = cfg.plane[1].remove_mode = remove;
      cfg.plane[0].trial_angle_deg = 90;
      cfg.plane[1].trial_angle_deg = 45;
      BalanceJob job;
      job.reset(cfg);
      job.start();
      CHECK(job.step() == JobStep::RUN0);
      const cplx none[2] = {0, 0};
      REQUIRE(job.record(stats_of(reading(none), 1502)));
      CHECK(job.test_rpm() == doctest::Approx(1502));
      CHECK(job.step() == JobStep::TRIAL1);
      const cplx t1[2] = {job.trial_vector(0), 0};
      REQUIRE(job.record(stats_of(reading(t1), 1502)));
      CHECK(job.step() == JobStep::TRIAL2);
      const cplx t2[2] = {0, job.trial_vector(1)};
      REQUIRE(job.record(stats_of(reading(t2), 1502)));
      REQUIRE(job.step() == JobStep::RESULT);
      REQUIRE(job.result().ok);
      CHECK(job.result().trial_check[0].ok);
      CHECK(job.result().trial_check[1].ok);

      // Fit what the screen says, the way an operator would.
      cplx fitted[2];
      for (int p = 0; p < 2; p++) {
        const auto &m = job.result().plane[p].mass;
        double ang = job.to_internal_angle(m.angle_deg);
        if (m.remove) ang += 180.0;  // removing mass at a spot == adding opposite
        fitted[p] = unbalance(m.mass_g, cfg.plane[p].corr_radius_mm, ang);
        CHECK(std::abs(fitted[p] + u0[p]) < 1e-6);
        if (remove) CHECK(job.result().plane[p].drill_depth_mm > 0);
      }
      job.begin_verify();
      REQUIRE(job.record(stats_of(reading(fitted), 1502)));
      CHECK(job.step() == JobStep::VERIFIED);
      CHECK(job.result().have_residual);
      CHECK(job.result().within_iso[0]);
      CHECK(job.result().within_iso[1]);
      CHECK(job.result().residual_gmm[0] < 1e-6);
      CHECK(job.result().reduction_pct[0] > 99.9);
    }
  }
}

TEST_CASE("job: single plane, kept trial weight, split positions, instructions") {
  cplx a[2] = {polar_deg(0.02, 75), polar_deg(0.01, 120)};
  cplx u0 = unbalance(2.0, 40, 123);
  JobConfig cfg;
  cfg.planes = 1;
  cfg.keep_last_trial = true;
  cfg.plane[0].remove_mode = false;
  cfg.plane[0].positions = 8;
  BalanceJob job;
  job.reset(cfg);
  job.start();
  CHECK(job.instruction().find("LOCK SPEED") != std::string::npos);
  cplx v0[2] = {a[0] * u0, a[1] * u0};
  REQUIRE(job.record(stats_of(v0, 1500)));
  CHECK(job.instruction().find("1455-1545 rpm") != std::string::npos);
  const cplx t = job.trial_vector(0);
  cplx v1[2] = {a[0] * (u0 + t), a[1] * (u0 + t)};
  REQUIRE(job.record(stats_of(v1, 1500)));
  REQUIRE(job.step() == JobStep::RESULT);
  const auto &pr = job.result().plane[0];
  // With the trial weight left on, the added weight must cancel u0 + t.
  CHECK(std::abs(pr.u + u0 + t) < 1e-6);
  REQUIRE(pr.use_split);
  const cplx sum = polar_deg(pr.split.mass_a, pr.split.angle_a) + polar_deg(pr.split.mass_b, pr.split.angle_b);
  CHECK(std::abs(sum - polar_deg(pr.mass.mass_g, pr.mass.angle_deg)) < 1e-9);
  CHECK(job.suggested_trial_g(0) == doctest::Approx(3.48).epsilon(0.01));
}

// ---------------------------------------------------------------------------
// Tach timebase (LRCK capture + DMA soft catch)

#include <algorithm>
#include "dsp_timebase.h"

TEST_CASE("timebase maps tach ticks to fractional frame indices across timer wrap") {
 for (bool drop_lrck : {false, true}) {
  CAPTURE(drop_lrck);
  const double timer_hz = 80e6;
  const double fs = 16000.0 * (1.0 + 37e-6);  // real crystal error
  const double t0 = 0.0123;                   // I2S starts 12.3 ms after the timer
  const uint32_t tick_offset = 0xFFFFFFFFu - 40000000u;  // wraps 0.5 s in
  auto ticks_at = [&](double t) { return static_cast<uint32_t>(tick_offset + static_cast<uint64_t>(std::llround(t * timer_hz))); };

  struct Ev { double t; int kind; uint32_t frames; };
  std::vector<Ev> ev;
  // LRCK rising edge is half a frame into its frame; the prescaler fires on
  // the 256th edge, i.e. frames 255, 511, ...
  for (int k = 0; k < 400; k++) {
    const int frame = 255 + 256 * k;
    if (drop_lrck && k % 37 == 20) continue;  // lost capture interrupt
    ev.push_back({t0 + (frame + 0.5) / fs, 1, 0});
  }
  // DMA complete for every 256-frame buffer, 4-9 us late.
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> lat(4e-6, 9e-6), when(0.05, 6.0);
  for (int m = 1; m < 400; m++) ev.push_back({t0 + 256.0 * m / fs + lat(rng), 2, static_cast<uint32_t>(256 * m)});
  std::vector<double> tach_t;
  for (int i = 0; i < 200; i++) tach_t.push_back(when(rng));
  std::sort(tach_t.begin(), tach_t.end());
  for (double t : tach_t) ev.push_back({t, 0, 0});
  std::sort(ev.begin(), ev.end(), [](const Ev &a, const Ev &b) { return a.t < b.t; });

  TachTimebase tb;
  tb.configure(timer_hz, 16000.0, 256);
  std::vector<double> got;
  for (auto &e : ev) {
    if (e.kind == 0) tb.on_tach(ticks_at(e.t));
    if (e.kind == 1) tb.on_lrck(ticks_at(e.t));
    if (e.kind == 2) tb.on_soft(ticks_at(e.t), e.frames);
    double f;
    while (tb.pop_tach(f)) got.push_back(f);
  }
  CHECK(tb.aligned());
  CHECK(tb.realigns() == 0);
  CHECK(tb.measured_fs() == doctest::Approx(fs).epsilon(1e-6));
  // Every tach after alignment (~30 ms in) must land on the right frame.
  int checked = 0;
  double worst = 0;
  for (double f : got) {
    // find the matching true time
    double best = 1e9;
    for (double t : tach_t) best = std::min(best, std::fabs(f - (t - t0) * fs));
    worst = std::max(worst, best);
    checked++;
  }
  CHECK(checked >= 195);
  INFO("worst frame error " << worst);
  CHECK(worst < 0.01);
 }
}

// ---------------------------------------------------------------------------
// On-device demo simulator drives the full wizard end to end.

#include "dsp_sim.h"

TEST_CASE("demo simulator + job: coast-downs give a correction that works") {
  RevSim sim;
  JobConfig cfg;
  BalanceJob job;
  job.reset(cfg);
  job.start();
  MeasureConfig mc;
  Measurement meas(mc);
  RevResult buf[16];
  auto run = [&](const cplx f1, const cplx f2) {
    sim.set_fitted(f1, f2);
    meas.reset();
    if (job.test_rpm() > 0) meas.set_target_rpm(job.test_rpm());
    for (int pass = 0; pass < 10 && !meas.stats().ready; pass++) {
      sim.blast(1700);
      while (sim.rpm() > 1300) {
        const int n = sim.step(0.05, buf, 16);
        for (int i = 0; i < n; i++) {
          if (meas.config().target_rpm <= 0 && buf[i].rpm < 1520) meas.set_target_rpm(1500);  // "LOCK SPEED"
          meas.add(buf[i]);
        }
      }
    }
    REQUIRE(meas.stats().ready);
    REQUIRE(job.record(meas.stats()));
  };
  run(0, 0);
  run(job.trial_vector(0), 0);
  run(0, job.trial_vector(1));
  REQUIRE(job.result().ok);
  cplx fit[2];
  for (int p = 0; p < 2; p++) {
    const auto &m = job.result().plane[p].mass;
    double ang = job.to_internal_angle(m.angle_deg) + (m.remove ? 180.0 : 0.0);
    fit[p] = unbalance(m.mass_g, cfg.plane[p].corr_radius_mm, ang);
    CHECK(std::abs(fit[p] + sim.u0[p]) / std::abs(sim.u0[p]) < 0.05);
  }
  job.begin_verify();
  run(fit[0], fit[1]);
  CHECK(job.result().within_iso[0]);
  CHECK(job.result().within_iso[1]);
}
