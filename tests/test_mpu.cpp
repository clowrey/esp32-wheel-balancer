// Host tests for the digital-sensor path: resampling MPU-6050 readings onto
// the capture-timer grid, and a tire balanced end to end through the same
// ModelCore the firmware runs.
#include "third_party/doctest.h"

#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "dsp_job.h"
#include "dsp_measurement.h"
#include "dsp_resample.h"
#include "dsp_revolution.h"
#include "sim_mpu.h"
#include "ui_core.h"

using namespace bal;

namespace {

constexpr double kTimerHz = 80e6;

RevolutionConfig mpu_rev_config(double fs, double delay_s) {
  RevolutionConfig rc;
  rc.fs = fs;
  rc.delay_s = delay_s;
  rc.min_rpm = 30;
  rc.max_rpm = 3000;
  rc.ring_size = 1u << 12;
  return rc;
}

// Moves whatever the resampler has ready into the analyzer, the way the
// acquisition task does.
void drain(StreamResampler &rs, RevolutionAnalyzer &an) {
  if (rs.take_restart()) an.reset();
  float a[64], b[64];
  size_t n;
  while ((n = rs.pop_frames(a, b, 64)) > 0) an.push_samples(a, b, n);
  double t;
  while (rs.pop_tach(t)) an.push_tach(t);
}

}  // namespace

TEST_CASE("resampler: two sensors on their own clocks land on one grid, across timer wrap") {
  // A steady 4.2 Hz rotor. Sensor A runs 0.3% slow, sensor B 0.5% fast, and
  // they start at different instants. Both must come out with the right 1x
  // amplitude and phase against the tach.
  const double f_rot = 4.2, fs = 250.0;
  const double amp[2] = {0.040, 0.025}, lag[2] = {63.0, 201.0};
  const double rate[2] = {249.25, 251.25}, start[2] = {0.0031, 0.0102};
  const uint32_t tick_offset = 0xFFFFFFFFu - 160000000u;  // wraps 2 s in
  auto ticks_at = [&](double t) {
    return static_cast<uint32_t>(tick_offset + static_cast<uint64_t>(std::llround(t * kTimerHz)));
  };
  auto value = [&](int s, double t) {
    return 0.2 + amp[s] * std::cos(kTwoPi * f_rot * t - lag[s] / kDegPerRad);  // 0.2 g of tilt on top
  };

  StreamResampler rs;
  rs.configure(kTimerHz, fs, 2);
  RevolutionAnalyzer an(mpu_rev_config(fs, 0));
  std::vector<RevResult> revs;
  RevResult r;
  double next[2] = {start[0], start[1]};
  double next_tach = 0.05;  // the mark passes the pickup at t = 0.05 + k / f_rot
  const double tach0 = next_tach;
  while (next[0] < 12.0) {
    int which = 2;
    double t = next_tach;
    for (int s = 0; s < 2; s++)
      if (next[s] < t) {
        t = next[s];
        which = s;
      }
    if (which == 2) {
      rs.push_tach(ticks_at(t));
      next_tach += 1.0 / f_rot;
    } else {
      // lag is defined against the tach mark, so shift time by the first pulse
      rs.push(which, ticks_at(t), static_cast<float>(value(which, t - tach0)));
      next[which] += 1.0 / rate[which];
    }
    drain(rs, an);
    while (an.pop(r)) revs.push_back(r);
  }
  REQUIRE(revs.size() >= 40);
  CHECK(rs.restarts() == 0);
  CHECK(rs.measured_rate(0) == doctest::Approx(rate[0]).epsilon(1e-4));
  CHECK(rs.measured_rate(1) == doctest::Approx(rate[1]).epsilon(1e-4));
  for (size_t i = 1; i < revs.size(); i++) {
    const RevResult &v = revs[i];
    CHECK(v.rpm == doctest::Approx(f_rot * 60).epsilon(1e-5));
    for (int s = 0; s < 2; s++) {
      CHECK(std::abs(v.order[s][0]) == doctest::Approx(amp[s]).epsilon(0.01));
      CHECK(wrap180(arg_deg(v.order[s][0]) - lag[s]) == doctest::Approx(0).epsilon(0.3).scale(1));
    }
  }
}

TEST_CASE("resampler: lost readings are bridged, a long dropout restarts the grid") {
  const double fs = 250.0;
  StreamResampler rs;
  rs.configure(kTimerHz, fs, 1);
  auto ticks_at = [](double t) { return static_cast<uint32_t>(std::llround(t * kTimerHz)); };
  float a[64], b[64];
  uint64_t frames = 0;
  double worst = 0;
  // A slow ramp, so interpolation across the holes is exact.
  for (int n = 0; n < 1000; n++) {
    if (n % 97 == 50 || n % 97 == 51) continue;  // two readings in a row lost
    const double t = 0.01 + n / 250.0;
    rs.push(0, ticks_at(t), static_cast<float>(3.0 * t));
    size_t got;
    while ((got = rs.pop_frames(a, b, 64)) > 0) {
      for (size_t i = 0; i < got; i++) {
        const double tg = 0.01 + static_cast<double>(frames + i) / fs;  // grid starts at the first reading
        worst = std::max(worst, std::fabs(a[i] - 3.0 * tg));
        CHECK(b[i] == 0.0f);  // single-sensor mode: channel B reads zero
      }
      frames += got;
    }
  }
  CHECK(rs.restarts() == 0);
  CHECK(frames >= 995);
  CHECK(worst < 1e-4);
  CHECK_FALSE(rs.take_restart());

  // Half a second of silence: the grid starts over and says so once.
  rs.push(0, ticks_at(4.6), 1.0f);
  CHECK(rs.restarts() == 1);
  CHECK(rs.take_restart());
  CHECK_FALSE(rs.take_restart());
  rs.push(0, ticks_at(4.604), 2.0f);
  CHECK(rs.pop_frames(a, b, 64) == 2);  // grid points at the first reading and one period on
  CHECK(a[0] == doctest::Approx(1.0f));
  CHECK(a[1] == doctest::Approx(2.0f));

  // A tach edge lands on the new grid by plain division.
  rs.push_tach(ticks_at(4.6 + 10.5 / fs));
  double idx;
  REQUIRE(rs.pop_tach(idx));
  CHECK(idx == doctest::Approx(10.5).epsilon(1e-6));

  // Forty seconds with no events at all: the 32-bit timer has gone more than
  // half way round, so the next reading looks 13 s old. It must start a new
  // grid, not be thrown away as out of order.
  rs.push(0, ticks_at(4.604 + 40.0), 5.0f);
  CHECK(rs.restarts() == 2);
  CHECK(rs.take_restart());
  rs.push(0, ticks_at(4.608 + 40.0), 6.0f);
  CHECK(rs.pop_frames(a, b, 64) == 2);
  CHECK(a[0] == doctest::Approx(5.0f));

  // More than a handful of readings lost in a row is not bridged either.
  rs.push(0, ticks_at(4.608 + 40.0 + 8 / fs), 7.0f);
  CHECK(rs.restarts() == 3);
}

TEST_CASE("orders above the usable band read zero") {
  // 600 rpm sampled at 250 Hz: 25 samples per revolution, so order 10
  // (100 Hz) fits under 0.45*fs = 112 Hz but at 720 rpm it does not.
  for (double rpm : {600.0, 720.0}) {
    RevolutionAnalyzer an(mpu_rev_config(250, 0));
    const double period = 250.0 * 60.0 / rpm;
    std::vector<float> x(2000), z(2000, 0.0f);
    for (int n = 0; n < 2000; n++) x[n] = static_cast<float>(std::cos(kTwoPi * n / period) + 0.5 * std::cos(kTwoPi * 10 * n / period));
    an.push_samples(x.data(), z.data(), 2000);
    for (int k = 0; k < 20; k++) an.push_tach(k * period);
    RevResult r;
    REQUIRE(an.pop(r));
    CHECK(std::abs(r.order[0][0]) == doctest::Approx(1.0).epsilon(0.02));
    if (rpm == 600.0)
      CHECK(std::abs(r.order[0][9]) == doctest::Approx(0.5).epsilon(0.05));
    else
      CHECK(std::abs(r.order[0][9]) == 0.0);
  }
}

TEST_CASE("tire preset: trial helper is capped, wording has no air, one sensor means one plane") {
  ModelCore m;
  m.apply_preset(Preset::TIRE);
  BalanceJob &job = m.job();
  const JobConfig &cfg = job.config();
  CHECK(cfg.rotor_mass_kg == doctest::Approx(20));
  CHECK_FALSE(cfg.plane[0].remove_mode);
  CHECK(m.settings().quantity == Quantity::DISPLACEMENT_UM);
  // G6.3, 20 kg, 800 rpm: 1504 g*mm, half per plane = 3.8 g at 200 mm.
  CHECK(iso_uper_gmm(cfg.grade, cfg.rotor_mass_kg, cfg.service_rpm) / 2 / 200 == doctest::Approx(3.76).epsilon(0.01));
  // 10% of the wheel's weight at 240 rpm would be 155 g; the cap gives 38 g.
  job.start();
  job.set_test_rpm(240);
  CHECK(suggest_trial_gmm(20, 240, 0.1) / 200 == doctest::Approx(155).epsilon(0.01));
  CHECK(job.suggested_trial_capped(0));
  CHECK(job.suggested_trial_g(0) == doctest::Approx(37.6).epsilon(0.01));
  CHECK(job.instruction().find("air") == std::string::npos);
  CHECK(job.instruction().find("216-264 rpm") != std::string::npos);

  // The motor-rotor preset is unchanged by the cap (PLAN §6.1: 3.5 g).
  m.apply_preset(Preset::MOTOR_ROTOR);
  m.job().start();
  m.job().set_test_rpm(1500);
  CHECK_FALSE(m.job().suggested_trial_capped(0));
  CHECK(m.job().suggested_trial_g(0) == doctest::Approx(3.48).epsilon(0.01));
  CHECK(m.job().instruction().find("air") != std::string::npos);

  // One sensor: two planes cannot be solved, so the job drops to one.
  m.apply_preset(Preset::TIRE);
  m.job().set_max_planes(1);
  m.job().mutable_config().planes = 2;  // what the setup screen toggle does
  m.job().start();
  CHECK(m.job().config().planes == 1);
}

TEST_CASE("best-effort readiness after 4x the minimum revolutions") {
  // A reading so small that its phase never settles.
  for (int factor : {0, 4}) {
    MeasureConfig mc;
    mc.target_rpm = 240;
    mc.window = 0.1;
    mc.target_revs = 20;
    mc.max_phase_unc_deg = 3;
    mc.amp_floor = 1e-9;
    mc.quantity = Quantity::ACCEL_G;
    mc.best_effort_factor = factor;
    Measurement meas(mc);
    std::mt19937 rng(3);
    std::normal_distribution<double> g(0.0, 1.0);
    RevResult r;
    r.rpm = 240;
    r.omega = 240 * kTwoPi / 60;
    for (int i = 0; i < 80; i++) {
      r.order[0][0] = cplx(0.001 + 0.002 * g(rng), 0.002 * g(rng));
      r.order[1][0] = cplx(0.002 * g(rng), 0.001 + 0.002 * g(rng));
      meas.add(r);
      if (i == 78) CHECK_FALSE(meas.stats().ready);
    }
    CHECK(meas.stats().ready == (factor == 4));
    CHECK(meas.stats().best_effort == (factor == 4));
  }
}

// ---------------------------------------------------------------------------
// End to end: a car wheel on a soft rig, two MPU-6050s, hand spins. Readings
// go through the resampler and analyzer into ModelCore with the tire preset,
// exactly the path the firmware takes, and the wizard is walked to a verify.

TEST_CASE("simulated tire balance with two MPU-6050s and hand spins") {
  sim::MpuRig rig;
  // Soft rig: the hub moves about U/M. Sensor A is on the bearing next to
  // the wheel, B on the far one (the couple reverses there).
  rig.a_true[0][0] = polar_deg(3.0e-6, 182);
  rig.a_true[0][1] = polar_deg(4.2e-6, 178);
  rig.a_true[1][0] = polar_deg(0.8e-6, 175);
  rig.a_true[1][1] = polar_deg(1.0e-6, 5);
  rig.u0[0] = unbalance(35.0, 200, 70);   // 7000 g*mm on the inner rim
  rig.u0[1] = unbalance(25.0, 200, 130);  // 5000 g*mm on the outer rim

  ModelCore m;
  m.apply_preset(Preset::TIRE);
  m.set_digital_sensor(true);
  BalanceJob &job = m.job();
  job.mutable_config().plane[0].trial_angle_deg = 0;
  job.mutable_config().plane[1].trial_angle_deg = 90;
  job.start();

  uint32_t seed = 40;
  double clock_s = 1.0;
  auto run = [&](cplx f0, cplx f1) {
    const cplx extra[2] = {f0, f1};
    rig.seed = seed++;
    StreamResampler rs;
    rs.configure(kTimerHz, 250.0, 2);
    RevolutionAnalyzer an(mpu_rev_config(250.0, 0));
    const uint32_t delay_ticks = static_cast<uint32_t>(rig.dlpf_delay_s * kTimerHz + 0.5);
    m.start_run();
    RevResult r;
    int passes = 0;
    // The operator keeps spinning the wheel until the screen says ready.
    while (!m.run_stats().ready && passes < 12) {
      passes++;
      rig.time_origin_s = clock_s;
      const double dur = sim::run_hand_spins(
          rig, extra, 1, 300, 190, 30.0,
          [&](int s, uint32_t ticks, float g) {
            rs.push(s, ticks - delay_ticks, g);  // stamped when the motion happened, as acq_mpu does
            drain(rs, an);
            while (an.pop(r)) {
              if (job.test_rpm() <= 0 && r.rpm < 245) m.lock_speed(240);  // "LOCK SPEED"
              m.feed_rev(r, static_cast<uint32_t>(clock_s * 1000));
            }
          },
          [&](uint32_t ticks) { rs.push_tach(ticks); });
      clock_s += dur;
      m.tick(static_cast<uint32_t>(clock_s * 1000));
      rig.seed = seed++;
    }
    const MeasureStats st = m.run_stats();
    std::printf("tire run: %d spins, %d revs, A %.1f um @ %.0f deg (+/- %.1f), B %.1f um @ %.0f deg (+/- %.1f)%s\n", passes,
                st.revs, st.ch[0].amp, st.ch[0].phase_deg, st.ch[0].phase_unc_deg, st.ch[1].amp, st.ch[1].phase_deg,
                st.ch[1].phase_unc_deg, st.best_effort ? "  [best effort]" : "");
    REQUIRE(st.ready);
    std::string err;
    REQUIRE_MESSAGE(m.accept_run(&err), err);
    return passes;
  };

  CHECK(run(0, 0) <= 4);
  CHECK(job.test_rpm() == doctest::Approx(240));
  run(job.trial_vector(0), 0);
  run(0, job.trial_vector(1));
  REQUIRE(job.step() == JobStep::RESULT);
  REQUIRE(job.result().ok);
  CHECK(job.result().trial_check[0].ok);
  CHECK(job.result().trial_check[1].ok);

  cplx fit[2];
  for (int p = 0; p < 2; p++) {
    const auto &pm = job.result().plane[p].mass;
    CHECK_FALSE(pm.remove);  // tire preset adds weights
    fit[p] = unbalance(pm.mass_g, job.config().plane[p].corr_radius_mm, job.to_internal_angle(pm.angle_deg));
    const cplx want = -rig.u0[p];
    const double amp_err = std::abs(std::abs(fit[p]) / std::abs(want) - 1.0);
    const double ph_err = std::fabs(wrap180(arg_deg(fit[p]) - arg_deg(want)));
    std::printf("tire plane %d: add %.1f g at %.0f deg (true %.1f g at %.0f deg; err %.1f%%, %.1f deg)\n", p + 1, pm.mass_g,
                pm.angle_deg, std::abs(want) / 200, arg_deg(want), amp_err * 100, ph_err);
    CHECK(amp_err < 0.10);
    CHECK(ph_err < 6.0);
  }

  job.begin_verify();
  run(fit[0], fit[1]);
  REQUIRE(job.step() == JobStep::VERIFIED);
  const JobResult &res = job.result();
  std::printf("tire residual: %.0f / %.0f g*mm (limit %.0f), vibration down %.0f%% / %.0f%%\n", res.residual_gmm[0],
              res.residual_gmm[1], res.plane[0].iso_limit_gmm, res.reduction_pct[0], res.reduction_pct[1]);
  CHECK(res.within_iso[0]);
  CHECK(res.within_iso[1]);
  CHECK(res.reduction_pct[0] > 85);
}
