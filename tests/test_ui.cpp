// Host tests for the touchscreen UI: it is tapped by label, the way a person uses it, with the
// demo wheel behind it (tools/ui_sim/harness.h).
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "third_party/doctest.h"

#include <cmath>

#include "harness.h"

using namespace bal;

namespace {

// The frame as it is now must be exactly what drawing everything afresh gives: only the bands
// that changed are drawn each frame, so a band that was missed would show stale pixels.
void check_partial_matches_full(Driver &d, const char *where) {
  const std::vector<uint16_t> partial = d.pixels;
  std::fill(d.pixels.begin(), d.pixels.end(), 0x1234);
  d.ui.redraw();
  d.frame();
  size_t differ = 0;
  for (size_t i = 0; i < partial.size(); i++) differ += partial[i] != d.pixels[i];
  INFO(where);
  CHECK(differ == 0);
}

void run_job_to_result(Driver &d) {
  REQUIRE(d.tap("New balance"));
  REQUIRE(d.tap("START"));
  d.advance(0.5);
  for (int i = 0; i < 3; i++) {
    d.run();
    REQUIRE(d.tap("ACCEPT"));
  }
  d.advance(0.2);
}

}  // namespace

TEST_CASE("a whole balancing job, by taps") {
  Driver d;
  CHECK(d.ui.current() == BalancerUi::HOME);
  CHECK_FALSE(d.has("Trim run"));  // no saved machine yet: the tile is disabled
  REQUIRE(d.tap("New balance"));
  CHECK(d.ui.current() == BalancerUi::SETUP);
  REQUIRE(d.tap("START"));
  CHECK(d.ui.current() == BalancerUi::MEASURE);
  CHECK(d.m.job().step() == JobStep::RUN0);
  CHECK_FALSE(d.has("ACCEPT"));  // not enough revolutions yet: disabled
  d.run();
  CHECK(d.has("ACCEPT"));
  REQUIRE(d.tap("ACCEPT"));
  CHECK(d.m.job().step() == JobStep::TRIAL1);
  d.run();
  REQUIRE(d.tap("ACCEPT"));
  CHECK(d.m.job().step() == JobStep::TRIAL2);
  d.run();
  REQUIRE(d.tap("ACCEPT"));
  CHECK(d.ui.current() == BalancerUi::RESULT);
  CHECK(d.m.job().result().ok);

  REQUIRE(d.tap("VERIFY RUN"));
  CHECK(d.ui.current() == BalancerUi::MEASURE);
  d.run();
  REQUIRE(d.tap("ACCEPT"));
  CHECK(d.m.job().step() == JobStep::VERIFIED);
  CHECK(d.ui.current() == BalancerUi::RESULT);
  CHECK(d.m.job().result().within_iso[0]);
  CHECK(d.m.job().result().within_iso[1]);

  REQUIRE(d.tap("Save machine"));
  CHECK(d.m.has_machine());
  CHECK(d.ui.toast_text().find("saved") != std::string::npos);
  REQUIRE(d.tap("Home"));
  CHECK(d.has("Trim run"));
  REQUIRE(d.tap("Trim run"));
  CHECK(d.ui.current() == BalancerUi::MEASURE);
}

TEST_CASE("partial redraws give the same picture as drawing everything") {
  Driver d;
  check_partial_matches_full(d, "home");
  REQUIRE(d.tap("New balance"));
  check_partial_matches_full(d, "rotor setup");
  REQUIRE(d.tap("Plane 2"));
  check_partial_matches_full(d, "rotor setup, plane 2");
  REQUIRE(d.tap("Trial mass"));
  d.type("12");
  check_partial_matches_full(d, "number pad");
  REQUIRE(d.tap("Cancel"));
  check_partial_matches_full(d, "number pad closed");
  REQUIRE(d.tap("START"));
  d.advance(1.0);
  check_partial_matches_full(d, "measure, waiting");
  d.m.sim_spin();
  d.advance(2.0);
  check_partial_matches_full(d, "measure, spinning");
  d.run();
  check_partial_matches_full(d, "measure, ready");
  REQUIRE(d.tap("ACCEPT"));
  d.run();
  REQUIRE(d.tap("ACCEPT"));
  d.run();
  REQUIRE(d.tap("ACCEPT"));
  d.advance(0.2);
  check_partial_matches_full(d, "correction");
  d.m.settings().mass_oz = true;
  d.advance(0.2);
  check_partial_matches_full(d, "correction in ounces");
  REQUIRE(d.tap("Home"));
  REQUIRE(d.tap("Live vibration"));
  fake_sensor_readings(d.m.live_mut());
  d.m.sim_spin();
  d.advance(1.0);
  check_partial_matches_full(d, "live, orders");
  REQUIRE(d.tap("Raw readings"));
  d.advance(0.4);
  check_partial_matches_full(d, "live, raw readings");
  REQUIRE(d.tap("Home"));
  REQUIRE(d.tap("Settings"));
  d.advance(0.2);
  check_partial_matches_full(d, "settings");
  REQUIRE(d.tap("FIND AXES (guided)"));
  REQUIRE(d.tap("START"));
  d.advance(1.0);
  check_partial_matches_full(d, "axis finder, running");
}

TEST_CASE("the number pad") {
  Driver d;
  REQUIRE(d.tap("New balance"));
  JobConfig &cfg = d.m.job().mutable_config();

  REQUIRE(d.tap("Rotor mass"));
  CHECK(d.ui.number_pad_open());
  CHECK_FALSE(d.has("Home"));  // the screen under a pop-up does not react
  d.type("12.5");
  REQUIRE(d.tap("OK"));
  CHECK_FALSE(d.ui.number_pad_open());
  CHECK(cfg.rotor_mass_kg == doctest::Approx(12.5));

  REQUIRE(d.tap("Rotor mass"));
  d.type("99");
  REQUIRE(d.tap("Cancel"));
  CHECK(cfg.rotor_mass_kg == doctest::Approx(12.5));

  // Deleting and the sign
  REQUIRE(d.tap("Trial angle"));
  d.type("45");
  REQUIRE(d.tap("Del"));
  REQUIRE(d.tap("-"));
  REQUIRE(d.tap("OK"));
  CHECK(cfg.plane[0].trial_angle_deg == doctest::Approx(-4));

  // Out of range: clamped
  REQUIRE(d.tap("Home"));
  REQUIRE(d.tap("Settings"));
  REQUIRE(d.tap("Speed window"));
  d.type("50");
  REQUIRE(d.tap("OK"));
  CHECK(d.m.settings().window == doctest::Approx(0.10));

  // Trial mass is typed in the unit shown
  d.m.settings().mass_oz = true;
  REQUIRE(d.tap("Home"));
  REQUIRE(d.tap("New balance"));
  REQUIRE(d.tap("Trial mass"));
  d.type("2");
  REQUIRE(d.tap("OK"));
  CHECK(cfg.plane[0].trial_mass_g == doctest::Approx(2 * 28.349523125));
}

TEST_CASE("settings and the axis finder") {
  Driver d;
  fake_sensor_readings(d.m.live_mut());
  REQUIRE(d.tap("Settings"));
  CHECK(d.m.settings().axis[1] == 2);
  REQUIRE(d.tap("B axis X"));
  CHECK(d.m.settings().axis[1] == 0);
  CHECK(d.ui.toast_text().find("Sensor B now reads X") != std::string::npos);

  const bool oz = d.m.settings().mass_oz;
  REQUIRE(d.tap("Weight units"));
  CHECK(d.m.settings().mass_oz != oz);

  // No teach pin on the demo: holding the button only says so
  REQUIRE(d.tap("TEACH TACH (hold)"));
  CHECK(d.ui.toast_text().find("No teach pin") != std::string::npos);

  REQUIRE(d.tap("FIND AXES (guided)"));
  CHECK(d.ui.axis_finder_open());
  CHECK_FALSE(d.has("SAVE"));  // the settings screen under it does not react
  REQUIRE(d.tap("START"));
  d.m.live_mut().axis_rms[0][1] = 0.08f;  // pushed along Y
  d.advance(4.5);
  CHECK(d.m.settings().axis[0] == 1);
  REQUIRE(d.tap("Sensor B"));
  REQUIRE(d.tap("Close"));
  CHECK_FALSE(d.ui.axis_finder_open());
  CHECK(d.has("SAVE"));
}

TEST_CASE("nothing is drawn when nothing changes") {
  Driver d;
  d.frame();
  const int before = d.frames_drawn;
  d.frame();
  d.frame();
  CHECK(d.frames_drawn == before);
}

TEST_CASE("bad numbers in the data do not upset the drawing") {
  Driver d;
  LiveData &lv = d.m.live_mut();
  fake_sensor_readings(lv);
  const float nan = std::nanf("");
  lv.raw[0][100] = nan;
  lv.raw[1][7] = INFINITY;
  lv.wave[0][3] = nan;
  lv.axis_rms[0][1] = nan;
  REQUIRE(d.tap("Live vibration"));
  REQUIRE(d.tap("Raw readings"));
  d.advance(0.4);
  check_partial_matches_full(d, "live, raw readings with NaN in them");
  REQUIRE(d.tap("Home"));
  REQUIRE(d.tap("Settings"));
  REQUIRE(d.tap("FIND AXES (guided)"));
  d.advance(0.4);
  check_partial_matches_full(d, "axis finder with NaN in the shake");
}

TEST_CASE("two fonts of one size are told apart when deciding what to redraw") {
  std::vector<uint16_t> a(static_cast<size_t>(SCREEN_W) * SCREEN_H), b(a.size());
  Canvas ca(a.data(), SCREEN_W, SCREEN_H), cb(b.data(), SCREEN_W, SCREEN_H);
  ca.begin_dry();
  ca.text(FONT_R20, 10, 10, "Plane 1", kText);
  const uint32_t ra = ca.end_dry()[0];
  cb.begin_dry();
  cb.text(FONT_M20, 10, 10, "Plane 1", kText);
  const uint32_t rb = cb.end_dry()[0];
  CHECK(ra != rb);
}
