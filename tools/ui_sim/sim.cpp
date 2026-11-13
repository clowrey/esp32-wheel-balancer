// Renders the touchscreen UI on a PC: taps its way through a whole balancing job with the demo
// wheel (the same ModelCore and BalancerUi code as the firmware) and takes a picture of each
// screen on the way.
//
//   sh tools/ui_sim/run.sh [outdir]      builds this, runs it, leaves PNG files (default docs/ui)
#include <cstdio>
#include <string>

#include "harness.h"

using namespace bal;

static std::string g_out;
static int g_failed = 0;

static void shot(const Driver &d, const char *name) {
  const std::string path = g_out + "/" + name + ".ppm";
  if (!write_ppm(path, d.pixels)) {
    std::printf("could not write %s\n", path.c_str());
    g_failed++;
    return;
  }
  std::printf("wrote %s\n", path.c_str());
}

static void step(bool ok, const char *what) {
  if (!ok) {
    std::printf("FAILED: %s\n", what);
    g_failed++;
  }
}

int main(int argc, char **argv) {
  g_out = argc > 1 ? argv[1] : ".";
  Driver d;
  d.advance(0.2);
  shot(d, "01_home");

  step(d.tap("New balance"), "open the rotor setup");
  d.advance(0.2);
  shot(d, "02_setup");
  step(d.tap("Trial mass"), "open the number pad");
  d.type("45");
  shot(d, "13_number_pad");
  step(d.tap("Cancel"), "close the number pad");

  step(d.tap("START"), "start the job");
  d.advance(0.5);
  shot(d, "03_run0_start");
  std::vector<uint16_t> mid;
  d.run(245, &mid);
  if (!mid.empty()) write_ppm(g_out + "/04_run0_collecting.ppm", mid), std::printf("wrote 04_run0_collecting\n");
  shot(d, "05_run0_ready");
  step(d.tap("ACCEPT"), "accept run 0");
  mid.clear();
  d.run(245, &mid);
  if (!mid.empty()) write_ppm(g_out + "/06_trial1_collecting.ppm", mid), std::printf("wrote 06_trial1_collecting\n");
  step(d.tap("ACCEPT"), "accept trial 1");
  d.run();
  step(d.tap("ACCEPT"), "accept trial 2");
  d.advance(0.2);
  step(d.ui.current() == BalancerUi::RESULT, "the correction is shown");
  shot(d, "07_result");
  d.m.settings().mass_oz = true;
  d.advance(0.2);
  shot(d, "07_result_oz");
  d.m.settings().mass_oz = false;

  step(d.tap("VERIFY RUN"), "start the verify run");
  d.run();
  step(d.tap("ACCEPT"), "accept the verify run");
  d.advance(0.2);
  shot(d, "08_verified");
  step(d.tap("Save machine"), "save the machine");
  d.advance(0.2);
  shot(d, "14_toast");

  step(d.tap("Home"), "back home");
  d.advance(4.0);  // the toast goes
  shot(d, "11_home_with_machine");

  step(d.tap("Live vibration"), "open live");
  fake_sensor_readings(d.m.live_mut());
  d.m.sim_spin();
  d.advance(1.0);
  shot(d, "09_live");
  step(d.tap("Raw readings"), "show the raw readings");
  d.advance(0.2);
  shot(d, "09_live_raw");
  step(d.tap("Orders"), "back to the orders");
  step(d.tap("Home"), "back home");

  step(d.tap("Settings"), "open settings");
  d.advance(0.2);
  shot(d, "10_settings");
  step(d.tap("FIND AXES (guided)"), "open the axis finder");
  d.advance(0.2);
  shot(d, "12_find_axes");
  step(d.tap("START"), "start finding an axis");
  d.advance(1.5);
  {
    // Sensor A pushed forward and back: most of the motion on its Y axis
    LiveData &lv = d.m.live_mut();
    lv.axis_rms[0][0] = 0.004f;
    lv.axis_rms[0][1] = 0.065f;
    lv.axis_rms[0][2] = 0.011f;
  }
  d.advance(0.2);
  shot(d, "12_find_axes_running");
  d.advance(3.0);
  step(d.m.settings().axis[0] == 1, "the axis finder picked Y for sensor A");
  shot(d, "12_find_axes_result");
  step(d.tap("Sensor B"), "go on to sensor B");
  step(d.tap("Close"), "close the axis finder");

  const auto &r = d.m.job().result();
  std::printf("verify: residual %.2f / %.2f g*mm, within ISO %d %d; %d frames drawn\n", r.residual_gmm[0],
              r.residual_gmm[1], r.within_iso[0], r.within_iso[1], d.frames_drawn);
  if (g_failed) std::printf("%d step(s) failed\n", g_failed);
  return g_failed ? 1 : 0;
}
