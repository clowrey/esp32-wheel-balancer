// Renders every screen of the balancer UI to images on a PC, driving the
// whole wizard with the demo rotor (same ModelCore + BalancerUi code as the
// firmware). Output: PPM files in the directory given as argv[1]. Pass "tire"
// as argv[2] to run the same script with the tire preset.
#include <cstdio>
#include <cstring>
#include <string>

#include "lvgl.h"
#include "ui.h"
#include "ui_core.h"

using namespace bal;

static constexpr int W = 1024, H = 600;
static uint16_t g_fb[W * H];
static uint8_t g_draw[W * H * 2];
static uint32_t g_now = 1;
static lv_display_t *g_disp;
static std::string g_out;
static std::string g_prefix;      // file name prefix ("tire_" for the tire preset)
static double g_lock_rpm = 1500;  // speed at which the script taps LOCK SPEED

static void flush_cb(lv_display_t *d, const lv_area_t *a, uint8_t *px) {
  const int w = a->x2 - a->x1 + 1;
  auto *src = reinterpret_cast<uint16_t *>(px);
  for (int y = a->y1; y <= a->y2; y++) std::memcpy(&g_fb[y * W + a->x1], &src[(y - a->y1) * w], w * 2);
  lv_display_flush_ready(d);
}
static uint32_t tick_cb() { return g_now; }

static void render(const char *name) {
  lv_obj_invalidate(lv_screen_active());
  lv_refr_now(g_disp);
  std::string path = g_out + "/" + g_prefix + name + ".ppm";
  FILE *f = std::fopen(path.c_str(), "wb");
  std::fprintf(f, "P6\n%d %d\n255\n", W, H);
  for (int i = 0; i < W * H; i++) {
    const uint16_t p = g_fb[i];
    const uint8_t rgb[3] = {static_cast<uint8_t>(((p >> 11) & 0x1f) * 255 / 31),
                            static_cast<uint8_t>(((p >> 5) & 0x3f) * 255 / 63),
                            static_cast<uint8_t>((p & 0x1f) * 255 / 31)};
    std::fwrite(rgb, 1, 3, f);
  }
  std::fclose(f);
  std::printf("wrote %s\n", path.c_str());
}

static void advance(ModelCore &m, BalancerUi &ui, double seconds) {
  const int steps = static_cast<int>(seconds * 50);
  for (int i = 0; i < steps; i++) {
    g_now += 20;
    m.tick(g_now);
    if (i % 10 == 0) ui.update();
    lv_timer_handler();
  }
  ui.update();
}

// One measurement run the way an operator does it with the demo rotor.
static void do_run(ModelCore &m, BalancerUi &ui, const char *mid_shot) {
  for (int pass = 0; pass < 30 && !m.run_stats().ready; pass++) {
    m.sim_blast();
    advance(m, ui, 0.3);  // let the speed readout catch up with the blast
    const double T = m.job().test_rpm() > 0 ? m.job().test_rpm() : g_lock_rpm;
    while (m.live().rpm == 0 || m.live().rpm > T * 0.95) {
      advance(m, ui, 0.1);
      if (m.job().test_rpm() <= 0 && m.live().rpm > 0 && m.live().rpm <= g_lock_rpm) m.lock_speed(0);  // LOCK SPEED
      if (mid_shot != nullptr && m.run_stats().revs >= 12) {
        render(mid_shot);
        mid_shot = nullptr;
      }
    }
  }
  advance(m, ui, 0.3);
}

int main(int argc, char **argv) {
  g_out = argc > 1 ? argv[1] : ".";
  lv_init();
  lv_tick_set_cb(tick_cb);
  g_disp = lv_display_create(W, H);
  lv_display_set_color_format(g_disp, LV_COLOR_FORMAT_RGB565);
  lv_display_set_buffers(g_disp, g_draw, nullptr, sizeof(g_draw), LV_DISPLAY_RENDER_MODE_FULL);
  lv_display_set_flush_cb(g_disp, flush_cb);

  ModelCore m;
  m.set_simulate(true);
  if (argc > 2 && std::strcmp(argv[2], "tire") == 0) {
    // Same script with the tire preset and digital sensors (balancer-tire.yaml).
    m.apply_preset(Preset::TIRE);
    m.set_digital_sensor(true);
    g_prefix = "tire_";
    g_lock_rpm = 245;
  }
  BalancerUi ui;
  ui.build(&m);
  advance(m, ui, 0.2);
  render("01_home");

  ui.show(BalancerUi::SETUP);
  advance(m, ui, 0.2);
  render("02_setup");

  ui.press_start();
  advance(m, ui, 0.5);
  render("03_run0_start");
  do_run(m, ui, "04_run0_collecting");
  render("05_run0_ready");
  ui.press_accept();
  do_run(m, ui, "06_trial1_collecting");
  ui.press_accept();
  do_run(m, ui, nullptr);
  ui.press_accept();
  advance(m, ui, 0.2);
  render("07_result");

  ui.press_verify();
  do_run(m, ui, nullptr);
  ui.press_accept();
  advance(m, ui, 0.2);
  render("08_verified");

  ui.show(BalancerUi::LIVE);
  m.sim_blast();
  advance(m, ui, 1.0);
  render("09_live");

  ui.show(BalancerUi::SETTINGS);
  advance(m, ui, 0.2);
  render("10_settings");

  m.save_machine();
  ui.show(BalancerUi::HOME);
  advance(m, ui, 0.2);
  render("11_home_with_machine");

  const auto &r = m.job().result();
  std::printf("verify: residual %.2f / %.2f g*mm, within ISO %d %d\n", r.residual_gmm[0], r.residual_gmm[1],
              r.within_iso[0], r.within_iso[1]);
  return 0;
}
