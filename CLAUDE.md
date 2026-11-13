# CLAUDE.md — guidance for Claude Code sessions in this repo

Two-plane dynamic balancer for car wheels on the Guition JC1060P470C
(ESP32-P4, 7" LCD). Read `PLAN.md` first; it is the source of truth for
architecture and phases. Decisions are in PLAN §0: two GY-521 (MPU-6050)
accelerometers on I²C, an optical tach pulse on a GPIO, ESPHome, and the
wheel is spun by hand on a soft rig (no drive). Speed is unregulated, so
measurements use the speed window and coast-down capture in PLAN §5.1.
This repo holds only that accelerometer firmware; the old IEPE/PCM1808 path
was removed on 2026-11-12 (it is in git history).

## Stack
- ESPHome (esp-idf framework) plus a local external component in
  `components/balancer`. This follows the same pattern as
  clowrey/CellKeeper-BMS-LCD, which runs on the same board.
- Board config: `hardware_guition_jc1060p470.yaml` (known-good, from CellKeeper).
  Don't change display, touch, PSRAM or LDO settings without a reason.
- UI: plain C++, no LVGL. The screens (`ui.cpp`, `ui_screens.cpp`) draw into
  an RGB565 frame in PSRAM (`ui_canvas.*`, from CellKeeper's `hmi_ui`) and
  only the 16-row bands that changed are sent to the panel. They talk only to
  `UiModel` (`ui_model.h`). Immediate mode: a screen runs top to bottom every
  frame and `button(...)` returns true when tapped (`ui_kit.h`). `ModelCore`
  (`ui_core.cpp`) is shared by the firmware, `tools/ui_sim` (renders every
  screen to PNG) and `tests/test_ui.cpp` (taps by label); check UI changes
  there before pushing. Fonts are generated: add a size or character in
  `tools/gen_ui_fonts.py` and run it, never edit `ui_fonts_data.cpp`.
- Sensor path: `acq_mpu` reads the MPU-6050s and `dsp_resample` puts them on
  one grid (PLAN §4, `docs/hardware/tire-rig.md`). Starting values for a
  wheel are in `ModelCore::load_defaults`.
- Component files are flat (ESPHome ignores subdirectories of external
  components): `dsp_*` pure C++, `acq_*` ESP-IDF, `ui*` pure C++, `balancer.*` glue.
  DSP namespace is `bal` (not `balancer`, which is ESPHome's).

## Rules
- `dsp_*` and `ui*` files are pure C++17: no ESPHome or IDF headers.
  It must build and pass tests on the host (`tests/`, CMake + doctest).
- Real-time acquisition (`acq_mpu`) runs in its own FreeRTOS
  task pinned to core 1. Never do sensor I²C, MCPWM or DSP work in
  ESPHome `loop()`; hand results over through a queue or double buffer. The
  MPU-6050 bus is a separate I²C port owned by that task, not ESPHome's.
- All timing comes from hardware capture. Tach edges and sensor data-ready
  edges are latched by MCPWM capture. No `micros()` in ISRs for timing (see PLAN §4).
- No bandpass filters in the measurement path. Synchronous averaging does
  the filtering.
- Pins and calibration values come from YAML config, never hard-coded.
- Settings changed on the screen are saved to flash at once and must survive
  firmware updates. Don't change the layout of `SavedState` (balancer.h) or
  the structs in it without migrating the previous layout: a size change
  silently resets the person's calibrated settings.
- Pin reference: `docs/hardware/pinout.md`. GPIO5 is avoided (LCD reset in
  the CellKeeper YAML).
- Run the simulator and cross-check tests (`tests/sim`, `tools/`) before
  claiming DSP or solver changes work.

## Build
- Firmware: `esphome compile balancer-tire.yaml` / `balancer-tire-demo.yaml`.
  The cloud dev workspace cannot reach Espressif's download hosts, so
  firmware is compiled by GitHub Actions; compile errors show up as check-run
  annotations and binaries on the `latest` pre-release.
- Host tests: `cmake -S tests -B build/tests && cmake --build build/tests && ctest --test-dir build/tests`
- UI simulator: `sh tools/ui_sim/run.sh [outdir]` (g++ and Python with Pillow;
  default outdir `docs/ui`).
- `secrets.yaml` is gitignored; copy `secrets.yaml.example`.
