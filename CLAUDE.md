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
- UI: C++ against LVGL 9.5 (`ui.cpp`), talking only to `UiModel`
  (`ui_model.h`). `ModelCore` (`ui_core.cpp`) is shared by the firmware and
  `tools/ui_preview`, which renders every screen to PNG on a PC; check UI
  changes there before pushing. New LVGL widgets or fonts must be added to
  UI_WIDGETS / UI_FONTS in `__init__.py`.
- Sensor path: `acq_mpu` reads the MPU-6050s and `dsp_resample` puts them on
  one grid (PLAN §4, `docs/hardware/tire-rig.md`). Starting values for a
  wheel are in `ModelCore::load_defaults`.
- Component files are flat (ESPHome ignores subdirectories of external
  components): `dsp_*` pure C++, `acq_*` ESP-IDF, `ui*` LVGL, `balancer.*` glue.
  DSP namespace is `bal` (not `balancer`, which is ESPHome's).

## Rules
- `dsp_*` files are pure C++17: no ESPHome, IDF or LVGL headers.
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
- UI preview: `cmake -S tools/ui_preview -B build/ui -DLVGL_DIR=<lvgl v9.5.0> && cmake --build build/ui && build/ui/ui_preview <outdir>`
- `secrets.yaml` is gitignored; copy `secrets.yaml.example`.
