# CLAUDE.md — guidance for Claude Code sessions in this repo

Two-plane dynamic balancer on the Guition JC1060P470C (ESP32-P4, 7" LCD).
Read `PLAN.md` first; it is the source of truth for architecture and phases.

## Stack
- ESPHome (esp-idf framework) plus a local external component in
  `components/balancer`. This follows the same pattern as
  clowrey/CellKeeper-BMS-LCD, which runs on the same board.
- Board config: `hardware_guition_jc1060p470.yaml` (known-good, from CellKeeper).
  Don't change display, touch, PSRAM or LDO settings without a reason.
- UI: ESPHome LVGL YAML (`ui_1024x600.yaml`). Custom drawing (rotor dials)
  is done in C++ inside the component.

## Rules
- `components/balancer/dsp/` is pure C++17: no ESPHome, IDF or LVGL headers.
  It must build and pass tests on the host (`tests/`, CMake + doctest).
- Real-time acquisition (`acq/`) runs in its own FreeRTOS task pinned to
  core 1. Never do I²S, MCPWM or DSP work in ESPHome `loop()`; hand results
  over through a queue or double buffer.
- All timing is in sample-index units. Tach edges and LRCK edges are latched
  by MCPWM capture hardware. No `micros()` in ISRs for timing (see PLAN §4).
- No bandpass filters in the measurement path. Synchronous averaging does
  the filtering.
- Pins and calibration values come from YAML config, never hard-coded.
- Pin reference: `docs/hardware/pinout.md`. GPIO5 is avoided (LCD reset in
  the CellKeeper YAML).
- Run the simulator and cross-check tests (`tests/sim`, `tools/`) before
  claiming DSP or solver changes work.

## Build
- `esphome compile balancer-jc1060p470.yaml` (once the entry YAML exists)
- Host tests: `cmake -S tests -B build/tests && cmake --build build/tests && ctest --test-dir build/tests`
- `secrets.yaml` is gitignored; copy `secrets.yaml.example`.
