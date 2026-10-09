# P4 Balancer

Two-plane dynamic balancer on the Guition JC1060P470C (ESP32-P4, 7"
1024×600 touchscreen). Two IEPE accelerometers and an optical tach in;
"drill 1.99 g at 29°" out, drawn on rotor dials on the LCD.
Default target: a 1 hp motor rotor on a stand, spun with compressed air.

![Correction screen](docs/ui/07_result.png)

## Status

| Phase | State |
|---|---|
| 0 Board bring-up | config compiles in CI; needs the bench check (LCD reset GPIO5 vs GPIO0) |
| 1 Acquisition (I2S + MCPWM capture) | written; timebase host-tested; needs the PCM1808 on the bench |
| 2 DSP + balancing math | done: 17 host tests, simulator exit test, pyPRB cross-check |
| 3 Touchscreen UI | done: all screens, previewed on PC ([docs/ui](docs/ui)) |
| 4 Real rotor | waiting on sensors + front end |
| 5 Persistence / export | settings, rotor setup and saved machine in flash; HA sensors |
| 6 Front-end PCB | later |

## Flashing

GitHub Actions compiles every push. The newest binaries are attached to the
**latest** pre-release (Releases → *Latest build*):

- `balancer-demo.factory.bin` — **start here.** Simulated air-driven rotor, no
  front end needed. Walk the whole procedure on the screen: AIR BLAST, LOCK
  SPEED, trial runs, correction, verify.
- `balancer.factory.bin` — real sensors (PCM1808 on JP1, tach on GPIO32).
- `bringup.factory.bin` — board only, one label (Phase 0 check).

Flash the `.factory.bin` over USB with [ESPHome Web](https://web.esphome.io)
or `esptool.py write_flash 0x0 <file>`, then use `.ota.bin` files or
`esphome run` for updates. To build locally: copy `secrets.yaml.example` to
`secrets.yaml`, then `esphome run balancer-demo.yaml`.

## How it works
- IEPE sensors → 4 mA current source → AC coupling → ×10 gain → PCM1808
  stereo ΔΣ ADC on I2S1. Both channels are sampled at the same instant.
- MCPWM capture latches tach edges, every 256th LRCK edge and each DMA
  completion on one 80 MHz timer, so every tach pulse is placed to a
  fraction of a sample.
- Each revolution's orders 1×–10× come from a direct DFT against the tach
  angle. Only revolutions inside the speed window count, and coast-down passes
  add up until the 1× phase is settled.
- Influence-coefficient solver: single and two plane, trim runs, add or
  drill, fixed positions, ISO 21940-11 limit, trial-weight helper.

## Repo map
- `PLAN.md` — architecture, decisions, phases, open questions
- `balancer-jc1060p470.yaml`, `balancer-demo.yaml` — device configs
- `hardware_guition_jc1060p470.yaml` — board config (from CellKeeper-BMS-LCD)
- `components/balancer/` — ESPHome component (flat layout, see below)
  - `dsp_*` pure C++: revolution analysis, measurement, solver, job, timebase, demo rotor
  - `acq_capture.*` I2S + MCPWM acquisition task (ESP-IDF)
  - `ui*.{h,cpp}` LVGL screens + shared model; `balancer.*` ESPHome glue
- `tests/` — host tests (`cmake -S tests -B build/tests && ctest --test-dir build/tests`)
- `tools/ui_preview/` — renders every screen on a PC
- `tools/crosscheck_pyprb.py` — solver cross-check against pyPRB
- `docs/hardware/` — pinout (JP1), front end, schematic
