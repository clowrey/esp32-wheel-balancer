# P4 Balancer

Two-plane dynamic balancer for car wheels on the Guition JC1060P470C
(ESP32-P4, 7" 1024×600 touchscreen). Two GY-521 (MPU-6050) accelerometers
on I²C and an optical tach in; "add 12.4 g at 137°" out, drawn on rotor
dials on the LCD. The wheel sits on a soft leaf-spring rig and is spun by
hand. Wiring and rig notes: [docs/hardware/tire-rig.md](docs/hardware/tire-rig.md).

![Correction screen](docs/ui/07_result.png)

## Status

| Phase | State |
|---|---|
| Board bring-up | display, touch and Wi-Fi run on the board |
| DSP + balancing math | done: host tests, simulator exit tests, pyPRB cross-check |
| Touchscreen UI | done: all screens, previewed on PC ([docs/ui](docs/ui)) |
| MPU-6050 acquisition | written and simulated end to end; bench test on the sensor PCB in progress |
| Persistence / export | settings, wheel setup and saved machine in flash; HA sensors |

## Flashing

GitHub Actions compiles every push. The newest binaries are attached to the
**latest** pre-release (Releases → *Latest build*):

- `balancer-tire-demo.factory.bin`: **start here.** Simulated wheel, no
  sensors needed. Walk the whole procedure on the screen: SPIN, LOCK SPEED,
  trial runs, correction, verify.
- `balancer-tire.factory.bin`: two MPU-6050 boards and the tach
  (wiring in [docs/hardware/tire-rig.md](docs/hardware/tire-rig.md)).

Flash the `.factory.bin` over USB with [ESPHome Web](https://web.esphome.io)
or `esptool.py write_flash 0x0 <file>`, then use `.ota.bin` files or
`esphome run` for updates. To build locally: copy `secrets.yaml.example` to
`secrets.yaml`, then `esphome run balancer-tire-demo.yaml`.

## How it works
- Each MPU-6050 raises INT when a reading is ready. One MCPWM capture timer
  latches those edges and the tach edges, so every reading and every tach
  pulse has a hardware timestamp.
- The two sensors run on their own clocks; each one is interpolated onto one
  uniform 250 Hz grid on the capture timer, after taking off its filter delay.
- Each revolution's orders 1×–10× come from a direct DFT against the tach
  angle. Only revolutions inside the speed window count, and coast-down
  passes add up until the 1× phase is settled.
- Influence-coefficient solver: single and two plane, trim runs, add or
  drill, fixed positions, ISO 21940-11 limit, trial-weight helper.

## Repo map
- `PLAN.md`: architecture, decisions, phases, open questions
- `balancer-tire.yaml`: device config, two MPU-6050 boards and the tach
- `balancer-tire-demo.yaml`: the same firmware with a simulated wheel
- `balancer-common.yaml`: board, network and display
- `hardware_guition_jc1060p470.yaml`: board config (from CellKeeper-BMS-LCD)
- `components/balancer/`: ESPHome component (flat layout)
  - `dsp_*` pure C++: revolution analysis, resampler, measurement, solver, job, demo wheel
  - `acq_mpu.*` MPU-6050 acquisition task: I²C reads timed by MCPWM capture
  - `ui*.{h,cpp}` LVGL screens and shared model; `balancer.*` ESPHome glue
- `tests/`: host tests (`cmake -S tests -B build/tests && ctest --test-dir build/tests`)
- `tools/ui_preview/`: renders every screen on a PC
- `tools/crosscheck_pyprb.py`: solver cross-check against pyPRB
- `docs/hardware/`: pinout (JP1), tire rig, schematic

## License
The firmware, tools and docs in this repo are MIT licensed (see `LICENSE`).
Third-party files keep their own terms:
- `tests/third_party/doctest.h`: doctest, MIT licensed.
- `docs/hardware/schematic/JC1060P470C_I_W_Y-V1.0.pdf`: Guition's schematic
  for the JC1060P470C board, included for reference. It belongs to Guition and
  is not covered by this repo's license.
