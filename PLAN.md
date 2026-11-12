# P4 Balancer — Project Plan

Two-plane dynamic balancer for car wheels on the **Guition JC1060P470C**
(ESP32-P4, 7" 1024×600 MIPI-DSI, GT911 touch, ESP32-C6 for Wi-Fi). Two
GY-521 (MPU-6050) accelerometers and an optical tach go into the board; the
LCD walks the operator through the runs and shows **how many grams to add,
and where**, on each rim edge.

---

## 0. Decisions

| Question | Decision | What it changes |
|---|---|---|
| Rotor | **Car wheel** on a soft leaf-spring rig | Setup defaults (§6.1), displacement units, wide speed window |
| Sensors | **Two GY-521 (MPU-6050) boards** on I²C, one INT line each | Acquisition in `acq_mpu`, resampling in `dsp_resample` (§4) |
| Tach | LX-101 optical sensor (or Chester's own), a pulse on a GPIO | GPIO32 input with configurable edge, MCPWM capture |
| Framework | **ESPHome** + external C++ component | As in §2 |
| Drive | **None.** The wheel is spun by hand and coasts | Speed is not regulated, so measurement uses a speed window and coast-down capture (§5.1) |

2026-11-12: the repo was cut down to this accelerometer firmware only. The
earlier IEPE + PCM1808 path for a 1 hp motor rotor was removed; it is in the
git history before that date if it is ever wanted again.

---

## 1. Goals and non-goals

**Goals**
- Single-plane and two-plane balancing with the influence-coefficient method:
  run 0 (as found), trial weight in plane 1, trial weight in plane 2, solve.
- Entire workflow on the touchscreen. No PC needed.
- Correction shown as mass (g) and angle on a rotor dial for each plane,
  measured from the tach mark in a configurable direction, with optional
  splitting onto fixed positions (spokes, holes).
- Trim runs that reuse stored influence coefficients (no new trial weights).
- Readings you can trust: hardware-timestamped samples, per-reading scatter,
  warnings when a trial weight's effect is too small or the coefficient
  matrix is ill-conditioned.
- Live vibration view: RPM, 1× amplitude and phase per channel, polar plots,
  order spectrum (1×–10×).
- ISO 21940-11 balance-grade helper (permissible residual unbalance).
- Save machines (influence coefficients, settings) to flash.
- Home Assistant / web export for free via ESPHome.

**Non-goals for v1**
- More than 2 planes or more than 2 sensors (the math is written for M×N
  least-squares so this can come later).
- Full vibration-analyzer features (envelope, bearing fault frequencies).
- Driving the wheel.

---

## 2. Framework decision

**ESPHome + a custom external component**, the same pattern as
CellKeeper-BMS-LCD.

- `hardware_guition_jc1060p470.yaml` is lifted from CellKeeper, so display,
  touch, backlight, PSRAM, LDOs and the C6 link are already known-good.
- The real-time parts (sensor reads, MCPWM capture, resampling, revolution
  analysis) live in C++ in `components/balancer`, on their own FreeRTOS task
  pinned to core 1, using the IDF drivers directly. ESPHome's `loop()` only
  consumes finished revolutions.
- The UI is C++ against LVGL 9 inside the component (dials, polar plots and
  the wizard were awkward in ESPHome LVGL YAML).
- OTA, web server and Home Assistant come for free.

---

## 3. Hardware

### 3.1 Signal chain

```
GY-521 A (0x68) ──┐  SDA GPIO1, SCL GPIO2 (own I²C port, read by the acquisition task)
GY-521 B (0x69) ──┘
INT A ──► GPIO3 ──► MCPWM capture ch1 (hardware timestamp of data ready)
INT B ──► GPIO4 ──► MCPWM capture ch2
Tach  ──► GPIO32 ─► MCPWM capture ch0
```

For slow, heavy rotors on a soft rig the signal is large (the hub moves
about U/M, hundreds of µm for a typical wheel), so a MEMS accelerometer at
400 µg/√Hz is enough. The rig must be soft: sway and twist resonances at a
third of the spin speed or lower (tap test). Near resonance the phase moves
with speed and nothing downstream can fix that.

### 3.2 Pin map (JP1 header)

JP1 exposes GPIO1, 2, 3, 4, 5, 20, 32, 33, 45, 46, 47, I²C bus_a (GPIO7/8),
3V3, VOUT-BAT and GND. **GPIO5 is used as the LCD reset in the CellKeeper
YAML** (the schematic shows LCD_RST on GPIO0), so it is avoided here.

| Function | GPIO | JP1 pin |
|---|---|---|
| Sensor SDA | 1 | 7 |
| Sensor SCL | 2 | 9 |
| INT, sensor A | 3 | 11 |
| INT, sensor B | 4 | 13 |
| Tach input | 32 | 19 |
| Spare (second sensor bus if needed) | 20, 33 | 17, 21 |
| Spare | 45, 46, 47 | 14, 12, 10 |

All pins are set in YAML (`balancer:` config), so they can be moved without
code changes. Full wiring: `docs/hardware/pinout.md` and
`docs/hardware/tire-rig.md`.

### 3.3 Rig

A leaf-spring stand holding a spindle in two bearings, one sensor on each
bearing housing, axis along the sway direction. Details, tap test and
safety notes in `docs/hardware/tire-rig.md`.

---

## 4. Timing architecture (the core of the design)

Every time is a hardware-latched capture value.

- Each sensor's data-ready edge and the tach are latched by one MCPWM capture
  timer. The acquisition task (`acq_mpu`, core 1) reads a sensor after its
  edge and discards the reading if another edge arrived before the read
  finished, so a reading is never paired with the wrong timestamp.
- The sensors run on their own oscillators at slightly different rates.
  `dsp_resample` interpolates each one onto a uniform grid defined by the
  capture timer (250 Hz by default). Tach edges land on that grid by
  division. Missing readings are bridged; a long dropout restarts the grid.
- Each sensor's low-pass group delay (4.9 ms at 44 Hz) is taken off its
  readings' timestamps before resampling, so the displayed phase is close to
  true even if the two boards carry different chips.
- A sensor that goes silent is re-initialised by the task; the bus is
  clocked free at start in case a reboot cut a transfer short.
- The sensor bus is not ESPHome's: it is a second I²C port owned by the
  acquisition task, so nothing in `loop()` touches it.

---

## 5. Signal processing

Runs in `components/balancer/dsp_*` as plain C++ with no ESPHome or IDF
dependencies, so it can be unit-tested on a PC.

1. **Revolution segmentation:** consecutive tach events define each
   revolution in fractional grid indices. Glitches (double counts) are
   rejected and a long gap restarts.
2. **Order extraction:** a direct DFT of each revolution against the tach
   angle gives the 1× vector (amplitude, phase relative to the tach mark)
   plus 2×–10× for the spectrum view. No bandpass: synchronous averaging
   does the filtering.
3. **Units:** acceleration in g; velocity in mm/s; displacement in µm (the
   default for a soft rig). Phase convention is configurable.
4. **Quality metrics:** the 1× vector's mean and its uncertainty in
   amplitude and phase over the accepted revolutions, plus speed mean and
   spread.
5. **Measurement states:** waiting for speed → collecting → ready. A reading
   is ready when enough revolutions were inside the window and the phase
   uncertainty is below the threshold (or after four times the minimum count
   for a reading too small for its phase to settle).

### 5.1 Hand spin: speed-window and coast-down capture

There is no speed control, so a reading can't wait for a steady speed.
Instead the acquisition works per revolution:

- Every revolution's 1× vector is computed on its own. Only revolutions
  whose speed is inside the **target window** (default ±10%) go into the
  average. Others are dropped.
- **Coast-down capture:** spin the wheel above the window and let it coast
  through. If one pass doesn't give enough revolutions, spin again; passes
  add up until the target count is reached and the phase is settled.
- **Lock speed:** run 0 sets the window (LOCK SPEED, or enter an RPM). Every
  later run in the job uses the same window, because influence coefficients
  are only valid at one speed.
- Revolutions where the speed changes too much from the previous one
  (default 3%) are rejected.

---

## 6. Balancing math (`dsp_balance.*`, `dsp_job.*`)

- Complex vectors throughout; rotor angles in degrees.
- **Single plane:** α = (V1 − V0)/T, W = −V0/α.
- **Two plane:** A(i,j) = (Vij − V0i)/Tj, then W = −A⁻¹·V0 for the 2×2 case.
  Written as general M×N least squares so more sensors or planes slot in.
- **Trial-weight handling:** leave trial weights on (correction is computed
  relative to them) or remove them.
- **Split** a correction onto N fixed positions.
- **Radius conversion:** if the correction radius differs from the trial
  radius, scale by r_trial/r_corr.
- **Add or remove:** each plane can be set to add mass (default for wheels)
  or remove mass (drilling).
- **Checks:** warn if a trial weight changes the reading too little, or if
  cond(A) is high. Report predicted residual.
- **ISO 21940-11:** U_per [g·mm] = 9549 · G · m [kg] / n [rpm], split
  between planes. Shown against the residual predicted from the influence
  coefficients.

Validated against `pyPRB` (`tools/crosscheck_pyprb.py`).

### 6.1 Defaults for a car wheel

Set in `ModelCore::load_defaults()` (`ui_core.cpp`); all editable on the
screen except the speed-change limit (`max_speed_change:` in YAML).

| Setting | Default | Note |
|---|---|---|
| Rotor mass | 20 kg | |
| Service speed, grade | 800 rpm, G6.3 | about 100 km/h; 3.8 g per rim edge at 200 mm |
| Trial weight | 30 g at 200 mm | |
| Correction | add weight, 200 mm | stick-on or clip-on weights |
| Units | µm | steady through a coast-down on a soft rig |
| Speed window | ±10% | only a handful of revolutions fit in ±3% at 4 rev/s |
| Speed change per revolution | 3% | a coasting wheel slows about 1% per revolution |
| Revolutions per run | 20, ready regardless after 80 | |

**Trial-weight helper:** suggests a trial mass whose force at the test speed
is about 10% of the rotor's weight, capped at a multiple of the plane's ISO
limit (the uncapped rule asks for far too much at a few hundred rpm).

---

## 7. UI (LVGL 9, 1024×600 landscape)

Status bar on every screen: RPM, sensor levels, tach status.

1. **Home:** New balance, Trim run (with a saved machine), Live vibration,
   Settings.
2. **Setup:** 1 or 2 planes; trial mass and radius per plane; correction
   radius; angle convention; fixed positions; add/remove mode per plane; ISO
   grade, rotor mass and service speed; trial-weight helper.
3. **Measure run (reused for each run):** instructions, speed with the
   target window, revolutions collected, LOCK SPEED, two polar plots with
   recent readings, amplitude and phase per channel, ACCEPT when ready.
4. **Result:** two rotor dials (plane 1, plane 2) with the correction angle
   and mass, split positions if configured, predicted residual vs ISO limit.
   Verify, Save machine, Trim.
5. **Live vibration:** amplitude, phase, RPM, 1×–10× orders, averaged
   revolution waveform, acquisition diagnostics.
6. **Settings:** sensor polarity, units, speed window, revolutions per run,
   phase threshold.

Screens rendered on a PC by `tools/ui_preview` are in `docs/ui/`.

---

## 8. Repository layout

ESPHome only copies top-level files of an external component, so the
component is flat, with prefixes instead of subdirectories.

```
esp32-wheel-balancer/
├── balancer-tire.yaml                  # device: two MPU-6050 boards and the tach
├── balancer-tire-demo.yaml             # device: same firmware, simulated wheel
├── balancer-common.yaml                # board, network, display
├── hardware_guition_jc1060p470.yaml    # board config from CellKeeper
├── components/balancer/
│   ├── __init__.py                     # YAML schema, codegen, LVGL widget/font registration
│   ├── balancer.{h,cpp}                # ESPHome glue: acquisition, NVS, sensors
│   ├── acq_mpu.{h,cpp}                 # ESP-IDF: MPU-6050 on I2C, timed by MCPWM capture
│   ├── dsp_types.h                     # conventions (lag-complex vectors, g*mm weights)
│   ├── dsp_resample.{h,cpp}            # sensors -> uniform grid on the capture timer
│   ├── dsp_revolution.{h,cpp}          # per-revolution DFT of orders 1-10
│   ├── dsp_measurement.{h,cpp}         # speed window / coast-down accumulator
│   ├── dsp_balance.{h,cpp}             # influence coefficients, split, drill, ISO
│   ├── dsp_job.{h,cpp}                 # wizard state and arithmetic
│   ├── dsp_sim.{h,cpp}                 # revolution-level demo wheel
│   ├── ui_model.h, ui_core.{h,cpp}     # what the UI needs + shared implementation
│   └── ui.{h,cpp}                      # LVGL 9 screens, dials, polar plots
├── tests/                              # doctest host suite + rotor and sensor simulators
├── tools/                              # balance_cli, pyPRB cross-check, ui_preview
├── ci/                                 # CI secrets stub
└── docs/  (hardware/, ui/ screenshots, research/)
```

---

## 9. Phases

### Phase 0 — Board bring-up  — DONE
- Board config from CellKeeper; display, touch and Wi-Fi run on the board.

### Phase 1 — DSP and math, host-first  — DONE
- Revolution analysis, measurement, single- and two-plane solvers, split,
  ISO, with doctest unit tests and a sample-level rotor simulator.
- Cross-check against pyPRB: agreement 2e-8 over 300 random cases.

### Phase 2 — UI  — DONE (screens in docs/ui; demo firmware)
- Wizard state machine in C++ (`dsp_job`), UI in C++ against LVGL 9.

### Phase 3 — MPU-6050 acquisition  — written; simulated end to end; bench test in progress
- Host test: two sensors on their own clocks with noise, lost readings and
  hand-spin coast-downs, through the resampler and `ModelCore`. 35 g and
  25 g on the two rim edges come back within 2% and 2°, and the verify run
  is inside the limit.
- On the bench: confirm both sensors report about 250 Hz on the LIVE screen,
  tap-test the rig, then balance a wheel with a known added weight.

### Phase 4 — Persistence and export  — settings, setup and machine in NVS; HA sensors done; CSV export open

---

## 10. Risks and open questions

| Risk | Mitigation |
|---|---|
| Rig resonance too close to the spin speed | Tap test before measuring; lengthen the spring (§3.1) |
| MPU-6050 readings mis-timed when the task is late | Readings that cannot be tied to one edge are dropped and bridged; count shown on the LIVE screen |
| Long I²C leads to the sensors on a moving rig | Short leads with a ground wire; 100 kHz option; a second bus on GPIO20/33 if needed |
| Too few revolutions in the window on one coast-down | Passes accumulate; the window and revolution count are adjustable |
| Board revision differences (I²C addresses differ between batches) | Scan on boot and show on the debug page |
| ESPHome loop task stack is only 8 KB | Keep large buffers on the heap (the demo's revolution buffer is) |
