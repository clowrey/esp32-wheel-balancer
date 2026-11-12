# P4 Balancer — Project Plan

Two-plane field/shop dynamic balancer on the **Guition JC1060P470C** (ESP32-P4,
7" 1024×600 MIPI-DSI, GT911 touch, ESP32-C6 for Wi-Fi). Two IEPE
accelerometers and an optical tach go into the board; the LCD walks the
operator through the runs and shows **how many grams to add, and where**, on
each correction plane.

It replaces the BMI160 lock-in sketch in `docs/legacy/original_lockin.ino`.
That sketch's problems (unsynchronized I²C sampling, a mistuned bandpass whose
phase had to be calibrated out, ISR races, single-rev RPM gating) are what
this design is built to avoid.

---

## 0. Decisions (2026-10-08)

| Question | Decision | What it changes |
|---|---|---|
| Default rotor | **1 hp induction-motor rotor** (56/143T frame class), balanced on a stand | Setup defaults, ISO helper, trial-weight helper (§6.1) |
| Sensors | **Buy 100 mV/g industrial IEPE** (PCB 603C01 class) | Front-end gain ×10, ±1.5 g full scale (§3.1) |
| Tach | **Chester builds the laser/optical sensor**; firmware only needs a 3.3 V logic pulse on a GPIO | No Schmitt/opto on our side; GPIO32 input with configurable edge and glitch filter |
| Framework | **ESPHome** + external C++ component | As in §2 |
| Rotor drive | **None. Rotor is spun with compressed air.** No motor drive, no valve output | Speed is not regulated, so measurement uses a speed window and coast-down capture (§5.1) |

**Added 2026-11-11: tire variant.** A second, quick-to-build sensor path for
car wheels, using parts on hand: two GY-521 (MPU-6050) boards on I²C, a
leaf-spring rig, the wheel spun by hand. Same board, UI, analysis and solver;
selected in YAML with `sensor_type: mpu6050` and `preset: tire`. The IEPE +
PCM1808 design stays the main path for stiff stands and fast rotors. Details
in §3.3 and `docs/hardware/tire-rig.md`.

---

## 1. Goals and non-goals

**Goals**
- Single-plane and two-plane balancing with the influence-coefficient method:
  run 0 (as found), trial weight in plane 1, trial weight in plane 2, solve.
- Entire workflow on the touchscreen. No PC needed.
- Correction shown as mass (g) and angle on a rotor dial for each plane,
  measured from the tach mark in a configurable direction, with optional
  splitting onto fixed positions (blades, holes, bolt circle).
- Trim runs that reuse stored influence coefficients (no new trial weights).
- Readings you can trust: phase-locked sampling, per-reading scatter, warnings
  when a trial weight's effect is too small or the coefficient matrix is
  ill-conditioned.
- Live vibration view: RPM, 1× amplitude (mm/s and g) and phase per channel,
  polar plots, order spectrum (1×–10×).
- ISO 21940-11 balance-grade helper (permissible residual unbalance).
- Save machines and jobs (influence coefficients, settings) to flash/SD.
- Home Assistant / web export for free via ESPHome (CSV of runs).

**Non-goals for v1**
- More than 2 planes or more than 2 sensors (the math is written for M×N
  least-squares so this can come later).
- Full vibration-analyzer features (envelope, bearing fault frequencies).
- Driving the rotor. It's spun with compressed air, so there's no motor drive
  or air-valve output in v1.

---

## 2. Framework decision

**Recommended: ESPHome + a custom external component**, the same pattern as
CellKeeper-BMS-LCD.

- `hardware_guition_jc1060p470.yaml` is lifted from CellKeeper, so display,
  touch, backlight, PSRAM, LDOs and the C6 link are already known-good.
- UI is ESPHome LVGL YAML, like `ui_1024x600.yaml` in CellKeeper.
- The real-time parts (I²S DMA, MCPWM capture, synchronous averaging) live in
  C++ in `components/balancer`, on their own FreeRTOS task pinned to core 1,
  using the IDF drivers directly. ESPHome's `loop()` only consumes finished
  results.
- The rotor dials (custom drawing) are done in C++ against the LVGL objects
  ESPHome creates, or with LVGL `meter` needles if that's enough.
- OTA, web server and Home Assistant come for free.

**Alternative:** plain ESP-IDF + `esp_lvgl_port` + `esp_lcd_jd9165` + GT911.
More control over LVGL and drawing, but the board bring-up has to be redone.
Only worth it if ESPHome's LVGL becomes a blocker for the dial or wizard UI.

> Decision gate at the end of Phase 1: if the rotor dial and wizard are
> awkward in ESPHome LVGL, switch the UI layer to C++ inside the component
> (still ESPHome) before considering plain IDF.

---

## 3. Hardware

### 3.1 Signal chain (per channel ×2)

```
IEPE accel ──► 4 mA CCS from +24 V ──► AC couple (≤0.1 Hz) ──► gain ×10 ──► RC AA ────┐
 (100 mV/g, PCB 603C01 class)            LT3092 or CRD                                   │
                                                                                          ▼
                                                              PCM1808 (stereo 24-bit ΔΣ, I²S)
                                                                     │  MCLK/BCK/LRCK from P4
                                                                     ▼
                                                          ESP32-P4 I2S1 (master) + DMA
Tach (Chester's laser/optical, 3.3 V pulse) ──► GPIO32 ──► MCPWM capture (hardware timestamp)
LRCK (loopback) ─────────────────────────► MCPWM capture ch2, prescale 256
IEPE bias (DC, via divider) ─► ADS1115 on I²C bus_a ─► open/short sensor detection
```

Details, BOM and the 24 V supply are in `docs/hardware/front-end.md`.

### 3.2 Pin map (JP1 header, from the JC1060P470C schematic)

JP1 exposes GPIO1, 2, 3, 4, 5, 20, 32, 33, 45, 46, 47, I²C bus_a (GPIO7/8),
3V3, VOUT-BAT and GND. **GPIO5 is used as the LCD reset in the CellKeeper
YAML** (the schematic shows LCD_RST on GPIO0), so it is avoided here.

| Function | GPIO | JP1 pin | Notes |
|---|---|---|---|
| PCM1808 SCKI (MCLK) | 1 | 7 | 256·fs, series 33 Ω |
| PCM1808 BCK | 2 | 9 | P4 is I²S master |
| PCM1808 LRCK | 3 | 11 | also looped into MCPWM capture |
| PCM1808 DOUT → P4 DIN | 4 | 13 | |
| Tach input | 32 | 19 | MCPWM capture ch0. 3.3 V logic pulse from the user-built sensor; edge and minimum period set in YAML |
| LRCK capture (jumper) | 33 | 21 | fallback if internal loopback of GPIO3 doesn't work |
| ADS1115 SDA / SCL | 7 / 8 | 23 / 25 | shared bus_a; 0x48 is free (GT911, ES8311, ES7210, RTC are elsewhere) |
| Spare: run/stop button | 45 | 14 | |
| Spare | 46 | 12 | |
| Spare | 20, 47 | 17, 10 | |

All pins are set in YAML (`balancer:` config), so they can be moved without
code changes. Onboard ES8311/ES7210 stay on I2S0 (GPIO9–13, 48) and are
untouched; the ES8311 + speaker (amp enable GPIO11) can be used for beeps.

**To verify on the bench before PCB work:** GPIO5 really is free or really
is the LCD reset on this board revision; GPIO3 can be an I²S output and an
MCPWM capture input at the same time through the GPIO matrix.

### 3.3 Tire variant: MPU-6050 on I²C

For slow, heavy rotors on a soft rig the signal is large (the hub moves about
U/M, hundreds of µm for a typical wheel), so a MEMS accelerometer at
400 µg/√Hz is enough. Two GY-521 boards share one I²C bus on JP1 (SDA GPIO1,
SCL GPIO2), one INT line each (GPIO3, GPIO4), tach on GPIO32 as before.

The rule from §4 still holds: every time is a hardware-latched capture value.

- Each sensor's data-ready edge and the tach are latched by one MCPWM capture
  timer. The acquisition task (`acq_mpu`, core 1) reads a sensor after its
  edge and discards the reading if another edge arrived before the read
  finished, so a reading is never paired with the wrong timestamp.
- The sensors run on their own oscillators at slightly different rates.
  `dsp_resample` interpolates each one onto a uniform grid defined by the
  capture timer (250 Hz by default). Tach edges land on that grid by
  division. Missing readings are bridged; a long dropout restarts the grid.
- The sensor's low-pass group delay (4.9 ms at 44 Hz) is removed like the
  PCM1808's delay.
- The sensor bus is not ESPHome's: it is a second I²C port owned by the
  acquisition task, so nothing in `loop()` touches it.

The `tire` preset (`ui_core.cpp`) sets a 20 kg wheel, G6.3 at 800 rpm, 30 g
trial weights at 200 mm, add-weight corrections, displacement units, a ±10%
window, up to 3% speed change per revolution and best-effort readiness after
four times the minimum revolutions. Presets keep separate saved state.

The rig must be soft: sway and twist resonances at a third of the spin speed
or lower (tap test). Near resonance the phase moves with speed and nothing
downstream can fix that.

---

## 4. Timing architecture (the core of the design)

Everything is expressed in **sample-index time**, so tach and vibration can
never drift relative to each other.

1. I2S1 runs as master at fs = 16 kHz (MCLK 4.096 MHz). DMA delivers frames
   continuously; the task keeps a 64-bit running sample counter `n`.
2. MCPWM capture ch2 timestamps every 256th LRCK rising edge (capture
   prescale 256), so there are about 62 interrupts/s. Capture value `c_k`
   corresponds to frame index `k·256 + n_off`, where `n_off` is a constant
   that cancels out of the balancing math.
3. MCPWM capture ch0 timestamps every tach edge, `c_tach`. It is converted to
   a fractional sample index by linear interpolation between the bracketing
   LRCK captures. Resolution is about 1/(capture clock), so tens of
   nanoseconds — about 0.001° at 5000 RPM.
4. Result: a queue of tach events in fractional sample units. ISR latency
   doesn't matter because both edges are latched in hardware.

**Fallback (Option B)** if the capture approach misbehaves: use a 4-channel
TDM ADC (PCM1865 or similar) and digitize the tach signal as a third
channel. Alignment is then perfect by construction, and the tach edge is
found by interpolated threshold crossing.

Fixed delays (PCM1808 digital filter ~17.4/fs, AC coupling, sensor
high-pass) are identical on both channels. They are compensated from
datasheet values so the displayed phase is close to true, but the balancing
result doesn't depend on that compensation.

---

## 5. Signal processing

Runs in `components/balancer/dsp/` as plain C++ with no ESPHome or IDF
dependencies, so it can be unit-tested on a PC.

1. **Pre-processing:** convert counts to g (per-channel sensitivity, e.g.
   10.670 mV/g, × front-end gain, ÷ ADC full scale), then a DC blocker.
   No bandpass.
2. **Revolution segmentation:** consecutive tach events define each
   revolution in fractional sample indices. Reject a revolution if its
   period differs from the running median by more than X% (glitches, double
   counts).
3. **Time-synchronous averaging (TSA):** resample each revolution into 128
   angle bins (linear or cubic interpolation) and accumulate over K
   revolutions.
4. **Order extraction:** DFT of the averaged revolution gives the 1× vector
   (amplitude, phase relative to the tach mark) plus 2×–10× for the
   spectrum view and diagnostics.
5. **Units:** acceleration in g; velocity in mm/s = a/(2πf) at each order.
   Phase convention is configurable (lag/lead, with or against rotation).
6. **Quality metrics:** split the window into sub-blocks and report the
   1× vector's mean, amplitude σ and phase σ. Also RPM mean and spread over
   the window, which replaces the single-rev RPM gate.
7. **Measurement states:** waiting for speed → settling → averaging →
   stable/accepted. A reading is accepted when RPM stays inside a band
   around the target and phase σ is below a threshold.

### 5.1 Air-driven rotor: speed-window and coast-down capture

With compressed air there's no speed control, so a reading can't wait for a
steady speed. Instead the acquisition works per revolution:

- Every revolution is resampled and its 1× vector computed on its own.
  Only revolutions whose speed is inside the **target window**
  (default ±3%) go into the average. Others are dropped.
- **Coast-down capture (default mode):** spin the rotor above the window,
  cut the air, and let it coast through the window. Revolutions in the
  window are accumulated. If one pass doesn't give enough revolutions,
  spin up and coast again; passes add up until the target count is reached
  (default 60 revolutions) and phase σ is below threshold. Coasting also
  removes the air jet's forces from the measurement.
- **Steady mode:** hold the air steady and accumulate in the window, for
  when the rotor holds speed well enough.
- **Lock speed:** run 0 sets the window ("Use current speed" button, or
  enter an RPM). Every later run in the job must use the same window,
  because influence coefficients are only valid at one speed.
- **Phase-vs-speed check:** within the window, fit the 1× phase against RPM.
  A steep slope means the stand has a resonance near the test speed, so the
  UI warns and suggests a different speed.
- Revolutions where speed changes too fast within one turn (deceleration
  above a set limit) are rejected.

---

## 6. Balancing math (`dsp/balance.*`)

- Complex vectors throughout (`std::complex<float>`); rotor angles in degrees.
- **Single plane:** α = (V1 − V0)/T, W = −V0/α.
- **Two plane:** A(i,j) = (Vij − V0i)/Tj, then W = −A⁻¹·V0 for the 2×2 case.
  Written as general M×N least squares so more sensors or planes slot in.
- **Trial-weight handling:** option to leave trial weights on (correction is
  computed relative to them) or remove them.
- **Split** a correction onto N fixed positions: the two adjacent positions
  that bracket the angle.
- **Combine** an existing correction weight with a new one (trim).
- **Radius conversion:** if the correction radius differs from the trial
  radius, scale by r_trial/r_corr.
- **Add or remove:** each plane can be set to *add mass* (weights or
  balance lugs) or *remove mass* (drilling end rings, typical for motor
  rotors). Remove mode shows the angle +180° and the mass to drill out,
  with an optional drill-depth helper (drill diameter, material density).
- **Checks:** warn if a trial weight changes amplitude by less than 30% or
  phase by less than 30° (rule of thumb). Warn if cond(A) is high.
  Report predicted residual vibration.
- **ISO 21940-11:** U_per [g·mm] = 9549 · G · m [kg] / n [rpm], split
  between planes by bearing load ratio. Display it against the residual
  predicted from the influence coefficients.

Validate against `pyPRB` and `hsbalance` with the same inputs (see Phase 2).

### 6.1 Defaults for a 1 hp motor rotor

All of these are editable in setup; they are starting points, not
measurements of Chester's rotor.

| Setting | Default | Note |
|---|---|---|
| Rotor mass | 3.5 kg | Weigh the actual rotor |
| Service speed (for ISO) | 1750 RPM | 4-pole 60 Hz; 3450 for 2-pole |
| Balance grade | G2.5 | ISO 21940-11, motors ≥80 mm shaft height above 950 RPM (143T ≈ 89 mm) |
| Test speed window | locked at run 0, ±3% | Air-driven; aim well clear of stand resonance |
| Planes | 2 (both end rings) | Single-plane option for narrow rotors |
| Correction radius | 40 mm | End-ring or lug radius; measure it |
| Correction mode | Remove (drill) | Switchable per plane |

With those defaults the ISO limit works out to U_per = 9549·2.5·3.5/1750
≈ 48 g·mm in total, about 24 g·mm per plane, which is about 0.6 g at 40 mm.
The ISO limit uses the **service** speed, not the air-driven test speed;
for a rigid rotor the correction is still valid at service speed.

**Trial-weight helper:** suggests a trial mass whose force at the test speed
is about 5–10% of the rotor's weight:
m_t·r = k·M·g/ω². For example, at 1500 RPM, 3.5 kg, k = 0.1 and r = 40 mm,
that's about 3.5 g. The helper also checks the trial run's effect against
the 30%/30° rule and asks for a bigger trial weight if the effect is too
small.

---

## 7. UI (LVGL, 1024×600 landscape)

Status bar on every screen: RPM, stability dot (red/amber/green), sensor
status (OK / open / short from IEPE bias), job name.

1. **Home:** big buttons — New balance, Trim run, Live vibration, Machines,
   Settings.
2. **Rotor setup (wizard step 0):** 1 or 2 planes; trial mass and radius per
   plane; correction radius; angle convention (with an illustration of the
   tach mark and rotation direction); fixed positions (none / N holes /
   N blades); add/remove mode per plane; ISO grade, rotor mass and service
   speed (defaults from §6.1); trial-weight helper. The test speed window
   is set at run 0.
3. **Measure run screen (reused for each run):**
   - Instructions at top ("Run 1: add 3.5 g at 0° on Plane 1, spin up with
     air, then let it coast").
   - Speed gauge with the target window highlighted, a revolutions-collected
     counter, and the coast/steady mode toggle.
   - Two polar plots (A and B) showing the live 1× vector with a fading
     trail of recent readings.
   - Amplitude (mm/s) and phase per channel, σ shown.
   - Accept button, enabled only when stable. Re-measure button.
4. **Result screen (the payoff):**
   - Two large rotor dials side by side (Plane 1, Plane 2), each showing:
     tach mark at 0°, rotation arrow, angle ticks, a weight marker and arrow
     at the correction angle, and the fixed positions if configured.
   - Big text: "ADD 12.4 g @ 137°", or for split mode
     "8.1 g @ hole 3 + 5.2 g @ hole 4".
   - Whether to remove or keep the trial weights.
   - Predicted residual vs ISO limit.
   - Buttons: Verify run, Save, Trim.
5. **Verify / trim:** measure after correction, show residual, compute trim
   from stored coefficients, repeat until within limit.
6. **Live vibration:** both channels — amplitude, phase, RPM trend, 1×–10×
   order bar chart, time waveform (averaged revolution).
7. **Machines:** saved rotor configurations and influence coefficients.
   Selecting one enables single-run trim balancing.
8. **Settings:** sensor sensitivity per channel, front-end gain, sample rate,
   averaging time / revolutions, stability thresholds, units (mm/s vs in/s,
   g vs m/s²), phase convention, backlight, Wi-Fi.

Wireframes go in `docs/ui/` (Phase 3).

---

## 8. Repository layout

ESPHome only copies top-level files of an external component, so the
component is flat, with prefixes instead of subdirectories.

```
p4-balancer/
├── balancer-jc1060p470.yaml            # device: real sensors
├── balancer-demo.yaml                  # device: simulated rotor (no front end)
├── balancer-tire.yaml                  # device: tire balancer, two MPU-6050 boards
├── balancer-tire-demo.yaml             # device: tire balancer, simulated wheel
├── balancer-common.yaml                # board, network, display: shared by all of the above
├── hardware_guition_jc1060p470.yaml    # board config from CellKeeper
├── components/balancer/
│   ├── __init__.py                     # YAML schema, codegen, LVGL widget/font registration
│   ├── balancer.{h,cpp}                # ESPHome glue: acquisition, NVS, sensors
│   ├── acq_capture.{h,cpp}             # ESP-IDF: I2S1 + MCPWM capture task
│   ├── acq_mpu.{h,cpp}                 # ESP-IDF: MPU-6050 on I2C, timed by MCPWM capture
│   ├── dsp_types.h                     # conventions (lag-complex vectors, g*mm weights)
│   ├── dsp_revolution.{h,cpp}          # per-revolution DFT of orders 1-10
│   ├── dsp_measurement.{h,cpp}         # speed window / coast-down accumulator
│   ├── dsp_balance.{h,cpp}             # influence coefficients, split, drill, ISO
│   ├── dsp_job.{h,cpp}                 # wizard state and arithmetic
│   ├── dsp_timebase.{h,cpp}            # tach ticks -> fractional sample index
│   ├── dsp_resample.{h,cpp}            # digital sensors -> uniform grid on the capture timer
│   ├── dsp_sim.{h,cpp}                 # revolution-level demo rotor
│   ├── ui_model.h, ui_core.{h,cpp}     # what the UI needs + shared implementation
│   └── ui.{h,cpp}                      # LVGL 9 screens, dials, polar plots
├── tests/                              # doctest host suite + sample-level rotor simulator
├── tools/                              # balance_cli, pyPRB cross-check, ui_preview
├── ci/                                 # CI secrets stub, bring-up config
└── docs/  (hardware/, ui/ screenshots, legacy/, research/)
```

---

## 9. Phases

### Phase 0 — Board bring-up (½ day)  — config builds in CI; bench check pending
- Copy the hardware YAML from CellKeeper and build a "hello" page.
- Confirm GPIO5's role on this revision.
- Confirm JP1 pin access and the 3V3 / VOUT-BAT levels.

### Phase 1 — Acquisition (2–3 days)  — written, timebase host-tested; bench test pending
- `i2s_adc`: I2S1 master, 16 kHz, 24-bit (32-bit slots), DMA, stereo.
  Start with a PCM1808 breakout board and a signal generator.
- `tach_capture`: MCPWM capture for tach and LRCK (prescale 256), fractional
  sample-index conversion, glitch rejection.
- Debug page: live waveforms, RPM, sample-rate check (counted LRCK vs
  expected).
- **Exit test:** feed the same sine to both channels plus a square wave
  derived from the same generator as the tach. Phase between channels
  < 0.1°, phase vs tach stable < 0.2° over 10 min.

### Phase 2 — DSP and math, host-first (2–3 days, parallel with Phase 1)  — DONE
- `dsp/` with doctest unit tests: TSA, order extraction, single- and
  two-plane solvers, split and combine, ISO calculation.
- Python simulator: rotor with known unbalance plus noise and speed jitter
  → synthetic I²S stream and tach events → firmware DSP → compare with the
  injected unbalance.
- Cross-check solver outputs against pyPRB and hsbalance.
- **Exit test:** simulator recovers injected unbalance within 2% / 1° at
  SNR 20 dB. **Result:** worst of 20 seeds 1.33% / 0.69° with three
  coast-downs per run; pyPRB agreement 2e-8 over 300 random cases.

### Phase 3 — UI (3–4 days)  — DONE (screens in docs/ui; demo firmware)
- Wireframes, then LVGL pages: home, setup, measure, result dial, live,
  settings.
- Wizard state machine in C++ (`dsp_job`), UI in C++ against LVGL 9 (the
  decision gate in §2 was taken: C++ UI inside the ESPHome component).
- Dial rendering (canvas or custom draw) with weight markers and fixed
  positions.

### Phase 4 — Integration on a real rotor (2 days)
- Front-end built on perfboard or a breakout. Test rig: a 1 hp motor rotor on
  a stand, spun with compressed air, with Chester's tach sensor. Start with a
  known added mass to check that the solver finds it.
- Full two-plane procedure; verify the residual drops to the predicted level.
- Repeatability: 5 runs with no changes, phase σ < 2°.

### Phase 4b — Tire rig with MPU-6050 sensors  — firmware written; simulated end to end; bench test pending
- `balancer-tire.yaml` on the 7" board, two GY-521 boards, leaf-spring rig.
- Host test: two sensors on their own clocks with noise, lost readings and
  hand-spin coast-downs, through the resampler and `ModelCore`. 35 g and
  25 g on the two rim edges come back within 2% and 2°, and the verify run
  is inside the limit.
- On the bench: confirm both sensors report about 250 Hz on the LIVE screen,
  tap-test the rig, then balance a wheel with a known added weight.

### Phase 5 — Persistence and export (1–2 days)  — settings, rotor setup and machine in NVS; HA sensors done; CSV export open
- Machines and jobs in NVS (small) or SD (larger, CSV).
- Web/HA export of run vectors; CSV download.

### Phase 6 — Front-end PCB (later)
- KiCad board: 2× IEPE (24 V boost, LT3092 CCS, coupling, gain), PCM1808,
  tach header (3.3 V, power out for Chester's sensor), ADS1115 bias
  monitor, JP1 header connector. BNC or MIL-C-5015 sensor connectors.

---

## 10. Risks and open questions

| Risk | Mitigation |
|---|---|
| GPIO3 can't be both I²S LRCK output and capture input | Jumper LRCK to GPIO33 (reserved) |
| MCPWM capture prescale semantics on P4 differ from the docs | Verify in Phase 1; fallback is Option B (TDM ADC with tach as a channel) |
| ESPHome LVGL too limited for the dial or wizard | Draw in C++ from the component; plain IDF is the last resort |
| PCM1808 minimum fs (16 kHz) is more than needed | It's fine — the P4 has plenty of headroom. Could decimate before TSA if CPU becomes tight. |
| Noise or ground loops from sensor cables | IEPE is single-ended — keep one ground point, use case-isolated sensors or isolating studs |
| Air jet adds noise and forces while it's blowing | Coast-down capture is the default; TSA rejects anything not synchronous with rotation |
| Stand resonance near the test speed makes phase speed-sensitive | Phase-vs-speed check in the window (§5.1); move the test speed |
| Too few revolutions in the window on one coast-down | Passes accumulate; the window and revolution count are adjustable |
| Board revision differences (I²C addresses differ between users) | Scan on boot and show on the debug page |

| Tire rig resonance too close to the spin speed | Tap test before measuring; lengthen the spring (§3.3) |
| MPU-6050 readings mis-timed when the task is late | Readings that cannot be tied to one edge are dropped and bridged; count shown on the LIVE screen |
| Long I²C leads to the sensors on a moving rig | Short leads with a ground wire; 100 kHz option |

**Still open**
1. Balancing stand: soft-bearing (pendulum or spring supports, measure above
   resonance) or hard-bearing (stiff supports, measure well below)? It
   changes the test-speed advice and whether readings can be shown as
   absolute unbalance in g·mm rather than only vibration.
2. Rotor details: actual mass, end-ring radius, and whether it has balance
   lugs or fan blades. Defaults in §6.1 assume drilling end rings.
3. Tach sensor output: idle level and pulse width, so the edge and glitch
   filter defaults match.
