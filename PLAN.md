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
- Driving the rotor motor (spare GPIOs are reserved for it).

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
IEPE accel ──► 4 mA CCS from +24 V ──► AC couple (≤0.1 Hz) ──► gain/atten ──► RC AA ──┐
 (100 mV/g preferred,                    LT3092 or CRD                                   │
  Dytran 3115A works)                                                                     ▼
                                                              PCM1808 (stereo 24-bit ΔΣ, I²S)
                                                                     │  MCLK/BCK/LRCK from P4
                                                                     ▼
                                                          ESP32-P4 I2S1 (master) + DMA
Optical/laser tach ──► 74LVC1G17 Schmitt ──► MCPWM capture (hardware timestamp)
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
| Tach input | 32 | 19 | MCPWM capture ch0, Schmitt-buffered |
| LRCK capture (jumper) | 33 | 21 | fallback if internal loopback of GPIO3 doesn't work |
| ADS1115 SDA / SCL | 7 / 8 | 23 / 25 | shared bus_a; 0x48 is free (GT911, ES8311, ES7210, RTC are elsewhere) |
| Spare: run/stop button | 45 | 14 | |
| Spare: motor enable / beeper | 46 | 12 | |
| Spare | 20, 47 | 17, 10 | |

All pins are set in YAML (`balancer:` config), so they can be moved without
code changes. Onboard ES8311/ES7210 stay on I2S0 (GPIO9–13, 48) and are
untouched; the ES8311 + speaker (amp enable GPIO11) can be used for beeps.

**To verify on the bench before PCB work:** GPIO5 really is free or really
is the LCD reset on this board revision; GPIO3 can be an I²S output and an
MCPWM capture input at the same time through the GPIO matrix.

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
- **Checks:** warn if a trial weight changes amplitude by less than 30% or
  phase by less than 30° (rule of thumb). Warn if cond(A) is high.
  Report predicted residual vibration.
- **ISO 21940-11:** U_per [g·mm] = 9549 · G · m [kg] / n [rpm], split
  between planes by bearing load ratio. Display it against the residual
  predicted from the influence coefficients.

Validate against `pyPRB` and `hsbalance` with the same inputs (see Phase 2).

---

## 7. UI (LVGL, 1024×600 landscape)

Status bar on every screen: RPM, stability dot (red/amber/green), sensor
status (OK / open / short from IEPE bias), job name.

1. **Home:** big buttons — New balance, Trim run, Live vibration, Machines,
   Settings.
2. **Rotor setup (wizard step 0):** 1 or 2 planes; trial mass and radius per
   plane; correction radius; angle convention (with an illustration of the
   tach mark and rotation direction); fixed positions (none / N holes /
   N blades); target RPM band; ISO grade and rotor mass (optional).
3. **Measure run screen (reused for each run):**
   - Instructions at top ("Run 1: add 5.0 g at 0° on Plane 1, spin up").
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

## 8. Repository layout (target)

```
p4-balancer/
├── balancer-jc1060p470.yaml            # device entry (name, wifi, packages)
├── hardware_guition_jc1060p470.yaml    # board config from CellKeeper
├── balancer_core.yaml                  # balancer: component config, sensors
├── ui_1024x600.yaml                    # LVGL pages
├── components/balancer/
│   ├── __init__.py                     # ESPHome config schema + codegen
│   ├── balancer.h / balancer.cpp       # ESPHome Component glue
│   ├── acq/                            # IDF-only: i2s_adc, tach_capture, timebase
│   ├── dsp/                            # pure C++: tsa, orders, balance, iso
│   └── ui/                             # C++ dial drawing, wizard state machine
├── tests/                              # host unit tests (CMake + doctest)
│   └── sim/                            # Python synthetic-signal generator
├── tools/                              # CSV export parser, pyPRB cross-check
└── docs/
    ├── hardware/  (front-end.md, pinout, schematic, KiCad later)
    ├── ui/        (wireframes)
    └── legacy/    (original BMI160 sketch + analysis)
```

---

## 9. Phases

### Phase 0 — Board bring-up (½ day)
- Copy the hardware YAML from CellKeeper and build a "hello" page.
- Confirm GPIO5's role on this revision.
- Confirm JP1 pin access and the 3V3 / VOUT-BAT levels.

### Phase 1 — Acquisition (2–3 days)
- `i2s_adc`: I2S1 master, 16 kHz, 24-bit (32-bit slots), DMA, stereo.
  Start with a PCM1808 breakout board and a signal generator.
- `tach_capture`: MCPWM capture for tach and LRCK (prescale 256), fractional
  sample-index conversion, glitch rejection.
- Debug page: live waveforms, RPM, sample-rate check (counted LRCK vs
  expected).
- **Exit test:** feed the same sine to both channels plus a square wave
  derived from the same generator as the tach. Phase between channels
  < 0.1°, phase vs tach stable < 0.2° over 10 min.

### Phase 2 — DSP and math, host-first (2–3 days, parallel with Phase 1)
- `dsp/` with doctest unit tests: TSA, order extraction, single- and
  two-plane solvers, split and combine, ISO calculation.
- Python simulator: rotor with known unbalance plus noise and speed jitter
  → synthetic I²S stream and tach events → firmware DSP → compare with the
  injected unbalance.
- Cross-check solver outputs against pyPRB and hsbalance.
- **Exit test:** simulator recovers injected unbalance within 2% / 1° at
  SNR 20 dB.

### Phase 3 — UI (3–4 days)
- Wireframes, then LVGL pages: home, setup, measure, result dial, live,
  settings.
- Wizard state machine in C++ (`ui/wizard`), driving LVGL from the component.
- Dial rendering (canvas or custom draw) with weight markers and fixed
  positions.

### Phase 4 — Integration on a real rotor (2 days)
- Front-end built on perfboard or a breakout. Sensors on a test rig (fan or
  grinder with a known added mass).
- Full two-plane procedure; verify the residual drops to the predicted level.
- Repeatability: 5 runs with no changes, phase σ < 2°.

### Phase 5 — Persistence and export (1–2 days)
- Machines and jobs in NVS (small) or SD (larger, CSV).
- Web/HA export of run vectors; CSV download.

### Phase 6 — Front-end PCB (later)
- KiCad board: 2× IEPE (24 V boost, LT3092 CCS, coupling, gain), PCM1808,
  tach input with Schmitt and optional opto, ADS1115 bias monitor, JP1
  header connector. BNC or MIL-C-5015 sensor connectors.

---

## 10. Risks and open questions

| Risk | Mitigation |
|---|---|
| GPIO3 can't be both I²S LRCK output and capture input | Jumper LRCK to GPIO33 (reserved) |
| MCPWM capture prescale semantics on P4 differ from the docs | Verify in Phase 1; fallback is Option B (TDM ADC with tach as a channel) |
| ESPHome LVGL too limited for the dial or wizard | Draw in C++ from the component; plain IDF is the last resort |
| PCM1808 minimum fs (16 kHz) is more than needed | It's fine — the P4 has plenty of headroom. Could decimate before TSA if CPU becomes tight. |
| Noise or ground loops from sensor cables | IEPE is single-ended — keep one ground point, use case-isolated sensors or isolating studs |
| Low SNR with 10 mV/g sensors | Gain stage; prefer 100 mV/g industrial sensors |
| Board revision differences (I²C addresses differ between users) | Scan on boot and show on the debug page |

**Questions for Chester**
1. Target rotors: speed range and typical size/mass? (Sets the default fs,
   averaging time and ISO helper defaults.)
2. Which sensors for v1 — the Dytran 3115As you have, or buy 100 mV/g
   units?
3. Tach: laser/optical module you already have, or design one in?
4. ESPHome (recommended) vs plain IDF — any preference?
5. Should the board drive the rotor motor (spin up to target RPM)?
