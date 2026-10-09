# Analog front end

Two identical IEPE channels into a PCM1808, plus a tach input. Phase 4 builds
this on perfboard / breakouts; Phase 6 turns it into a KiCad board.

## Per IEPE channel

```
+24V ─┬─ LT3092 (Rset → 4.0 mA)  ──┬──────────── sensor + (signal/power, coax centre)
      │                            │             sensor − (shield) ── AGND
      │                    100k/100k divider ─► ADS1115 AINx (bias monitor)
      │                            │
      │                         C_ac 10 µF film/X7R ─┬─ R_bias 330k → VREF(PCM1808 VCOM)
      │                                              │
      │                                       gain stage ×10 (OPA1612/OPA2192)
      │                                              │
      │                                        RC AA: 1k + 10 nF (≈16 kHz)
      │                                              │
      └──────────────────────────────────────────►  PCM1808 VINL / VINR
```

- **Excitation:** 2–10 mA constant current, 18–30 V compliance. LT3092 is the
  low-noise choice (as in Analog Devices CN0561); a 4 mA CRD is fine for a
  first build.
- **+24 V:** small boost from 5 V (TPS61040 / MT3608 module), followed by an
  RC or LDO filter. Keep switching noise away from the ADC.
- **Coupling corner:** 10 µF × 330 kΩ ≈ 0.05 Hz, so it has negligible phase
  effect at balancing speeds. Settling takes a few seconds; the firmware waits
  for the bias to settle (from the ADS1115 reading) before measuring.
- **Bias monitor:** IEPE bias normally sits around 8–14 V. About 24 V means
  the cable is open; under about 2 V means it's shorted. The ADS1115 reads it
  through a divider, giving the "sensor OK" indicator on the status bar.
- **PCM1808 input:** about 0.6·VCC p-p full scale (≈3 Vpp at 5 V AVDD),
  single-ended, biased at VCOM.

### Gain

v1 sensors are **100 mV/g industrial IEPE** (PCB 603C01 class, to buy).
Default gain is **×10**, giving about ±1.5 g full scale. A well-balanced 1 hp
rotor on a stand at ~1500 RPM produces only tens of mg at 1×, so the extra
gain matters more than headroom. Drop to ×3 (±5 g) if the air jet or a
rough stand clips the input.

| Sensor | Sensitivity | Gain | Full scale ≈ |
|---|---|---|---|
| PCB 603C01 / Dytran 3055 (default) | 100 mV/g | ×10 | ±1.5 g |
| same, rough conditions | 100 mV/g | ×3 | ±5 g |
| Dytran 3115A (on hand, 10.670 mV/g cal) | ~10.7 mV/g | ×20 | ±7 g |

Gain and per-sensor sensitivity (from each sensor's cal sheet) are
configuration values, not hard-coded.

## PCM1808 setup

- Mode: slave (MD1=MD0=0); the P4 drives SCKI, BCK and LRCK.
- Format: I²S 24-bit (FMT=0).
- fs = 16 kHz, SCKI = 256·fs = 4.096 MHz.
- The digital HPF is on (corner ~0.3 Hz at 16 kHz). That's fine; its phase at
  ≥5 Hz is negligible and identical on both channels.
- Supply: 5 V analog (AVDD) for full input range, 3.3 V digital (DVDD), so
  logic levels match the P4.

## Tach input

Chester builds the laser/optical sensor. The firmware only needs a clean
**3.3 V logic pulse, once per revolution, on GPIO32** (JP1 pin 19).

- Edge (rising/falling), internal pull-up, and minimum pulse period
  (glitch rejection) are YAML settings.
- If the sensor runs at 5 V, level-shift or divide down before GPIO32.
- Reflective tape mark = 0°. Rotation direction and angle convention are set
  in the UI.

## Bill of materials (prototype)

| Qty | Part | Note |
|---|---|---|
| 1 | PCM1808 breakout (common blue "PCM1808 ADC" board) | check MD/FMT strapping |
| 2 | LT3092 + Rset/Rout | or 4 mA CRD (e.g. CRD 4.0 mA) |
| 1 | 5→24 V boost module | low-ripple, add LC filter |
| 1 | OPA1612 or OPA2192 | dual low-noise op-amp for gain |
| 1 | ADS1115 breakout | bias monitor on I²C bus_a, addr 0x48 |
| 2 | BNC or MIL-C-5015 2-pin | sensor connectors |
| 2 | IEPE accelerometers, 100 mV/g (PCB 603C01 class) | to buy; keep the cal sheets |
| 2 | Magnetic bases or 1/4-28 studs | stud mount preferred for repeatability |
| 1 | Tach sensor + reflective tape | Chester's build; 3.3 V pulse to GPIO32 |
