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
      │                                       gain stage (OPA1612/OPA2192, G set per sensor)
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

### Gain by sensor

| Sensor | Sensitivity | Gain | Full scale ≈ |
|---|---|---|---|
| PCB 603C01 / Dytran 3055 (100 mV/g) | 100 mV/g | ×3 | ±5 g |
| Dytran 3115A (10.670 mV/g cal) | ~10.7 mV/g | ×20 | ±7 g |

Gain and sensitivity are configuration values, not hard-coded.

## PCM1808 setup

- Mode: slave (MD1=MD0=0); the P4 drives SCKI, BCK and LRCK.
- Format: I²S 24-bit (FMT=0).
- fs = 16 kHz, SCKI = 256·fs = 4.096 MHz.
- The digital HPF is on (corner ~0.3 Hz at 16 kHz). That's fine; its phase at
  ≥5 Hz is negligible and identical on both channels.
- Supply: 5 V analog (AVDD) for full input range, 3.3 V digital (DVDD), so
  logic levels match the P4.

## Tach input

- Optical reflective sensor or laser tach module (open-collector or 5 V push-pull).
- 10k pull-up to 3V3, then a 74LVC1G17 Schmitt buffer into GPIO32. Add an
  optocoupler option for long cables or noisy machines.
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
| 1 | 74LVC1G17 | tach Schmitt buffer |
| 2 | BNC or MIL-C-5015 2-pin | sensor connectors |
| 2 | IEPE accelerometers | 100 mV/g preferred; 3115A works with more gain |
| 1 | Optical/laser tach + reflective tape | |
