# P4 Balancer

Two-plane dynamic balancer on the Guition JC1060P470C (ESP32-P4, 7"
1024×600 touchscreen). Two IEPE accelerometers and an optical tach in;
"add 12.4 g at 137° on plane 1" out, drawn on rotor dials on the LCD.

**Status:** planning. See [PLAN.md](PLAN.md).

## How it works (short version)
- Two IEPE accelerometers feed a 4 mA current source, then AC coupling, gain,
  and a PCM1808 stereo ΔΣ ADC on I2S1. Both channels are sampled at the same
  instant.
- MCPWM capture latches tach edges and LRCK edges in hardware, so every tach
  pulse is known to a fraction of a sample.
- Time-synchronous averaging over whole revolutions, then the 1× vector per
  channel with a stability check.
- Influence-coefficient solver (single and two plane, trim runs, weight
  splitting, ISO 21940-11 residual).
- ESPHome + LVGL UI wizard: setup → run 0 → trial plane 1 → trial plane 2 →
  result dials → verify/trim.

## Repo map
- `PLAN.md` — architecture, phases, risks, open questions
- `CLAUDE.md` — conventions for Claude Code sessions
- `hardware_guition_jc1060p470.yaml` — board config (from CellKeeper-BMS-LCD)
- `docs/hardware/` — pinout (JP1 header), front end, schematic
- `docs/legacy/` — the original BMI160 lock-in sketch and its review
- `docs/research/sources.md` — references
