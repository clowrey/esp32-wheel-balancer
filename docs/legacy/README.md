# Legacy: BMI160 lock-in sketch

`original_lockin.ino` is the starting point this project replaces: two BMI160
IMUs on separate I²C buses, a once-per-rev tach on GPIO14, a software lock-in
at 1600 Hz, and 2-second averages printed over serial when RPM is between
4800 and 5000. It is kept for reference. The file is truncated as received
(closing braces are missing).

## Issues found in the review

1. **Unsynchronized sampling.** Each BMI160 samples on its own oscillator,
   while the ESP polls on a separate 625 µs timer, so samples are repeated or
   skipped and the two sensors are skewed. The calibration slopes
   (0.012 and 0.0191 °/RPM ≈ 2.0 and 3.2 ms of delay) reflect this.
2. **Bandpass centred at ~49 Hz** (Q ≈ 0.72), while the speed band is
   80–83 Hz. That adds RPM-dependent phase the linear calibration only
   approximates.
3. `Wire.read() | (Wire.read() << 8)` has unspecified evaluation order.
4. **ISR race** on `t_last_pulse` / `t_turn` can produce a wrapped `age` and
   a garbage θ.
5. No stall timeout; `speed_stable` latches forever.
6. The RPM gate uses only the last revolution, not the whole window.
7. Failed I²C reads feed zeros into the filter.
8. The window isn't an integer number of revolutions (minor leakage).

The new design addresses all of these: hardware-latched timing in
sample-index units, a simultaneous-sampling ADC, no bandpass,
time-synchronous averaging over whole revolutions, and window-level
quality gating.
