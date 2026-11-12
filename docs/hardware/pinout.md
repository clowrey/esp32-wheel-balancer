# JC1060P470C pinout (as used by p4-balancer)

Source: `schematic/JC1060P470C_I_W_Y-V1.0.pdf` (Guition, 9/06/2025) and the
working CellKeeper-BMS-LCD ESPHome config.

## JP1 "Expand IO" header (2×13, 2.54 mm)

![JP1](jp1-header.png)

| Pin | Signal | Pin | Signal |
|---|---|---|---|
| 1 | 3V3 | 2 | VOUT-BAT |
| 3 | 3V3 | 4 | VOUT-BAT |
| 5 | GND | 6 | GND |
| 7 | **GPIO1** — PCM1808 SCKI (MCLK) | 8 | NC |
| 9 | **GPIO2** — PCM1808 BCK | 10 | GPIO47 — spare |
| 11 | **GPIO3** — PCM1808 LRCK | 12 | GPIO46 — spare |
| 13 | **GPIO4** — PCM1808 DOUT | 14 | GPIO45 — spare (run/stop button) |
| 15 | GPIO5 — avoid (LCD reset in CellKeeper YAML) | 16 | GND |
| 17 | GPIO20 — spare | 18 | 3V3 |
| 19 | **GPIO32** — tach in (3.3 V pulse) | 20 | C6_U0RXD |
| 21 | GPIO33 — LRCK capture jumper (fallback) | 22 | C6_U0TXD |
| 23 | ES_I2C_SDA (GPIO7) — ADS1115 | 24 | C6_IO9 |
| 25 | ES_I2C_SCL (GPIO8) — ADS1115 | 26 | C6_CHIP_PU |

### Tire variant (`balancer-tire.yaml`)

The same header pins carry two GY-521 (MPU-6050) boards in place of the
PCM1808. Full notes in [tire-rig.md](tire-rig.md).

| Pin | Signal |
|---|---|
| 1 or 3 | 3V3 to both sensor boards |
| 5 | GND |
| 7 | GPIO1: SDA |
| 9 | GPIO2: SCL |
| 11 | GPIO3: INT, sensor A (address 0x68) |
| 13 | GPIO4: INT, sensor B (address 0x69, AD0 tied to VCC) |
| 16 | GND: tach sensor 0 V |
| 18 | 3V3: 4.7 kΩ pull-up to pin 19 |
| 19 | GPIO32: tach in (LX-101 black wire, open collector) |

## Onboard functions (do not reuse)

| Function | GPIO |
|---|---|
| I²C bus_a (GT911, ES8311, ES7210, RTC) | 7 SDA, 8 SCL |
| GT911 INT / RST | 21 / 22 |
| Backlight PWM | 23 |
| LCD reset | 0 per schematic; CellKeeper YAML uses 5 |
| ES8311/ES7210 I2S0 | MCLK 13, SCLK 12, LRCK 10, DSDIN 9, ES7210 SDOUT 48 |
| Speaker amp enable | 11 |
| ESP32-C6 SDIO | CLK 18, CMD 19, D0–D3 14–17, RST 54 |
| USB-serial-JTAG | 24/25 |
| UART0 (CN2 MX1.25 / RS485) | 37 TX, 38 RX |
| Battery voltage | 53 |
| Boot / user button | 35 |
| Strapping pins | 34–38 |

## Things to verify on the bench
- GPIO5: free, or really driving LCD reset on this revision?
- GPIO3 as I²S LRCK output **and** MCPWM capture input through the GPIO matrix.
- I²C addresses vary between board batches; scan on boot.
