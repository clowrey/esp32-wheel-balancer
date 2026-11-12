# References

## Board
- CellKeeper-BMS-LCD (working ESPHome config for this board): https://github.com/clowrey/CellKeeper-BMS-LCD
- Guition JC1060P470 overview: https://www.atomic14.com/esp32/boards/guition-jc1060p470/
- Home Assistant community thread with pin details: https://community.home-assistant.io/t/guition-esp32-p4-jc1060p470/959144
- esp_lcd_jd9165 (panel driver, for a plain-IDF fallback): https://components.espressif.com/components/espressif/esp_lcd_jd9165

## ESP-IDF peripherals
- MCPWM capture (prescale, software catch): https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/api-reference/peripherals/mcpwm/mcpwm_cap.html
- I2C master on ESP32-P4: https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/api-reference/peripherals/i2c.html

## Balancing math
- pyPRB (B&K-guide static and dynamic balancing): https://github.com/ladisk/pyPRB
- hsbalance (multi-plane influence coefficients): https://github.com/MagedMohamedTurk/Turbomachinery-Rotors-Balancing
- Single-plane DIY balancer (Arduino, IR tach): https://github.com/hstarmans/rotorbalancer
