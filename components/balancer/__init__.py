"""ESPHome external component: two-plane dynamic balancer (see PLAN.md).

Example:

balancer:
  id: rotor_balancer
  i2s: {mclk_pin: GPIO1, bclk_pin: GPIO2, lrclk_pin: GPIO3, din_pin: GPIO4}
  tach: {pin: GPIO32, edge: RISING}
  channel_a: {sensitivity: 100, gain: 10}
  channel_b: {sensitivity: 100, gain: 10}
  simulate: false
  rpm: {name: "Rotor RPM"}
"""

import esphome.codegen as cg
from esphome import pins
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_PIN,
    STATE_CLASS_MEASUREMENT,
)

DEPENDENCIES = ["lvgl"]
AUTO_LOAD = ["sensor"]
CODEOWNERS = ["@clowrey"]

balancer_ns = cg.esphome_ns.namespace("balancer")
BalancerComponent = balancer_ns.class_("BalancerComponent", cg.Component)

CONF_I2S = "i2s"
CONF_MCLK_PIN = "mclk_pin"
CONF_BCLK_PIN = "bclk_pin"
CONF_LRCLK_PIN = "lrclk_pin"
CONF_DIN_PIN = "din_pin"
CONF_PORT = "port"
CONF_SAMPLE_RATE = "sample_rate"
CONF_USE_APLL = "use_apll"
CONF_LRCK_CAPTURE_PIN = "lrck_capture_pin"
CONF_TACH = "tach"
CONF_EDGE = "edge"
CONF_PULLUP = "pullup"
CONF_ADC_FULL_SCALE = "adc_full_scale"
CONF_CHANNEL_A = "channel_a"
CONF_CHANNEL_B = "channel_b"
CONF_SENSITIVITY = "sensitivity"
CONF_GAIN = "gain"
CONF_INVERT = "invert"
CONF_SWAP_CHANNELS = "swap_channels"
CONF_ADC_DELAY = "adc_delay_us"
CONF_SIMULATE = "simulate"
CONF_RPM = "rpm"
CONF_AMPLITUDE_A = "amplitude_a"
CONF_AMPLITUDE_B = "amplitude_b"
CONF_PHASE_A = "phase_a"
CONF_PHASE_B = "phase_b"
CONF_TASK_CORE = "task_core"

# LVGL builds only the widgets and fonts the YAML uses. The UI is written in
# C++, so register what it needs during validation (before lvgl's codegen).
UI_WIDGETS = (
    "label",
    "button",
    "line",
    "bar",
    "textarea",
    "keyboard",
    "buttonmatrix",
    "flex",
)
UI_FONTS = ("montserrat_14", "montserrat_20", "montserrat_28", "montserrat_40", "montserrat_48")


def _register_lvgl_needs(config):
    from esphome.components.lvgl import defines as lvdf

    lvdf.add_lv_use(*UI_WIDGETS)
    lvdf.get_lv_fonts_used().update(UI_FONTS)
    return config


CHANNEL_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_SENSITIVITY, default=100.0): cv.positive_float,  # mV/g
        cv.Optional(CONF_GAIN, default=10.0): cv.positive_float,
        cv.Optional(CONF_INVERT, default=False): cv.boolean,
    }
)

I2S_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_MCLK_PIN, default=1): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_BCLK_PIN, default=2): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_LRCLK_PIN, default=3): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_DIN_PIN, default=4): pins.internal_gpio_input_pin_number,
        cv.Optional(CONF_PORT, default=1): cv.int_range(0, 2),
        cv.Optional(CONF_SAMPLE_RATE, default=16000): cv.int_range(8000, 48000),
        cv.Optional(CONF_USE_APLL, default=False): cv.boolean,
    }
)

TACH_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_PIN, default=32): pins.internal_gpio_input_pin_number,
        cv.Optional(CONF_EDGE, default="RISING"): cv.one_of("RISING", "FALLING", upper=True),
        cv.Optional(CONF_PULLUP, default=True): cv.boolean,
    }
)

_amp = sensor.sensor_schema(accuracy_decimals=3, state_class=STATE_CLASS_MEASUREMENT)
_phase = sensor.sensor_schema(unit_of_measurement="°", accuracy_decimals=1, state_class=STATE_CLASS_MEASUREMENT)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BalancerComponent),
            cv.Optional(CONF_I2S, default={}): I2S_SCHEMA,
            cv.Optional(CONF_LRCK_CAPTURE_PIN, default=-1): cv.Any(
                cv.int_range(-1, -1), pins.internal_gpio_input_pin_number
            ),
            cv.Optional(CONF_TACH, default={}): TACH_SCHEMA,
            cv.Optional(CONF_ADC_FULL_SCALE, default=1.5): cv.positive_float,  # volts peak
            cv.Optional(CONF_CHANNEL_A, default={}): CHANNEL_SCHEMA,
            cv.Optional(CONF_CHANNEL_B, default={}): CHANNEL_SCHEMA,
            cv.Optional(CONF_SWAP_CHANNELS, default=False): cv.boolean,
            cv.Optional(CONF_ADC_DELAY, default=1087.5): cv.float_range(min=0, max=100000),  # PCM1808: 17.4/fs
            cv.Optional(CONF_SIMULATE, default=False): cv.boolean,
            cv.Optional(CONF_TASK_CORE, default=1): cv.int_range(0, 1),
            cv.Optional(CONF_RPM): sensor.sensor_schema(
                unit_of_measurement="rpm",
                icon="mdi:rotate-right",
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_AMPLITUDE_A): _amp,
            cv.Optional(CONF_AMPLITUDE_B): _amp,
            cv.Optional(CONF_PHASE_A): _phase,
            cv.Optional(CONF_PHASE_B): _phase,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
    _register_lvgl_needs,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    i2s = config[CONF_I2S]
    cg.add(var.set_i2s(i2s[CONF_PORT], i2s[CONF_MCLK_PIN], i2s[CONF_BCLK_PIN], i2s[CONF_LRCLK_PIN],
                       i2s[CONF_DIN_PIN], i2s[CONF_SAMPLE_RATE], i2s[CONF_USE_APLL]))
    cg.add(var.set_lrck_capture_pin(config[CONF_LRCK_CAPTURE_PIN]))
    tach = config[CONF_TACH]
    cg.add(var.set_tach(tach[CONF_PIN], tach[CONF_EDGE] == "FALLING", tach[CONF_PULLUP]))
    cg.add(var.set_adc_full_scale(config[CONF_ADC_FULL_SCALE]))
    for idx, key in enumerate((CONF_CHANNEL_A, CONF_CHANNEL_B)):
        ch = config[key]
        cg.add(var.set_channel_default(idx, ch[CONF_SENSITIVITY], ch[CONF_GAIN], ch[CONF_INVERT]))
    cg.add(var.set_swap_channels(config[CONF_SWAP_CHANNELS]))
    cg.add(var.set_adc_delay_us(config[CONF_ADC_DELAY]))
    cg.add(var.set_simulate(config[CONF_SIMULATE]))
    cg.add(var.set_task_core(config[CONF_TASK_CORE]))

    for key, setter in (
        (CONF_RPM, "set_rpm_sensor"),
        (CONF_AMPLITUDE_A, "set_amp_sensor_a"),
        (CONF_AMPLITUDE_B, "set_amp_sensor_b"),
        (CONF_PHASE_A, "set_phase_sensor_a"),
        (CONF_PHASE_B, "set_phase_sensor_b"),
    ):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, setter)(sens))
