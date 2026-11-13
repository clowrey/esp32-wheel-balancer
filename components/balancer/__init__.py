"""ESPHome external component: two-plane dynamic balancer (see PLAN.md).

Two MPU-6050-family accelerometers on I2C and a tach, for a car wheel on a
soft rig (see docs/hardware/tire-rig.md). Example:

balancer:
  id: rotor_balancer
  simulate: false
  mpu6050:
    sda_pin: GPIO1
    scl_pin: GPIO2
    sensor_a: {address: 0x68, int_pin: GPIO3, axis: X}
    sensor_b: {address: 0x69, int_pin: GPIO4, axis: X}   # leave out for one sensor
  tach: {pin: GPIO32, edge: FALLING, teach_pin: GPIO20}
  rpm: {name: "Wheel speed"}
"""

import esphome.codegen as cg
from esphome import pins
from esphome.components import display, sensor, touchscreen
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_PIN,
    STATE_CLASS_MEASUREMENT,
)

DEPENDENCIES = ["display", "touchscreen"]
AUTO_LOAD = ["sensor", "web_server_base"]  # web_server_base: the recording download
CODEOWNERS = ["@clowrey"]

balancer_ns = cg.esphome_ns.namespace("balancer")
BalancerComponent = balancer_ns.class_("BalancerComponent", cg.Component)

CONF_SAMPLE_RATE = "sample_rate"
CONF_TACH = "tach"
CONF_EDGE = "edge"
CONF_PULLUP = "pullup"
CONF_TEACH_PIN = "teach_pin"
CONF_INVERT = "invert"
CONF_SIMULATE = "simulate"
CONF_RPM = "rpm"
CONF_AMPLITUDE_A = "amplitude_a"
CONF_AMPLITUDE_B = "amplitude_b"
CONF_PHASE_A = "phase_a"
CONF_PHASE_B = "phase_b"
CONF_TASK_CORE = "task_core"
CONF_MPU6050 = "mpu6050"
CONF_SDA_PIN = "sda_pin"
CONF_SCL_PIN = "scl_pin"
CONF_FREQUENCY = "frequency"
CONF_FILTER = "filter"
CONF_RANGE = "range"
CONF_SENSOR_A = "sensor_a"
CONF_SENSOR_B = "sensor_b"
CONF_ADDRESS = "address"
CONF_INT_PIN = "int_pin"
CONF_AXIS = "axis"
CONF_MAX_SPEED_CHANGE = "max_speed_change"
CONF_PC_LOG = "pc_log"
CONF_WEIGHT_UNIT = "weight_unit"
CONF_RECORD_MINUTES = "record_minutes"
# The screen the UI draws on and the touch panel over it (each one picked up by itself when
# the board has only one).
CONF_DISPLAY_ID = "display_id"
CONF_TOUCHSCREEN_ID = "touchscreen_id"

AXES = {"X": 0, "Y": 1, "Z": 2}

TACH_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_PIN, default=32): pins.internal_gpio_input_pin_number,
        cv.Optional(CONF_EDGE, default="RISING"): cv.one_of("RISING", "FALLING", upper=True),
        cv.Optional(CONF_PULLUP, default=True): cv.boolean,
        # Drives the sensor's teach input (LX-101 pink wire) through an NPN
        # transistor or N-MOSFET: high = teach. Never wire it to pink directly.
        cv.Optional(CONF_TEACH_PIN): pins.internal_gpio_output_pin_number,
    }
)

MPU_SENSOR_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ADDRESS): cv.i2c_address,
        cv.Required(CONF_INT_PIN): pins.internal_gpio_input_pin_number,
        # The sensor axis that points along the motion being measured.
        cv.Optional(CONF_AXIS, default="X"): cv.enum(AXES, upper=True),
        # Flip the sign if the board is mounted the other way round (also on the settings screen).
        cv.Optional(CONF_INVERT, default=False): cv.boolean,
        # Sensor B only: its own I2C bus, so a sensor that hangs or fails
        # can't stall the other. It runs on the P4's LP I2C controller, which
        # only reaches GPIO0..15. Leave both out to share sensor A's bus.
        cv.Optional(CONF_SDA_PIN): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_SCL_PIN): pins.internal_gpio_output_pin_number,
    }
)

# One or two MPU-6050-family accelerometers on their own I2C bus (not the
# board's touch bus: this one is read from the acquisition task).
MPU6050_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_SDA_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_SCL_PIN): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_FREQUENCY, default="400kHz"): cv.All(cv.frequency, cv.int_range(min=50000, max=400000)),
        # Readings per second (the chip divides 1 kHz) and its low-pass filter.
        cv.Optional(CONF_SAMPLE_RATE, default=250): cv.one_of(50, 100, 125, 200, 250, 500, 1000, int=True),
        cv.Optional(CONF_FILTER, default="44Hz"): cv.All(cv.frequency, cv.one_of(5, 10, 21, 44, 94, 184, int=True)),
        cv.Optional(CONF_RANGE, default=2): cv.one_of(2, 4, 8, 16, int=True),  # +/- g
        cv.Required(CONF_SENSOR_A): MPU_SENSOR_SCHEMA,
        cv.Optional(CONF_SENSOR_B): MPU_SENSOR_SCHEMA,
    }
)


def _check_mpu(config):
    mpu = config[CONF_MPU6050]
    if 2 * mpu[CONF_FILTER] > mpu[CONF_SAMPLE_RATE]:
        raise cv.Invalid(
            f"filter {mpu[CONF_FILTER]} Hz is too wide for {mpu[CONF_SAMPLE_RATE]} readings per second "
            "(keep it at or below half the sample rate)",
            path=[CONF_MPU6050, CONF_FILTER],
        )
    # Each reading is an 85-clock I2C transfer that has to finish well before
    # the next reading is due, or it cannot be tied to its timestamp.
    for key in (CONF_SDA_PIN, CONF_SCL_PIN):
        if key in mpu[CONF_SENSOR_A]:
            raise cv.Invalid(
                f"sensor_a uses the bus pins set above; '{key}' is only for sensor_b", path=[CONF_MPU6050, CONF_SENSOR_A, key]
            )
    own_bus = False
    if CONF_SENSOR_B in mpu:
        b = mpu[CONF_SENSOR_B]
        if (CONF_SDA_PIN in b) != (CONF_SCL_PIN in b):
            raise cv.Invalid("set both sda_pin and scl_pin for sensor_b, or neither", path=[CONF_MPU6050, CONF_SENSOR_B])
        own_bus = CONF_SDA_PIN in b
        if own_bus:
            for key in (CONF_SDA_PIN, CONF_SCL_PIN):
                if b[key] > 15:
                    raise cv.Invalid(
                        f"sensor_b's own bus runs on the P4's LP I2C controller, which only reaches GPIO0..15 "
                        f"(GPIO{b[key]} given)",
                        path=[CONF_MPU6050, CONF_SENSOR_B, key],
                    )
    sensors = 2 if CONF_SENSOR_B in mpu and not own_bus else 1  # sensors sharing one bus
    busy = sensors * 85.0 / mpu[CONF_FREQUENCY] * mpu[CONF_SAMPLE_RATE]
    if busy > 0.5:
        raise cv.Invalid(
            f"{sensors} sensor(s) at {mpu[CONF_SAMPLE_RATE]} readings per second need a faster bus than "
            f"{int(mpu[CONF_FREQUENCY] / 1000)} kHz: raise 'frequency' or lower 'sample_rate'",
            path=[CONF_MPU6050, CONF_SAMPLE_RATE],
        )
    if (
        CONF_SENSOR_B in mpu
        and not own_bus
        and mpu[CONF_SENSOR_B][CONF_ADDRESS] == mpu[CONF_SENSOR_A][CONF_ADDRESS]
    ):
        raise cv.Invalid(
            "sensor_a and sensor_b need different addresses (tie AD0 high on one board for 0x69)",
            path=[CONF_MPU6050, CONF_SENSOR_B, CONF_ADDRESS],
        )
    return config


_amp = sensor.sensor_schema(accuracy_decimals=3, state_class=STATE_CLASS_MEASUREMENT)
_phase = sensor.sensor_schema(unit_of_measurement="°", accuracy_decimals=1, state_class=STATE_CLASS_MEASUREMENT)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BalancerComponent),
            cv.GenerateID(CONF_DISPLAY_ID): cv.use_id(display.Display),
            cv.GenerateID(CONF_TOUCHSCREEN_ID): cv.use_id(touchscreen.Touchscreen),
            cv.Optional(CONF_TACH, default={}): TACH_SCHEMA,
            cv.Optional(CONF_SIMULATE, default=False): cv.boolean,
            cv.Optional(CONF_TASK_CORE, default=1): cv.int_range(0, 1),
            cv.Required(CONF_MPU6050): MPU6050_SCHEMA,
            # Largest speed change from one revolution to the next that still counts.
            cv.Optional(CONF_MAX_SPEED_CHANGE): cv.percentage,
            # Stream every raw reading to the USB log from boot (tools/mpu_log.py).
            # It can also be switched on the LIVE screen.
            cv.Optional(CONF_PC_LOG, default=False): cv.boolean,
            # Unit for balance weights on the screens until changed and saved in Settings.
            cv.Optional(CONF_WEIGHT_UNIT, default="g"): cv.one_of("g", "oz", lower=True),
            # Keep the last N minutes of raw readings in PSRAM, downloadable at
            # http://<board>/recording.bin (tools/mpu_recording.py). 0: off.
            cv.Optional(CONF_RECORD_MINUTES, default=10): cv.float_range(min=0, max=60),
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
    _check_mpu,
)


async def to_code(config):
    # Allocate the component at setup time instead of in static storage
    # (what cg.new_Pvariable does). Static data sits in the small block of
    # internal RAM that is all the heap there is until FreeRTOS starts; the
    # Wi-Fi co-processor driver and the system tasks fill the rest of it, and
    # this object is about 27 KB. With it there, 200 more bytes of state made
    # the board assert in vApplicationGetIdleTaskMemory on every boot.
    var = cg.Pvariable(
        config[CONF_ID], cg.RawExpression("new esphome::balancer::BalancerComponent()")
    )
    await cg.register_component(var, config)
    cg.add(var.set_display(await cg.get_variable(config[CONF_DISPLAY_ID])))
    cg.add(var.set_touchscreen(await cg.get_variable(config[CONF_TOUCHSCREEN_ID])))

    # The revolution analyser's sample rings (up to a few hundred KB) are
    # std::vectors. With PSRAM in caps-only mode, new/malloc can only use
    # internal RAM and the first one throws bad_alloc in setup(). Let large
    # allocations fall through to PSRAM (small ones stay internal).
    from esphome.components.esp32 import add_idf_sdkconfig_option

    add_idf_sdkconfig_option("CONFIG_SPIRAM_USE_CAPS_ALLOC", False)
    add_idf_sdkconfig_option("CONFIG_SPIRAM_USE_MALLOC", True)

    tach = config[CONF_TACH]
    cg.add(var.set_tach(tach[CONF_PIN], tach[CONF_EDGE] == "FALLING", tach[CONF_PULLUP]))
    if CONF_TEACH_PIN in tach:
        cg.add(var.set_tach_teach_pin(tach[CONF_TEACH_PIN]))
    cg.add(var.set_simulate(config[CONF_SIMULATE]))
    cg.add(var.set_task_core(config[CONF_TASK_CORE]))
    cg.add(var.set_pc_log_at_boot(config[CONF_PC_LOG]))
    cg.add(var.set_weight_unit_oz(config[CONF_WEIGHT_UNIT] == "oz"))
    cg.add(var.set_record_minutes(config[CONF_RECORD_MINUTES]))
    if CONF_MAX_SPEED_CHANGE in config:
        cg.add(var.set_max_speed_change(config[CONF_MAX_SPEED_CHANGE]))
    mpu = config[CONF_MPU6050]
    cg.add(
        var.set_mpu_bus(
            mpu[CONF_SDA_PIN],
            mpu[CONF_SCL_PIN],
            int(mpu[CONF_FREQUENCY]),
            mpu[CONF_SAMPLE_RATE],
            int(mpu[CONF_FILTER]),
            mpu[CONF_RANGE],
        )
    )
    for idx, key in enumerate((CONF_SENSOR_A, CONF_SENSOR_B)):
        if key in mpu:
            sens = mpu[key]
            cg.add(
                var.set_mpu_sensor(
                    idx, int(sens[CONF_ADDRESS]), sens[CONF_INT_PIN], AXES[str(sens[CONF_AXIS])], sens[CONF_INVERT]
                )
            )
            if CONF_SDA_PIN in sens:
                cg.add(var.set_mpu_bus_b(sens[CONF_SDA_PIN], sens[CONF_SCL_PIN]))

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
