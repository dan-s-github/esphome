import esphome.codegen as cg
from esphome.components import sensor, uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_CURRENT,
    CONF_ENERGY,
    CONF_EXTERNAL_TEMPERATURE,
    CONF_ID,
    CONF_INTERNAL_TEMPERATURE,
    CONF_POWER,
    CONF_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_KILOWATT_HOURS,
    UNIT_VOLT,
    UNIT_WATT,
)

CONF_REFERENCE_VOLTAGE = "reference_voltage"
CONF_RESISTOR_SHUNT = "resistor_shunt"
CONF_RESISTOR_ONE = "resistor_one"
CONF_RESISTOR_TWO = "resistor_two"

CONF_CURRENT_REFERENCE = "current_reference"
CONF_ENERGY_REFERENCE = "energy_reference"
CONF_POWER_REFERENCE = "power_reference"
CONF_VOLTAGE_REFERENCE = "voltage_reference"

DEPENDENCIES = ["uart"]

bl0940_v2_ns = cg.esphome_ns.namespace("bl0940_v2")
BL0940_V2 = bl0940_v2_ns.class_("BL0940_V2", cg.PollingComponent, uart.UARTDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BL0940_V2),
            cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_ENERGY): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOWATT_HOURS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_INTERNAL_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_EXTERNAL_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_REFERENCE_VOLTAGE): cv.float_,
            cv.Optional(CONF_RESISTOR_SHUNT): cv.float_,
            cv.Optional(CONF_RESISTOR_ONE): cv.float_,
            cv.Optional(CONF_RESISTOR_TWO): cv.float_,
            cv.Optional(CONF_CURRENT_REFERENCE): cv.float_,
            cv.Optional(CONF_ENERGY_REFERENCE): cv.float_,
            cv.Optional(CONF_POWER_REFERENCE): cv.float_,
            cv.Optional(CONF_VOLTAGE_REFERENCE): cv.float_,
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if voltage_config := config.get(CONF_VOLTAGE):
        sens = await sensor.new_sensor(voltage_config)
        cg.add(var.set_voltage_sensor(sens))
    if current_config := config.get(CONF_CURRENT):
        sens = await sensor.new_sensor(current_config)
        cg.add(var.set_current_sensor(sens))
    if power_config := config.get(CONF_POWER):
        sens = await sensor.new_sensor(power_config)
        cg.add(var.set_power_sensor(sens))
    if energy_config := config.get(CONF_ENERGY):
        sens = await sensor.new_sensor(energy_config)
        cg.add(var.set_energy_sensor(sens))
    if internal_temperature_config := config.get(CONF_INTERNAL_TEMPERATURE):
        sens = await sensor.new_sensor(internal_temperature_config)
        cg.add(var.set_internal_temperature_sensor(sens))
    if external_temperature_config := config.get(CONF_EXTERNAL_TEMPERATURE):
        sens = await sensor.new_sensor(external_temperature_config)
        cg.add(var.set_external_temperature_sensor(sens))

    # Calibrate based on schematics
    if (vref := config.get(CONF_REFERENCE_VOLTAGE, None)) is not None:
        cg.add(var.set_reference_voltage(vref))
    if (rl := config.get(CONF_RESISTOR_SHUNT, None)) is not None:
        cg.add(var.set_resistor_shunt(rl))
    if (rOne := config.get(CONF_RESISTOR_ONE, None)) is not None:
        cg.add(var.set_resistor_one(rOne))
    if (rTwo := config.get(CONF_RESISTOR_TWO, None)) is not None:
        cg.add(var.set_resistor_one(rTwo))

    # Calibrate using measured values
    if (current_reference := config.get(CONF_CURRENT_REFERENCE, None)) is not None:
        cg.add(var.set_current_reference(current_reference))
    if (voltage_reference := config.get(CONF_VOLTAGE_REFERENCE, None)) is not None:
        cg.add(var.set_voltage_reference(voltage_reference))
    if (power_reference := config.get(CONF_POWER_REFERENCE, None)) is not None:
        cg.add(var.set_power_reference(power_reference))
    if (energy_reference := config.get(CONF_ENERGY_REFERENCE, None)) is not None:
        cg.add(var.set_energy_reference(energy_reference))