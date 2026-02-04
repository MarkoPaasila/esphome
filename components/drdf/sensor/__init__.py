"""DRDF sensor platform: sensor that filters an input sensor with DRDF and optional diagnostic sensors."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ALPHA, CONF_ID, CONF_NAME

from .. import (
    CONF_DEADBAND_MULTIPLIER,
    CONF_INPUT_SENSOR_ID,
    CONF_LOWER_BOUND,
    CONF_UPPER_BOUND,
    DrdfSensor,
)

DEPENDENCIES = ["drdf", "sensor"]

CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(DrdfSensor)
    .extend(
        {
            cv.Required(CONF_INPUT_SENSOR_ID): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_ALPHA, default=0.01): cv.positive_float,
            cv.Optional(CONF_DEADBAND_MULTIPLIER, default=3.82): cv.positive_float,
            cv.Optional(CONF_UPPER_BOUND): sensor.sensor_schema(),
            cv.Optional(CONF_LOWER_BOUND): sensor.sensor_schema(),
        }
    ),
    cv.has_at_least_one_key(CONF_ID, CONF_NAME),
)


async def to_code(config):
    var = cg.new_Pvariable(
        config[CONF_ID],
        config.get(CONF_ALPHA, 0.01),
        config.get(CONF_DEADBAND_MULTIPLIER, 3.82),
    )
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)

    input_sensor = await cg.get_variable(config[CONF_INPUT_SENSOR_ID])
    cg.add(var.set_input_sensor(input_sensor))

    if CONF_UPPER_BOUND in config:
        ub_sens = await sensor.new_sensor(config[CONF_UPPER_BOUND])
        cg.add(var.set_upper_bound_sensor(ub_sens))
    if CONF_LOWER_BOUND in config:
        lb_sens = await sensor.new_sensor(config[CONF_LOWER_BOUND])
        cg.add(var.set_lower_bound_sensor(lb_sens))
