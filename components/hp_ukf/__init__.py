"""HP-UKF external component for ESPHome."""

import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ID
from esphome.components import sensor

DEPENDENCIES = ["sensor"]

hp_ukf_ns = cg.esphome_ns.namespace("hp_ukf")
HpUkfComponent = hp_ukf_ns.class_("HpUkfComponent", cg.PollingComponent)

CONF_HP_UKF = "hp_ukf"
CONF_UPDATE_INTERVAL = "update_interval"
CONF_INLET_TEMPERATURE = "inlet_temperature"
CONF_INLET_HUMIDITY = "inlet_humidity"
CONF_OUTLET_TEMPERATURE = "outlet_temperature"
CONF_OUTLET_HUMIDITY = "outlet_humidity"
CONF_TRACK_TEMPERATURE_DERIVATIVES = "track_temperature_derivatives"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(HpUkfComponent),
        cv.Optional(CONF_UPDATE_INTERVAL, default="1s"): cv.update_interval,
        cv.Optional(CONF_INLET_TEMPERATURE): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_INLET_HUMIDITY): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_OUTLET_TEMPERATURE): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_OUTLET_HUMIDITY): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_TRACK_TEMPERATURE_DERIVATIVES, default=True): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
    cg.add(var.set_track_temperature_derivatives(config[CONF_TRACK_TEMPERATURE_DERIVATIVES]))
    if CONF_INLET_TEMPERATURE in config:
        sens = await cg.get_variable(config[CONF_INLET_TEMPERATURE])
        cg.add(var.set_inlet_temperature_sensor(sens))
    if CONF_INLET_HUMIDITY in config:
        sens = await cg.get_variable(config[CONF_INLET_HUMIDITY])
        cg.add(var.set_inlet_humidity_sensor(sens))
    if CONF_OUTLET_TEMPERATURE in config:
        sens = await cg.get_variable(config[CONF_OUTLET_TEMPERATURE])
        cg.add(var.set_outlet_temperature_sensor(sens))
    if CONF_OUTLET_HUMIDITY in config:
        sens = await cg.get_variable(config[CONF_OUTLET_HUMIDITY])
        cg.add(var.set_outlet_humidity_sensor(sens))
