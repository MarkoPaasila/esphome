"""HP-UKF external component for ESPHome."""

import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ID

DEPENDENCIES = []

hp_ukf_ns = cg.esphome_ns.namespace("hp_ukf")
HpUkfComponent = hp_ukf_ns.class_("HpUkfComponent", cg.Component)

CONF_HP_UKF = "hp_ukf"
CONF_UPDATE_INTERVAL = "update_interval"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(HpUkfComponent),
        cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.update_interval,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
