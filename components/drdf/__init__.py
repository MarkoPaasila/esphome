"""DRDF (Dynamic Reversals based Deadband Filter) sensor filter for ESPHome."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ALPHA, CONF_ID, CONF_NAME

DEPENDENCIES = ["sensor"]

# Optional top-level config: adding "drdf:" in YAML loads this component so the
# "drdf" filter is registered before sensor configs that use "filters: - drdf" are validated.
CONFIG_SCHEMA = cv.Schema({})

CONF_EMA_MULTIPLIER = "ema_multiplier"
CONF_INPUT_SENSOR_ID = "input_sensor_id"
CONF_DEADBAND_MULTIPLIER = "deadband_multiplier"
CONF_UPPER_BOUND = "upper_bound"
CONF_LOWER_BOUND = "lower_bound"

drdf_ns = cg.esphome_ns.namespace("drdf")
DrdfFilter = drdf_ns.class_("DrdfFilter", sensor.Filter)
DrdfSensor = drdf_ns.class_("DrdfSensor", cg.Component, sensor.Sensor)

DRDF_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_ALPHA, default=0.01): cv.positive_float,
        cv.Optional(CONF_EMA_MULTIPLIER, default=3.82): cv.positive_float,
    }
)


@sensor.FILTER_REGISTRY.register("drdf", DrdfFilter, DRDF_SCHEMA)
async def drdf_filter_to_code(config, filter_id):
    """Generate the DRDF filter C++ code."""
    var = cg.new_Pvariable(
        filter_id,
        config.get(CONF_ALPHA, 0.01),
        config.get(CONF_EMA_MULTIPLIER, 3.82),
    )
    return var


async def to_code(config):
    """No C++ for top-level drdf; only ensures component loads so the filter is registered."""
    pass
