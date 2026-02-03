# Copyright (c) 2021 Veli Matti Lastumäki (Velsku at lampopumput.info),
# Joonas Ihonen (puu at lampopumput.info),
# Ilkka Roivainen (iro at lampopumput.info)
#
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# ESPHome external component - MitsuRunner defrost controller

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, button, sensor, switch, text_sensor
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

CODEOWNERS = ["@lampopumput"]
DEPENDENCIES = ["sensor", "switch", "text_sensor", "binary_sensor", "button"]

CONF_HEAT_EXCHANGER_TEMPERATURE_SENSOR = "heat_exchanger_temperature_sensor_id"
CONF_OUTDOOR_TEMPERATURE_SENSOR = "outdoor_temperature_sensor_id"
CONF_DEFROST_RELAY = "defrost_relay_id"
CONF_SMART_DEFROST_LOGIC_STATE = "smart_defrost_logic_state"
CONF_DEFROST_ALLOWED_SENSOR = "defrost_allowed_sensor"
CONF_TEMPERATURE_DELTA_SENSOR = "temperature_delta_sensor"
CONF_SMART_DEFROST_LOGIC = "smart_defrost_logic"
CONF_DEFROST_NOW = "defrost_now"

# Optional timing/threshold constants (from constants.h)
CONF_TEMPERATURE_DELTA_TO_DEFROST = "temperature_delta_to_defrost"
CONF_OUTDOOR_TEMPERATURE_TO_ENTER_OFF_STATE = "outdoor_temperature_to_enter_off_state"
CONF_OUTDOOR_TEMPERATURE_TO_EXIT_OFF_STATE = "outdoor_temperature_to_exit_off_state"
CONF_HEAT_EXCHANGER_MAX_TEMPERATURE = "heat_exchanger_max_temperature"
CONF_TEMPERATURE_DELTA_DEFROSTING_STARTED = "temperature_delta_defrosting_started"
CONF_TEMPERATURE_DELTA_EXCESS_TIME_MIN = "temperature_delta_excess_time_min"
CONF_TEMPERATURE_DELTA_DECREASING_EXCESS_TIME_MIN = "temperature_delta_decreasing_excess_time_min"
CONF_MAX_HEATING_TIME_MIN = "max_heating_time_min"
CONF_MIN_HEATING_TIME_MIN = "min_heating_time_min"
CONF_DEFROST_DURATION_MIN = "defrost_duration_min"
CONF_DEFROST_TIMEOUT_MIN = "defrost_timeout_min"
CONF_RESET_SENSOR_DELAY_SEC = "reset_sensor_delay_sec"
CONF_INITIALIZE_DELAY_SEC = "initialize_delay_sec"

mitsurunner_ns = cg.esphome_ns.namespace("mitsurunner")
MitsurunnerComponent = mitsurunner_ns.class_("MitsurunnerComponent", cg.Component, cg.PollingComponent)
MitsurunnerDefrostAllowedSwitch = mitsurunner_ns.class_(
    "MitsurunnerDefrostAllowedSwitch", switch.Switch
)
MitsurunnerDefrostNowButton = mitsurunner_ns.class_(
    "MitsurunnerDefrostNowButton", button.Button
)


def validate_constants(config):
    min_heating = config[CONF_MIN_HEATING_TIME_MIN]
    defrost_duration = config[CONF_DEFROST_DURATION_MIN]
    max_heating = config[CONF_MAX_HEATING_TIME_MIN]
    if min_heating < defrost_duration:
        raise cv.Invalid(
            f"min_heating_time_min ({min_heating}) must be >= defrost_duration_min ({defrost_duration})"
        )
    if max_heating < min_heating:
        raise cv.Invalid(
            f"max_heating_time_min ({max_heating}) must be >= min_heating_time_min ({min_heating})"
        )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MitsurunnerComponent),
            cv.Required(CONF_HEAT_EXCHANGER_TEMPERATURE_SENSOR): cv.use_id(sensor.Sensor),
            cv.Required(CONF_OUTDOOR_TEMPERATURE_SENSOR): cv.use_id(sensor.Sensor),
            cv.Required(CONF_DEFROST_RELAY): cv.use_id(switch.Switch),
            cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.update_interval,
            cv.Optional(CONF_SMART_DEFROST_LOGIC_STATE): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_DEFROST_ALLOWED_SENSOR): binary_sensor.binary_sensor_schema(),
            cv.Optional(CONF_TEMPERATURE_DELTA_SENSOR): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
                device_class=DEVICE_CLASS_TEMPERATURE,
            ),
            cv.Optional(CONF_SMART_DEFROST_LOGIC): switch.switch_schema(
                MitsurunnerDefrostAllowedSwitch
            ),
            cv.Optional(CONF_DEFROST_NOW): button.button_schema(
                MitsurunnerDefrostNowButton
            ),
            cv.Optional(CONF_TEMPERATURE_DELTA_TO_DEFROST, default=-5.0): cv.float_,
            cv.Optional(CONF_OUTDOOR_TEMPERATURE_TO_ENTER_OFF_STATE, default=3.0): cv.float_,
            cv.Optional(CONF_OUTDOOR_TEMPERATURE_TO_EXIT_OFF_STATE, default=2.0): cv.float_,
            cv.Optional(CONF_HEAT_EXCHANGER_MAX_TEMPERATURE, default=10.0): cv.float_,
            cv.Optional(CONF_TEMPERATURE_DELTA_DEFROSTING_STARTED, default=1.0): cv.float_,
            cv.Optional(CONF_TEMPERATURE_DELTA_EXCESS_TIME_MIN, default=8): cv.positive_int,
            cv.Optional(CONF_TEMPERATURE_DELTA_DECREASING_EXCESS_TIME_MIN, default=5): cv.positive_int,
            cv.Optional(CONF_MAX_HEATING_TIME_MIN, default=180): cv.positive_int,
            cv.Optional(CONF_MIN_HEATING_TIME_MIN, default=50): cv.positive_int,
            cv.Optional(CONF_DEFROST_DURATION_MIN, default=30): cv.positive_int,
            cv.Optional(CONF_DEFROST_TIMEOUT_MIN, default=10): cv.positive_int,
            cv.Optional(CONF_RESET_SENSOR_DELAY_SEC, default=25): cv.positive_int,
            cv.Optional(CONF_INITIALIZE_DELAY_SEC, default=60): cv.positive_int,
        }
    ).extend(cv.COMPONENT_SCHEMA).extend(cv.polling_component_schema("60s")),
    validate_constants,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))

    heat_exchanger = await cg.get_variable(config[CONF_HEAT_EXCHANGER_TEMPERATURE_SENSOR])
    outdoor = await cg.get_variable(config[CONF_OUTDOOR_TEMPERATURE_SENSOR])
    relay = await cg.get_variable(config[CONF_DEFROST_RELAY])
    cg.add(var.set_heat_exchanger_sensor(heat_exchanger))
    cg.add(var.set_outdoor_sensor(outdoor))
    cg.add(var.set_allow_defrost_relay(relay))

    cg.add(var.set_temperature_delta_to_defrost(config[CONF_TEMPERATURE_DELTA_TO_DEFROST]))
    cg.add(var.set_outdoor_temperature_to_enter_off_state(config[CONF_OUTDOOR_TEMPERATURE_TO_ENTER_OFF_STATE]))
    cg.add(var.set_outdoor_temperature_to_exit_off_state(config[CONF_OUTDOOR_TEMPERATURE_TO_EXIT_OFF_STATE]))
    cg.add(var.set_heat_exchanger_max_temperature(config[CONF_HEAT_EXCHANGER_MAX_TEMPERATURE]))
    cg.add(var.set_temperature_delta_defrosting_started(config[CONF_TEMPERATURE_DELTA_DEFROSTING_STARTED]))
    cg.add(var.set_temperature_delta_excess_time_min(config[CONF_TEMPERATURE_DELTA_EXCESS_TIME_MIN]))
    cg.add(var.set_temperature_delta_decreasing_excess_time_min(config[CONF_TEMPERATURE_DELTA_DECREASING_EXCESS_TIME_MIN]))
    cg.add(var.set_max_heating_time_min(config[CONF_MAX_HEATING_TIME_MIN]))
    cg.add(var.set_min_heating_time_min(config[CONF_MIN_HEATING_TIME_MIN]))
    cg.add(var.set_defrost_duration_min(config[CONF_DEFROST_DURATION_MIN]))
    cg.add(var.set_defrost_timeout_min(config[CONF_DEFROST_TIMEOUT_MIN]))
    cg.add(var.set_reset_sensor_delay_sec(config[CONF_RESET_SENSOR_DELAY_SEC]))
    cg.add(var.set_initialize_delay_sec(config[CONF_INITIALIZE_DELAY_SEC]))
    if CONF_SMART_DEFROST_LOGIC_STATE in config:
        state_conf = dict(config[CONF_SMART_DEFROST_LOGIC_STATE])
        if CONF_NAME not in state_conf or state_conf[CONF_NAME] is None:
            state_conf[CONF_NAME] = "Smart Defrost Logic State"
        state_ts = await text_sensor.new_text_sensor(state_conf)
        cg.add(var.set_state_text_sensor(state_ts))
    if CONF_DEFROST_ALLOWED_SENSOR in config:
        defrost_bs = await binary_sensor.new_binary_sensor(
            config[CONF_DEFROST_ALLOWED_SENSOR]
        )
        cg.add(var.set_defrost_allowed_sensor(defrost_bs))
    if CONF_TEMPERATURE_DELTA_SENSOR in config:
        tds = await sensor.new_sensor(config[CONF_TEMPERATURE_DELTA_SENSOR])
        cg.add(var.set_temperature_delta_sensor(tds))
    if CONF_SMART_DEFROST_LOGIC in config:
        defrost_sw = await switch.new_switch(config[CONF_SMART_DEFROST_LOGIC])
        cg.add(defrost_sw.set_component(var))
        cg.add(var.set_defrost_allowed_switch(defrost_sw))
    if CONF_DEFROST_NOW in config:
        defrost_now_conf = dict(config[CONF_DEFROST_NOW])
        if CONF_NAME not in defrost_now_conf or defrost_now_conf[CONF_NAME] is None:
            defrost_now_conf[CONF_NAME] = "Defrost Now"
        defrost_now_btn = await button.new_button(defrost_now_conf)
        cg.add(defrost_now_btn.set_component(var))
