"""HP-UKF external component for ESPHome."""

import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_PERCENT,
)
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
CONF_FILTERED_INLET_TEMPERATURE = "filtered_inlet_temperature"
CONF_FILTERED_INLET_HUMIDITY = "filtered_inlet_humidity"
CONF_FILTERED_OUTLET_TEMPERATURE = "filtered_outlet_temperature"
CONF_FILTERED_OUTLET_HUMIDITY = "filtered_outlet_humidity"
CONF_FILTERED_INLET_TEMPERATURE_DERIVATIVE = "filtered_inlet_temperature_derivative"
CONF_FILTERED_OUTLET_TEMPERATURE_DERIVATIVE = "filtered_outlet_temperature_derivative"
CONF_FILTERED_INLET_HUMIDITY_DERIVATIVE = "filtered_inlet_humidity_derivative"
CONF_FILTERED_OUTLET_HUMIDITY_DERIVATIVE = "filtered_outlet_humidity_derivative"
CONF_EM_AUTOTUNE = "em_autotune"
CONF_EM_LAMBDA_Q = "em_lambda_q"
CONF_EM_LAMBDA_R_INLET = "em_lambda_r_inlet"
CONF_EM_LAMBDA_R_OUTLET = "em_lambda_r_outlet"
CONF_EM_Q_T_IN = "em_q_t_in"
CONF_EM_Q_RH_IN = "em_q_rh_in"
CONF_EM_Q_T_OUT = "em_q_t_out"
CONF_EM_Q_RH_OUT = "em_q_rh_out"
CONF_EM_Q_DT_IN = "em_q_dt_in"
CONF_EM_Q_DT_OUT = "em_q_dt_out"
CONF_EM_Q_DRH_IN = "em_q_drh_in"
CONF_EM_Q_DRH_OUT = "em_q_drh_out"
CONF_EM_R_T_IN = "em_r_t_in"
CONF_EM_R_RH_IN = "em_r_rh_in"
CONF_EM_R_T_OUT = "em_r_t_out"
CONF_EM_R_RH_OUT = "em_r_rh_out"
CONF_EM_LAMBDA_Q_SENSOR = "em_lambda_q_sensor"
CONF_EM_LAMBDA_R_INLET_SENSOR = "em_lambda_r_inlet_sensor"
CONF_EM_LAMBDA_R_OUTLET_SENSOR = "em_lambda_r_outlet_sensor"


def _em_lambda(value):
    v = float(value)
    if v <= 0 or v > 1:
        raise cv.Invalid("lambda must be in (0, 1]")
    return v


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(HpUkfComponent),
        cv.Optional(CONF_UPDATE_INTERVAL, default="1s"): cv.update_interval,
        cv.Optional(CONF_INLET_TEMPERATURE): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_INLET_HUMIDITY): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_OUTLET_TEMPERATURE): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_OUTLET_HUMIDITY): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_TRACK_TEMPERATURE_DERIVATIVES, default=True): cv.boolean,
        cv.Optional(
            CONF_FILTERED_INLET_TEMPERATURE,
            default={CONF_NAME: "Filtered Inlet Temperature"},
        ): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(
            CONF_FILTERED_INLET_HUMIDITY,
            default={CONF_NAME: "Filtered Inlet Humidity"},
        ): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(
            CONF_FILTERED_OUTLET_TEMPERATURE,
            default={CONF_NAME: "Filtered Outlet Temperature"},
        ): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(
            CONF_FILTERED_OUTLET_HUMIDITY,
            default={CONF_NAME: "Filtered Outlet Humidity"},
        ): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(
            CONF_FILTERED_INLET_TEMPERATURE_DERIVATIVE,
            default={CONF_NAME: "Filtered Inlet Temperature Derivative"},
        ): sensor.sensor_schema(
            unit_of_measurement="°C/s",
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(
            CONF_FILTERED_OUTLET_TEMPERATURE_DERIVATIVE,
            default={CONF_NAME: "Filtered Outlet Temperature Derivative"},
        ): sensor.sensor_schema(
            unit_of_measurement="°C/s",
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(
            CONF_FILTERED_INLET_HUMIDITY_DERIVATIVE,
            default={CONF_NAME: "Filtered Inlet Humidity Derivative"},
        ): sensor.sensor_schema(
            unit_of_measurement="%/s",
            accuracy_decimals=4,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(
            CONF_FILTERED_OUTLET_HUMIDITY_DERIVATIVE,
            default={CONF_NAME: "Filtered Outlet Humidity Derivative"},
        ): sensor.sensor_schema(
            unit_of_measurement="%/s",
            accuracy_decimals=4,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_EM_AUTOTUNE, default=False): cv.boolean,
        cv.Optional(CONF_EM_LAMBDA_Q, default=0.995): _em_lambda,
        cv.Optional(CONF_EM_LAMBDA_R_INLET, default=0.998): _em_lambda,
        cv.Optional(CONF_EM_LAMBDA_R_OUTLET, default=0.98): _em_lambda,
        cv.Optional(CONF_EM_Q_T_IN): sensor.sensor_schema(
            unit_of_measurement="°C²",
            accuracy_decimals=6,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_EM_Q_RH_IN): sensor.sensor_schema(
            unit_of_measurement="%²",
            accuracy_decimals=6,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_EM_Q_T_OUT): sensor.sensor_schema(
            unit_of_measurement="°C²",
            accuracy_decimals=6,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_EM_Q_RH_OUT): sensor.sensor_schema(
            unit_of_measurement="%²",
            accuracy_decimals=6,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_EM_Q_DT_IN): sensor.sensor_schema(
            unit_of_measurement="(°C/s)²",
            accuracy_decimals=6,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_EM_Q_DT_OUT): sensor.sensor_schema(
            unit_of_measurement="(°C/s)²",
            accuracy_decimals=6,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_EM_Q_DRH_IN): sensor.sensor_schema(
            unit_of_measurement="(%/s)²",
            accuracy_decimals=6,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_EM_Q_DRH_OUT): sensor.sensor_schema(
            unit_of_measurement="(%/s)²",
            accuracy_decimals=6,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_EM_R_T_IN): sensor.sensor_schema(
            unit_of_measurement="°C²",
            accuracy_decimals=6,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_EM_R_RH_IN): sensor.sensor_schema(
            unit_of_measurement="%²",
            accuracy_decimals=6,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_EM_R_T_OUT): sensor.sensor_schema(
            unit_of_measurement="°C²",
            accuracy_decimals=6,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_EM_R_RH_OUT): sensor.sensor_schema(
            unit_of_measurement="%²",
            accuracy_decimals=6,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_EM_LAMBDA_Q_SENSOR): sensor.sensor_schema(
            accuracy_decimals=3,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_EM_LAMBDA_R_INLET_SENSOR): sensor.sensor_schema(
            accuracy_decimals=3,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_EM_LAMBDA_R_OUTLET_SENSOR): sensor.sensor_schema(
            accuracy_decimals=3,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
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

    cg.add(var.set_em_autotune(config[CONF_EM_AUTOTUNE]))
    cg.add(var.set_em_lambda_q(config[CONF_EM_LAMBDA_Q]))
    cg.add(var.set_em_lambda_r_inlet(config[CONF_EM_LAMBDA_R_INLET]))
    cg.add(var.set_em_lambda_r_outlet(config[CONF_EM_LAMBDA_R_OUTLET]))

    sens = await sensor.new_sensor(config[CONF_FILTERED_INLET_TEMPERATURE])
    cg.add(var.set_filtered_inlet_temperature_sensor(sens))
    sens = await sensor.new_sensor(config[CONF_FILTERED_INLET_HUMIDITY])
    cg.add(var.set_filtered_inlet_humidity_sensor(sens))
    sens = await sensor.new_sensor(config[CONF_FILTERED_OUTLET_TEMPERATURE])
    cg.add(var.set_filtered_outlet_temperature_sensor(sens))
    sens = await sensor.new_sensor(config[CONF_FILTERED_OUTLET_HUMIDITY])
    cg.add(var.set_filtered_outlet_humidity_sensor(sens))
    sens = await sensor.new_sensor(config[CONF_FILTERED_INLET_TEMPERATURE_DERIVATIVE])
    cg.add(var.set_filtered_inlet_temperature_derivative_sensor(sens))
    sens = await sensor.new_sensor(config[CONF_FILTERED_OUTLET_TEMPERATURE_DERIVATIVE])
    cg.add(var.set_filtered_outlet_temperature_derivative_sensor(sens))
    sens = await sensor.new_sensor(config[CONF_FILTERED_INLET_HUMIDITY_DERIVATIVE])
    cg.add(var.set_filtered_inlet_humidity_derivative_sensor(sens))
    sens = await sensor.new_sensor(config[CONF_FILTERED_OUTLET_HUMIDITY_DERIVATIVE])
    cg.add(var.set_filtered_outlet_humidity_derivative_sensor(sens))

    if CONF_EM_Q_T_IN in config:
        sens = await sensor.new_sensor(config[CONF_EM_Q_T_IN])
        cg.add(var.set_em_q_t_in_sensor(sens))
    if CONF_EM_Q_RH_IN in config:
        sens = await sensor.new_sensor(config[CONF_EM_Q_RH_IN])
        cg.add(var.set_em_q_rh_in_sensor(sens))
    if CONF_EM_Q_T_OUT in config:
        sens = await sensor.new_sensor(config[CONF_EM_Q_T_OUT])
        cg.add(var.set_em_q_t_out_sensor(sens))
    if CONF_EM_Q_RH_OUT in config:
        sens = await sensor.new_sensor(config[CONF_EM_Q_RH_OUT])
        cg.add(var.set_em_q_rh_out_sensor(sens))
    if CONF_EM_Q_DT_IN in config:
        sens = await sensor.new_sensor(config[CONF_EM_Q_DT_IN])
        cg.add(var.set_em_q_dt_in_sensor(sens))
    if CONF_EM_Q_DT_OUT in config:
        sens = await sensor.new_sensor(config[CONF_EM_Q_DT_OUT])
        cg.add(var.set_em_q_dt_out_sensor(sens))
    if CONF_EM_Q_DRH_IN in config:
        sens = await sensor.new_sensor(config[CONF_EM_Q_DRH_IN])
        cg.add(var.set_em_q_drh_in_sensor(sens))
    if CONF_EM_Q_DRH_OUT in config:
        sens = await sensor.new_sensor(config[CONF_EM_Q_DRH_OUT])
        cg.add(var.set_em_q_drh_out_sensor(sens))
    if CONF_EM_R_T_IN in config:
        sens = await sensor.new_sensor(config[CONF_EM_R_T_IN])
        cg.add(var.set_em_r_t_in_sensor(sens))
    if CONF_EM_R_RH_IN in config:
        sens = await sensor.new_sensor(config[CONF_EM_R_RH_IN])
        cg.add(var.set_em_r_rh_in_sensor(sens))
    if CONF_EM_R_T_OUT in config:
        sens = await sensor.new_sensor(config[CONF_EM_R_T_OUT])
        cg.add(var.set_em_r_t_out_sensor(sens))
    if CONF_EM_R_RH_OUT in config:
        sens = await sensor.new_sensor(config[CONF_EM_R_RH_OUT])
        cg.add(var.set_em_r_rh_out_sensor(sens))
    if CONF_EM_LAMBDA_Q_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_EM_LAMBDA_Q_SENSOR])
        cg.add(var.set_em_lambda_q_sensor(sens))
    if CONF_EM_LAMBDA_R_INLET_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_EM_LAMBDA_R_INLET_SENSOR])
        cg.add(var.set_em_lambda_r_inlet_sensor(sens))
    if CONF_EM_LAMBDA_R_OUTLET_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_EM_LAMBDA_R_OUTLET_SENSOR])
        cg.add(var.set_em_lambda_r_outlet_sensor(sens))
