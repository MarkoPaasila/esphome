"""HP-EKF external component for ESPHome."""

import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ID
from esphome.components import sensor

DEPENDENCIES = ["sensor"]

hp_ekf_ns = cg.esphome_ns.namespace("hp_ekf")
HpEkfComponent = hp_ekf_ns.class_("HpEkfComponent", cg.PollingComponent)

CONF_HP_EKF = "hp_ekf"
CONF_UPDATE_INTERVAL = "update_interval"
CONF_PRESSURE_PA = "pressure_pa"
CONF_USE_INLET_TEMP = "use_inlet_temp"
CONF_USE_INLET_RH = "use_inlet_rh"
CONF_USE_OUTLET_TEMP = "use_outlet_temp"
CONF_USE_OUTLET_RH = "use_outlet_rh"
CONF_INLET_TEMP_SENSOR = "inlet_temperature_id"
CONF_INLET_RH_SENSOR = "inlet_rh_id"
CONF_OUTLET_TEMP_SENSOR = "outlet_temperature_id"
CONF_OUTLET_RH_SENSOR = "outlet_rh_id"

CONF_INITIAL_STATE = "initial_state"
CONF_TIN_C = "tin_c"
CONF_TOUT_C = "tout_c"
CONF_AHIN_GM3 = "ahin_gm3"
CONF_AHOUT_GM3 = "ahout_gm3"
CONF_TIN_RATE_CPS = "tin_rate_cps"
CONF_TOUT_RATE_CPS = "tout_rate_cps"

CONF_INITIAL_COVARIANCE = "initial_covariance"
CONF_P_TIN = "tin"
CONF_P_TOUT = "tout"
CONF_P_AHIN = "ahin"
CONF_P_AHOUT = "ahout"
CONF_P_TIN_RATE = "tin_rate"
CONF_P_TOUT_RATE = "tout_rate"

CONF_PROCESS_NOISE = "process_noise"
CONF_Q_ACC_IN = "q_acc_in"
CONF_Q_ACC_OUT = "q_acc_out"
CONF_Q_AHIN = "q_ahin"
CONF_Q_AHOUT = "q_ahout"

CONF_MEASUREMENT_NOISE = "measurement_noise"
CONF_R_TIN = "tin"
CONF_R_TOUT = "tout"
CONF_R_RHIN = "rhin"
CONF_R_RHOUT = "rhout"

CONF_GATING = "gating"
CONF_TEMP_D2 = "temp_d2"
CONF_RH_D2 = "rh_d2"

CONF_CLAMPS = "clamps"
CONF_TEMP_C = "temp_c"
CONF_RATE_CPS = "rate_cps"
CONF_MIN = "min"
CONF_MAX = "max"

INITIAL_STATE_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_TIN_C): cv.float_,
        cv.Optional(CONF_TOUT_C): cv.float_,
        cv.Optional(CONF_AHIN_GM3): cv.float_,
        cv.Optional(CONF_AHOUT_GM3): cv.float_,
        cv.Optional(CONF_TIN_RATE_CPS): cv.float_,
        cv.Optional(CONF_TOUT_RATE_CPS): cv.float_,
    }
)

INITIAL_COVARIANCE_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_P_TIN): cv.positive_float,
        cv.Optional(CONF_P_TOUT): cv.positive_float,
        cv.Optional(CONF_P_AHIN): cv.positive_float,
        cv.Optional(CONF_P_AHOUT): cv.positive_float,
        cv.Optional(CONF_P_TIN_RATE): cv.positive_float,
        cv.Optional(CONF_P_TOUT_RATE): cv.positive_float,
    }
)

PROCESS_NOISE_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_Q_ACC_IN, default=0.02): cv.positive_float,
        cv.Optional(CONF_Q_ACC_OUT, default=0.02): cv.positive_float,
        cv.Optional(CONF_Q_AHIN, default=0.02): cv.positive_float,
        cv.Optional(CONF_Q_AHOUT, default=0.02): cv.positive_float,
    }
)

MEASUREMENT_NOISE_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_R_TIN, default=0.04): cv.positive_float,
        cv.Optional(CONF_R_TOUT, default=0.04): cv.positive_float,
        cv.Optional(CONF_R_RHIN, default=0.0004): cv.positive_float,
        cv.Optional(CONF_R_RHOUT, default=0.0004): cv.positive_float,
    }
)

GATING_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_TEMP_D2, default=6.63): cv.positive_float,
        cv.Optional(CONF_RH_D2, default=6.63): cv.positive_float,
    }
)

CLAMPS_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_TEMP_C): cv.Schema(
            {
                cv.Optional(CONF_MIN, default=-40.0): cv.float_,
                cv.Optional(CONF_MAX, default=100.0): cv.float_,
            }
        ),
        cv.Optional(CONF_RATE_CPS): cv.Schema(
            {
                cv.Optional(CONF_MIN, default=-1.0): cv.float_,
                cv.Optional(CONF_MAX, default=1.0): cv.float_,
            }
        ),
    }
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(HpEkfComponent),
            cv.Optional(CONF_PRESSURE_PA, default=101325.0): cv.positive_float,
            cv.Optional(CONF_USE_INLET_TEMP, default=True): cv.boolean,
            cv.Optional(CONF_USE_INLET_RH, default=True): cv.boolean,
            cv.Optional(CONF_USE_OUTLET_TEMP, default=True): cv.boolean,
            cv.Optional(CONF_USE_OUTLET_RH, default=True): cv.boolean,
            cv.Optional(CONF_INLET_TEMP_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_INLET_RH_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_OUTLET_TEMP_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_OUTLET_RH_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_INITIAL_STATE): INITIAL_STATE_SCHEMA,
            cv.Optional(CONF_INITIAL_COVARIANCE): INITIAL_COVARIANCE_SCHEMA,
            cv.Optional(CONF_PROCESS_NOISE): PROCESS_NOISE_SCHEMA,
            cv.Optional(CONF_MEASUREMENT_NOISE): MEASUREMENT_NOISE_SCHEMA,
            cv.Optional(CONF_GATING): GATING_SCHEMA,
            cv.Optional(CONF_CLAMPS): CLAMPS_SCHEMA,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(cv.polling_component_schema("500ms"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_UPDATE_INTERVAL in config:
        cg.add(var.set_update_interval_ms(config[CONF_UPDATE_INTERVAL]))
    cg.add(var.set_pressure_pa(config[CONF_PRESSURE_PA]))
    cg.add(var.set_use_inlet_temp(config[CONF_USE_INLET_TEMP]))
    cg.add(var.set_use_inlet_rh(config[CONF_USE_INLET_RH]))
    cg.add(var.set_use_outlet_temp(config[CONF_USE_OUTLET_TEMP]))
    cg.add(var.set_use_outlet_rh(config[CONF_USE_OUTLET_RH]))

    if CONF_INLET_TEMP_SENSOR in config:
        sens = await cg.get_variable(config[CONF_INLET_TEMP_SENSOR])
        cg.add(var.set_inlet_temp_sensor(sens))
    if CONF_INLET_RH_SENSOR in config:
        sens = await cg.get_variable(config[CONF_INLET_RH_SENSOR])
        cg.add(var.set_inlet_rh_sensor(sens))
    if CONF_OUTLET_TEMP_SENSOR in config:
        sens = await cg.get_variable(config[CONF_OUTLET_TEMP_SENSOR])
        cg.add(var.set_outlet_temp_sensor(sens))
    if CONF_OUTLET_RH_SENSOR in config:
        sens = await cg.get_variable(config[CONF_OUTLET_RH_SENSOR])
        cg.add(var.set_outlet_rh_sensor(sens))

    if CONF_INITIAL_STATE in config:
        is_c = config[CONF_INITIAL_STATE]
        cg.add(
            var.set_initial_state(
                is_c.get(CONF_TIN_C, 20.0),
                is_c.get(CONF_TOUT_C, 20.0),
                is_c.get(CONF_AHIN_GM3, 8.0),
                is_c.get(CONF_AHOUT_GM3, 8.0),
                is_c.get(CONF_TIN_RATE_CPS, 0.0),
                is_c.get(CONF_TOUT_RATE_CPS, 0.0),
            )
        )

    if CONF_INITIAL_COVARIANCE in config:
        ic = config[CONF_INITIAL_COVARIANCE]
        cg.add(
            var.set_initial_covariance_diagonal(
                ic.get(CONF_P_TIN, 4.0),
                ic.get(CONF_P_TOUT, 4.0),
                ic.get(CONF_P_AHIN, 4.0),
                ic.get(CONF_P_AHOUT, 4.0),
                ic.get(CONF_P_TIN_RATE, 0.25),
                ic.get(CONF_P_TOUT_RATE, 0.25),
            )
        )

    if CONF_PROCESS_NOISE in config:
        pn = config[CONF_PROCESS_NOISE]
        cg.add(
            var.set_Q_temp_accel(pn[CONF_Q_ACC_IN], pn[CONF_Q_ACC_OUT])
        )
        cg.add(var.set_Q_ah(pn[CONF_Q_AHIN], pn[CONF_Q_AHOUT]))

    if CONF_MEASUREMENT_NOISE in config:
        mn = config[CONF_MEASUREMENT_NOISE]
        cg.add(var.set_R_Tin(mn[CONF_R_TIN]))
        cg.add(var.set_R_Tout(mn[CONF_R_TOUT]))
        cg.add(var.set_R_RHin(mn[CONF_R_RHIN]))
        cg.add(var.set_R_RHout(mn[CONF_R_RHOUT]))

    if CONF_GATING in config:
        g = config[CONF_GATING]
        cg.add(var.set_gate_thresholds(g[CONF_TEMP_D2], g[CONF_RH_D2]))

    if CONF_CLAMPS in config:
        cl = config[CONF_CLAMPS]
        if CONF_TEMP_C in cl:
            cg.add(var.set_temp_bounds(cl[CONF_TEMP_C][CONF_MIN], cl[CONF_TEMP_C][CONF_MAX]))
        if CONF_RATE_CPS in cl:
            cg.add(var.set_rate_bounds(cl[CONF_RATE_CPS][CONF_MIN], cl[CONF_RATE_CPS][CONF_MAX]))
