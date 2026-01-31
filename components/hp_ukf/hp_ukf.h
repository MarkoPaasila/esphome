#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/climate/climate.h"
#include "hp_ukf_ukf.h"

namespace esphome {
namespace hp_ukf {

class HpUkfComponent : public PollingComponent {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;

  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_inlet_temperature_sensor(sensor::Sensor *s) { inlet_temperature_ = s; }
  void set_inlet_humidity_sensor(sensor::Sensor *s) { inlet_humidity_ = s; }
  void set_outlet_temperature_sensor(sensor::Sensor *s) { outlet_temperature_ = s; }
  void set_outlet_humidity_sensor(sensor::Sensor *s) { outlet_humidity_ = s; }
  void set_track_temperature_derivatives(bool v) { track_derivatives_ = v; }

  void set_filtered_inlet_temperature_sensor(sensor::Sensor *s) { filtered_inlet_temperature_ = s; }
  void set_filtered_inlet_humidity_sensor(sensor::Sensor *s) { filtered_inlet_humidity_ = s; }
  void set_filtered_outlet_temperature_sensor(sensor::Sensor *s) { filtered_outlet_temperature_ = s; }
  void set_filtered_outlet_humidity_sensor(sensor::Sensor *s) { filtered_outlet_humidity_ = s; }
  void set_filtered_inlet_temperature_derivative_sensor(sensor::Sensor *s) {
    filtered_inlet_temperature_derivative_ = s;
  }
  void set_filtered_outlet_temperature_derivative_sensor(sensor::Sensor *s) {
    filtered_outlet_temperature_derivative_ = s;
  }
  void set_filtered_inlet_humidity_derivative_sensor(sensor::Sensor *s) {
    filtered_inlet_humidity_derivative_ = s;
  }
  void set_filtered_outlet_humidity_derivative_sensor(sensor::Sensor *s) {
    filtered_outlet_humidity_derivative_ = s;
  }

  void set_em_autotune(bool v) { em_autotune_ = v; }
  void set_em_lambda_q(float v) { em_lambda_q_ = v; }
  void set_em_lambda_r_inlet(float v) { em_lambda_r_inlet_ = v; }
  void set_em_lambda_r_outlet(float v) { em_lambda_r_outlet_ = v; }
  void set_em_inflation(float v) { em_inflation_ = v; }

  void set_em_q_t_in_sensor(sensor::Sensor *s) { em_q_t_in_ = s; }
  void set_em_q_rh_in_sensor(sensor::Sensor *s) { em_q_rh_in_ = s; }
  void set_em_q_t_out_sensor(sensor::Sensor *s) { em_q_t_out_ = s; }
  void set_em_q_rh_out_sensor(sensor::Sensor *s) { em_q_rh_out_ = s; }
  void set_em_q_dt_in_sensor(sensor::Sensor *s) { em_q_dt_in_ = s; }
  void set_em_q_dt_out_sensor(sensor::Sensor *s) { em_q_dt_out_ = s; }
  void set_em_q_drh_in_sensor(sensor::Sensor *s) { em_q_drh_in_ = s; }
  void set_em_q_drh_out_sensor(sensor::Sensor *s) { em_q_drh_out_ = s; }
  void set_em_r_t_in_sensor(sensor::Sensor *s) { em_r_t_in_ = s; }
  void set_em_r_rh_in_sensor(sensor::Sensor *s) { em_r_rh_in_ = s; }
  void set_em_r_t_out_sensor(sensor::Sensor *s) { em_r_t_out_ = s; }
  void set_em_r_rh_out_sensor(sensor::Sensor *s) { em_r_rh_out_ = s; }
  void set_em_lambda_q_sensor(sensor::Sensor *s) { em_lambda_q_sensor_ = s; }
  void set_em_lambda_r_inlet_sensor(sensor::Sensor *s) { em_lambda_r_inlet_sensor_ = s; }
  void set_em_lambda_r_outlet_sensor(sensor::Sensor *s) { em_lambda_r_outlet_sensor_ = s; }

  void set_atmospheric_pressure(float pa) { pressure_pa_ = pa; }
  void set_delivered_power_lag_tau_s(float s) { delivered_power_lag_tau_s_ = s; }
  void set_inlet_absolute_humidity_sensor(sensor::Sensor *s) { inlet_absolute_humidity_ = s; }
  void set_inlet_dew_point_sensor(sensor::Sensor *s) { inlet_dew_point_ = s; }
  void set_inlet_enthalpy_sensor(sensor::Sensor *s) { inlet_enthalpy_ = s; }
  void set_inlet_humidity_ratio_sensor(sensor::Sensor *s) { inlet_humidity_ratio_ = s; }
  void set_outlet_absolute_humidity_sensor(sensor::Sensor *s) { outlet_absolute_humidity_ = s; }
  void set_outlet_dew_point_sensor(sensor::Sensor *s) { outlet_dew_point_ = s; }
  void set_outlet_enthalpy_sensor(sensor::Sensor *s) { outlet_enthalpy_ = s; }
  void set_outlet_humidity_ratio_sensor(sensor::Sensor *s) { outlet_humidity_ratio_ = s; }

  void set_air_flow_sensor(sensor::Sensor *s) { air_flow_ = s; }
  void set_filtered_air_flow_sensor(sensor::Sensor *s) { filtered_air_flow_ = s; }
  void set_delivered_power_sensor(sensor::Sensor *s) { delivered_power_ = s; }
  void set_delivered_power_lag_sensor(sensor::Sensor *s) { delivered_power_lag_ = s; }
  void set_virtual_coil(bool v) { virtual_coil_ = v; }
  void set_coil_tau_s(float s) { coil_tau_s_ = s; }
  void set_outlet_air_tau_s(float s) { outlet_air_tau_s_ = s; }
  void set_filtered_virtual_coil_temperature_sensor(sensor::Sensor *s) {
    filtered_virtual_coil_temperature_ = s;
  }

  void set_climate(climate::Climate *c) { climate_ = c; }
  void set_compressor_frequency_sensor(sensor::Sensor *s) { compressor_frequency_ = s; }
  void set_power_sensor(sensor::Sensor *s) { power_sensor_ = s; }
  void set_outside_temperature_sensor(sensor::Sensor *s) { outside_temperature_ = s; }
  void set_outside_coil_temperature_before_sensor(sensor::Sensor *s) {
    outside_coil_temperature_before_ = s;
  }
  void set_outside_coil_temperature_after_sensor(sensor::Sensor *s) {
    outside_coil_temperature_after_ = s;
  }
  void set_inside_room_temperature_sensor(sensor::Sensor *s) { inside_room_temperature_ = s; }
  void set_inside_room_humidity_sensor(sensor::Sensor *s) { inside_room_humidity_ = s; }

 protected:
  sensor::Sensor *inlet_temperature_{nullptr};
  sensor::Sensor *inlet_humidity_{nullptr};
  sensor::Sensor *outlet_temperature_{nullptr};
  sensor::Sensor *outlet_humidity_{nullptr};
  bool track_derivatives_{true};

  sensor::Sensor *filtered_inlet_temperature_{nullptr};
  sensor::Sensor *filtered_inlet_humidity_{nullptr};
  sensor::Sensor *filtered_outlet_temperature_{nullptr};
  sensor::Sensor *filtered_outlet_humidity_{nullptr};
  sensor::Sensor *filtered_inlet_temperature_derivative_{nullptr};
  sensor::Sensor *filtered_outlet_temperature_derivative_{nullptr};
  sensor::Sensor *filtered_inlet_humidity_derivative_{nullptr};
  sensor::Sensor *filtered_outlet_humidity_derivative_{nullptr};

  bool em_autotune_{false};
  float em_lambda_q_{0.995f};
  float em_lambda_r_inlet_{0.998f};
  float em_lambda_r_outlet_{0.98f};
  float em_inflation_{0.5f};

  sensor::Sensor *em_q_t_in_{nullptr};
  sensor::Sensor *em_q_rh_in_{nullptr};
  sensor::Sensor *em_q_t_out_{nullptr};
  sensor::Sensor *em_q_rh_out_{nullptr};
  sensor::Sensor *em_q_dt_in_{nullptr};
  sensor::Sensor *em_q_dt_out_{nullptr};
  sensor::Sensor *em_q_drh_in_{nullptr};
  sensor::Sensor *em_q_drh_out_{nullptr};
  sensor::Sensor *em_r_t_in_{nullptr};
  sensor::Sensor *em_r_rh_in_{nullptr};
  sensor::Sensor *em_r_t_out_{nullptr};
  sensor::Sensor *em_r_rh_out_{nullptr};
  sensor::Sensor *em_lambda_q_sensor_{nullptr};
  sensor::Sensor *em_lambda_r_inlet_sensor_{nullptr};
  sensor::Sensor *em_lambda_r_outlet_sensor_{nullptr};

  float pressure_pa_{101325.0f};
  sensor::Sensor *inlet_absolute_humidity_{nullptr};
  sensor::Sensor *inlet_dew_point_{nullptr};
  sensor::Sensor *inlet_enthalpy_{nullptr};
  sensor::Sensor *inlet_humidity_ratio_{nullptr};
  sensor::Sensor *outlet_absolute_humidity_{nullptr};
  sensor::Sensor *outlet_dew_point_{nullptr};
  sensor::Sensor *outlet_enthalpy_{nullptr};
  sensor::Sensor *outlet_humidity_ratio_{nullptr};

  sensor::Sensor *air_flow_{nullptr};
  sensor::Sensor *filtered_air_flow_{nullptr};
  sensor::Sensor *delivered_power_{nullptr};
  sensor::Sensor *delivered_power_lag_{nullptr};
  float delivered_power_lag_tau_s_{30.0f};
  bool virtual_coil_{false};
  float coil_tau_s_{60.0f};
  float outlet_air_tau_s_{20.0f};
  sensor::Sensor *filtered_virtual_coil_temperature_{nullptr};

  climate::Climate *climate_{nullptr};
  sensor::Sensor *compressor_frequency_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *outside_temperature_{nullptr};
  sensor::Sensor *outside_coil_temperature_before_{nullptr};
  sensor::Sensor *outside_coil_temperature_after_{nullptr};
  sensor::Sensor *inside_room_temperature_{nullptr};
  sensor::Sensor *inside_room_humidity_{nullptr};

  HpUkfFilter filter_;
  uint32_t last_update_ms_{0};
  bool initialized_{false};
};

}  // namespace hp_ukf
}  // namespace esphome
