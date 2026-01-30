#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
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

  HpUkfFilter filter_;
  uint32_t last_update_ms_{0};
  bool initialized_{false};
};

}  // namespace hp_ukf
}  // namespace esphome
