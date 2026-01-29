#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace hp_ekf {

static constexpr int EKF_STATE_DIM = 6;
// State indices: 0 T_in, 1 T_out, 2 AH_in, 3 AH_out, 4 T_in_rate, 5 T_out_rate

class HpEkfComponent : public PollingComponent {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;

  float get_setup_priority() const override { return setup_priority::DATA; }

  // Config
  void set_update_interval_ms(uint32_t ms);
  void set_pressure_pa(float p);
  void set_use_inlet_temp(bool enabled);
  void set_use_inlet_rh(bool enabled);
  void set_use_outlet_temp(bool enabled);
  void set_use_outlet_rh(bool enabled);
  void set_initial_state(float Tin, float Tout, float AHin, float AHout, float Tin_rate, float Tout_rate);
  void set_initial_covariance_diagonal(float p_Tin, float p_Tout, float p_AHin, float p_AHout, float p_Tin_rate,
                                       float p_Tout_rate);
  void set_gate_thresholds(float temp_thresh, float rh_thresh);
  void set_rate_bounds(float min_rate_c_per_s, float max_rate_c_per_s);
  void set_temp_bounds(float min_c, float max_c);
  void set_Q_temp_accel(float q_acc_in, float q_acc_out);
  void set_Q_ah(float q_AHin, float q_AHout);
  void set_R_Tin(float r_Tin);
  void set_R_Tout(float r_Tout);
  void set_R_RHin(float r_RHin);
  void set_R_RHout(float r_RHout);
  void set_dt_bounds(float dt_min_s, float dt_max_s);

  // Input sensors
  void set_inlet_temp_sensor(sensor::Sensor *s);
  void set_inlet_rh_sensor(sensor::Sensor *s);
  void set_outlet_temp_sensor(sensor::Sensor *s);
  void set_outlet_rh_sensor(sensor::Sensor *s);

  // Output sensors (component owns and publishes to these)
  sensor::Sensor *get_tin_filtered_sensor() { return &tin_filtered_sensor_; }
  sensor::Sensor *get_tout_filtered_sensor() { return &tout_filtered_sensor_; }
  sensor::Sensor *get_ahin_filtered_sensor() { return &ahin_filtered_sensor_; }
  sensor::Sensor *get_ahout_filtered_sensor() { return &ahout_filtered_sensor_; }
  sensor::Sensor *get_tin_rate_filtered_sensor() { return &tin_rate_filtered_sensor_; }
  sensor::Sensor *get_tout_rate_filtered_sensor() { return &tout_rate_filtered_sensor_; }
  sensor::Sensor *get_rhin_est_sensor() { return &rhin_est_sensor_; }
  sensor::Sensor *get_rhout_est_sensor() { return &rhout_est_sensor_; }

  // Getters
  float get_Tin() const { return x_[0]; }
  float get_Tout() const { return x_[1]; }
  float get_AHin() const { return x_[2]; }
  float get_AHout() const { return x_[3]; }
  float get_Tin_rate() const { return x_[4]; }
  float get_Tout_rate() const { return x_[5]; }
  float get_RHin_est() const;
  float get_RHout_est() const;

 protected:
  void predict(float dt);
  void build_F(float dt);
  void build_Q(float dt);
  bool apply_temperature_update(int state_idx, float z, float R);
  bool apply_rh_update(bool is_inlet, float z_rh, float R);
  float es(float T_c) const;
  float rh_from_TAH(float T_c, float AH) const;
  void rh_jacobian_inlet(float &dRH_dT, float &dRH_dAH) const;
  void rh_jacobian_outlet(float &dRH_dT, float &dRH_dAH) const;
  void clamp_state();
  void symmetrize_P();
  bool is_valid_float(float v) const;

  float x_[EKF_STATE_DIM]{};
  float P_[EKF_STATE_DIM][EKF_STATE_DIM]{};
  float F_[EKF_STATE_DIM][EKF_STATE_DIM]{};
  float Q_[EKF_STATE_DIM][EKF_STATE_DIM]{};

  float x0_[EKF_STATE_DIM]{};
  float P0_[EKF_STATE_DIM]{};
  bool initial_state_set_{false};
  bool initial_covariance_set_{false};

  uint32_t last_predict_us_{0};

  float pressure_pa_{101325.0f};
  bool use_inlet_temp_{true};
  bool use_inlet_rh_{true};
  bool use_outlet_temp_{true};
  bool use_outlet_rh_{true};

  float r_Tin_{0.04f};
  float r_Tout_{0.04f};
  float r_RHin_{0.0004f};
  float r_RHout_{0.0004f};

  float q_acc_in_{0.02f};
  float q_acc_out_{0.02f};
  float q_AH_in_{0.02f};
  float q_AH_out_{0.02f};

  float gate_temp_d2_{6.63f};
  float gate_rh_d2_{6.63f};

  float temp_min_c_{-40.0f};
  float temp_max_c_{100.0f};
  float rate_min_cps_{-1.0f};
  float rate_max_cps_{1.0f};

  float dt_min_s_{1e-3f};
  float dt_max_s_{1.0f};

  sensor::Sensor *inlet_temp_sensor_{nullptr};
  sensor::Sensor *inlet_rh_sensor_{nullptr};
  sensor::Sensor *outlet_temp_sensor_{nullptr};
  sensor::Sensor *outlet_rh_sensor_{nullptr};

  sensor::Sensor tin_filtered_sensor_;
  sensor::Sensor tout_filtered_sensor_;
  sensor::Sensor ahin_filtered_sensor_;
  sensor::Sensor ahout_filtered_sensor_;
  sensor::Sensor tin_rate_filtered_sensor_;
  sensor::Sensor tout_rate_filtered_sensor_;
  sensor::Sensor rhin_est_sensor_;
  sensor::Sensor rhout_est_sensor_;

  static constexpr float K_ABS_ZERO = 273.15f;
  static constexpr float ES_A = 17.625f;
  static constexpr float ES_B = 243.04f;
  static constexpr float ES_C_PA = 610.94f;
  static constexpr float AH_COEF = 216.7f;
};

}  // namespace hp_ekf
}  // namespace esphome
