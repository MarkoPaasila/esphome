#pragma once

#include <cstdint>

namespace esphome {
namespace hp_ukf {

// Time-discrete Unscented Kalman Filter for heat pump inlet/outlet state.
// State layouts:
//   n=4:  [T_in, RH_in, T_out, RH_out]
//   n=8:  [T_in, RH_in, T_out, RH_out, dT_in, dT_out, dRH_in, dRH_out]
//   n=6:  [T_in, RH_in, T_out, RH_out, air_flow_L_s, delivered_power_kW]
//   n=10: [T_in, RH_in, T_out, RH_out, air_flow_L_s, delivered_power_kW, dT_in, dT_out, dRH_in, dRH_out]
//   n=7:  [T_in, RH_in, T_out, RH_out, air_flow_L_s, delivered_power_kW, Tvcoil]  (virtual coil enabled)
//   n=11: [T_in, RH_in, T_out, RH_out, air_flow_L_s, delivered_power_kW, dT_in, dT_out, dRH_in, dRH_out, Tvcoil]
//         When n=11, T_out is evolved only as T_out + dT_out*dt (not lagged toward Tvcoil) so dT_out tracks outlet.
// Measurements (m=5): [T_in, RH_in, T_out, RH_out, air_flow_L_s] (z[4] used only when air flow sensor configured).
class HpUkfFilter {
 public:
  static constexpr int N_MAX = 11;
  static constexpr int M = 5;

  HpUkfFilter() = default;

  // Configure state dimension: 4, 6, 7, 8, 10, or 11 (see state layout comment above).
  void set_state_dimension(int n);
  int get_state_dimension() const { return n_; }

  // Atmospheric pressure (Pa) for delivered power from air flow and enthalpy. Call when n>=6.
  void set_atmospheric_pressure(float pa) { pressure_pa_ = pa; }

  // First-order lag time constant (s) for delivered power state; 0 = instantaneous. Call when n>=6.
  void set_delivered_power_lag_tau_s(float s) { tau_delivered_power_s_ = s; }

  // Virtual coil: first-order lag time constant (s) for Tvcoil toward G(f_comp, T_outside). Call when n>=7.
  void set_coil_tau_s(float s) { tau_coil_s_ = s; }
  // Outlet air temperature first-order lag toward Tvcoil (s). Call when n>=7.
  void set_outlet_air_tau_s(float s) { tau_outlet_air_s_ = s; }

  // Set initial state and covariance. Call once before first predict/update.
  void set_state(const float *x);
  void set_covariance(const float *P);
  void set_initial_state(const float *x, const float *P);

  // Time-discrete predict with elapsed time dt in seconds.
  void predict(float dt);

  // Control input: set before predict(). action: 0=OFF, 2=COOLING, 3=HEATING, 4=IDLE, 5=DRYING, 6=FAN (ClimateAction).
  // hp_state: inferred HpState code (0=OFF, 1=FAN_ONLY, 2=HEATING_IDLE, 3=HEATING_RAMP_UP, 4=HEATING_ACTIVE,
  //   5=DEFROSTING, 6=DEFROST_END, 7=DEFROST_RAMP_UP, 8=COOLING_IDLE, 9=COOLING_RAMP_UP, 10=COOLING_ACTIVE,
  //   11=STABILIZING, 12=UNKNOWN). Use 255 to ignore hp_state (e.g. state machine not run).
  // compressor_freq_hz: Hz or 0/NaN when unknown. power_kw: input power in kW (NaN when not configured).
  // T_outside, T_coil_before, T_coil_after, T_room in °C; rh_room in %. NAN = not available.
  void set_control_input(uint8_t action, uint8_t hp_state, float compressor_freq_hz, float power_kw,
                         float T_outside, float T_coil_before, float T_coil_after, float T_room, float rh_room);

  // Update with measurement z[M] and mask (true = measurement available).
  void update(const float *z, const bool *mask);

  // Current state and covariance (read-only).
  const float *get_state() const { return x_; }
  const float *get_covariance() const { return P_; }

  // Optional: set process/measurement noise (defaults set in .cpp).
  void set_process_noise(const float *Q);
  void set_measurement_noise(const float *R);

  // EM auto-tune: separate forgetting factors for Q and for R (inlet vs outlet).
  void enable_em_autotune(bool enable) { em_enabled_ = enable; }
  void set_em_lambda_q(float v) { em_lambda_q_ = v; }
  void set_em_lambda_r_inlet(float v) { em_lambda_r_inlet_ = v; }
  void set_em_lambda_r_outlet(float v) { em_lambda_r_outlet_ = v; }
  void set_em_inflation(float v) { em_inflation_ = v; }
  bool em_autotune_enabled() const { return em_enabled_; }
  float get_em_lambda_q() const { return em_lambda_q_; }
  float get_em_lambda_r_inlet() const { return em_lambda_r_inlet_; }
  float get_em_lambda_r_outlet() const { return em_lambda_r_outlet_; }

  // Getters for diagonal Q and R (for sensor exposure). q_diag has n_ elements, r_diag has M.
  void get_process_noise_diag(float *q_diag) const;
  void get_measurement_noise_diag(float *r_diag) const;

 private:
  int n_{8};
  float x_[N_MAX]{};
  float P_[N_MAX * N_MAX]{};
  float Q_[N_MAX * N_MAX]{};
  float R_[M * M]{};

  bool em_enabled_{false};
  float em_lambda_q_{0.995f};
  float em_lambda_r_inlet_{0.998f};
  float em_lambda_r_outlet_{0.98f};
  float em_inflation_{0.5f};
  static constexpr float R_MIN = 1e-6f;
  static constexpr float Q_MIN = 1e-10f;

  // UKF parameters: alpha, beta, kappa -> lambda = alpha^2 * (n + kappa) - n
  // alpha must be >= 1 (or kappa large) so lambda >= 0; else weights are invalid and P becomes non-PSD -> NaN state
  float alpha_{1.0f};
  float beta_{2.0f};
  float kappa_{0.0f};
  float lambda_{0.0f};
  float wm0_{0.0f};
  float wc0_{0.0f};
  float wm_{0.0f};
  float wc_{0.0f};

  float pressure_pa_{101325.0f};
  float tau_delivered_power_s_{30.0f};  // s; 0 = instantaneous
  float tau_coil_s_{60.0f};             // s; virtual coil thermal time constant (n>=7)
  float tau_outlet_air_s_{20.0f};      // s; outlet air lag toward Tvcoil (n>=7)

  uint8_t control_action_{0};
  uint8_t control_hp_state_{255};  // 255 = ignore (state machine not run)
  float control_compressor_hz_{0.0f};
  float control_power_kw_{0.0f};  // NaN when not configured
  float control_T_outside_{0.0f};  // °C, NAN when not available
  float control_T_coil_before_{0.0f};
  float control_T_coil_after_{0.0f};
  float control_T_room_{0.0f};
  float control_rh_room_{0.0f};  // %, NAN when not available

  void update_weights();
  void state_transition(const float *x_in, float dt, float *x_out) const;
  void cholesky_factor(int dim, const float *A, float *L) const;
  void sigma_points(int dim, float *chi) const;
};

}  // namespace hp_ukf
}  // namespace esphome
