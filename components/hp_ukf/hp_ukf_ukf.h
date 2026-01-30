#pragma once

namespace esphome {
namespace hp_ukf {

// Time-discrete Unscented Kalman Filter for heat pump inlet/outlet state.
// State (n=8): [T_in, RH_in, T_out, RH_out, dT_in, dT_out, dRH_in, dRH_out].
// Measurements (m=4): [T_in, RH_in, T_out, RH_out].
// Supports n=4 (no derivatives) or n=8 (with all derivatives).
class HpUkfFilter {
 public:
  static constexpr int N_MAX = 8;
  static constexpr int M = 4;

  HpUkfFilter() = default;

  // Configure state dimension: 4 (no derivatives) or 8 (with dT_in, dT_out, dRH_in, dRH_out).
  void set_state_dimension(int n);
  int get_state_dimension() const { return n_; }

  // Set initial state and covariance. Call once before first predict/update.
  void set_state(const float *x);
  void set_covariance(const float *P);
  void set_initial_state(const float *x, const float *P);

  // Time-discrete predict with elapsed time dt in seconds.
  void predict(float dt);

  // Update with measurement z[4] and mask (true = measurement available).
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

  void update_weights();
  void state_transition(const float *x_in, float dt, float *x_out) const;
  void cholesky_factor(int dim, const float *A, float *L) const;
  void sigma_points(int dim, float *chi) const;
};

}  // namespace hp_ukf
}  // namespace esphome
