#include "hp_ukf_ukf.h"
#include <cmath>
#include <algorithm>

namespace esphome {
namespace hp_ukf {

// Psychrometric helpers (Magnus, T in °C, p in Pa). Single precision only.
static float saturation_vapor_pressure_pa(float T_c) {
  float denom = 243.04f + T_c;
  if (denom < 1.0f)
    denom = 1.0f;
  return 610.94f * expf(17.625f * T_c / denom);
}

static float humidity_ratio_kg_kg(float pv_pa, float p_pa) {
  float p_minus_pv = p_pa - pv_pa;
  if (p_minus_pv < 10.0f)
    return 0.0f;
  return 0.62198f * pv_pa / p_minus_pv;
}

static float enthalpy_kj_kg(float T_c, float W_kg_kg) {
  return 1.006f * T_c + W_kg_kg * (2501.0f + 1.86f * T_c);
}

static float delivered_power_kw(float T_in, float rh_in, float T_out, float rh_out,
                                float air_flow_L_s, float pressure_pa) {
  float pv_in = (rh_in / 100.0f) * saturation_vapor_pressure_pa(T_in);
  float W_in = humidity_ratio_kg_kg(pv_in, pressure_pa);
  float h_in = enthalpy_kj_kg(T_in, W_in);
  float pv_out = (rh_out / 100.0f) * saturation_vapor_pressure_pa(T_out);
  float W_out = humidity_ratio_kg_kg(pv_out, pressure_pa);
  float h_out = enthalpy_kj_kg(T_out, W_out);
  float T_in_K = T_in + 273.15f;
  if (T_in_K < 1.0f)
    T_in_K = 1.0f;
  float rho = pressure_pa / (287.0f * T_in_K);
  float air_flow_m3_s = air_flow_L_s / 1000.0f;
  float m_dot = air_flow_m3_s * rho;
  return m_dot * (h_out - h_in);
}

void HpUkfFilter::set_state_dimension(int n) {
  n_ = (n == 4 || n == 6 || n == 8 || n == 10) ? n : 8;
  update_weights();
  for (int i = 0; i < n_ * n_; i++)
    Q_[i] = 0.0f;
  Q_[0 * n_ + 0] = 0.001074034f;   // T_in °C²
  Q_[1 * n_ + 1] = 0.01395682f;    // RH_in %²
  Q_[2 * n_ + 2] = 0.002934240f;   // T_out °C²
  Q_[3 * n_ + 3] = 0.03639173f;    // RH_out %²
  if (n_ >= 6) {
    Q_[4 * n_ + 4] = 1.0f;         // air_flow (L/s)²
    Q_[5 * n_ + 5] = 0.01f;        // delivered_power (kW)²
  }
  if (n_ >= 8 && n_ < 10) {
    Q_[4 * n_ + 4] = 0.0001467592f;  // dT_in
    Q_[5 * n_ + 5] = 0.0005582764f;  // dT_out
    Q_[6 * n_ + 6] = 0.002286525f;   // dRH_in
    Q_[7 * n_ + 7] = 0.005536150f;   // dRH_out
  }
  if (n_ >= 10) {
    Q_[6 * n_ + 6] = 0.0001467592f;  // dT_in
    Q_[7 * n_ + 7] = 0.0005582764f;  // dT_out
    Q_[8 * n_ + 8] = 0.002286525f;   // dRH_in
    Q_[9 * n_ + 9] = 0.005536150f;   // dRH_out
  }
  for (int i = 0; i < M * M; i++)
    R_[i] = 0.0f;
  R_[0 * M + 0] = 0.0007687177f;  // T_in °C²
  R_[1 * M + 1] = 0.02196444f;    // RH_in %²
  R_[2 * M + 2] = 0.002987398f;   // T_out °C²
  R_[3 * M + 3] = 0.05590067f;    // RH_out %²
  R_[4 * M + 4] = 10.0f;         // air_flow (L/s)²
}

void HpUkfFilter::update_weights() {
  lambda_ = alpha_ * alpha_ * (n_ + kappa_) - n_;
  float nlam = n_ + lambda_;
  wm0_ = lambda_ / nlam;
  wc0_ = lambda_ / nlam + (1.0f - alpha_ * alpha_ + beta_);
  wm_ = 0.5f / nlam;
  wc_ = 0.5f / nlam;
}

void HpUkfFilter::set_state(const float *x) {
  for (int i = 0; i < n_; i++)
    x_[i] = x[i];
}

void HpUkfFilter::set_covariance(const float *P) {
  for (int i = 0; i < n_ * n_; i++)
    P_[i] = P[i];
}

void HpUkfFilter::set_initial_state(const float *x, const float *P) {
  set_state(x);
  set_covariance(P);
}

void HpUkfFilter::set_process_noise(const float *Q) {
  for (int i = 0; i < n_ * n_; i++)
    Q_[i] = Q[i];
}

void HpUkfFilter::set_measurement_noise(const float *R) {
  for (int i = 0; i < M * M; i++)
    R_[i] = R[i];
}

void HpUkfFilter::get_process_noise_diag(float *q_diag) const {
  for (int i = 0; i < n_; i++)
    q_diag[i] = Q_[i * n_ + i];
}

void HpUkfFilter::get_measurement_noise_diag(float *r_diag) const {
  for (int i = 0; i < M; i++)
    r_diag[i] = R_[i * M + i];
}

void HpUkfFilter::set_control_input(uint8_t action, float compressor_freq_hz, float power_kw,
                                    float T_outside, float T_coil_before, float T_coil_after,
                                    float T_room, float rh_room) {
  control_action_ = action;
  control_compressor_hz_ = compressor_freq_hz;
  control_power_kw_ = power_kw;
  control_T_outside_ = T_outside;
  control_T_coil_before_ = T_coil_before;
  control_T_coil_after_ = T_coil_after;
  control_T_room_ = T_room;
  control_rh_room_ = rh_room;
}

// Threshold below which input power (kW) is treated as "no power" (correlated to compressor off).
static constexpr float POWER_NO_POWER_THRESHOLD_KW = 0.01f;

// True when control input indicates no heating/cooling power (OFF, IDLE, FAN, DRY, compressor off, or power near 0).
static bool control_no_power(uint8_t action, float compressor_hz, float power_kw) {
  if (action == 0 || action == 4 || action == 5 || action == 6)  // OFF, IDLE, DRYING, FAN
    return true;
  if (!std::isfinite(compressor_hz) || compressor_hz <= 0.0f)
    return true;
  if (std::isfinite(power_kw) && power_kw >= 0.0f && power_kw < POWER_NO_POWER_THRESHOLD_KW)
    return true;
  return false;
}

void HpUkfFilter::state_transition(const float *x_in, float dt, float *x_out) const {
  bool no_power = control_no_power(control_action_, control_compressor_hz_, control_power_kw_);

  if (n_ >= 10) {
    x_out[0] = x_in[0] + x_in[6] * dt;   // T_in
    x_out[1] = x_in[1] + x_in[8] * dt;   // RH_in
    x_out[2] = x_in[2] + x_in[7] * dt;   // T_out
    x_out[3] = x_in[3] + x_in[9] * dt;   // RH_out
    x_out[4] = x_in[4];  // air_flow L/s
    if (no_power) {
      x_out[5] = 0.0f;
    } else {
      float p_kw = delivered_power_kw(x_out[0], x_out[1], x_out[2], x_out[3], x_out[4], pressure_pa_);
      x_out[5] = std::isfinite(p_kw) ? p_kw : x_in[5];
    }
    x_out[6] = x_in[6];
    x_out[7] = x_in[7];
    x_out[8] = x_in[8];
    x_out[9] = x_in[9];
  } else if (n_ >= 8) {
    x_out[0] = x_in[0] + x_in[4] * dt;
    x_out[1] = x_in[1] + x_in[6] * dt;
    x_out[2] = x_in[2] + x_in[5] * dt;
    x_out[3] = x_in[3] + x_in[7] * dt;
    x_out[4] = x_in[4];
    x_out[5] = x_in[5];
    x_out[6] = x_in[6];
    x_out[7] = x_in[7];
  } else if (n_ >= 6) {
    x_out[0] = x_in[0];
    x_out[1] = x_in[1];
    x_out[2] = x_in[2];
    x_out[3] = x_in[3];
    x_out[4] = x_in[4];
    if (no_power) {
      x_out[5] = 0.0f;
    } else {
      float p_kw = delivered_power_kw(x_out[0], x_out[1], x_out[2], x_out[3], x_out[4], pressure_pa_);
      x_out[5] = std::isfinite(p_kw) ? p_kw : x_in[5];
    }
  } else {
    x_out[0] = x_in[0];
    x_out[1] = x_in[1];
    x_out[2] = x_in[2];
    x_out[3] = x_in[3];
  }
}

void HpUkfFilter::cholesky_factor(int dim, const float *A, float *L) const {
  for (int i = 0; i < dim * dim; i++)
    L[i] = 0.0f;
  for (int i = 0; i < dim; i++) {
    for (int j = 0; j <= i; j++) {
      float s = A[i * dim + j];
      for (int k = 0; k < j; k++)
        s -= L[i * dim + k] * L[j * dim + k];
      if (i == j)
        L[i * dim + j] = (s > 1e-10f) ? std::sqrt(s) : 1e-5f;
      else
        L[i * dim + j] = s / (L[j * dim + j] + 1e-10f);
    }
  }
}

// chi: (2n+1) columns, each column length n. Stored row-major as chi[n * (2*n+1)].
// Sigma points use (n+lambda)*P = L*L^T, then x +/- L columns.
void HpUkfFilter::sigma_points(int dim, float *chi) const {
  float P_scaled[N_MAX * N_MAX];
  float scale = dim + lambda_;
  for (int i = 0; i < dim * dim; i++)
    P_scaled[i] = scale * P_[i];
  float L[N_MAX * N_MAX];
  cholesky_factor(dim, P_scaled, L);
  for (int i = 0; i < dim; i++)
    chi[i * (2 * dim + 1)] = x_[i];
  for (int j = 0; j < dim; j++) {
    for (int i = 0; i < dim; i++) {
      chi[i * (2 * dim + 1) + j + 1] = x_[i] + L[i * dim + j];
      chi[i * (2 * dim + 1) + dim + 1 + j] = x_[i] - L[i * dim + j];
    }
  }
}

void HpUkfFilter::predict(float dt) {
  dt = std::max(1e-6f, std::min(dt, 3600.0f));
  int dim = n_;
  int n_sigma = 2 * dim + 1;
  float chi[N_MAX * (2 * N_MAX + 1)];
  sigma_points(dim, chi);

  float x_pred[N_MAX];
  for (int i = 0; i < dim; i++)
    x_pred[i] = wm0_ * chi[i * n_sigma];
  for (int k = 1; k < n_sigma; k++) {
    float x_prop[N_MAX];
    for (int i = 0; i < dim; i++)
      x_prop[i] = chi[i * n_sigma + k];
    float x_out[N_MAX];
    state_transition(x_prop, dt, x_out);
    for (int i = 0; i < dim; i++)
      x_pred[i] += wm_ * x_out[i];
  }
  for (int i = 0; i < dim; i++)
    x_[i] = x_pred[i];

  float P_pred[N_MAX * N_MAX];
  for (int i = 0; i < dim * dim; i++)
    P_pred[i] = 0.0f;
  for (int k = 0; k < n_sigma; k++) {
    float x_prop[N_MAX];
    for (int i = 0; i < dim; i++)
      x_prop[i] = chi[i * n_sigma + k];
    float x_out[N_MAX];
    state_transition(x_prop, dt, x_out);
    float w = (k == 0) ? wc0_ : wc_;
    for (int i = 0; i < dim; i++)
      for (int j = 0; j < dim; j++)
        P_pred[i * dim + j] += w * (x_out[i] - x_[i]) * (x_out[j] - x_[j]);
  }
  for (int i = 0; i < dim * dim; i++)
    P_[i] = P_pred[i] + Q_[i];
}

void HpUkfFilter::update(const float *z, const bool *mask) {
  int dim = n_;
  int m_avail = 0;
  int idx[M];
  for (int i = 0; i < M; i++) {
    if (mask[i]) {
      idx[m_avail] = i;
      m_avail++;
    }
  }
  if (m_avail == 0)
    return;

  int n_sigma = 2 * dim + 1;
  float chi[N_MAX * (2 * N_MAX + 1)];
  sigma_points(dim, chi);

  float z_pred[M];
  for (int i = 0; i < M; i++)
    z_pred[i] = wm0_ * chi[i * n_sigma];
  for (int k = 1; k < n_sigma; k++) {
    float w = wm_;
    for (int i = 0; i < M; i++)
      z_pred[i] += w * chi[i * n_sigma + k];
  }

  float z_avail[M];
  float z_pred_avail[M];
  for (int i = 0; i < m_avail; i++) {
    z_avail[i] = z[idx[i]];
    z_pred_avail[i] = z_pred[idx[i]];
  }

  float Pzz[M * M];
  for (int i = 0; i < m_avail * m_avail; i++)
    Pzz[i] = 0.0f;
  for (int k = 0; k < n_sigma; k++) {
    float w = (k == 0) ? wc0_ : wc_;
    float dz[M];
    for (int i = 0; i < m_avail; i++)
      dz[i] = chi[idx[i] * n_sigma + k] - z_pred_avail[i];
    for (int i = 0; i < m_avail; i++)
      for (int j = 0; j < m_avail; j++)
        Pzz[i * m_avail + j] += w * dz[i] * dz[j];
  }
  // Save Pzz prior (before adding R) for EM R adaptation.
  float Pzz_prior_ii[M];
  for (int i = 0; i < m_avail; i++)
    Pzz_prior_ii[i] = Pzz[i * m_avail + i];
  for (int i = 0; i < m_avail; i++)
    Pzz[i * m_avail + i] += R_[idx[i] * M + idx[i]];

  float Pxz[N_MAX * M];
  for (int i = 0; i < dim * m_avail; i++)
    Pxz[i] = 0.0f;
  for (int k = 0; k < n_sigma; k++) {
    float w = (k == 0) ? wc0_ : wc_;
    float dx[N_MAX], dz[M];
    for (int i = 0; i < dim; i++)
      dx[i] = chi[i * n_sigma + k] - x_[i];
    for (int i = 0; i < m_avail; i++)
      dz[i] = chi[idx[i] * n_sigma + k] - z_pred_avail[i];
    for (int i = 0; i < dim; i++)
      for (int j = 0; j < m_avail; j++)
        Pxz[i * m_avail + j] += w * dx[i] * dz[j];
  }

  // Pzz^{-1} via Gauss-Jordan (m_avail x m_avail)
  float Pzz_inv[M * M];
  for (int i = 0; i < m_avail; i++)
    for (int j = 0; j < m_avail; j++)
      Pzz_inv[i * m_avail + j] = (i == j) ? 1.0f : 0.0f;
  float Pzz_work[M * M];
  for (int i = 0; i < m_avail * m_avail; i++)
    Pzz_work[i] = Pzz[i];
  for (int col = 0; col < m_avail; col++) {
    int pivot = col;
    float v = std::abs(Pzz_work[col * m_avail + col]);
    for (int row = col + 1; row < m_avail; row++) {
      float v2 = std::abs(Pzz_work[row * m_avail + col]);
      if (v2 > v) {
        v = v2;
        pivot = row;
      }
    }
    if (pivot != col) {
      for (int j = 0; j < m_avail; j++) {
        std::swap(Pzz_work[col * m_avail + j], Pzz_work[pivot * m_avail + j]);
        std::swap(Pzz_inv[col * m_avail + j], Pzz_inv[pivot * m_avail + j]);
      }
    }
    float div = Pzz_work[col * m_avail + col];
    if (std::abs(div) < 1e-10f)
      div = 1e-10f;
    for (int j = 0; j < m_avail; j++) {
      Pzz_work[col * m_avail + j] /= div;
      Pzz_inv[col * m_avail + j] /= div;
    }
    for (int row = 0; row < m_avail; row++) {
      if (row == col)
        continue;
      float fac = Pzz_work[row * m_avail + col];
      for (int j = 0; j < m_avail; j++) {
        Pzz_work[row * m_avail + j] -= fac * Pzz_work[col * m_avail + j];
        Pzz_inv[row * m_avail + j] -= fac * Pzz_inv[col * m_avail + j];
      }
    }
  }

  float K[N_MAX * M];
  for (int i = 0; i < dim; i++)
    for (int j = 0; j < m_avail; j++) {
      K[i * m_avail + j] = 0.0f;
      for (int r = 0; r < m_avail; r++)
        K[i * m_avail + j] += Pxz[i * m_avail + r] * Pzz_inv[r * m_avail + j];
    }

  float innov[M];
  for (int i = 0; i < m_avail; i++)
    innov[i] = z_avail[i] - z_pred_avail[i];
  float corr[N_MAX];
  for (int i = 0; i < dim; i++) {
    float dx = 0.0f;
    for (int j = 0; j < m_avail; j++)
      dx += K[i * m_avail + j] * innov[j];
    corr[i] = dx;
    x_[i] += dx;
  }

  // Joseph form: P = (I - K*H)*P*(I - K*H)' + K*R*K'
  // H for available measurements: H_avail has rows idx[0..m_avail-1] = identity rows.
  // (I - K*H) for reduced: I - K*H_avail, H_avail is m_avail x n, rows are unit vectors for idx[].
  float IKH[N_MAX * N_MAX];
  for (int i = 0; i < dim * dim; i++)
    IKH[i] = (i % (dim + 1) == 0) ? 1.0f : 0.0f;
  for (int i = 0; i < dim; i++)
    for (int j = 0; j < m_avail; j++)
      IKH[i * dim + idx[j]] -= K[i * m_avail + j];
  float P_new[N_MAX * N_MAX];
  for (int i = 0; i < dim; i++)
    for (int j = 0; j < dim; j++) {
      P_new[i * dim + j] = 0.0f;
      for (int r = 0; r < dim; r++)
        P_new[i * dim + j] += IKH[i * dim + r] * P_[r * dim + j];
    }
  float P_tmp[N_MAX * N_MAX];
  for (int i = 0; i < dim; i++)
    for (int j = 0; j < dim; j++) {
      P_tmp[i * dim + j] = 0.0f;
      for (int r = 0; r < dim; r++)
        P_tmp[i * dim + j] += P_new[i * dim + r] * IKH[j * dim + r];
    }
  for (int i = 0; i < dim; i++)
    for (int j = 0; j < dim; j++) {
      float KRK = 0.0f;
      for (int r = 0; r < m_avail; r++)
        for (int s = 0; s < m_avail; s++)
          KRK += K[i * m_avail + r] * R_[idx[r] * M + idx[s]] * K[j * m_avail + s];
      P_[i * dim + j] = P_tmp[i * dim + j] + KRK;
    }

  // EM auto-tune: R adaptation then Q adaptation (diagonal, with forgetting factors).
  if (em_enabled_) {
    for (int i = 0; i < m_avail; i++) {
      int g = idx[i];
      float lambda_r = (g <= 1) ? em_lambda_r_inlet_ : em_lambda_r_outlet_;
      float r_est = innov[i] * innov[i] - Pzz_prior_ii[i];
      if (r_est < R_MIN)
        r_est = R_MIN;
      r_est *= (1.0f + em_inflation_);
      float r_old = R_[g * M + g];
      R_[g * M + g] = lambda_r * r_old + (1.0f - lambda_r) * r_est;
      if (R_[g * M + g] < R_MIN)
        R_[g * M + g] = R_MIN;
    }
    for (int j = 0; j < dim; j++) {
      float q_est = corr[j] * corr[j];
      if (q_est < Q_MIN)
        q_est = Q_MIN;
      q_est *= (1.0f + em_inflation_);
      float q_old = Q_[j * dim + j];
      Q_[j * dim + j] = em_lambda_q_ * q_old + (1.0f - em_lambda_q_) * q_est;
      if (Q_[j * dim + j] < Q_MIN)
        Q_[j * dim + j] = Q_MIN;
    }
  }
}

}  // namespace hp_ukf
}  // namespace esphome
