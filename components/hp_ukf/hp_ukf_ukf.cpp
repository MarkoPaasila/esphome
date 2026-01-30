#include "hp_ukf_ukf.h"
#include <cmath>
#include <algorithm>

namespace esphome {
namespace hp_ukf {

void HpUkfFilter::set_state_dimension(int n) {
  n_ = (n == 4 || n == 8) ? n : 8;
  update_weights();
  // Default process noise: small for T/RH, larger for derivatives (random walk).
  for (int i = 0; i < n_ * n_; i++)
    Q_[i] = 0.0f;
  Q_[0 * n_ + 0] = 1e-5f;   // T_in
  Q_[1 * n_ + 1] = 1e-4f;   // RH_in
  Q_[2 * n_ + 2] = 1e-5f;   // T_out
  Q_[3 * n_ + 3] = 1e-4f;   // RH_out
  if (n_ >= 8) {
    Q_[4 * n_ + 4] = 0.01f;  // dT_in
    Q_[5 * n_ + 5] = 0.01f;  // dT_out
    Q_[6 * n_ + 6] = 0.001f;  // dRH_in
    Q_[7 * n_ + 7] = 0.001f;  // dRH_out
  }
  // Default measurement noise: typical sensor accuracy.
  for (int i = 0; i < M * M; i++)
    R_[i] = 0.0f;
  R_[0 * M + 0] = 0.3f * 0.3f;   // T_in
  R_[1 * M + 1] = 2.0f * 2.0f;   // RH_in
  R_[2 * M + 2] = 0.3f * 0.3f;   // T_out
  R_[3 * M + 3] = 2.0f * 2.0f;   // RH_out
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

void HpUkfFilter::state_transition(const float *x_in, float dt, float *x_out) const {
  x_out[0] = x_in[0] + (n_ >= 8 ? x_in[4] * dt : 0.0f);   // T_in
  x_out[1] = x_in[1] + (n_ >= 8 ? x_in[6] * dt : 0.0f);   // RH_in
  x_out[2] = x_in[2] + (n_ >= 8 ? x_in[5] * dt : 0.0f);   // T_out
  x_out[3] = x_in[3] + (n_ >= 8 ? x_in[7] * dt : 0.0f);   // RH_out
  if (n_ >= 8) {
    x_out[4] = x_in[4];  // dT_in
    x_out[5] = x_in[5];  // dT_out
    x_out[6] = x_in[6];  // dRH_in
    x_out[7] = x_in[7];  // dRH_out
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

  float z_avail[4];
  float z_pred_avail[4];
  for (int i = 0; i < m_avail; i++) {
    z_avail[i] = z[idx[i]];
    z_pred_avail[i] = z_pred[idx[i]];
  }

  float Pzz[4 * 4];
  for (int i = 0; i < m_avail * m_avail; i++)
    Pzz[i] = 0.0f;
  for (int k = 0; k < n_sigma; k++) {
    float w = (k == 0) ? wc0_ : wc_;
    float dz[4];
    for (int i = 0; i < m_avail; i++)
      dz[i] = chi[idx[i] * n_sigma + k] - z_pred_avail[i];
    for (int i = 0; i < m_avail; i++)
      for (int j = 0; j < m_avail; j++)
        Pzz[i * m_avail + j] += w * dz[i] * dz[j];
  }
  // Save Pzz prior (before adding R) for EM R adaptation.
  float Pzz_prior_ii[4];
  for (int i = 0; i < m_avail; i++)
    Pzz_prior_ii[i] = Pzz[i * m_avail + i];
  for (int i = 0; i < m_avail; i++)
    Pzz[i * m_avail + i] += R_[idx[i] * M + idx[i]];

  float Pxz[N_MAX * 4];
  for (int i = 0; i < dim * m_avail; i++)
    Pxz[i] = 0.0f;
  for (int k = 0; k < n_sigma; k++) {
    float w = (k == 0) ? wc0_ : wc_;
    float dx[N_MAX], dz[4];
    for (int i = 0; i < dim; i++)
      dx[i] = chi[i * n_sigma + k] - x_[i];
    for (int i = 0; i < m_avail; i++)
      dz[i] = chi[idx[i] * n_sigma + k] - z_pred_avail[i];
    for (int i = 0; i < dim; i++)
      for (int j = 0; j < m_avail; j++)
        Pxz[i * m_avail + j] += w * dx[i] * dz[j];
  }

  // Pzz^{-1} via Gauss-Jordan (m_avail x m_avail)
  float Pzz_inv[4 * 4];
  for (int i = 0; i < m_avail; i++)
    for (int j = 0; j < m_avail; j++)
      Pzz_inv[i * m_avail + j] = (i == j) ? 1.0f : 0.0f;
  float Pzz_work[4 * 4];
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

  float K[N_MAX * 4];
  for (int i = 0; i < dim; i++)
    for (int j = 0; j < m_avail; j++) {
      K[i * m_avail + j] = 0.0f;
      for (int r = 0; r < m_avail; r++)
        K[i * m_avail + j] += Pxz[i * m_avail + r] * Pzz_inv[r * m_avail + j];
    }

  float innov[4];
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
      float r_old = R_[g * M + g];
      R_[g * M + g] = lambda_r * r_old + (1.0f - lambda_r) * r_est;
      if (R_[g * M + g] < R_MIN)
        R_[g * M + g] = R_MIN;
    }
    for (int j = 0; j < dim; j++) {
      float q_est = corr[j] * corr[j];
      if (q_est < Q_MIN)
        q_est = Q_MIN;
      float q_old = Q_[j * dim + j];
      Q_[j * dim + j] = em_lambda_q_ * q_old + (1.0f - em_lambda_q_) * q_est;
      if (Q_[j * dim + j] < Q_MIN)
        Q_[j * dim + j] = Q_MIN;
    }
  }
}

}  // namespace hp_ukf
}  // namespace esphome
