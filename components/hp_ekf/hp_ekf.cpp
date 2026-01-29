#include "hp_ekf.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace hp_ekf {

static const char *const TAG = "hp_ekf";

static constexpr float S_EPSILON = 1e-6f;
static constexpr float T_ES_MIN = -40.0f;
static constexpr float T_ES_MAX = 100.0f;
static constexpr float T_REJECT_MIN = -50.0f;
static constexpr float T_REJECT_MAX = 120.0f;
static constexpr float RH_PERCENT_THRESH = 1.2f;

// --- Config setters (trivial) ---
void HpEkfComponent::set_update_interval_ms(uint32_t ms) { set_interval(ms); }
void HpEkfComponent::set_pressure_pa(float p) { pressure_pa_ = p; }
void HpEkfComponent::set_use_inlet_temp(bool enabled) { use_inlet_temp_ = enabled; }
void HpEkfComponent::set_use_inlet_rh(bool enabled) { use_inlet_rh_ = enabled; }
void HpEkfComponent::set_use_outlet_temp(bool enabled) { use_outlet_temp_ = enabled; }
void HpEkfComponent::set_use_outlet_rh(bool enabled) { use_outlet_rh_ = enabled; }

void HpEkfComponent::set_initial_state(float Tin, float Tout, float AHin, float AHout, float Tin_rate,
                                       float Tout_rate) {
  x0_[0] = Tin;
  x0_[1] = Tout;
  x0_[2] = AHin;
  x0_[3] = AHout;
  x0_[4] = Tin_rate;
  x0_[5] = Tout_rate;
  initial_state_set_ = true;
}

void HpEkfComponent::set_initial_covariance_diagonal(float p_Tin, float p_Tout, float p_AHin, float p_AHout,
                                                     float p_Tin_rate, float p_Tout_rate) {
  P0_[0] = p_Tin;
  P0_[1] = p_Tout;
  P0_[2] = p_AHin;
  P0_[3] = p_AHout;
  P0_[4] = p_Tin_rate;
  P0_[5] = p_Tout_rate;
  initial_covariance_set_ = true;
}

void HpEkfComponent::set_gate_thresholds(float temp_thresh, float rh_thresh) {
  gate_temp_d2_ = temp_thresh;
  gate_rh_d2_ = rh_thresh;
}

void HpEkfComponent::set_rate_bounds(float min_rate_c_per_s, float max_rate_c_per_s) {
  rate_min_cps_ = min_rate_c_per_s;
  rate_max_cps_ = max_rate_c_per_s;
}

void HpEkfComponent::set_temp_bounds(float min_c, float max_c) {
  temp_min_c_ = min_c;
  temp_max_c_ = max_c;
}

void HpEkfComponent::set_Q_temp_accel(float q_acc_in, float q_acc_out) {
  q_acc_in_ = q_acc_in;
  q_acc_out_ = q_acc_out;
}

void HpEkfComponent::set_Q_ah(float q_AHin, float q_AHout) {
  q_AH_in_ = q_AHin;
  q_AH_out_ = q_AHout;
}

void HpEkfComponent::set_R_Tin(float r_Tin) { r_Tin_ = r_Tin; }
void HpEkfComponent::set_R_Tout(float r_Tout) { r_Tout_ = r_Tout; }
void HpEkfComponent::set_R_RHin(float r_RHin) { r_RHin_ = r_RHin; }
void HpEkfComponent::set_R_RHout(float r_RHout) { r_RHout_ = r_RHout; }
void HpEkfComponent::set_dt_bounds(float dt_min_s, float dt_max_s) {
  dt_min_s_ = dt_min_s;
  dt_max_s_ = dt_max_s;
}

void HpEkfComponent::set_inlet_temp_sensor(sensor::Sensor *s) { inlet_temp_sensor_ = s; }
void HpEkfComponent::set_inlet_rh_sensor(sensor::Sensor *s) { inlet_rh_sensor_ = s; }
void HpEkfComponent::set_outlet_temp_sensor(sensor::Sensor *s) { outlet_temp_sensor_ = s; }
void HpEkfComponent::set_outlet_rh_sensor(sensor::Sensor *s) { outlet_rh_sensor_ = s; }

// --- Math helpers ---
float HpEkfComponent::es(float T_c) const {
  float t = T_c;
  if (t < T_ES_MIN)
    t = T_ES_MIN;
  else if (t > T_ES_MAX)
    t = T_ES_MAX;
  return ES_C_PA * std::exp(ES_A * t / (t + ES_B));
}

float HpEkfComponent::rh_from_TAH(float T_c, float AH) const {
  float e = es(T_c);
  if (e <= 0.0f || !is_valid_float(e))
    return 0.0f;
  float rh = AH * (T_c + K_ABS_ZERO) / (AH_COEF * e);
  if (rh < 0.0f)
    rh = 0.0f;
  else if (rh > 1.0f)
    rh = 1.0f;
  return rh;
}

void HpEkfComponent::rh_jacobian_inlet(float &dRH_dT, float &dRH_dAH) const {
  float T = x_[0];
  float AH = x_[2];
  float e = es(T);
  if (e <= 0.0f) {
    dRH_dT = 0.0f;
    dRH_dAH = 0.0f;
    return;
  }
  float Tb = T + ES_B;
  float factor = (T + K_ABS_ZERO) * ES_A * ES_B / (Tb * Tb);
  dRH_dT = AH * (1.0f / AH_COEF) * (1.0f - factor) / e;
  dRH_dAH = (T + K_ABS_ZERO) / (AH_COEF * e);
}

void HpEkfComponent::rh_jacobian_outlet(float &dRH_dT, float &dRH_dAH) const {
  float T = x_[1];
  float AH = x_[3];
  float e = es(T);
  if (e <= 0.0f) {
    dRH_dT = 0.0f;
    dRH_dAH = 0.0f;
    return;
  }
  float Tb = T + ES_B;
  float factor = (T + K_ABS_ZERO) * ES_A * ES_B / (Tb * Tb);
  dRH_dT = AH * (1.0f / AH_COEF) * (1.0f - factor) / e;
  dRH_dAH = (T + K_ABS_ZERO) / (AH_COEF * e);
}

bool HpEkfComponent::is_valid_float(float v) const {
  return std::isfinite(v);
}

void HpEkfComponent::clamp_state() {
  if (x_[0] < temp_min_c_)
    x_[0] = temp_min_c_;
  else if (x_[0] > temp_max_c_)
    x_[0] = temp_max_c_;
  if (x_[1] < temp_min_c_)
    x_[1] = temp_min_c_;
  else if (x_[1] > temp_max_c_)
    x_[1] = temp_max_c_;
  if (x_[2] < 0.0f)
    x_[2] = 0.0f;
  if (x_[3] < 0.0f)
    x_[3] = 0.0f;
  if (x_[4] < rate_min_cps_)
    x_[4] = rate_min_cps_;
  else if (x_[4] > rate_max_cps_)
    x_[4] = rate_max_cps_;
  if (x_[5] < rate_min_cps_)
    x_[5] = rate_min_cps_;
  else if (x_[5] > rate_max_cps_)
    x_[5] = rate_max_cps_;
}

void HpEkfComponent::symmetrize_P() {
  for (int i = 0; i < EKF_STATE_DIM; i++) {
    for (int j = i + 1; j < EKF_STATE_DIM; j++) {
      float v = 0.5f * (P_[i][j] + P_[j][i]);
      P_[i][j] = v;
      P_[j][i] = v;
    }
  }
}

// --- F matrix: identity with F[0][4]=dt, F[1][5]=dt ---
void HpEkfComponent::build_F(float dt) {
  for (int i = 0; i < EKF_STATE_DIM; i++)
    for (int j = 0; j < EKF_STATE_DIM; j++)
      F_[i][j] = (i == j) ? 1.0f : 0.0f;
  F_[0][4] = dt;
  F_[1][5] = dt;
}

// --- Q: block-diagonal. Blocks (0,4) and (1,5) constant-velocity; diagonals 2,3 for AH ---
void HpEkfComponent::build_Q(float dt) {
  for (int i = 0; i < EKF_STATE_DIM; i++)
    for (int j = 0; j < EKF_STATE_DIM; j++)
      Q_[i][j] = 0.0f;

  float dt2 = dt * dt;
  float dt3 = dt2 * dt;

  Q_[0][0] = q_acc_in_ * dt3 / 3.0f;
  Q_[0][4] = q_acc_in_ * dt2 / 2.0f;
  Q_[4][0] = Q_[0][4];
  Q_[4][4] = q_acc_in_ * dt;

  Q_[1][1] = q_acc_out_ * dt3 / 3.0f;
  Q_[1][5] = q_acc_out_ * dt2 / 2.0f;
  Q_[5][1] = Q_[1][5];
  Q_[5][5] = q_acc_out_ * dt;

  Q_[2][2] = q_AH_in_ * dt;
  Q_[3][3] = q_AH_out_ * dt;
}

// --- Predict: x = F*x, P = F*P*F' + Q ---
void HpEkfComponent::predict(float dt) {
  build_F(dt);
  build_Q(dt);

  float x_new[EKF_STATE_DIM];
  for (int i = 0; i < EKF_STATE_DIM; i++) {
    x_new[i] = 0.0f;
    for (int j = 0; j < EKF_STATE_DIM; j++)
      x_new[i] += F_[i][j] * x_[j];
  }
  for (int i = 0; i < EKF_STATE_DIM; i++)
    x_[i] = x_new[i];

  float FP[EKF_STATE_DIM][EKF_STATE_DIM];
  for (int i = 0; i < EKF_STATE_DIM; i++) {
    for (int j = 0; j < EKF_STATE_DIM; j++) {
      FP[i][j] = 0.0f;
      for (int k = 0; k < EKF_STATE_DIM; k++)
        FP[i][j] += F_[i][k] * P_[k][j];
    }
  }
  for (int i = 0; i < EKF_STATE_DIM; i++) {
    for (int j = 0; j < EKF_STATE_DIM; j++) {
      P_[i][j] = Q_[i][j];
      for (int k = 0; k < EKF_STATE_DIM; k++)
        P_[i][j] += FP[i][k] * F_[j][k];
    }
  }
}

// --- Temperature update: H = row with 1 at state_idx. y = z - x[state_idx], S = P[i][i]+R, K = P col / S, Joseph ---
bool HpEkfComponent::apply_temperature_update(int state_idx, float z, float R) {
  float y = z - x_[state_idx];
  float S = P_[state_idx][state_idx] + R + S_EPSILON;
  if (S <= 0.0f || !is_valid_float(S))
    return false;
  float d2 = y * y / S;
  if (d2 > gate_temp_d2_) {
    ESP_LOGW(TAG, "Temp update gated (state_idx=%d, d2=%.4f)", state_idx, d2);
    return false;
  }

  float K[EKF_STATE_DIM];
  for (int i = 0; i < EKF_STATE_DIM; i++)
    K[i] = P_[i][state_idx] / S;

  for (int i = 0; i < EKF_STATE_DIM; i++)
    x_[i] += K[i] * y;

  // Joseph: P = (I - K*H)*P*(I-K*H)' + K*R*K'. H is row with 1 at state_idx.
  float IKH[EKF_STATE_DIM][EKF_STATE_DIM];
  for (int i = 0; i < EKF_STATE_DIM; i++)
    for (int j = 0; j < EKF_STATE_DIM; j++)
      IKH[i][j] = (i == j) ? (1.0f - K[i] * (j == state_idx ? 1.0f : 0.0f)) : (-K[i] * (j == state_idx ? 1.0f : 0.0f));
  // Actually IKH[i][j] = delta_ij - K[i]*H[j], H[j]= (j==state_idx)?1:0 => IKH[i][j] = delta_ij - K[i]*(j==state_idx)
  for (int i = 0; i < EKF_STATE_DIM; i++) {
    for (int j = 0; j < EKF_STATE_DIM; j++) {
      IKH[i][j] = (i == j) ? 1.0f : 0.0f;
      if (j == state_idx)
        IKH[i][j] -= K[i];
    }
  }

  float IKHP[EKF_STATE_DIM][EKF_STATE_DIM];
  for (int i = 0; i < EKF_STATE_DIM; i++) {
    for (int j = 0; j < EKF_STATE_DIM; j++) {
      IKHP[i][j] = 0.0f;
      for (int k = 0; k < EKF_STATE_DIM; k++)
        IKHP[i][j] += IKH[i][k] * P_[k][j];
    }
  }
  for (int i = 0; i < EKF_STATE_DIM; i++) {
    for (int j = 0; j < EKF_STATE_DIM; j++) {
      P_[i][j] = K[i] * R * K[j];
      for (int k = 0; k < EKF_STATE_DIM; k++)
        P_[i][j] += IKHP[i][k] * IKH[j][k];
    }
  }
  symmetrize_P();
  clamp_state();
  return true;
}

// --- RH update: h = rh_from_TAH(T, AH), H from Jacobian, same flow ---
bool HpEkfComponent::apply_rh_update(bool is_inlet, float z_rh, float R) {
  if (z_rh < 0.0f)
    z_rh = 0.0f;
  else if (z_rh > 1.0f)
    z_rh = 1.0f;

  float dRH_dT, dRH_dAH;
  float h;
  float H[EKF_STATE_DIM];
  for (int i = 0; i < EKF_STATE_DIM; i++)
    H[i] = 0.0f;

  if (is_inlet) {
    rh_jacobian_inlet(dRH_dT, dRH_dAH);
    h = rh_from_TAH(x_[0], x_[2]);
    H[0] = dRH_dT;
    H[2] = dRH_dAH;
  } else {
    rh_jacobian_outlet(dRH_dT, dRH_dAH);
    h = rh_from_TAH(x_[1], x_[3]);
    H[1] = dRH_dT;
    H[3] = dRH_dAH;
  }

  float y = z_rh - h;
  float S = S_EPSILON;
  for (int i = 0; i < EKF_STATE_DIM; i++)
    for (int j = 0; j < EKF_STATE_DIM; j++)
      S += H[i] * P_[i][j] * H[j];
  S += R;
  if (S <= 0.0f || !is_valid_float(S))
    return false;
  float d2 = y * y / S;
  if (d2 > gate_rh_d2_) {
    ESP_LOGW(TAG, "RH update gated (is_inlet=%d, d2=%.4f)", is_inlet ? 1 : 0, d2);
    return false;
  }

  float K[EKF_STATE_DIM];
  for (int i = 0; i < EKF_STATE_DIM; i++) {
    K[i] = 0.0f;
    for (int j = 0; j < EKF_STATE_DIM; j++)
      K[i] += P_[i][j] * H[j];
    K[i] /= S;
  }

  for (int i = 0; i < EKF_STATE_DIM; i++)
    x_[i] += K[i] * y;

  // Joseph: I - K*H
  float IKH[EKF_STATE_DIM][EKF_STATE_DIM];
  for (int i = 0; i < EKF_STATE_DIM; i++) {
    for (int j = 0; j < EKF_STATE_DIM; j++)
      IKH[i][j] = (i == j ? 1.0f : 0.0f) - K[i] * H[j];
  }
  float IKHP[EKF_STATE_DIM][EKF_STATE_DIM];
  for (int i = 0; i < EKF_STATE_DIM; i++) {
    for (int j = 0; j < EKF_STATE_DIM; j++) {
      IKHP[i][j] = 0.0f;
      for (int k = 0; k < EKF_STATE_DIM; k++)
        IKHP[i][j] += IKH[i][k] * P_[k][j];
    }
  }
  for (int i = 0; i < EKF_STATE_DIM; i++) {
    for (int j = 0; j < EKF_STATE_DIM; j++) {
      P_[i][j] = K[i] * R * K[j];
      for (int k = 0; k < EKF_STATE_DIM; k++)
        P_[i][j] += IKHP[i][k] * IKH[j][k];
    }
  }
  symmetrize_P();
  clamp_state();
  return true;
}

float HpEkfComponent::get_RHin_est() const {
  return rh_from_TAH(x_[0], x_[2]);
}

float HpEkfComponent::get_RHout_est() const {
  return rh_from_TAH(x_[1], x_[3]);
}

void HpEkfComponent::setup() {
  if (initial_state_set_) {
    for (int i = 0; i < EKF_STATE_DIM; i++)
      x_[i] = x0_[i];
  } else {
    x_[0] = 20.0f;
    x_[1] = 20.0f;
    x_[2] = 8.0f;
    x_[3] = 8.0f;
    x_[4] = 0.0f;
    x_[5] = 0.0f;
    if (inlet_temp_sensor_ && inlet_rh_sensor_) {
      float t = inlet_temp_sensor_->state;
      float rh = inlet_rh_sensor_->state;
      if (is_valid_float(t) && is_valid_float(rh)) {
        if (rh > RH_PERCENT_THRESH)
          rh /= 100.0f;
        if (rh >= 0.0f && rh <= 1.0f && t >= T_REJECT_MIN && t <= T_REJECT_MAX) {
          float e = es(t);
          if (e > 0.0f)
            x_[2] = AH_COEF * rh * e / (t + K_ABS_ZERO);
        }
      }
    }
    if (outlet_temp_sensor_ && outlet_rh_sensor_) {
      float t = outlet_temp_sensor_->state;
      float rh = outlet_rh_sensor_->state;
      if (is_valid_float(t) && is_valid_float(rh)) {
        if (rh > RH_PERCENT_THRESH)
          rh /= 100.0f;
        if (rh >= 0.0f && rh <= 1.0f && t >= T_REJECT_MIN && t <= T_REJECT_MAX) {
          float e = es(t);
          if (e > 0.0f)
            x_[3] = AH_COEF * rh * e / (t + K_ABS_ZERO);
        }
      }
    }
  }

  if (initial_covariance_set_) {
    for (int i = 0; i < EKF_STATE_DIM; i++) {
      for (int j = 0; j < EKF_STATE_DIM; j++)
        P_[i][j] = (i == j) ? P0_[i] : 0.0f;
    }
  } else {
    float p0[EKF_STATE_DIM] = {4.0f, 4.0f, 4.0f, 4.0f, 0.25f, 0.25f};
    for (int i = 0; i < EKF_STATE_DIM; i++) {
      for (int j = 0; j < EKF_STATE_DIM; j++)
        P_[i][j] = (i == j) ? p0[i] : 0.0f;
    }
  }

  last_predict_us_ = micros();

  tin_filtered_sensor_.set_name("HP EKF Inlet Temperature");
  tin_filtered_sensor_.set_unit_of_measurement("\xC2\xB0C");  // °C
  tin_filtered_sensor_.set_device_class("temperature");
  App.register_sensor(&tin_filtered_sensor_);

  tout_filtered_sensor_.set_name("HP EKF Outlet Temperature");
  tout_filtered_sensor_.set_unit_of_measurement("\xC2\xB0C");
  tout_filtered_sensor_.set_device_class("temperature");
  App.register_sensor(&tout_filtered_sensor_);

  ahin_filtered_sensor_.set_name("HP EKF Inlet Absolute Humidity");
  ahin_filtered_sensor_.set_unit_of_measurement("g/m\xC2\xB3");  // g/m³
  App.register_sensor(&ahin_filtered_sensor_);

  ahout_filtered_sensor_.set_name("HP EKF Outlet Absolute Humidity");
  ahout_filtered_sensor_.set_unit_of_measurement("g/m\xC2\xB3");
  App.register_sensor(&ahout_filtered_sensor_);

  tin_rate_filtered_sensor_.set_name("HP EKF Inlet Temperature Rate");
  tin_rate_filtered_sensor_.set_unit_of_measurement("\xC2\xB0C/s");
  App.register_sensor(&tin_rate_filtered_sensor_);

  tout_rate_filtered_sensor_.set_name("HP EKF Outlet Temperature Rate");
  tout_rate_filtered_sensor_.set_unit_of_measurement("\xC2\xB0C/s");
  App.register_sensor(&tout_rate_filtered_sensor_);

  rhin_est_sensor_.set_name("HP EKF Inlet RH (est)");
  rhin_est_sensor_.set_device_class("humidity");
  App.register_sensor(&rhin_est_sensor_);

  rhout_est_sensor_.set_name("HP EKF Outlet RH (est)");
  rhout_est_sensor_.set_device_class("humidity");
  App.register_sensor(&rhout_est_sensor_);

  ESP_LOGCONFIG(TAG, "HP-EKF: update_interval=%u ms", this->get_interval());
  ESP_LOGCONFIG(TAG, "  use_inlet_temp=%d use_inlet_rh=%d use_outlet_temp=%d use_outlet_rh=%d",
                use_inlet_temp_ ? 1 : 0, use_inlet_rh_ ? 1 : 0, use_outlet_temp_ ? 1 : 0, use_outlet_rh_ ? 1 : 0);
  ESP_LOGD(TAG, "  Q acc in/out=%.4f/%.4f Q AH in/out=%.4f/%.4f", q_acc_in_, q_acc_out_, q_AH_in_, q_AH_out_);
  ESP_LOGD(TAG, "  R Tin/Tout/RHin/RHout=%.4f/%.4f/%.6f/%.6f", r_Tin_, r_Tout_, r_RHin_, r_RHout_);
}

void HpEkfComponent::update() {
  uint32_t now_us = micros();
  float dt = (float) (now_us - last_predict_us_) / 1e6f;
  if (dt < dt_min_s_)
    dt = dt_min_s_;
  else if (dt > dt_max_s_)
    dt = dt_max_s_;

  predict(dt);
  last_predict_us_ = now_us;

  float z_tin = NAN, z_rhin = NAN, z_tout = NAN, z_rhout = NAN;
  if (inlet_temp_sensor_ && use_inlet_temp_) {
    z_tin = inlet_temp_sensor_->state;
    if (!is_valid_float(z_tin) || z_tin < T_REJECT_MIN || z_tin > T_REJECT_MAX)
      z_tin = NAN;
  }
  if (inlet_rh_sensor_ && use_inlet_rh_) {
    z_rhin = inlet_rh_sensor_->state;
    if (!is_valid_float(z_rhin))
      z_rhin = NAN;
    else if (z_rhin > RH_PERCENT_THRESH)
      z_rhin /= 100.0f;
    if (z_rhin < 0.0f || z_rhin > 1.0f)
      z_rhin = NAN;
  }
  if (outlet_temp_sensor_ && use_outlet_temp_) {
    z_tout = outlet_temp_sensor_->state;
    if (!is_valid_float(z_tout) || z_tout < T_REJECT_MIN || z_tout > T_REJECT_MAX)
      z_tout = NAN;
  }
  if (outlet_rh_sensor_ && use_outlet_rh_) {
    z_rhout = outlet_rh_sensor_->state;
    if (!is_valid_float(z_rhout))
      z_rhout = NAN;
    else if (z_rhout > RH_PERCENT_THRESH)
      z_rhout /= 100.0f;
    if (z_rhout < 0.0f || z_rhout > 1.0f)
      z_rhout = NAN;
  }

  if (is_valid_float(z_tin))
    apply_temperature_update(0, z_tin, r_Tin_);
  if (is_valid_float(z_rhin))
    apply_rh_update(true, z_rhin, r_RHin_);
  if (is_valid_float(z_tout))
    apply_temperature_update(1, z_tout, r_Tout_);
  if (is_valid_float(z_rhout))
    apply_rh_update(false, z_rhout, r_RHout_);

  float rhin = get_RHin_est();
  float rhout = get_RHout_est();

  if (is_valid_float(x_[0]))
    tin_filtered_sensor_.publish_state(x_[0]);
  if (is_valid_float(x_[1]))
    tout_filtered_sensor_.publish_state(x_[1]);
  if (is_valid_float(x_[2]))
    ahin_filtered_sensor_.publish_state(x_[2]);
  if (is_valid_float(x_[3]))
    ahout_filtered_sensor_.publish_state(x_[3]);
  if (is_valid_float(x_[4]))
    tin_rate_filtered_sensor_.publish_state(x_[4]);
  if (is_valid_float(x_[5]))
    tout_rate_filtered_sensor_.publish_state(x_[5]);
  if (is_valid_float(rhin))
    rhin_est_sensor_.publish_state(rhin);
  if (is_valid_float(rhout))
    rhout_est_sensor_.publish_state(rhout);

  for (int i = 0; i < EKF_STATE_DIM; i++) {
    if (!is_valid_float(x_[i])) {
      ESP_LOGW(TAG, "NaN/Inf in state[%d] after update", i);
      break;
    }
  }
}

void HpEkfComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "HP-EKF component");
  ESP_LOGCONFIG(TAG, "  Update interval: %u ms", this->get_interval());
  ESP_LOGCONFIG(TAG, "  Pressure: %.0f Pa", pressure_pa_);
  ESP_LOGCONFIG(TAG, "  Temp bounds: [%.1f, %.1f] °C", temp_min_c_, temp_max_c_);
  ESP_LOGCONFIG(TAG, "  Rate bounds: [%.2f, %.2f] °C/s", rate_min_cps_, rate_max_cps_);
}

}  // namespace hp_ekf
}  // namespace esphome
