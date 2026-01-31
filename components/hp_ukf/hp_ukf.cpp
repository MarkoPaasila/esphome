#include "hp_ukf.h"
#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/climate/climate_mode.h"
#include <cmath>
#include <algorithm>
#ifdef USE_ESP32
#include "esp_system.h"
#endif
#ifdef USE_ESP8266
#include "Esp.h"
#endif

namespace esphome {
namespace hp_ukf {

static const char *const TAG = "hp_ukf";

static uint32_t get_free_heap_bytes() {
#ifdef USE_ESP32
  return esp_get_free_heap_size();
#elif defined(USE_ESP8266)
  return ESP.getFreeHeap();
#else
  return 0U;
#endif
}

static float read_sensor(sensor::Sensor *s) {
  if (s != nullptr && s->has_state())
    return s->get_state();
  return NAN;
}

// Psychrometric helpers (Magnus over water, T in °C, pressures in Pa). Single precision only.
static float saturation_vapor_pressure_pa(float T_c) {
  float denom = 243.04f + T_c;
  if (denom < 1.0f)
    denom = 1.0f;
  return 610.94f * expf(17.625f * T_c / denom);
}

static float partial_pressure_vapor(float T_c, float rh) {
  return (rh / 100.0f) * saturation_vapor_pressure_pa(T_c);
}

static float absolute_humidity_g_m3(float T_c, float pv_pa) {
  float T_k = T_c + 273.15f;
  if (T_k < 1.0f)
    T_k = 1.0f;
  return 1000.0f * pv_pa / (461.495f * T_k);
}

static float dew_point_c(float pv_pa) {
  if (pv_pa <= 1.0f)
    return NAN;
  float ln_pv = logf(pv_pa / 610.94f);
  float denom = 17.625f - ln_pv;
  if (denom < 0.1f)
    denom = 0.1f;
  return 243.04f * ln_pv / denom;
}

static float humidity_ratio_kg_kg(float pv_pa, float p_pa) {
  float p_minus_pv = p_pa - pv_pa;
  if (p_minus_pv < 10.0f)
    return NAN;
  return 0.62198f * pv_pa / p_minus_pv;
}

static float enthalpy_kj_kg(float T_c, float W_kg_kg) {
  return 1.006f * T_c + W_kg_kg * (2501.0f + 1.86f * T_c);
}

// Delivered power (kW) from inlet/outlet T/RH, air flow (L/s), and pressure (Pa).
static float delivered_power_kw(float T_in, float rh_in, float T_out, float rh_out,
                                float air_flow_L_s, float pressure_pa) {
  float pv_in = partial_pressure_vapor(T_in, rh_in);
  float W_in = humidity_ratio_kg_kg(pv_in, pressure_pa);
  if (!std::isfinite(W_in))
    return NAN;
  float h_in = enthalpy_kj_kg(T_in, W_in);
  float pv_out = partial_pressure_vapor(T_out, rh_out);
  float W_out = humidity_ratio_kg_kg(pv_out, pressure_pa);
  if (!std::isfinite(W_out))
    return NAN;
  float h_out = enthalpy_kj_kg(T_out, W_out);
  float T_in_K = T_in + 273.15f;
  if (T_in_K < 1.0f)
    T_in_K = 1.0f;
  float rho = pressure_pa / (287.0f * T_in_K);
  float air_flow_m3_s = air_flow_L_s / 1000.0f;
  float m_dot = air_flow_m3_s * rho;
  return m_dot * (h_out - h_in);
}

static void publish_psychrometric(float T_c, float rh, float p_pa,
                                  sensor::Sensor *abs_hum, sensor::Sensor *dew, sensor::Sensor *enthalpy,
                                  sensor::Sensor *hum_ratio) {
  if (!std::isfinite(T_c) || !std::isfinite(rh))
    return;
  float p_sat = saturation_vapor_pressure_pa(T_c);
  float pv = (rh / 100.0f) * p_sat;

  if (abs_hum) {
    float ah = absolute_humidity_g_m3(T_c, pv);
    if (std::isfinite(ah))
      abs_hum->publish_state(ah);
  }
  if (dew) {
    float tdp = dew_point_c(pv);
    if (std::isfinite(tdp))
      dew->publish_state(tdp);
  }

  float W = humidity_ratio_kg_kg(pv, p_pa);
  if (hum_ratio && std::isfinite(W))
    hum_ratio->publish_state(W * 1000.0f);
  if (enthalpy && std::isfinite(W)) {
    float h = enthalpy_kj_kg(T_c, W);
    if (std::isfinite(h))
      enthalpy->publish_state(h);
  }
}

void HpUkfComponent::setup() {
  uint32_t t_setup_start_us = micros();
  ESP_LOGCONFIG(TAG, "Setting up HP-UKF component");
  if (air_flow_ != nullptr) {
    filter_.set_state_dimension(virtual_coil_ ? (track_derivatives_ ? 11 : 7) : (track_derivatives_ ? 10 : 6));
    filter_.set_atmospheric_pressure(pressure_pa_);
    filter_.set_delivered_power_lag_tau_s(delivered_power_lag_tau_s_);
    if (virtual_coil_) {
      filter_.set_coil_tau_s(coil_tau_s_);
      filter_.set_outlet_air_tau_s(outlet_air_tau_s_);
    }
  } else {
    filter_.set_state_dimension(track_derivatives_ ? 8 : 4);
  }

  float x0[HpUkfFilter::N_MAX] = {20.0f, 50.0f, 20.0f, 50.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float t_in = read_sensor(inlet_temperature_);
  float rh_in = read_sensor(inlet_humidity_);
  float t_out = read_sensor(outlet_temperature_);
  float rh_out = read_sensor(outlet_humidity_);
  if (!std::isnan(t_in))
    x0[0] = t_in;
  if (!std::isnan(rh_in))
    x0[1] = rh_in;
  if (!std::isnan(t_out))
    x0[2] = t_out;
  if (!std::isnan(rh_out))
    x0[3] = rh_out;

  int n = filter_.get_state_dimension();
  if (n >= 6 && air_flow_ != nullptr) {
    float af = read_sensor(air_flow_);
    x0[4] = std::isfinite(af) ? af : 100.0f;
    float p_kw = delivered_power_kw(x0[0], x0[1], x0[2], x0[3], x0[4], pressure_pa_);
    x0[5] = std::isfinite(p_kw) ? p_kw : 0.0f;
  }
  if (n >= 7) {
    int tvcoil_idx = n - 1;
    float T_coil_after = read_sensor(outside_coil_temperature_after_);
    float T_outside = read_sensor(outside_temperature_);
    if (std::isfinite(T_coil_after))
      x0[tvcoil_idx] = T_coil_after;
    else if (std::isfinite(T_outside))
      x0[tvcoil_idx] = T_outside;
    else
      x0[tvcoil_idx] = x0[2];
  }

  float P0[HpUkfFilter::N_MAX * HpUkfFilter::N_MAX];
  for (int i = 0; i < n * n; i++)
    P0[i] = 0.0f;
  for (int i = 0; i < n; i++)
    P0[i * n + i] = 1.0f;
  filter_.set_initial_state(x0, P0);

  if (em_autotune_) {
    filter_.enable_em_autotune(true);
    filter_.set_em_lambda_q(em_lambda_q_);
    filter_.set_em_lambda_r_inlet(em_lambda_r_inlet_);
    filter_.set_em_lambda_r_outlet(em_lambda_r_outlet_);
    filter_.set_em_inflation(em_inflation_);
  }

  // Publish initial state so sensors show values immediately (avoids NaN/unknown
  // before first update and when source sensors haven't reported yet).
  const float *x = filter_.get_state();
  if (filtered_inlet_temperature_)
    filtered_inlet_temperature_->publish_state(x[0]);
  if (filtered_inlet_humidity_)
    filtered_inlet_humidity_->publish_state(x[1]);
  if (filtered_outlet_temperature_)
    filtered_outlet_temperature_->publish_state(x[2]);
  if (filtered_outlet_humidity_)
    filtered_outlet_humidity_->publish_state(x[3]);
  if (n >= 6) {
    if (filtered_air_flow_ && std::isfinite(x[4]))
      filtered_air_flow_->publish_state(x[4]);
    if (delivered_power_ && std::isfinite(x[5]))
      delivered_power_->publish_state(x[5]);
    if (delivered_power_lag_ && std::isfinite(x[5]))
      delivered_power_lag_->publish_state(x[5]);
  }
  if (n >= 7 && filtered_virtual_coil_temperature_ && std::isfinite(x[n - 1]))
    filtered_virtual_coil_temperature_->publish_state(x[n - 1]);
  if (track_derivatives_ && n >= 8) {
    int d0 = (n >= 10) ? 6 : 4;
    if (filtered_inlet_temperature_derivative_)
      filtered_inlet_temperature_derivative_->publish_state(x[d0 + 0]);
    if (filtered_outlet_temperature_derivative_)
      filtered_outlet_temperature_derivative_->publish_state(x[d0 + 1]);
    if (filtered_inlet_humidity_derivative_)
      filtered_inlet_humidity_derivative_->publish_state(x[d0 + 2]);
    if (filtered_outlet_humidity_derivative_)
      filtered_outlet_humidity_derivative_->publish_state(x[d0 + 3]);
  }

  {
    int em_sensor_count = (em_q_t_in_ ? 1 : 0) + (em_q_rh_in_ ? 1 : 0) + (em_q_t_out_ ? 1 : 0) + (em_q_rh_out_ ? 1 : 0)
        + (em_r_t_in_ ? 1 : 0) + (em_r_rh_in_ ? 1 : 0) + (em_r_t_out_ ? 1 : 0) + (em_r_rh_out_ ? 1 : 0);
    ESP_LOGI(TAG, "em_autotune=%s em_q_r_sensors=%d (Q/R only shown when both enabled and sensors configured)",
             em_autotune_ ? "on" : "off", em_sensor_count);
    ESP_LOGD(TAG, "setup: em_autotune=%d em_sensor_count=%d", em_autotune_ ? 1 : 0, em_sensor_count);
  }
  if (em_autotune_) {
    float q_diag[HpUkfFilter::N_MAX], r_diag[HpUkfFilter::M];
    filter_.get_process_noise_diag(q_diag);
    filter_.get_measurement_noise_diag(r_diag);
    ESP_LOGD(TAG, "setup Q/R diag: q[0]=%.6f q[2]=%.6f r[0]=%.6f r[2]=%.6f",
             q_diag[0], q_diag[2], r_diag[0], r_diag[2]);
    int n = filter_.get_state_dimension();
    ESP_LOGD(TAG, "Q/R diagonal (copy to hp_ukf initial config):");
    ESP_LOGD(TAG, "  q_t_in: %.6e  q_rh_in: %.6e  q_t_out: %.6e  q_rh_out: %.6e",
             q_diag[0], q_diag[1], q_diag[2], q_diag[3]);
    if (n >= 10) {
      ESP_LOGD(TAG, "  q_dt_in: %.6e  q_dt_out: %.6e  q_drh_in: %.6e  q_drh_out: %.6e",
               q_diag[6], q_diag[7], q_diag[8], q_diag[9]);
    } else if (n >= 8) {
      ESP_LOGD(TAG, "  q_dt_in: %.6e  q_dt_out: %.6e  q_drh_in: %.6e  q_drh_out: %.6e",
               q_diag[4], q_diag[5], q_diag[6], q_diag[7]);
    }
    if (n >= 7)
      ESP_LOGD(TAG, "  q_tvcoil: %.6e", q_diag[n - 1]);
    ESP_LOGD(TAG, "  r_t_in: %.6e  r_rh_in: %.6e  r_t_out: %.6e  r_rh_out: %.6e",
             r_diag[0], r_diag[1], r_diag[2], r_diag[3]);
    if (em_q_t_in_) em_q_t_in_->publish_state(q_diag[0]);
    if (em_q_rh_in_) em_q_rh_in_->publish_state(q_diag[1]);
    if (em_q_t_out_) em_q_t_out_->publish_state(q_diag[2]);
    if (em_q_rh_out_) em_q_rh_out_->publish_state(q_diag[3]);
    if (n >= 10) {
      if (em_q_dt_in_) em_q_dt_in_->publish_state(q_diag[6]);
      if (em_q_dt_out_) em_q_dt_out_->publish_state(q_diag[7]);
      if (em_q_drh_in_) em_q_drh_in_->publish_state(q_diag[8]);
      if (em_q_drh_out_) em_q_drh_out_->publish_state(q_diag[9]);
    } else if (n >= 8) {
      if (em_q_dt_in_) em_q_dt_in_->publish_state(q_diag[4]);
      if (em_q_dt_out_) em_q_dt_out_->publish_state(q_diag[5]);
      if (em_q_drh_in_) em_q_drh_in_->publish_state(q_diag[6]);
      if (em_q_drh_out_) em_q_drh_out_->publish_state(q_diag[7]);
    }
    if (em_r_t_in_) em_r_t_in_->publish_state(r_diag[0]);
    if (em_r_rh_in_) em_r_rh_in_->publish_state(r_diag[1]);
    if (em_r_t_out_) em_r_t_out_->publish_state(r_diag[2]);
    if (em_r_rh_out_) em_r_rh_out_->publish_state(r_diag[3]);
    if (em_lambda_q_sensor_) em_lambda_q_sensor_->publish_state(em_lambda_q_);
    if (em_lambda_r_inlet_sensor_) em_lambda_r_inlet_sensor_->publish_state(em_lambda_r_inlet_);
    if (em_lambda_r_outlet_sensor_) em_lambda_r_outlet_sensor_->publish_state(em_lambda_r_outlet_);
  }

  publish_psychrometric(x[0], x[1], pressure_pa_,
                        inlet_absolute_humidity_, inlet_dew_point_, inlet_enthalpy_, inlet_humidity_ratio_);
  publish_psychrometric(x[2], x[3], pressure_pa_,
                        outlet_absolute_humidity_, outlet_dew_point_, outlet_enthalpy_, outlet_humidity_ratio_);

  uint32_t t_setup_end_us = micros();
  uint32_t free_heap = get_free_heap_bytes();
  ESP_LOGD(TAG, "setup: %.2f ms, free_heap %u bytes",
           (t_setup_end_us - t_setup_start_us) / 1000.0f, (unsigned) free_heap);
  last_update_ms_ = millis();
  initialized_ = true;
}

void HpUkfComponent::update() {
  if (this->is_failed() || !initialized_)
    return;

  uint32_t t0_us = micros();
  uint32_t heap_before = get_free_heap_bytes();
  uint32_t now_ms = millis();
  float dt_s = (now_ms - last_update_ms_) / 1000.0f;
  dt_s = std::max(1e-6f, std::min(dt_s, 3600.0f));

  uint8_t action = 0;
  float compressor_hz = 0.0f;
  float power_kw = NAN;
  if (climate_ != nullptr) {
    action = static_cast<uint8_t>(climate_->action);
  }
  if (compressor_frequency_ != nullptr && compressor_frequency_->has_state()) {
    float v = compressor_frequency_->get_state();
    compressor_hz = std::isfinite(v) ? v : 0.0f;
  }
  if (power_sensor_ != nullptr && power_sensor_->has_state()) {
    float w = power_sensor_->get_state();
    if (std::isfinite(w) && w >= 0.0f) {
      power_kw = w / 1000.0f;  // sensor in W -> kW
    }
  }
  float T_outside = read_sensor(outside_temperature_);
  float T_coil_before = read_sensor(outside_coil_temperature_before_);
  float T_coil_after = read_sensor(outside_coil_temperature_after_);
  float T_room = read_sensor(inside_room_temperature_);
  float rh_room = read_sensor(inside_room_humidity_);
  if (std::isfinite(T_room) || std::isfinite(rh_room)) {
    ESP_LOGD(TAG, "Indoor T_room=%.2f °C rh_room=%.1f %%", T_room, rh_room);
  }
  filter_.set_control_input(action, compressor_hz, power_kw, T_outside, T_coil_before, T_coil_after,
                            T_room, rh_room);

  filter_.predict(dt_s);
  uint32_t t_after_predict_us = micros();

  float z[HpUkfFilter::M];
  bool mask[HpUkfFilter::M];
  z[0] = read_sensor(inlet_temperature_);
  z[1] = read_sensor(inlet_humidity_);
  z[2] = read_sensor(outlet_temperature_);
  z[3] = read_sensor(outlet_humidity_);
  mask[0] = !std::isnan(z[0]);
  mask[1] = !std::isnan(z[1]);
  mask[2] = !std::isnan(z[2]);
  mask[3] = !std::isnan(z[3]);
  if (air_flow_ != nullptr) {
    z[4] = read_sensor(air_flow_);
    mask[4] = std::isfinite(z[4]);
  } else {
    z[4] = 0.0f;
    mask[4] = false;
  }

  filter_.update(z, mask);
  uint32_t t_end_us = micros();
  uint32_t heap_after = get_free_heap_bytes();
  ESP_LOGD(TAG, "update: predict %.2f ms, update %.2f ms, total %.2f ms, free_heap %u -> %u bytes",
           (t_after_predict_us - t0_us) / 1000.0f, (t_end_us - t_after_predict_us) / 1000.0f,
           (t_end_us - t0_us) / 1000.0f, (unsigned) heap_before, (unsigned) heap_after);
  last_update_ms_ = now_ms;

  const float *x = filter_.get_state();
  // Only publish finite values so we don't overwrite with NaN (e.g. when source
  // sensors haven't reported yet or filter is still converging).
  if (filtered_inlet_temperature_ && std::isfinite(x[0]))
    filtered_inlet_temperature_->publish_state(x[0]);
  if (filtered_inlet_humidity_ && std::isfinite(x[1]))
    filtered_inlet_humidity_->publish_state(x[1]);
  if (filtered_outlet_temperature_ && std::isfinite(x[2]))
    filtered_outlet_temperature_->publish_state(x[2]);
  if (filtered_outlet_humidity_ && std::isfinite(x[3]))
    filtered_outlet_humidity_->publish_state(x[3]);
  int n = filter_.get_state_dimension();
  if (n >= 6) {
    if (filtered_air_flow_ && std::isfinite(x[4]))
      filtered_air_flow_->publish_state(x[4]);
    if (delivered_power_ && std::isfinite(x[5]))
      delivered_power_->publish_state(x[5]);
    if (delivered_power_lag_ && std::isfinite(x[5]))
      delivered_power_lag_->publish_state(x[5]);
  }
  if (n >= 7 && filtered_virtual_coil_temperature_ && std::isfinite(x[n - 1]))
    filtered_virtual_coil_temperature_->publish_state(x[n - 1]);
  if (track_derivatives_ && n >= 8) {
    int d0 = (n >= 10) ? 6 : 4;
    if (filtered_inlet_temperature_derivative_ && std::isfinite(x[d0 + 0]))
      filtered_inlet_temperature_derivative_->publish_state(x[d0 + 0]);
    if (filtered_outlet_temperature_derivative_ && std::isfinite(x[d0 + 1]))
      filtered_outlet_temperature_derivative_->publish_state(x[d0 + 1]);
    if (filtered_inlet_humidity_derivative_ && std::isfinite(x[d0 + 2]))
      filtered_inlet_humidity_derivative_->publish_state(x[d0 + 2]);
    if (filtered_outlet_humidity_derivative_ && std::isfinite(x[d0 + 3]))
      filtered_outlet_humidity_derivative_->publish_state(x[d0 + 3]);
  }

  if (em_autotune_) {
    float q_diag[HpUkfFilter::N_MAX], r_diag[HpUkfFilter::M];
    filter_.get_process_noise_diag(q_diag);
    filter_.get_measurement_noise_diag(r_diag);
    static uint32_t s_update_count;
    if (s_update_count < 3) {
      ESP_LOGI(TAG, "update#%u Q/R: q[0]=%.6f r[0]=%.6f finite=%d %d",
               s_update_count, q_diag[0], r_diag[0],
               std::isfinite(q_diag[0]) ? 1 : 0, std::isfinite(r_diag[0]) ? 1 : 0);
      s_update_count++;
    }
    int n = filter_.get_state_dimension();
    // Debug: Q/R diagonal in copy-paste form for use as compile-time initial values
    ESP_LOGD(TAG, "Q/R diagonal (copy to hp_ukf initial config):");
    ESP_LOGD(TAG, "  q_t_in: %.6e  q_rh_in: %.6e  q_t_out: %.6e  q_rh_out: %.6e",
             q_diag[0], q_diag[1], q_diag[2], q_diag[3]);
    if (n >= 10) {
      ESP_LOGD(TAG, "  q_dt_in: %.6e  q_dt_out: %.6e  q_drh_in: %.6e  q_drh_out: %.6e",
               q_diag[6], q_diag[7], q_diag[8], q_diag[9]);
    } else if (n >= 8) {
      ESP_LOGD(TAG, "  q_dt_in: %.6e  q_dt_out: %.6e  q_drh_in: %.6e  q_drh_out: %.6e",
               q_diag[4], q_diag[5], q_diag[6], q_diag[7]);
    }
    if (n >= 7)
      ESP_LOGD(TAG, "  q_tvcoil: %.6e", q_diag[n - 1]);
    ESP_LOGD(TAG, "  r_t_in: %.6e  r_rh_in: %.6e  r_t_out: %.6e  r_rh_out: %.6e",
             r_diag[0], r_diag[1], r_diag[2], r_diag[3]);
    if (em_q_t_in_ && std::isfinite(q_diag[0])) em_q_t_in_->publish_state(q_diag[0]);
    if (em_q_rh_in_ && std::isfinite(q_diag[1])) em_q_rh_in_->publish_state(q_diag[1]);
    if (em_q_t_out_ && std::isfinite(q_diag[2])) em_q_t_out_->publish_state(q_diag[2]);
    if (em_q_rh_out_ && std::isfinite(q_diag[3])) em_q_rh_out_->publish_state(q_diag[3]);
    if (n >= 10) {
      if (em_q_dt_in_ && std::isfinite(q_diag[6])) em_q_dt_in_->publish_state(q_diag[6]);
      if (em_q_dt_out_ && std::isfinite(q_diag[7])) em_q_dt_out_->publish_state(q_diag[7]);
      if (em_q_drh_in_ && std::isfinite(q_diag[8])) em_q_drh_in_->publish_state(q_diag[8]);
      if (em_q_drh_out_ && std::isfinite(q_diag[9])) em_q_drh_out_->publish_state(q_diag[9]);
    } else if (n >= 8) {
      if (em_q_dt_in_ && std::isfinite(q_diag[4])) em_q_dt_in_->publish_state(q_diag[4]);
      if (em_q_dt_out_ && std::isfinite(q_diag[5])) em_q_dt_out_->publish_state(q_diag[5]);
      if (em_q_drh_in_ && std::isfinite(q_diag[6])) em_q_drh_in_->publish_state(q_diag[6]);
      if (em_q_drh_out_ && std::isfinite(q_diag[7])) em_q_drh_out_->publish_state(q_diag[7]);
    }
    if (em_r_t_in_ && std::isfinite(r_diag[0])) em_r_t_in_->publish_state(r_diag[0]);
    if (em_r_rh_in_ && std::isfinite(r_diag[1])) em_r_rh_in_->publish_state(r_diag[1]);
    if (em_r_t_out_ && std::isfinite(r_diag[2])) em_r_t_out_->publish_state(r_diag[2]);
    if (em_r_rh_out_ && std::isfinite(r_diag[3])) em_r_rh_out_->publish_state(r_diag[3]);
    if (em_lambda_q_sensor_) em_lambda_q_sensor_->publish_state(em_lambda_q_);
    if (em_lambda_r_inlet_sensor_) em_lambda_r_inlet_sensor_->publish_state(em_lambda_r_inlet_);
    if (em_lambda_r_outlet_sensor_) em_lambda_r_outlet_sensor_->publish_state(em_lambda_r_outlet_);
  }

  if (std::isfinite(x[0]) && std::isfinite(x[1])) {
    publish_psychrometric(x[0], x[1], pressure_pa_,
                          inlet_absolute_humidity_, inlet_dew_point_, inlet_enthalpy_, inlet_humidity_ratio_);
  }
  if (std::isfinite(x[2]) && std::isfinite(x[3])) {
    publish_psychrometric(x[2], x[3], pressure_pa_,
                          outlet_absolute_humidity_, outlet_dew_point_, outlet_enthalpy_, outlet_humidity_ratio_);
  }
}

void HpUkfComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "HP-UKF component");
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  Track derivatives (dT_in, dT_out, dRH_in, dRH_out): %s",
                track_derivatives_ ? "yes" : "no");
  ESP_LOGCONFIG(TAG, "  Inlet temperature sensor: %s", inlet_temperature_ ? "set" : "not set");
  ESP_LOGCONFIG(TAG, "  Inlet humidity sensor: %s", inlet_humidity_ ? "set" : "not set");
  ESP_LOGCONFIG(TAG, "  Outlet temperature sensor: %s", outlet_temperature_ ? "set" : "not set");
  ESP_LOGCONFIG(TAG, "  Outlet humidity sensor: %s", outlet_humidity_ ? "set" : "not set");
  ESP_LOGCONFIG(TAG, "  Air flow sensor (L/s): %s", air_flow_ ? "set" : "not set");
  ESP_LOGCONFIG(TAG, "  Climate (action input): %s", climate_ ? "set" : "not set");
  ESP_LOGCONFIG(TAG, "  Compressor frequency sensor (Hz): %s", compressor_frequency_ ? "set" : "not set");
  ESP_LOGCONFIG(TAG, "  Power sensor (W, control input): %s", power_sensor_ ? "set" : "not set");
  ESP_LOGCONFIG(TAG, "  Inside room temperature: %s", inside_room_temperature_ ? "set" : "not set");
  ESP_LOGCONFIG(TAG, "  Inside room humidity: %s", inside_room_humidity_ ? "set" : "not set");
  ESP_LOGCONFIG(TAG, "  Atmospheric pressure: %.2f hPa", pressure_pa_ / 100.0f);
  ESP_LOGCONFIG(TAG, "  EM auto-tune: %s", em_autotune_ ? "enabled" : "disabled");
  if (em_autotune_) {
    ESP_LOGCONFIG(TAG, "  EM lambda_q=%.3f, lambda_r_inlet=%.3f, lambda_r_outlet=%.3f, inflation=%.2f",
                  em_lambda_q_, em_lambda_r_inlet_, em_lambda_r_outlet_, em_inflation_);
    int em_sensors = (em_q_t_in_ ? 1 : 0) + (em_q_rh_in_ ? 1 : 0) + (em_q_t_out_ ? 1 : 0) + (em_q_rh_out_ ? 1 : 0)
        + (em_r_t_in_ ? 1 : 0) + (em_r_rh_in_ ? 1 : 0) + (em_r_t_out_ ? 1 : 0) + (em_r_rh_out_ ? 1 : 0);
    ESP_LOGCONFIG(TAG, "  EM Q/R sensors configured: %d", em_sensors);
  }
}

}  // namespace hp_ukf
}  // namespace esphome
