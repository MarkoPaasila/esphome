#include "hp_ukf.h"
#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include <cmath>
#include <algorithm>

namespace esphome {
namespace hp_ukf {

static const char *const TAG = "hp_ukf";

static float read_sensor(sensor::Sensor *s) {
  if (s != nullptr && s->has_state())
    return s->get_state();
  return NAN;
}

void HpUkfComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HP-UKF component");
  filter_.set_state_dimension(track_derivatives_ ? 8 : 4);

  float x0[HpUkfFilter::N_MAX] = {20.0f, 50.0f, 20.0f, 50.0f, 0.0f, 0.0f, 0.0f, 0.0f};
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

  float P0[HpUkfFilter::N_MAX * HpUkfFilter::N_MAX];
  int n = filter_.get_state_dimension();
  for (int i = 0; i < n * n; i++)
    P0[i] = 0.0f;
  for (int i = 0; i < n; i++)
    P0[i * n + i] = 1.0f;
  filter_.set_initial_state(x0, P0);

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
  if (track_derivatives_) {
    if (filtered_inlet_temperature_derivative_)
      filtered_inlet_temperature_derivative_->publish_state(x[4]);
    if (filtered_outlet_temperature_derivative_)
      filtered_outlet_temperature_derivative_->publish_state(x[5]);
    if (filtered_inlet_humidity_derivative_)
      filtered_inlet_humidity_derivative_->publish_state(x[6]);
    if (filtered_outlet_humidity_derivative_)
      filtered_outlet_humidity_derivative_->publish_state(x[7]);
  }

  last_update_ms_ = millis();
  initialized_ = true;
}

void HpUkfComponent::update() {
  if (this->is_failed() || !initialized_)
    return;

  uint32_t now_ms = millis();
  float dt_s = (now_ms - last_update_ms_) / 1000.0f;
  dt_s = std::max(1e-6f, std::min(dt_s, 3600.0f));

  filter_.predict(dt_s);

  float z[HpUkfFilter::M];
  bool mask[HpUkfFilter::M];
  z[0] = read_sensor(inlet_temperature_);
  z[1] = read_sensor(inlet_humidity_);
  z[2] = read_sensor(outlet_temperature_);
  z[3] = read_sensor(outlet_humidity_);
  for (int i = 0; i < HpUkfFilter::M; i++)
    mask[i] = !std::isnan(z[i]);

  filter_.update(z, mask);
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
  if (track_derivatives_) {
    if (filtered_inlet_temperature_derivative_ && std::isfinite(x[4]))
      filtered_inlet_temperature_derivative_->publish_state(x[4]);
    if (filtered_outlet_temperature_derivative_ && std::isfinite(x[5]))
      filtered_outlet_temperature_derivative_->publish_state(x[5]);
    if (filtered_inlet_humidity_derivative_ && std::isfinite(x[6]))
      filtered_inlet_humidity_derivative_->publish_state(x[6]);
    if (filtered_outlet_humidity_derivative_ && std::isfinite(x[7]))
      filtered_outlet_humidity_derivative_->publish_state(x[7]);
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
}

}  // namespace hp_ukf
}  // namespace esphome
