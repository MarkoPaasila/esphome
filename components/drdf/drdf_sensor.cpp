#include "drdf_sensor.h"
#include <cmath>
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace drdf {

static const char *const TAG = "drdf.sensor";

void DrdfSensor::setup() {
  if (input_sensor_ == nullptr) {
    ESP_LOGE(TAG, "Input sensor not set");
    this->mark_failed();
    return;
  }
  input_sensor_->add_on_state_callback([this](float value) { this->on_input_state_(value); });
  if (input_sensor_->has_state()) {
    this->on_input_state_(input_sensor_->get_state());
  }
}

void DrdfSensor::on_input_state_(float value) {
  optional<float> out = filter_.new_value(value);
  if (out.has_value()) {
    this->publish_state(*out);
  }
  float ub = filter_.get_upper_bound();
  float lb = filter_.get_lower_bound();
  if (upper_bound_sensor_ != nullptr && std::isfinite(ub)) {
    upper_bound_sensor_->publish_state(ub);
  }
  if (lower_bound_sensor_ != nullptr && std::isfinite(lb)) {
    lower_bound_sensor_->publish_state(lb);
  }
}

void DrdfSensor::dump_config() {
  if (filter_.is_bias_enabled()) {
    ESP_LOGCONFIG(TAG, "DRDF sensor (bias correction: bias_ema_alpha=%.3f):",
                  filter_.get_bias_ema_alpha());
  } else {
    ESP_LOGCONFIG(TAG, "DRDF sensor (bias disabled, midpoint):");
  }
  LOG_SENSOR("  ", "Input", input_sensor_);
  if (upper_bound_sensor_ != nullptr) {
    LOG_SENSOR("  ", "Upper bound", upper_bound_sensor_);
  }
  if (lower_bound_sensor_ != nullptr) {
    LOG_SENSOR("  ", "Lower bound", lower_bound_sensor_);
  }
}

}  // namespace drdf
}  // namespace esphome
