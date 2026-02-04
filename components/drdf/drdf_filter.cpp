#include "drdf_filter.h"
#include <cmath>

namespace esphome {
namespace drdf {

DrdfFilter::DrdfFilter(float alpha, float ema_multiplier)
    : alpha_(alpha), ema_multiplier_(ema_multiplier) {}

optional<float> DrdfFilter::new_value(float value) {
  const float current_value = value;

  // Initialize bounds on first reading
  if (std::isnan(upper_bound_) || std::isnan(lower_bound_)) {
    upper_bound_ = current_value + deadband_size_ / 2.0f;
    lower_bound_ = current_value - deadband_size_ / 2.0f;
    trend_ = 0;
    previous_trend_ = 0;
    return (upper_bound_ + lower_bound_) / 2.0f;
  }

  // Determine current trend direction
  int current_trend;
  if (current_value > previous_value_) {
    current_trend = 1;
  } else if (current_value < previous_value_) {
    current_trend = -1;
  } else {
    current_trend = 0;
  }

  // Check if trend has reversed (not neutral and different from previous trend)
  bool trend_reversed =
      (trend_ != 0 && current_trend != 0 && trend_ != current_trend);

  // Detect consecutive reversals and update EMA
  if (trend_reversed) {
    if (reversal_detected_) {
      float reversal_diff = fabsf(current_value - reversal_value_);
      if (ema_value_ == 0.0f) {
        ema_value_ = reversal_diff;
      } else {
        ema_value_ =
            alpha_ * reversal_diff + (1.0f - alpha_) * ema_value_;
      }
      deadband_size_ = ema_value_ * ema_multiplier_;
      reversal_value_ = current_value;
    } else {
      reversal_detected_ = true;
      reversal_value_ = current_value;
    }
  } else {
    reversal_detected_ = false;
  }

  // Slide bounds when value exceeds them (band width = deadband_size_)
  if (current_value > upper_bound_) {
    upper_bound_ = current_value;
    lower_bound_ = current_value - deadband_size_;
  } else if (current_value < lower_bound_) {
    lower_bound_ = current_value;
    upper_bound_ = current_value + deadband_size_;
  }

  previous_value_ = current_value;
  previous_trend_ = trend_;
  trend_ = current_trend;

  return (upper_bound_ + lower_bound_) / 2.0f;
}

}  // namespace drdf
}  // namespace esphome
