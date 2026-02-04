#pragma once

#include "esphome/components/sensor/filter.h"

namespace esphome {
namespace drdf {

/** Dynamic Reversals based Deadband Filter (DRDF).
 *
 * Adapts deadband size from observed trend reversals; outputs the midpoint
 * of a sliding deadband. See README for behavior and tuning.
 */
class DrdfFilter : public sensor::Filter {
 public:
  DrdfFilter(float alpha, float ema_multiplier, float bias_ema_alpha = 0.1f);

  optional<float> new_value(float value) override;

  float get_upper_bound() const { return upper_bound_; }
  float get_lower_bound() const { return lower_bound_; }

 protected:
  float upper_bound_{NAN};
  float lower_bound_{NAN};
  float previous_value_{0.0f};
  int8_t trend_{0};           // -1 = down, 0 = neutral, 1 = up
  int8_t previous_trend_{0};
  float reversal_value_{NAN};
  float ema_value_{0.0f};
  float deadband_size_{0.0f};
  float alpha_{0.01f};
  float ema_multiplier_{3.82f};
  float bias_ema_alpha_{0.1f};
  float bias_ema_{0.0f};
  bool reversal_detected_{false};
};

}  // namespace drdf
}  // namespace esphome
