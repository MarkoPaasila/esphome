#pragma once

#include "drdf_filter.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace drdf {

class DrdfSensor : public Component, public sensor::Sensor {
 public:
  DrdfSensor(float alpha, float ema_multiplier) : filter_(alpha, ema_multiplier) {}

  void set_input_sensor(sensor::Sensor *sensor) { input_sensor_ = sensor; }
  void set_upper_bound_sensor(sensor::Sensor *sensor) { upper_bound_sensor_ = sensor; }
  void set_lower_bound_sensor(sensor::Sensor *sensor) { lower_bound_sensor_ = sensor; }

  void setup() override;
  void dump_config() override;

 protected:
  void on_input_state_(float value);

  sensor::Sensor *input_sensor_{nullptr};
  DrdfFilter filter_;
  sensor::Sensor *upper_bound_sensor_{nullptr};
  sensor::Sensor *lower_bound_sensor_{nullptr};
};

}  // namespace drdf
}  // namespace esphome
