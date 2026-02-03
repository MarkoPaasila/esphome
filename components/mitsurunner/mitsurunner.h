/*
 * Copyright (c) 2021 Veli Matti Lastumäki (Velsku at lampopumput.info),
 * Joonas Ihonen (puu at lampopumput.info),
 * Ilkka Roivainen (iro at lampopumput.info)
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/core/time.h"

namespace esphome {
namespace mitsurunner {

class MitsurunnerComponent;

class MitsurunnerDefrostAllowedSwitch : public switch_::Switch {
 public:
  void set_component(MitsurunnerComponent *component) { component_ = component; }

 protected:
  void write_state(bool state) override;

 private:
  MitsurunnerComponent *component_{nullptr};
};

class MitsurunnerManualDefrostSwitch : public switch_::Switch {
 public:
  void set_component(MitsurunnerComponent *component) { component_ = component; }

 protected:
  void write_state(bool state) override;

 private:
  MitsurunnerComponent *component_{nullptr};
};

enum MitsurunnerState {
  ST_SENSOR_FAULT = -6,
  ST_LONG_DEFROSTING_STARTED = -5,
  ST_RESET = -4,
  ST_OFF = -3,
  ST_DEFROSTING_STARTED = -2,
  ST_HEATING_MIN_TIME = -1,
  ST_IDLE = 0,
  ST_TEMP_EXCEEDED = 1,
  ST_TEMP_EXCEEDED_TEMP_DECREASING = 2,
  ST_START_DEFROSTING = 3,
};

class MitsurunnerComponent : public PollingComponent {
 public:
  void set_heat_exchanger_sensor(sensor::Sensor *s) { heat_exchanger_sensor_ = s; }
  void set_outdoor_sensor(sensor::Sensor *s) { outdoor_sensor_ = s; }
  void set_allow_defrost_relay(switch_::Switch *s) { allow_defrost_relay_ = s; }
  void set_expose_state_text_sensor(bool b) { expose_state_text_sensor_ = b; }
  void set_defrost_allowed_sensor(binary_sensor::BinarySensor *s) {
    defrost_allowed_sensor_ = s;
  }
  void set_temperature_delta_sensor(sensor::Sensor *s) { temperature_delta_sensor_ = s; }
  void set_defrost_allowed_switch(MitsurunnerDefrostAllowedSwitch *s) {
    defrost_allowed_switch_ = s;
  }
  void set_expose_manual_defrost_switch(bool b) { expose_manual_defrost_switch_ = b; }

  void set_defrost_allowed_enabled(bool allowed);

  void request_manual_defrost();

  void set_temperature_delta_to_defrost(float v) { temperature_delta_to_defrost_ = v; }
  void set_outdoor_temperature_to_enter_off_state(float v) { outdoor_temperature_to_enter_off_state_ = v; }
  void set_outdoor_temperature_to_exit_off_state(float v) { outdoor_temperature_to_exit_off_state_ = v; }
  void set_heat_exchanger_max_temperature(float v) { heat_exchanger_max_temperature_ = v; }
  void set_temperature_delta_defrosting_started(float v) { temperature_delta_defrosting_started_ = v; }
  void set_temperature_delta_excess_time_min(int v) { temperature_delta_excess_time_min_ = v; }
  void set_temperature_delta_decreasing_excess_time_min(int v) { temperature_delta_decreasing_excess_time_min_ = v; }
  void set_max_heating_time_min(int v) { max_heating_time_min_ = v; }
  void set_min_heating_time_min(int v) { min_heating_time_min_ = v; }
  void set_defrost_duration_min(int v) { defrost_duration_min_ = v; }
  void set_defrost_timeout_min(int v) { defrost_timeout_min_ = v; }
  void set_reset_sensor_delay_sec(int v) { reset_sensor_delay_sec_ = v; }
  void set_initialize_delay_sec(int v) { initialize_delay_sec_ = v; }

  void setup() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_runner_on(bool on) { runner_on_ = on; }
  void set_long_defrosting(bool v) { long_defrosting_ = v; }
  void set_manual_defrosting(bool v) { manual_defrosting_ = v; }
  bool get_runner_on() const { return runner_on_; }
  bool get_long_defrosting() const { return long_defrosting_; }
  bool get_manual_defrosting() const { return manual_defrosting_; }

 private:
  sensor::Sensor *heat_exchanger_sensor_{nullptr};
  sensor::Sensor *outdoor_sensor_{nullptr};
  switch_::Switch *allow_defrost_relay_{nullptr};
  text_sensor::TextSensor *state_text_sensor_{nullptr};  // created in setup() when expose_state_text_sensor_
  binary_sensor::BinarySensor *defrost_allowed_sensor_{nullptr};
  sensor::Sensor *temperature_delta_sensor_{nullptr};
  MitsurunnerDefrostAllowedSwitch *defrost_allowed_switch_{nullptr};
  MitsurunnerManualDefrostSwitch *manual_defrost_switch_{nullptr};
  bool expose_state_text_sensor_{true};
  bool expose_manual_defrost_switch_{true};
  bool defrost_allowed_{false};
  float last_temperature_delta_{0.0f};

  float temperature_delta_to_defrost_{-5.0f};
  float outdoor_temperature_to_enter_off_state_{3.0f};
  float outdoor_temperature_to_exit_off_state_{2.0f};
  float heat_exchanger_max_temperature_{10.0f};
  float temperature_delta_defrosting_started_{1.0f};
  int temperature_delta_excess_time_min_{8};
  int temperature_delta_decreasing_excess_time_min_{5};
  int max_heating_time_min_{180};
  int min_heating_time_min_{50};
  int defrost_duration_min_{30};
  int defrost_timeout_min_{10};
  int reset_sensor_delay_sec_{25};
  int initialize_delay_sec_{60};
  static constexpr float TEMP_SANITY_MIN = -60.0f;
  static constexpr float TEMP_SANITY_MAX = 60.0f;
  static constexpr uint32_t SENSOR_STALE_MS = 5 * 60 * 1000;   // 5 minutes
  static constexpr int SENSOR_FAULT_NO_DEFROST_TIME_MIN = 60;

  MitsurunnerState state_{ST_RESET};
  MitsurunnerState previous_state_{ST_RESET};
  MitsurunnerState last_published_state_{static_cast<MitsurunnerState>(-10)};  // sentinel so first run publishes
  bool last_published_defrost_allowed_{true};  // opposite of initial defrost_allowed_ so first run publishes
  float last_published_temperature_delta_{0.0f};
  bool temperature_delta_published_{false};
  bool runner_on_{true};
  bool long_defrosting_{false};
  bool manual_defrosting_{false};
  bool max_heating_time_passed_{false};
  bool state_time_passed_{false};
  float outdoor_temp_{0.0f};
  uint32_t state_timer_start_ms_{0};
  uint32_t state_timer_duration_ms_{0};
  uint32_t max_heating_timer_start_ms_{0};
  uint32_t max_heating_timer_duration_ms_{0};
  uint32_t boot_complete_start_ms_{0};
  uint32_t boot_complete_duration_ms_{0};
  uint32_t sensor_fault_timer_start_ms_{0};
  uint32_t sensor_fault_timer_duration_ms_{0};
  uint32_t last_outdoor_valid_ms_{0};
  static constexpr int DELTA_MIN_WINDOW = 5;
  float delta_min_buf_[DELTA_MIN_WINDOW];
  float delta_min_current_{0.0f};
  int delta_min_idx_{0};
  int delta_min_count_{0};

  void run_state_machine_(uint32_t now_ms);
  float get_delta_min_() const;
  void push_delta_min_(float delta);
  void publish_state_text_();
  void publish_defrost_allowed_();
  bool check_state_timer_(uint32_t now_ms);
  bool timer_elapsed_(uint32_t start_ms, uint32_t duration_ms, uint32_t now_ms) const;
  bool should_enter_off_(float heat_exchanger_temp, float outdoor_temp) const;
  bool is_temperature_sane_(float t) const;
  bool sensor_fault_detected_(float heat_exchanger_temp, float outdoor_temp,
                              bool heat_exchanger_has_state, bool outdoor_has_state,
                              uint32_t now_ms) const;
  void enter_sensor_fault_(uint32_t now_ms);
  void enter_reset_(uint32_t now_ms);
  void enter_off_();
  void enter_idle_(uint32_t now_ms);
  void enter_temp_exceeded_(uint32_t now_ms);
  void enter_temp_exceeded_temp_decreasing_(uint32_t now_ms);
  void enter_start_defrosting_(uint32_t now_ms);
  void enter_defrosting_started_(uint32_t now_ms);
  void enter_long_defrosting_started_(uint32_t now_ms);
  void enter_heating_min_time_(uint32_t now_ms);

  void start_state_timer_ms_(uint32_t ms, uint32_t now_ms);
  void start_forced_defrost_timer_(uint32_t now_ms);
  void stop_forced_defrost_timer_();
  void recompute_delta_min_();
};

}  // namespace mitsurunner
}  // namespace esphome
