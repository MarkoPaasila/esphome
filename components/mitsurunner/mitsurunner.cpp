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

#include "mitsurunner.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mitsurunner {

static const char *const TAG = "mitsurunner";

void MitsurunnerDefrostAllowedSwitch::write_state(bool state) {
  if (component_ != nullptr) {
    component_->set_defrost_allowed_enabled(state);
  }
  publish_state(state);
}

void MitsurunnerManualDefrostSwitch::write_state(bool state) {
  if (component_ != nullptr && state) {
    component_->request_manual_defrost();
  }
  publish_state(false);
}

void MitsurunnerComponent::request_manual_defrost() {
  manual_defrosting_ = true;
}

void MitsurunnerComponent::setup() {
  if (allow_defrost_relay_ != nullptr) {
    allow_defrost_relay_->turn_off();
  }
  state_ = ST_RESET;
  previous_state_ = ST_RESET;
  runner_on_ = true;
  long_defrosting_ = false;
  manual_defrosting_ = false;
  max_heating_time_passed_ = false;
  state_time_passed_ = false;
  outdoor_temp_ = 0.0f;
  state_timer_duration_ms_ = 0;
  max_heating_timer_duration_ms_ = 0;
  for (int i = 0; i < DELTA_MIN_WINDOW; i++) {
    delta_min_buf_[i] = 0.0f;
  }
  delta_min_idx_ = 0;
  delta_min_count_ = 0;
  delta_min_current_ = 0.0f;
  boot_complete_start_ms_ = millis();
  boot_complete_duration_ms_ = (uint32_t) initialize_delay_sec_ * 1000;

  if (defrost_allowed_sensor_ != nullptr) {
    defrost_allowed_sensor_->publish_state(defrost_allowed_);  // Initial state so entity is announced
    last_published_defrost_allowed_ = defrost_allowed_;
  }
  if (temperature_delta_sensor_ != nullptr) {
    temperature_delta_sensor_->publish_state(0.0f);  // Initial state so entity is announced
    last_published_temperature_delta_ = 0.0f;
    temperature_delta_published_ = true;
  }
  if (defrost_allowed_switch_ != nullptr) {
    defrost_allowed_switch_->publish_state(runner_on_);  // Initial state so entity is announced
  }
  if (expose_manual_defrost_switch_) {
    manual_defrost_switch_ = new MitsurunnerManualDefrostSwitch();
    manual_defrost_switch_->set_name("Manual defrost request");
    manual_defrost_switch_->set_component(this);
    manual_defrost_switch_->publish_state(false);
    App.register_switch(manual_defrost_switch_);
  }
  enter_reset_(millis());
}

void MitsurunnerComponent::update() {
  uint32_t now_ms = millis();
  if (!timer_elapsed_(boot_complete_start_ms_, boot_complete_duration_ms_, now_ms)) {
    return;
  }
  if (outdoor_sensor_ != nullptr && outdoor_sensor_->has_state()) {
    float v = outdoor_sensor_->state;
    if (!std::isnan(v) && is_temperature_sane_(v)) {
      outdoor_temp_ = v;
      last_outdoor_valid_ms_ = now_ms;
    }
  }
  run_state_machine_(now_ms);
  publish_state_text_();
  publish_defrost_allowed_();
  if (temperature_delta_sensor_ != nullptr) {
    if (!temperature_delta_published_ || last_temperature_delta_ != last_published_temperature_delta_) {
      temperature_delta_sensor_->publish_state(last_temperature_delta_);
      last_published_temperature_delta_ = last_temperature_delta_;
      temperature_delta_published_ = true;
    }
  }
}

float MitsurunnerComponent::get_delta_min_() const {
  return delta_min_current_;
}

void MitsurunnerComponent::recompute_delta_min_() {
  if (delta_min_count_ == 0) {
    delta_min_current_ = 0.0f;
    return;
  }
  float m = delta_min_buf_[0];
  for (int i = 1; i < delta_min_count_; i++) {
    if (delta_min_buf_[i] < m) m = delta_min_buf_[i];
  }
  delta_min_current_ = m;
}

void MitsurunnerComponent::push_delta_min_(float delta) {
  if (delta_min_count_ < DELTA_MIN_WINDOW) {
    delta_min_buf_[delta_min_idx_] = delta;
    if (delta_min_count_ == 0 || delta < delta_min_current_)
      delta_min_current_ = delta;
    delta_min_idx_ = (delta_min_idx_ + 1) % DELTA_MIN_WINDOW;
    delta_min_count_++;
    return;
  }
  float old_val = delta_min_buf_[delta_min_idx_];
  delta_min_buf_[delta_min_idx_] = delta;
  delta_min_idx_ = (delta_min_idx_ + 1) % DELTA_MIN_WINDOW;
  if (old_val <= delta_min_current_)
    recompute_delta_min_();
  else if (delta < delta_min_current_)
    delta_min_current_ = delta;
}

bool MitsurunnerComponent::timer_elapsed_(uint32_t start_ms, uint32_t duration_ms, uint32_t now_ms) const {
  if (duration_ms == 0) return true;
  return (uint32_t) (now_ms - start_ms) >= duration_ms;
}

bool MitsurunnerComponent::check_state_timer_(uint32_t now_ms) {
  if (state_timer_duration_ms_ == 0) return false;
  if (timer_elapsed_(state_timer_start_ms_, state_timer_duration_ms_, now_ms)) {
    state_time_passed_ = true;
    return true;
  }
  return false;
}

void MitsurunnerComponent::start_state_timer_ms_(uint32_t ms, uint32_t now_ms) {
  state_timer_start_ms_ = now_ms;
  state_timer_duration_ms_ = ms;
  state_time_passed_ = false;
}

void MitsurunnerComponent::start_forced_defrost_timer_(uint32_t now_ms) {
  max_heating_timer_start_ms_ = now_ms;
  max_heating_timer_duration_ms_ = (uint32_t) max_heating_time_min_ * 60 * 1000;
  max_heating_time_passed_ = false;
}

void MitsurunnerComponent::stop_forced_defrost_timer_() {
  max_heating_timer_duration_ms_ = 0;
}

bool MitsurunnerComponent::is_temperature_sane_(float t) const {
  return !std::isnan(t) && t >= TEMP_SANITY_MIN && t <= TEMP_SANITY_MAX;
}

bool MitsurunnerComponent::should_enter_off_(float heat_exchanger_temp, float outdoor_temp) const {
  return heat_exchanger_temp > heat_exchanger_max_temperature_ ||
         outdoor_temp > outdoor_temperature_to_enter_off_state_ || !runner_on_;
}

bool MitsurunnerComponent::sensor_fault_detected_(float heat_exchanger_temp, float outdoor_temp,
                                                  bool heat_exchanger_has_state,
                                                  bool outdoor_has_state,
                                                  uint32_t now_ms) const {
  if (!heat_exchanger_has_state)
    return true;
  if (std::isnan(heat_exchanger_temp) || !is_temperature_sane_(heat_exchanger_temp))
    return true;
  if (!outdoor_has_state)
    return true;
  if (std::isnan(outdoor_temp) || !is_temperature_sane_(outdoor_temp))
    return true;
  if ((uint32_t) (now_ms - last_outdoor_valid_ms_) > SENSOR_STALE_MS)
    return true;
  return false;
}

void MitsurunnerComponent::enter_sensor_fault_(uint32_t now_ms) {
  if (allow_defrost_relay_) allow_defrost_relay_->turn_off();
  defrost_allowed_ = false;
  previous_state_ = state_;
  state_ = ST_SENSOR_FAULT;
  sensor_fault_timer_start_ms_ = now_ms;
  sensor_fault_timer_duration_ms_ = (uint32_t) SENSOR_FAULT_NO_DEFROST_TIME_MIN * 60 * 1000;
}

void MitsurunnerComponent::set_defrost_allowed_enabled(bool allowed) {
  runner_on_ = allowed;
  if (!allowed) {
    if (allow_defrost_relay_ != nullptr) {
      allow_defrost_relay_->turn_off();
    }
    defrost_allowed_ = false;
    state_timer_duration_ms_ = 0;
    stop_forced_defrost_timer_();
    previous_state_ = state_;
    state_ = ST_OFF;
  }
}

void MitsurunnerComponent::enter_reset_(uint32_t now_ms) {
  if (allow_defrost_relay_) allow_defrost_relay_->turn_off();
  defrost_allowed_ = false;
  long_defrosting_ = false;
  manual_defrosting_ = false;
  previous_state_ = state_;
  state_ = ST_RESET;
  start_state_timer_ms_((uint32_t) reset_sensor_delay_sec_ * 1000, now_ms);
}

void MitsurunnerComponent::enter_off_() {
  if (allow_defrost_relay_) allow_defrost_relay_->turn_off();
  defrost_allowed_ = false;
  long_defrosting_ = false;
  manual_defrosting_ = false;
  state_timer_duration_ms_ = 0;
  stop_forced_defrost_timer_();
  previous_state_ = state_;
  state_ = ST_OFF;
}

void MitsurunnerComponent::enter_idle_(uint32_t now_ms) {
  if (allow_defrost_relay_) allow_defrost_relay_->turn_off();
  defrost_allowed_ = false;
  state_timer_duration_ms_ = 0;
  previous_state_ = state_;
  state_ = ST_IDLE;
  start_forced_defrost_timer_(now_ms);
}

void MitsurunnerComponent::enter_temp_exceeded_(uint32_t now_ms) {
  if (allow_defrost_relay_) allow_defrost_relay_->turn_off();
  defrost_allowed_ = false;
  previous_state_ = state_;
  state_ = ST_TEMP_EXCEEDED;
  start_state_timer_ms_((uint32_t) temperature_delta_excess_time_min_ * 60 * 1000, now_ms);
}

void MitsurunnerComponent::enter_temp_exceeded_temp_decreasing_(uint32_t now_ms) {
  if (allow_defrost_relay_) allow_defrost_relay_->turn_off();
  defrost_allowed_ = false;
  previous_state_ = state_;
  state_ = ST_TEMP_EXCEEDED_TEMP_DECREASING;
  start_state_timer_ms_((uint32_t) temperature_delta_decreasing_excess_time_min_ * 60 * 1000, now_ms);
}

void MitsurunnerComponent::enter_start_defrosting_(uint32_t now_ms) {
  if (allow_defrost_relay_) allow_defrost_relay_->turn_on();
  defrost_allowed_ = true;
  manual_defrosting_ = false;
  previous_state_ = state_;
  state_ = ST_START_DEFROSTING;
  start_state_timer_ms_((uint32_t) defrost_timeout_min_ * 60 * 1000, now_ms);
}

void MitsurunnerComponent::enter_defrosting_started_(uint32_t now_ms) {
  if (allow_defrost_relay_) allow_defrost_relay_->turn_on();
  defrost_allowed_ = true;
  long_defrosting_ = false;
  manual_defrosting_ = false;
  previous_state_ = state_;
  state_ = ST_DEFROSTING_STARTED;
  start_state_timer_ms_((uint32_t) defrost_duration_min_ * 60 * 1000, now_ms);
  start_forced_defrost_timer_(now_ms);
}

void MitsurunnerComponent::enter_long_defrosting_started_(uint32_t now_ms) {
  if (allow_defrost_relay_) allow_defrost_relay_->turn_off();
  defrost_allowed_ = false;
  long_defrosting_ = false;
  manual_defrosting_ = false;
  previous_state_ = state_;
  state_ = ST_LONG_DEFROSTING_STARTED;
  start_state_timer_ms_((uint32_t) defrost_duration_min_ * 60 * 1000, now_ms);
  start_forced_defrost_timer_(now_ms);
}

void MitsurunnerComponent::enter_heating_min_time_(uint32_t now_ms) {
  if (allow_defrost_relay_) allow_defrost_relay_->turn_off();
  defrost_allowed_ = false;
  long_defrosting_ = false;
  manual_defrosting_ = false;
  previous_state_ = state_;
  state_ = ST_HEATING_MIN_TIME;
  start_state_timer_ms_((uint32_t) (min_heating_time_min_ - defrost_duration_min_) * 60 * 1000, now_ms);
}

void MitsurunnerComponent::publish_state_text_() {
  if (state_text_sensor_ == nullptr) return;
  if (state_ == last_published_state_) return;
  last_published_state_ = state_;
  const char *text = "Internal error";
  switch (state_) {
    case ST_SENSOR_FAULT: text = "Sensor fault"; break;
    case ST_LONG_DEFROSTING_STARTED: text = "Long defrosting"; break;
    case ST_RESET: text = "Reset"; break;
    case ST_OFF: text = "Off"; break;
    case ST_DEFROSTING_STARTED: text = "Defrosting"; break;
    case ST_HEATING_MIN_TIME: text = "Forced heating"; break;
    case ST_IDLE: text = "Idle"; break;
    case ST_TEMP_EXCEEDED: text = "Temp exceeded"; break;
    case ST_TEMP_EXCEEDED_TEMP_DECREASING: text = "Temp exceeded - decreasing"; break;
    case ST_START_DEFROSTING: text = "Starting defrosting"; break;
    default: break;
  }
  state_text_sensor_->publish_state(text);
}

void MitsurunnerComponent::publish_defrost_allowed_() {
  if (defrost_allowed_sensor_ != nullptr &&
      defrost_allowed_ != last_published_defrost_allowed_) {
    last_published_defrost_allowed_ = defrost_allowed_;
    defrost_allowed_sensor_->publish_state(defrost_allowed_);
  }
}

void MitsurunnerComponent::run_state_machine_(uint32_t now_ms) {
  if (heat_exchanger_sensor_ == nullptr || outdoor_sensor_ == nullptr) return;

  bool heat_exchanger_has_state = heat_exchanger_sensor_->has_state();
  float heat_exchanger_temp = heat_exchanger_has_state ? heat_exchanger_sensor_->state : 0.0f;
  bool outdoor_has_state = outdoor_sensor_->has_state();
  float outdoor_temp = outdoor_has_state ? outdoor_sensor_->state : outdoor_temp_;

  if (sensor_fault_detected_(heat_exchanger_temp, outdoor_temp, heat_exchanger_has_state,
                             outdoor_has_state, now_ms)) {
    if (state_ != ST_SENSOR_FAULT) {
      enter_sensor_fault_(now_ms);
    } else {
      if (timer_elapsed_(sensor_fault_timer_start_ms_, sensor_fault_timer_duration_ms_, now_ms)) {
        if (allow_defrost_relay_) allow_defrost_relay_->turn_off();
        defrost_allowed_ = false;
      }
    }
    last_temperature_delta_ = 0.0f;
    return;
  }

  if (state_ == ST_SENSOR_FAULT) {
    if (should_enter_off_(heat_exchanger_temp, outdoor_temp))
      enter_off_();
    else
      enter_reset_(now_ms);
    return;
  }

  if (!heat_exchanger_has_state) return;

  // Delta = heat exchanger − outdoor; negative when coil is colder than outside (frost condition).
  float temperature_delta = heat_exchanger_temp - outdoor_temp;
  last_temperature_delta_ = temperature_delta;

  push_delta_min_(temperature_delta);
  float delta_min = get_delta_min_();

  check_state_timer_(now_ms);
  if (max_heating_timer_duration_ms_ != 0 &&
      timer_elapsed_(max_heating_timer_start_ms_, max_heating_timer_duration_ms_, now_ms)) {
    max_heating_time_passed_ = true;
  }

  switch (state_) {
    case ST_RESET:
      if (state_time_passed_) {
        if (should_enter_off_(heat_exchanger_temp, outdoor_temp)) {
          enter_off_();
        } else if (temperature_delta >= temperature_delta_defrosting_started_) {
          enter_defrosting_started_(now_ms);
        } else {
          enter_idle_(now_ms);
        }
      }
      break;

    case ST_IDLE:
      if (should_enter_off_(heat_exchanger_temp, outdoor_temp)) {
        enter_off_();
      } else if (max_heating_time_passed_ || manual_defrosting_) {
        enter_start_defrosting_(now_ms);
      } else if (temperature_delta <= temperature_delta_to_defrost_) {
        enter_temp_exceeded_(now_ms);
      }
      break;

    case ST_OFF:
      // Hysteresis: enter OFF when outdoor > enter threshold (3°C), exit when outdoor < exit (2°C)
      if (heat_exchanger_temp < heat_exchanger_max_temperature_ &&
          outdoor_temp < outdoor_temperature_to_exit_off_state_ && runner_on_) {
        start_forced_defrost_timer_(now_ms);
        enter_idle_(now_ms);
      }
      break;

    case ST_TEMP_EXCEEDED:
      if (delta_min_count_ >= DELTA_MIN_WINDOW) {
        if ((delta_min < temperature_delta && state_time_passed_) ||
            max_heating_time_passed_ || manual_defrosting_) {
          enter_start_defrosting_(now_ms);
        } else if (delta_min >= temperature_delta && state_time_passed_) {
          enter_temp_exceeded_temp_decreasing_(now_ms);
        } else if (temperature_delta > temperature_delta_to_defrost_) {
          enter_idle_(now_ms);
        }
      } else {
        if (max_heating_time_passed_ || manual_defrosting_) {
          enter_start_defrosting_(now_ms);
        } else if (temperature_delta > temperature_delta_to_defrost_) {
          enter_idle_(now_ms);
        }
      }
      break;

    case ST_TEMP_EXCEEDED_TEMP_DECREASING:
      if (temperature_delta > temperature_delta_to_defrost_) {
        enter_idle_(now_ms);
      } else if (max_heating_time_passed_ || state_time_passed_ || manual_defrosting_) {
        enter_start_defrosting_(now_ms);
      }
      break;

    case ST_START_DEFROSTING:
      if (state_time_passed_) {
        long_defrosting_ = true;
        enter_long_defrosting_started_(now_ms);
      } else if (temperature_delta >= temperature_delta_defrosting_started_) {
        if (long_defrosting_) {
          enter_long_defrosting_started_(now_ms);
        } else {
          enter_defrosting_started_(now_ms);
        }
      }
      break;

    case ST_LONG_DEFROSTING_STARTED:
    case ST_DEFROSTING_STARTED:
      if (state_time_passed_) {
        enter_heating_min_time_(now_ms);
      }
      break;

    case ST_HEATING_MIN_TIME:
      if (state_time_passed_) {
        enter_idle_(now_ms);
      } else if (max_heating_time_passed_) {
        enter_start_defrosting_(now_ms);
      }
      break;

    default:
      enter_idle_(now_ms);
      break;
  }
}

}  // namespace mitsurunner
}  // namespace esphome
