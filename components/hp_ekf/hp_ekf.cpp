#include "hp_ekf.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hp_ekf {

static const char *const TAG = "hp_ekf";

void HpEkfComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HP-EKF component");
  // Add your initialization here
}

void HpEkfComponent::update() {
  // Called at update_interval - add your periodic logic here
  ESP_LOGD(TAG, "HP-EKF update");
}

void HpEkfComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "HP-EKF component");
  ESP_LOGCONFIG(TAG, "  Update interval: %u ms", this->get_interval());
}

}  // namespace hp_ekf
}  // namespace esphome
