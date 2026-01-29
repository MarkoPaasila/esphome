#include "hp_ukf.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hp_ukf {

static const char *const TAG = "hp_ukf";

void HpUkfComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HP-UKF component");
  // Add your initialization here
}

void HpUkfComponent::update() {
  // Called at update_interval - add your periodic logic here
  ESP_LOGD(TAG, "HP-UKF update");
}

void HpUkfComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "HP-UKF component");
  ESP_LOGCONFIG(TAG, "  Update interval: %u ms", this->get_interval());
}

}  // namespace hp_ukf
}  // namespace esphome
