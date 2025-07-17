#include "jsdrive.h"
#include "esphome/core/log.h"

namespace esphome {
namespace jsdrive {

static const char *const TAG = "jsdrive";

const char *jsdrive_operation_to_str(JSDriveOperation op) {
  switch (op) {
  case JSDRIVE_OPERATION_IDLE:
    return "IDLE";
  case JSDRIVE_OPERATION_RAISING:
    return "RAISING";
  case JSDRIVE_OPERATION_LOWERING:
    return "LOWERING";
  default:
    return "UNKNOWN";
  }
}

static int segs_to_num(uint8_t segments) {
  switch (segments & 0x7f) {
  case 0x3f:
    return 0;
  case 0x06:
    return 1;
  case 0x5b:
    return 2;
  case 0x4f:
    return 3;
  case 0x67:
    return 4;
  case 0x6d:
    return 5;
  case 0x7d:
    return 6;
  case 0x07:
    return 7;
  case 0x7f:
    return 8;
  case 0x6f:
    return 9;
  default:
    ESP_LOGV(TAG, "unknown digit: %02f", segments & 0x7f);
  }
  return -1;
}


void JSDrive::dump_config() {
  ESP_LOGCONFIG(TAG, "JSDrive Desk");
  if (this->desk_uart_ != nullptr)
    ESP_LOGCONFIG(TAG, "  Message Length: %d", this->message_length_);
  LOG_SENSOR("", "Height", this->height_sensor_);
  LOG_BINARY_SENSOR("  ", "Up", this->up_bsensor_);
  LOG_BINARY_SENSOR("  ", "Down", this->down_bsensor_);
  LOG_BINARY_SENSOR("  ", "Memory1", this->memory1_bsensor_);
  LOG_BINARY_SENSOR("  ", "Memory2", this->memory2_bsensor_);
  LOG_BINARY_SENSOR("  ", "Memory3", this->memory3_bsensor_);
}

void JSDrive::move_to(float height) {
  ESP_LOGD(TAG, "move_to called: target=%.1f, current=%.1f", height,
           this->current_pos_);

  if (this->desk_uart_ == nullptr) {
    ESP_LOGE(TAG, "move_to: desk_uart is null");
    return;
  }

  this->moving_ = true;
  this->target_pos_ = height;
  this->move_dir_ = height > this->current_pos_;
  this->current_operation =
      this->move_dir_ ? JSDRIVE_OPERATION_RAISING : JSDRIVE_OPERATION_LOWERING;

  ESP_LOGD(TAG, "Movement started: direction=%s, moving_=%d",
           this->move_dir_ ? "UP" : "DOWN", this->moving_);
}

void JSDrive::stop() {
  this->moving_ = false;
  this->current_operation = JSDRIVE_OPERATION_IDLE;
}

void JSDrive::simulate_button_press(uint8_t button_mask) {
  ESP_LOGD(TAG, "simulate_button_press called with mask: 0x%02x", button_mask);

  if (this->desk_uart_ == nullptr) {
    ESP_LOGE(TAG, "simulate_button_press: desk_uart is null");
    return;
  }

  static uint8_t buf[] = {0xa5, 0, 0, 0, 0xff};
  buf[2] = button_mask;
  buf[3] = 0xff - button_mask;

  ESP_LOGD(TAG, "Sending button command: %02x %02x %02x %02x %02x", buf[0],
           buf[1], buf[2], buf[3], buf[4]);

  this->desk_uart_->write_array(buf, 5);

  if (this->up_bsensor_ != nullptr)
    this->up_bsensor_->publish_state(button_mask & JSDRIVE_BUTTON_UP);
  if (this->down_bsensor_ != nullptr)
    this->down_bsensor_->publish_state(button_mask & JSDRIVE_BUTTON_DOWN);
  if (this->memory1_bsensor_ != nullptr)
    this->memory1_bsensor_->publish_state(button_mask & JSDRIVE_BUTTON_MEMORY1);
  if (this->memory2_bsensor_ != nullptr)
    this->memory2_bsensor_->publish_state(button_mask & JSDRIVE_BUTTON_MEMORY2);
  if (this->memory3_bsensor_ != nullptr)
    this->memory3_bsensor_->publish_state(button_mask & JSDRIVE_BUTTON_MEMORY3);
}
} // namespace jsdrive
} // namespace esphome
