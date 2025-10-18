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
    /*ESP_LOGV(TAG, "unknown digit: %02f", segments & 0x7f);*/
  }
  return -1;
}

void JSDrive::setup() {
  if (this->remote_pin_ != nullptr) {
    this->remote_pin_->setup();
    this->remote_pin_prev_ = this->remote_pin_->digital_read();
  }
  if (this->desk_pin_ != nullptr) {
    this->desk_pin_->setup();
    this->desk_pin_->digital_write(false);  // Initialize LOW
  }
}

void JSDrive::loop() {
  uint8_t c;
  bool have_data = false;
  if (this->desk_uart_ != nullptr) {
    float num;
    while (this->desk_uart_->available()) {
      this->desk_uart_->read_byte(&c);
      if (this->remote_uart_ != nullptr)
        this->remote_uart_->write_byte(c);
      if (!this->desk_rx_) {
        if (c == 0x5a)
          this->desk_rx_ = true;
        continue;
      }
      this->desk_buffer_.push_back(c);
      if (this->desk_buffer_.size() < this->message_length_ - 1)
        continue;
      this->desk_rx_ = false;
      uint8_t *d = this->desk_buffer_.data();
      uint8_t csum = d[0] + d[1] + d[2];
      if (this->message_length_ > 5)
        csum += d[3];
      uint8_t tcsum = this->message_length_ == 5 ? d[3] : d[4];
      if (csum != tcsum) {
        ESP_LOGE(TAG, "desk checksum mismatch: %02x != %02x", csum, tcsum);
        this->desk_buffer_.clear();
        continue;
      }
      do {
        if ((this->message_length_ == 6) && (d[3] != 1)) {
          ESP_LOGV(TAG, "unknown message type %02x", d[3]);
          break;
        }
        if ((d[0] | d[1] | d[2]) == 0)
          break;
        int d0 = segs_to_num(d[0]);
        int d1 = segs_to_num(d[1]);
        int d2 = segs_to_num(d[2]);
        if (d0 < 0 || d1 < 0 || d2 < 0)
          break;
        num = segs_to_num(d[0]) * 100 + segs_to_num(d[1]) * 10 +
              segs_to_num(d[2]);
        have_data = true;
        if (d[1] & 0x80)
          num /= 10.0;
      } while (false);
      this->desk_buffer_.clear();
    }
    if (have_data && (this->height_sensor_ != nullptr) &&
        (this->current_pos_ != num)) {
      this->height_sensor_->publish_state(num);
      this->current_pos_ = num;
    }
  }
  if (this->moving_) {
    if ((this->move_dir_ && (this->current_pos_ >= this->target_pos_)) ||
        (!this->move_dir_ && (this->current_pos_ <= this->target_pos_))) {
      this->moving_ = false;
    } else {
      static uint8_t buf[] = {0xa5, 0, 0, 0, 0xff};
      buf[2] = (this->move_dir_ ? 0x20 : 0x40);
      buf[3] = 0xff - buf[2];
      this->desk_uart_->write_array(buf, 5);
    }
  }
  uint8_t buttons = 0;
  have_data = false;
  if (this->remote_uart_ != nullptr) {
    while (this->remote_uart_->available()) {
      this->remote_uart_->read_byte(&c);
      if (!this->rem_rx_) {
        if (c == 0xa5)
          this->rem_rx_ = true;
        continue;
      }
      this->rem_buffer_.push_back(c);
      if (this->rem_buffer_.size() < 4)
        continue;
      this->rem_rx_ = false;
      uint8_t *d = this->rem_buffer_.data();
      uint8_t csum = d[0] + d[1] + d[2];
      if (csum != d[3]) {
        ESP_LOGE(TAG, "remote checksum mismatch: %02x != %02x", csum, d[3]);
        this->rem_buffer_.clear();
        continue;
      }
      buttons = d[1];
      ESP_LOGI(TAG, "Received buttons from remote: 0x%02X", buttons);
      have_data = true;
      this->rem_buffer_.clear();
    }
    if (have_data) {
      if (this->up_bsensor_ != nullptr)
        this->up_bsensor_->publish_state(buttons & 0x20);
      if (this->down_bsensor_ != nullptr)
        this->down_bsensor_->publish_state(buttons & 0x40);
      if (this->memory1_bsensor_ != nullptr)
        this->memory1_bsensor_->publish_state(buttons & 2);
      if (this->memory2_bsensor_ != nullptr)
        this->memory2_bsensor_->publish_state(buttons & 4);
      if (this->memory3_bsensor_ != nullptr)
        this->memory3_bsensor_->publish_state(buttons & 8);
      if (!this->moving_ && this->desk_uart_ != nullptr) {
        uint8_t buf[] = {0xa5, 0, buttons, (uint8_t)(0xff - buttons), 0xff};
        this->desk_uart_->write_array(buf, 5);
      }
    }
  }
  if (this->remote_pin_ != nullptr) {
    bool pin_state = this->remote_pin_->digital_read();
    if (pin_state != this->remote_pin_prev_) {
      this->remote_pin_prev_ = pin_state;
      if (this->desk_pin_ != nullptr) {
        this->desk_pin_->digital_write(pin_state);
      }
    }
  }
}

void JSDrive::dump_config() {
  ESP_LOGCONFIG(TAG, "JSDrive Desk");
  if (this->desk_uart_ != nullptr)
    ESP_LOGCONFIG(TAG, "  Message Length: %d", this->message_length_);
  LOG_PIN("Remote Pin", this->remote_pin_);
  LOG_PIN("Desk Pin", this->desk_pin_);
  LOG_SENSOR("", "Height", this->height_sensor_);
  LOG_BINARY_SENSOR("  ", "Up", this->up_bsensor_);
  LOG_BINARY_SENSOR("  ", "Down", this->down_bsensor_);
  LOG_BINARY_SENSOR("  ", "Memory1", this->memory1_bsensor_);
  LOG_BINARY_SENSOR("  ", "Memory2", this->memory2_bsensor_);
  LOG_BINARY_SENSOR("  ", "Memory3", this->memory3_bsensor_);
}

void JSDrive::move_to(float height) {
  if (this->desk_uart_ == nullptr)
    return;
  this->moving_ = true;
  this->target_pos_ = height;
  this->move_dir_ = height > this->current_pos_;
  this->current_operation =
      this->move_dir_ ? JSDRIVE_OPERATION_RAISING : JSDRIVE_OPERATION_LOWERING;
}

void JSDrive::stop() {
  this->moving_ = false;
  this->current_operation = JSDRIVE_OPERATION_IDLE;
}

void JSDrive::press_preset1() {
  if (this->desk_pin_ != nullptr) {
    this->desk_pin_->digital_write(true);
  }
  if (this->desk_uart_ != nullptr) {
    uint8_t buttons = 2;
    ESP_LOGI(TAG, "Sending preset1 to desk: buttons=0x%02X", buttons);
    uint8_t buf[] = {0xa5, 0, buttons, (uint8_t)(0xff - buttons), 0xff};
    this->desk_uart_->write_array(buf, 5);
    delay(100);
  }
  if (this->desk_pin_ != nullptr) {
    delay(100);
    this->desk_pin_->digital_write(false);
  }
}

void JSDrive::press_preset2() {
  if (this->desk_pin_ != nullptr) {
    this->desk_pin_->digital_write(true);
  }
  if (this->desk_uart_ != nullptr) {
    uint8_t buttons = 4;
    ESP_LOGI(TAG, "Sending preset2 to desk: buttons=0x%02X", buttons);
    uint8_t buf[] = {0xa5, 0, buttons, (uint8_t)(0xff - buttons), 0xff};
    this->desk_uart_->write_array(buf, 5);
  }
  if (this->desk_pin_ != nullptr) {
    delay(100);
    this->desk_pin_->digital_write(false);
  }
}

void JSDrive::press_preset3() {
  if (this->desk_pin_ != nullptr) {
    this->desk_pin_->digital_write(true);
  }
  if (this->desk_uart_ != nullptr) {
    uint8_t buttons = 8;
    ESP_LOGI(TAG, "Sending preset3 to desk: buttons=0x%02X", buttons);
    uint8_t buf[] = {0xa5, 0, buttons, (uint8_t)(0xff - buttons), 0xff};
    this->desk_uart_->write_array(buf, 5);
  }
  if (this->desk_pin_ != nullptr) {
    delay(100);
    this->desk_pin_->digital_write(false);
  }
}

void JSDrive::wake_desk() {
  if (this->desk_pin_ != nullptr) {
    this->desk_pin_->digital_write(true);
    delay(10);  // Brief pulse
    this->desk_pin_->digital_write(false);
  }
}

} // namespace jsdrive
} // namespace esphome
