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
    if (have_data) {
      this->height_known_ = true;
      if (this->current_pos_ != num) {
        ESP_LOGV(TAG, "desk height: %.1f", num);
        if (this->height_sensor_ != nullptr)
          this->height_sensor_->publish_state(num);
        this->current_pos_ = num;
      }
      if (this->wake_test_active_)
        this->end_wake_test("desk responded");
    }
  }
  if (this->moving_) {
    if (this->move_settling_) {
      uint8_t buf[] = {0xa5, 0, 0, 0xff, 0xff};
      if (millis() - this->move_last_send_ >= JSDRIVE_MOVE_SEND_INTERVAL) {
        this->desk_uart_->write_array(buf, 5);
        this->move_last_send_ = millis();
      }
      if (millis() - this->move_settled_at_ >= JSDRIVE_MOVE_SETTLE_TIME) {
        float error = this->target_pos_ - this->current_pos_;
        if (fabsf(error) <= JSDRIVE_MOVE_TOLERANCE) {
          ESP_LOGV(TAG, "move target %.1f reached at %.1f", this->target_pos_,
                   this->current_pos_);
          this->moving_ = false;
          this->current_operation = JSDRIVE_OPERATION_IDLE;
        } else {
          this->move_dir_ = error > 0;
          float half_error = fabsf(error) / 2.0f;
          float margin = half_error < JSDRIVE_BRAKING_MARGIN ? half_error
                                                              : JSDRIVE_BRAKING_MARGIN;
          this->stop_pos_ = this->target_pos_ +
                            (this->move_dir_ ? -margin : margin);
          this->move_settling_ = false;
          ESP_LOGV(TAG, "correcting move %s: current %.1f, target %.1f, brake %.1f",
                   this->move_dir_ ? "up" : "down", this->current_pos_,
                   this->target_pos_, this->stop_pos_);
        }
      }
    } else if ((this->move_dir_ && (this->current_pos_ >= this->stop_pos_)) ||
               (!this->move_dir_ && (this->current_pos_ <= this->stop_pos_))) {
      ESP_LOGV(TAG, "move target %.1f braking at %.1f", this->target_pos_,
               this->current_pos_);
      this->move_settling_ = true;
      this->move_settled_at_ = millis();
      this->move_last_send_ = this->move_settled_at_ - JSDRIVE_MOVE_SEND_INTERVAL;
    } else {
      static uint8_t buf[] = {0xa5, 0, 0, 0xff, 0xff};
      buf[2] = (this->move_dir_ ? 0x20 : 0x40);
      buf[3] = 0xff - buf[2];
      if (millis() - this->move_last_send_ >= JSDRIVE_MOVE_SEND_INTERVAL) {
        this->desk_uart_->write_array(buf, 5);
        this->move_last_send_ = millis();
      }
    }
  }
  if (this->wake_test_active_) {
    if (millis() - this->wake_test_started_ >= this->wake_test_duration_) {
      this->end_wake_test("timed out");
    } else if (millis() - this->wake_test_last_send_ >= JSDRIVE_WAKE_TEST_SEND_INTERVAL) {
      uint8_t buf[] = {0xa5, 0, 0, 0xff, 0xff};
      this->desk_uart_->write_array(buf, 5);
      this->wake_test_last_send_ = millis();
    }
  }
  if (this->preset_buttons_ != 0) {
    uint32_t elapsed = millis() - this->preset_started_;
    if (elapsed >= JSDRIVE_PRESET_HOLD_TIME) {
      uint8_t buf[] = {0xa5, 0, 0, 0xff, 0xff};
      ESP_LOGV(TAG, "preset %02x: releasing after %u ms and %u frames",
               this->preset_buttons_, (unsigned) elapsed, this->preset_send_count_);
      this->desk_uart_->write_array(buf, 5);
      this->preset_buttons_ = 0;
      if (this->desk_pin_ != nullptr) {
        ESP_LOGV(TAG, "desk wake pin set: low");
        this->desk_pin_->digital_write(false);
      }
    } else if (millis() - this->preset_last_send_ >= JSDRIVE_PRESET_SEND_INTERVAL) {
      uint8_t buf[] = {0xa5, 0, this->preset_buttons_,
                       (uint8_t) (0xff - this->preset_buttons_), 0xff};
      if (this->preset_send_count_ == 0) {
        ESP_LOGV(TAG, "preset %02x: sending first preset frame", this->preset_buttons_);
      }
      this->desk_uart_->write_array(buf, 5);
      this->preset_last_send_ = millis();
      this->preset_send_count_++;
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
      if (!this->buttons_logged_ || buttons != this->last_logged_buttons_) {
        ESP_LOGV(TAG, "remote frame: a5 %02x %02x %02x %02x", d[0], d[1],
                 d[2], d[3]);
        this->last_logged_buttons_ = buttons;
        this->buttons_logged_ = true;
      }
      have_data = true;
      this->rem_buffer_.clear();
    }
    if (have_data) {
      if (this->wake_test_active_)
        this->end_wake_test("physical controller pressed");
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
      if (!this->moving_ && this->preset_buttons_ == 0 &&
          this->desk_uart_ != nullptr) {
        uint8_t buf[] = {0xa5, 0, buttons, (uint8_t)(0xff - buttons), 0xff};
        this->desk_uart_->write_array(buf, 5);
      }
    }
  }
  if (this->remote_pin_ != nullptr && this->preset_buttons_ == 0) {
    bool pin_state = this->remote_pin_->digital_read();
    if (pin_state != this->remote_pin_prev_) {
      this->remote_pin_prev_ = pin_state;
      ESP_LOGV(TAG, "remote wake pin changed: %s", pin_state ? "high" : "low");
      if (this->desk_pin_ != nullptr) {
        ESP_LOGV(TAG, "desk wake pin set: %s", pin_state ? "high" : "low");
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
  if (this->desk_uart_ == nullptr) {
    ESP_LOGW(TAG, "ignoring move to %.1f: desk UART unavailable", height);
    return;
  }
  if (!this->height_known_) {
    ESP_LOGW(TAG, "ignoring move to %.1f: no valid desk height received", height);
    return;
  }
  this->moving_ = true;
  this->move_settling_ = false;
  this->target_pos_ = height;
  this->move_dir_ = height > this->current_pos_;
  this->stop_pos_ = this->move_dir_ ? height - JSDRIVE_BRAKING_MARGIN
                                    : height + JSDRIVE_BRAKING_MARGIN;
  this->move_last_send_ = millis() - JSDRIVE_MOVE_SEND_INTERVAL;
  this->current_operation =
      this->move_dir_ ? JSDRIVE_OPERATION_RAISING : JSDRIVE_OPERATION_LOWERING;
  ESP_LOGV(TAG, "starting move %s: current %.1f, target %.1f, brake %.1f",
           this->move_dir_ ? "up" : "down", this->current_pos_, height,
           this->stop_pos_);
}

void JSDrive::stop() {
  ESP_LOGV(TAG, "stopping move at %.1f", this->current_pos_);
  if (this->wake_test_active_)
    this->end_wake_test("stopped");
  else if (this->desk_uart_ != nullptr) {
    uint8_t buf[] = {0xa5, 0, 0, 0xff, 0xff};
    this->desk_uart_->write_array(buf, 5);
  }
  this->moving_ = false;
  this->move_settling_ = false;
  this->current_operation = JSDRIVE_OPERATION_IDLE;
}

void JSDrive::press_preset1() {
  this->press_preset(0x02);
}

void JSDrive::press_preset2() {
  this->press_preset(0x04);
}

void JSDrive::press_preset3() {
  this->press_preset(0x08);
}

void JSDrive::press_preset4() {
  this->press_preset(0x10);
}

void JSDrive::press_preset(uint8_t buttons) {
  if (this->desk_uart_ != nullptr) {
    if (this->wake_test_active_)
      this->end_wake_test("preset requested");
    ESP_LOGV(TAG, "pressing preset button: %02x", buttons);
    if (this->desk_pin_ != nullptr) {
      ESP_LOGV(TAG, "desk wake pin set: high");
      this->desk_pin_->digital_write(true);
    }
    this->moving_ = false;
    this->current_operation = JSDRIVE_OPERATION_IDLE;
    this->preset_started_ = millis();
    this->preset_buttons_ = buttons;
    this->preset_last_send_ = this->preset_started_ - JSDRIVE_PRESET_SEND_INTERVAL;
    this->preset_send_count_ = 0;
  }
}

void JSDrive::wake_desk_for(uint32_t duration_ms) {
  if (this->desk_uart_ == nullptr || this->desk_pin_ == nullptr) {
    ESP_LOGW(TAG, "ignoring wake test: desk UART or wake pin unavailable");
    return;
  }
  if (this->moving_ || this->preset_buttons_ != 0 || this->wake_test_active_) {
    ESP_LOGW(TAG, "ignoring wake test: desk operation already active");
    return;
  }
  ESP_LOGV(TAG, "starting desk wake test for %u ms", (unsigned) duration_ms);
  this->wake_test_active_ = true;
  this->wake_test_started_ = millis();
  this->wake_test_duration_ = duration_ms;
  this->wake_test_last_send_ = this->wake_test_started_ - JSDRIVE_WAKE_TEST_SEND_INTERVAL;
  this->desk_pin_->digital_write(true);
}

void JSDrive::end_wake_test(const char *reason) {
  if (!this->wake_test_active_)
    return;
  uint8_t buf[] = {0xa5, 0, 0, 0xff, 0xff};
  this->desk_uart_->write_array(buf, 5);
  this->desk_pin_->digital_write(false);
  ESP_LOGV(TAG, "desk wake test %s after %u ms", reason,
           (unsigned) (millis() - this->wake_test_started_));
  this->wake_test_active_ = false;
}

} // namespace jsdrive
} // namespace esphome
