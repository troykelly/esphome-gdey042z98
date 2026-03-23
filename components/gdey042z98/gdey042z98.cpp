// ESPHome display driver for Good Display GDEY042Z98
// 4.2" tri-color (black/white/red) e-paper, SSD1683 controller
// Based on GxEPD2_420c_GDEY042Z98 by Jean-Marc Zingg
// https://github.com/ZinggJM/GxEPD2
//
// Panel: GDEY042Z98 : https://www.good-display.com/product/387.html
// Controller: SSD1683 : https://v4.cecdn.yun300.cn/100001_1909185148/SSD1683.PDF

#include "gdey042z98.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace gdey042z98 {

static const char *const TAG = "gdey042z98";

float GDEY042Z98::get_setup_priority() const { return setup_priority::PROCESSOR; }

void GDEY042Z98::setup() {
  ESP_LOGCONFIG(TAG, "Setting up GDEY042Z98 e-paper display...");

  // Setup pins
  this->dc_pin_->setup();
  this->dc_pin_->digital_write(false);
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
  }
  if (this->busy_pin_ != nullptr) {
    this->busy_pin_->setup();
  }

  // Setup SPI
  this->spi_setup();

  // Allocate buffer: two planes (black/white + red)
  this->init_internal_(2 * EPD_PLANE_SIZE);
  if (this->buffer_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate display buffer!");
    this->mark_failed();
    return;
  }

  // Fill buffer with white (black plane = 0xFF, red plane = 0x00)
  memset(this->buffer_, 0xFF, EPD_PLANE_SIZE);                    // black/white plane: all white
  memset(this->buffer_ + EPD_PLANE_SIZE, 0x00, EPD_PLANE_SIZE);   // red plane: no red

  ESP_LOGD(TAG, "Buffer allocated: %u bytes (%u per plane)", 2 * EPD_PLANE_SIZE, EPD_PLANE_SIZE);
}

void GDEY042Z98::update() {
  // Let ESPHome drawing routines fill the buffer
  this->do_update_();

  // Initialize display if needed
  this->init_display_();

  // Write black/white buffer to RAM (command 0x24)
  ESP_LOGD(TAG, "Writing black/white buffer to display RAM...");
  this->write_buffer_(0x24, this->buffer_, EPD_PLANE_SIZE);

  // Write red buffer to RAM (command 0x26)
  // The red plane needs to be inverted when sending to the controller
  // In GxEPD2: writeImage for color uses !invert, which inverts the data
  ESP_LOGD(TAG, "Writing red buffer to display RAM...");
  this->set_partial_ram_area_(0, 0, EPD_WIDTH, EPD_HEIGHT);
  this->command_(0x26);
  this->start_data_();
  for (uint32_t i = 0; i < EPD_PLANE_SIZE; i++) {
    this->write_byte(~this->buffer_[EPD_PLANE_SIZE + i]);
  }
  this->end_data_();

  // Trigger full display update
  ESP_LOGD(TAG, "Triggering display update...");
  this->update_full_();

  ESP_LOGD(TAG, "Display update complete.");
}

void GDEY042Z98::on_safe_shutdown() {
  ESP_LOGD(TAG, "Safe shutdown - entering deep sleep...");
  this->deep_sleep_();
}

void GDEY042Z98::dump_config() {
  LOG_DISPLAY("", "GDEY042Z98 E-Paper", this);
  ESP_LOGCONFIG(TAG, "  Model: Good Display GDEY042Z98");
  ESP_LOGCONFIG(TAG, "  Controller: SSD1683");
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", EPD_WIDTH, EPD_HEIGHT);
  ESP_LOGCONFIG(TAG, "  Colors: Black, White, Red");
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  Busy Pin: ", this->busy_pin_);
  ESP_LOGCONFIG(TAG, "  Reset Duration: %ums", this->reset_duration_);
  LOG_UPDATE_INTERVAL(this);
}

void GDEY042Z98::fill(Color color) {
  uint8_t bw_value, red_value;

  if (color.is_on()) {
    // Black
    bw_value = 0x00;
    red_value = 0x00;
  } else if (color.red > 127 && color.green < 128 && color.blue < 128) {
    // Red
    bw_value = 0xFF;
    red_value = 0xFF;
  } else {
    // White (default)
    bw_value = 0xFF;
    red_value = 0x00;
  }

  memset(this->buffer_, bw_value, EPD_PLANE_SIZE);
  memset(this->buffer_ + EPD_PLANE_SIZE, red_value, EPD_PLANE_SIZE);
}

void GDEY042Z98::draw_absolute_pixel_internal(int x, int y, Color color) {
  if (x < 0 || x >= EPD_WIDTH || y < 0 || y >= EPD_HEIGHT)
    return;

  uint32_t byte_pos = (y * EPD_WIDTH + x) / 8;
  uint8_t bit_mask = 0x80 >> (x % 8);

  // In the black/white plane: 1 = white, 0 = black
  // In the red plane: 1 = red, 0 = not red
  if (color.red > 127 && color.green < 128 && color.blue < 128) {
    // Red pixel
    this->buffer_[byte_pos] |= bit_mask;
    this->buffer_[EPD_PLANE_SIZE + byte_pos] |= bit_mask;
  } else if (color.is_on()) {
    // Black pixel
    this->buffer_[byte_pos] &= ~bit_mask;
    this->buffer_[EPD_PLANE_SIZE + byte_pos] &= ~bit_mask;
  } else {
    // White pixel
    this->buffer_[byte_pos] |= bit_mask;
    this->buffer_[EPD_PLANE_SIZE + byte_pos] &= ~bit_mask;
  }
}

// --- SPI helpers ---

void GDEY042Z98::start_command_() {
  this->dc_pin_->digital_write(false);
  this->enable();
}

void GDEY042Z98::end_command_() { this->disable(); }

void GDEY042Z98::start_data_() {
  this->dc_pin_->digital_write(true);
  this->enable();
}

void GDEY042Z98::end_data_() { this->disable(); }

void GDEY042Z98::command_(uint8_t cmd) {
  this->start_command_();
  this->write_byte(cmd);
  this->end_command_();
}

void GDEY042Z98::data_(uint8_t val) {
  this->start_data_();
  this->write_byte(val);
  this->end_data_();
}

// --- Display control ---

void GDEY042Z98::reset_() {
  if (this->reset_pin_ == nullptr)
    return;

  this->reset_pin_->digital_write(false);
  delay(this->reset_duration_);
  this->reset_pin_->digital_write(true);
  delay(this->reset_duration_);
}

bool GDEY042Z98::wait_until_idle_(uint32_t timeout_ms) {
  if (this->busy_pin_ == nullptr) {
    delay(timeout_ms);
    return true;
  }

  uint32_t start = millis();
  while (this->busy_pin_->digital_read()) {  // HIGH = busy for SSD1683
    if (millis() - start > timeout_ms) {
      ESP_LOGW(TAG, "Timeout waiting for display idle after %ums", timeout_ms);
      return false;
    }
    delay(10);
    App.feed_wdt();
  }
  return true;
}

void GDEY042Z98::init_display_() {
  if (this->init_display_done_)
    return;

  ESP_LOGD(TAG, "Initializing display...");

  // Hardware reset
  this->reset_();

  // Software reset (0x12)
  this->command_(0x12);
  delay(10);

  // Driver output control (0x01)
  // Set gate driver output: (HEIGHT-1) = 299 = 0x012B
  this->command_(0x01);
  this->data_((EPD_HEIGHT - 1) % 256);  // 0x2B = 43
  this->data_((EPD_HEIGHT - 1) / 256);  // 0x01
  this->data_(0x00);

  // Border waveform control (0x3C)
  this->command_(0x3C);
  this->data_(0x05);

  // Read built-in temperature sensor (0x18)
  this->command_(0x18);
  this->data_(0x80);

  // Set RAM area for full screen
  this->set_partial_ram_area_(0, 0, EPD_WIDTH, EPD_HEIGHT);

  this->init_display_done_ = true;
  ESP_LOGD(TAG, "Display initialized.");
}

void GDEY042Z98::set_partial_ram_area_(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  this->command_(0x11);
  this->data_(0x03);

  this->command_(0x44);
  this->data_(x / 8);
  this->data_((x + w - 1) / 8);

  this->command_(0x45);
  this->data_(y % 256);
  this->data_(y / 256);
  this->data_((y + h - 1) % 256);
  this->data_((y + h - 1) / 256);

  this->command_(0x4E);
  this->data_(x / 8);

  this->command_(0x4F);
  this->data_(y % 256);
  this->data_(y / 256);
}

void GDEY042Z98::write_buffer_(uint8_t command, const uint8_t *buffer, uint32_t length) {
  this->set_partial_ram_area_(0, 0, EPD_WIDTH, EPD_HEIGHT);
  this->command_(command);
  this->start_data_();
  for (uint32_t i = 0; i < length; i++) {
    this->write_byte(buffer[i]);
  }
  this->end_data_();
}

void GDEY042Z98::power_on_() {
  if (this->power_is_on_)
    return;

  ESP_LOGD(TAG, "Powering on display...");
  this->command_(0x22);
  this->data_(0xC0);
  this->command_(0x20);
  this->wait_until_idle_(1000);
  this->power_is_on_ = true;
}

void GDEY042Z98::power_off_() {
  if (!this->power_is_on_)
    return;

  ESP_LOGD(TAG, "Powering off display...");
  this->command_(0x22);
  this->data_(0xC3);
  this->command_(0x20);
  this->wait_until_idle_(1000);
  this->power_is_on_ = false;
}

void GDEY042Z98::update_full_() {
  this->command_(0x22);
  this->data_(0xF7);
  this->command_(0x20);
  this->wait_until_idle_(30000);
  this->power_is_on_ = false;
  ESP_LOGD(TAG, "Full update complete.");
}

void GDEY042Z98::deep_sleep_() {
  this->power_off_();
  if (this->reset_pin_ != nullptr) {
    this->command_(0x10);
    this->data_(0x11);
    this->init_display_done_ = false;
    ESP_LOGD(TAG, "Display entered deep sleep.");
  }
}

}  // namespace gdey042z98
}  // namespace esphome
