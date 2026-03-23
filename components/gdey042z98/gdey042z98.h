#pragma once

// ESPHome display driver for Good Display GDEY042Z98
// 4.2" tri-color (black/white/red) e-paper, SSD1683 controller
// Based on GxEPD2_420c_GDEY042Z98 by Jean-Marc Zingg
// https://github.com/ZinggJM/GxEPD2

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/display/display_buffer.h"

namespace esphome {
namespace gdey042z98 {

static const uint16_t EPD_WIDTH = 400;
static const uint16_t EPD_HEIGHT = 300;
static const uint32_t EPD_PLANE_SIZE = EPD_WIDTH * EPD_HEIGHT / 8;

class GDEY042Z98 : public display::DisplayBuffer,
                   public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                         spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_2MHZ> {
 public:
  // Pin setters
  void set_dc_pin(GPIOPin *dc_pin) { this->dc_pin_ = dc_pin; }
  void set_reset_pin(GPIOPin *reset_pin) { this->reset_pin_ = reset_pin; }
  void set_busy_pin(GPIOPin *busy_pin) { this->busy_pin_ = busy_pin; }
  void set_reset_duration(uint32_t duration) { this->reset_duration_ = duration; }

  // Component overrides
  float get_setup_priority() const override;
  void setup() override;
  void update() override;
  void on_safe_shutdown() override;
  void dump_config() override;

  // Display type - tri-color
  display::DisplayType get_display_type() override { return display::DisplayType::DISPLAY_TYPE_COLOR; }

  void fill(Color color) override;

 protected:
  void draw_absolute_pixel_internal(int x, int y, Color color) override;
  int get_width_internal() override { return EPD_WIDTH; }
  int get_height_internal() override { return EPD_HEIGHT; }

  // SPI helpers
  void command_(uint8_t cmd);
  void data_(uint8_t val);
  void start_command_();
  void end_command_();
  void start_data_();
  void end_data_();

  // Display control
  void reset_();
  bool wait_until_idle_(uint32_t timeout_ms = 30000);
  void init_display_();
  void set_partial_ram_area_(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
  void write_buffer_(uint8_t command, const uint8_t *buffer, uint32_t length);
  void power_on_();
  void power_off_();
  void update_full_();
  void deep_sleep_();

  // Pins
  GPIOPin *dc_pin_{nullptr};
  GPIOPin *reset_pin_{nullptr};
  GPIOPin *busy_pin_{nullptr};

  uint32_t reset_duration_{2};
  bool power_is_on_{false};
  bool init_display_done_{false};
};

}  // namespace gdey042z98
}  // namespace esphome
