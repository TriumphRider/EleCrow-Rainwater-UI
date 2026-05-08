#pragma once

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <Wire.h>

class LGFX_CrowPanel : public lgfx::LGFX_Device {
public:
  lgfx::Bus_RGB _bus_instance;
  lgfx::Panel_RGB _panel_instance;
  lgfx::Touch_GT911 _touch_instance;

  LGFX_CrowPanel(void) {
    { // Panel config
      auto cfg = _panel_instance.config();
      cfg.memory_width  = 800;
      cfg.memory_height = 480;
      cfg.panel_width   = 800;
      cfg.panel_height  = 480;
      cfg.offset_x      = 0;
      cfg.offset_y      = 0;
      _panel_instance.config(cfg);
    }

    { // Panel detail
      auto cfg = _panel_instance.config_detail();
      cfg.use_psram = 2;
      _panel_instance.config_detail(cfg);
    }

    { // RGB bus config
      auto cfg = _bus_instance.config();
      cfg.panel = &_panel_instance;

      // RGB data pins (DIS02050A V1.1)
      cfg.pin_d0  = GPIO_NUM_21; // B0
      cfg.pin_d1  = GPIO_NUM_47; // B1
      cfg.pin_d2  = GPIO_NUM_48; // B2
      cfg.pin_d3  = GPIO_NUM_45; // B3
      cfg.pin_d4  = GPIO_NUM_38; // B4
      cfg.pin_d5  = GPIO_NUM_9;  // G0
      cfg.pin_d6  = GPIO_NUM_10; // G1
      cfg.pin_d7  = GPIO_NUM_11; // G2
      cfg.pin_d8  = GPIO_NUM_12; // G3
      cfg.pin_d9  = GPIO_NUM_13; // G4
      cfg.pin_d10 = GPIO_NUM_14; // G5
      cfg.pin_d11 = GPIO_NUM_7;  // R0
      cfg.pin_d12 = GPIO_NUM_17; // R1
      cfg.pin_d13 = GPIO_NUM_18; // R2
      cfg.pin_d14 = GPIO_NUM_3;  // R3
      cfg.pin_d15 = GPIO_NUM_46; // R4

      cfg.pin_henable = GPIO_NUM_42; // DE
      cfg.pin_vsync   = GPIO_NUM_41; // VSYNC
      cfg.pin_hsync   = GPIO_NUM_40; // HSYNC
      cfg.pin_pclk    = GPIO_NUM_39; // PCLK

      cfg.freq_write = 15000000; // 15 MHz — reduced from 18 to ease PSRAM bus pressure

      cfg.hsync_polarity     = 0;
      cfg.hsync_front_porch  = 8;
      cfg.hsync_pulse_width  = 4;
      cfg.hsync_back_porch   = 8;
      cfg.vsync_polarity     = 0;
      cfg.vsync_front_porch  = 8;
      cfg.vsync_pulse_width  = 4;
      cfg.vsync_back_porch   = 8;
      cfg.pclk_idle_high     = 1;

      _bus_instance.config(cfg);
    }

    _panel_instance.setBus(&_bus_instance);

    { // Touch GT911 config
      auto cfg = _touch_instance.config();
      cfg.x_min = 0;
      cfg.x_max = 800;
      cfg.y_min = 0;
      cfg.y_max = 480;

      cfg.i2c_port = 0;
      cfg.i2c_addr = 0x5D;
      cfg.pin_sda  = GPIO_NUM_15;
      cfg.pin_scl  = GPIO_NUM_16;
      cfg.pin_int  = GPIO_NUM_1;
      cfg.pin_rst  = GPIO_NUM_2;
      cfg.freq     = 400000;

      cfg.bus_shared = true;

      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);
    }

    setPanel(&_panel_instance);
  }
};

// Backlight control (I2C 0x30)
inline void backlight_init() {
  Wire.begin(15, 16);
  delay(10);
  Wire.beginTransmission(0x30);
  Wire.write(0x10);
  Wire.endTransmission();
}

inline void backlight_set(uint8_t val) {
  Wire.beginTransmission(0x30);
  Wire.write(val);
  Wire.endTransmission();
}