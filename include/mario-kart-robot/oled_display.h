#pragma once
#include <Adafruit_SSD1306.h>
#include <mario-kart-robot\config.h>

namespace OLED {
    extern Adafruit_SSD1306 display_handler;
    
    void begin_oled();
    void display_text(String text);
}