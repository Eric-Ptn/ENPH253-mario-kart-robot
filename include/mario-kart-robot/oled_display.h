#include <Adafruit_SSD1306.h>

namespace OLED {
    Adafruit_SSD1306 display_handler;

    void begin_oled();
    void display_text(String text);
}