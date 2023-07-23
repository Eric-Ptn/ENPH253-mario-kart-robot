#include <mario-kart-robot\oled_display.h>
#include <mario-kart-robot\config.h>

namespace OLED {
  Adafruit_SSD1306 display_handler = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

  void begin_oled(){
    display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  }
  
  void display_text(String text) {
    display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0,0);
    display_handler.println(text);
    display_handler.display();
  }
}