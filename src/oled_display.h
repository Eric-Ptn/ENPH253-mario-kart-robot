#pragma once
#include <Adafruit_SSD1306.h>

// static to ensure local file scope
// don't use #define, that will make it global to all files i think

static const int SCREEN_WIDTH = 128;
static const int SCREEN_HEIGHT = 64;
static const int OLED_RESET = -1; // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void display_text(String text) {
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  delay(100);
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.println(text);
  display_handler.display();
  delay(100);
}