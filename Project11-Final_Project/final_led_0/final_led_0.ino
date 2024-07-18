#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

// Initialize the OLED display
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);

// Variables to control the blinking
bool heartVisible = true;
unsigned long previousMillis = 0;
const long interval = 500;  // interval at which to blink (milliseconds)

// Function to draw a heart shape
void drawHeart(int x, int y) {
  u8g2.drawPixel(x, y + 3);
  u8g2.drawPixel(x + 1, y + 2);
  u8g2.drawPixel(x + 2, y + 1);
  u8g2.drawPixel(x + 3, y + 1);
  u8g2.drawPixel(x + 4, y + 2);
  u8g2.drawPixel(x + 5, y + 3);
  u8g2.drawPixel(x + 5, y + 4);
  u8g2.drawPixel(x + 4, y + 5);
  u8g2.drawPixel(x + 3, y + 6);
  u8g2.drawPixel(x + 2, y + 5);
  u8g2.drawPixel(x + 1, y + 4);
  u8g2.drawPixel(x, y + 3);
}

void setup(void) {
  // Initialize the display
  u8g2.begin();
}

void loop(void) {
  unsigned long currentMillis = millis();

  // Check if it's time to toggle the heart's visibility
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    heartVisible = !heartVisible;
  }

  // Clear the internal memory
  u8g2.clearBuffer();

  // Set the font for text
  u8g2.setFont(u8g2_font_ncenB08_tr);

  // Draw "Shirel" on the display
  u8g2.drawStr(0, 20, "Shirel");

  // Draw the heart conditionally based on the blink state
  if (heartVisible) {
    drawHeart(50, 15);  // Adjust the coordinates as needed
  }

  // Draw "JoeJoe" on the display
  u8g2.drawStr(70, 20, "JoeJoe");

  // Transfer internal memory to the display
  u8g2.sendBuffer();

  // Small delay to ensure smooth blinking (if needed)
  delay(50);
}
