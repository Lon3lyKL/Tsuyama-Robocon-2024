#include <Adafruit_NeoPixel.h>

#define PIN_WS2812B 22  // The ESP32 pin GPIO16 connected to WS2812B
#define NUM_PIXELS 5    // The number of LEDs (pixels) on WS2812B LED strip

Adafruit_NeoPixel ws2812b(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);

void displayBinary(int number) {
  ws2812b.clear();  // Clear all LEDs

  for (int bit = 0; bit < NUM_PIXELS; bit++) {
    if (number & (1 << bit)) {  // Check if the bit is set in 'number'
      ws2812b.setPixelColor(bit, ws2812b.Color(0, 255, 0));  // Turn ON LED (green)
    } else {
      ws2812b.setPixelColor(bit, ws2812b.Color(0, 0, 0));    // Turn OFF LED (black/off)
    }
  }

  ws2812b.show();  // Update the LED strip
}

void setup() {
  ws2812b.begin();  // Initialize WS2812B strip object (REQUIRED)
}

void loop() {
  // Loop through numbers from 0 to 25
  for (int number = 0; number <= 25; number++) {
    displayBinary(number);  // Display binary representation on the LED strip
    delay(1000);            // Pause for 1 second
  }
}
