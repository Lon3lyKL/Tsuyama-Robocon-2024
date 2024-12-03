#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define SENSOR_PIN 15 // GPIO36 (VP) is an analog pin
#define PIN_WS2812B 22  // The ESP32 pin GPIO16 connected to WS2812B
#define NUM_PIXELS 5    // The number of LEDs (pixels) on WS2812B LED strip

Adafruit_NeoPixel ws2812b(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);

int calib_count = 0;
float base_total = 0;
int base_calib = 0;
float base_dist = 0;
float dist = 0;
float prev_dist = 0;
int pucks = 0;

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
  Serial.begin(115200);
  ws2812b.begin();  // Initialize WS2812B strip object
}

void loop() {
  int rawValue = analogRead(SENSOR_PIN); // Read the raw ADC value (0-4095)
  float voltage = (rawValue / 4095.0) * 5.0; // Convert to voltage (for 3.3V ADC reference)
  
  // Convert voltage to distance (approximation based on the sensor's datasheet)
  float dist = 27.86 / (voltage - 0.42); // Calibration formula

  unsigned long curr_time = millis();

  /*if (base_calib == 0) {
    base_total += dist;
    calib_count++;
    if (curr_time >= 7000) {
      base_dist = base_total / calib_count;
      base_calib = 1;
      Serial.println("Calibration completed.");
    }
  }*/
  base_calib = 1;
  base_dist = 16.70;

  if (base_calib == 1) {
    pucks = round((base_dist - dist) / 0.44);
    //if (pucks <= 1) pucks++;
    displayBinary(pucks);  // Display binary representation on the LED strip
    Serial.print("Base Distance: ");
    Serial.print(base_dist);
    Serial.print("\tDistance: ");
    Serial.print(dist, 2);
    Serial.print(" cm\tPucks: ");
    Serial.println(pucks);
  }
  
  delay(100);
}