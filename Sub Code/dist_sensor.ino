#include <Arduino.h>

#define SENSOR_PIN 13 // GPIO36 (VP) is an analog pin

void setup() {
  Serial.begin(115200);
}

void loop() {
  int rawValue = analogRead(SENSOR_PIN); // Read the raw ADC value (0-4095)
  float voltage = (rawValue / 4095.0) * 5.0; // Convert to voltage (for 3.3V ADC reference)
  
  // Convert voltage to distance (approximation based on the sensor's datasheet)
  float distance = 27.86 / (voltage - 0.42); // Calibration formula
  
  Serial.print("Raw Value: ");
  Serial.print(rawValue);
  Serial.print("\tVoltage: ");
  Serial.print(voltage, 2);
  Serial.print(" V\tDistance: ");
  Serial.print(distance, 2);
  Serial.println(" cm");
  
  delay(100); // Small delay for stability
}
