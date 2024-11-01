#include <ESP32Servo.h>

Servo flywheel;

const int pwmPinflywheel = 13;

void setup() {
  Serial.begin(115200);
  flywheel.attach(pwmPinflywheel, 1000, 2000);
  flywheel.write(0);
  delay(10000);
}

void loop() {
  flywheel.write(50);
  delay(1000);
  flywheel.write(0);
  delay(1000);
}
