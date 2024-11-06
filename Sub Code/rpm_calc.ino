#include <ESP32Servo.h>
#include <PS4Controller.h>

Servo flywheel;

const int pwmPinflywheel = 13;
const int digitalPin = 12;

int digitalVal;
int flywheel_vel = 0;
int prevDigitalVal = LOW;

float rpm = 0;

unsigned long lastPulseTime = 0;
unsigned long timeDifference = 0;

void setup() {
  pinMode(digitalPin, INPUT);
  Serial.begin(115200);
  PS4.begin();
  while (!PS4.isConnected()) {
    Serial.println("Waiting for PS4 controller to connect...");
    delay(1000);
  }
  flywheel.attach(pwmPinflywheel, 1000, 2000);
  flywheel.write(0);
  delay(10000);
}

void loop() {
  static bool leftPressed = false;
  static bool rightPressed = false;
  if (PS4.Left()) {
    if (!leftPressed) {
      flywheel_vel -= 3;
      leftPressed = true;
    }
  } else leftPressed = false;
  
  if (PS4.Right()) {
    if (!rightPressed) {
      flywheel_vel += 3;
      rightPressed = true; 
    }
  } else rightPressed = false;

  if (PS4.R2()) flywheel.write(flywheel_vel);
  else flywheel.write(0);

  digitalVal = digitalRead(digitalPin);

  // Detect transition from LOW to HIGH
  if (digitalVal == HIGH && prevDigitalVal == LOW) {
    unsigned long currentTime = micros();

    timeDifference = currentTime - lastPulseTime;

    if (timeDifference > 0) {
      rpm = (60000000.0 / timeDifference); 
    }
    lastPulseTime = currentTime;
  }

  prevDigitalVal = digitalVal;

  Serial.print("RPM: ");
  Serial.print(rpm, 5);
  Serial.print("\tvel ");
  Serial.println(flywheel_vel);

  delay(1);
}
