#define HALL 12  // Pin number for Hall sensor input (using GPIO12 for ESP32)
#include <ESP32Servo.h>
#include <PS4Controller.h>

Servo flywheel;

const int pwmPinflywheel = 13;

int flywheel_vel = 0;
volatile unsigned char tick = 0;
volatile unsigned long tickTime_isr;
unsigned long tickTime = 0;
float rpm = 0; // Variable to store RPM value

void setup() {
  Serial.begin(115200);
  pinMode(HALL, INPUT);  // Set GPIO12 as input pin for Hall sensor signal
  attachInterrupt(HALL, hall_ISR, FALLING); // Set interrupt for falling edge on GPIO12
  PS4.begin();
  
  while (!PS4.isConnected()) {
    Serial.println("Waiting for PS4 controller to connect...");
    delay(1000);
  }
  
  Serial.println("Waiting for ESC...");
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

  if (PS4.R2()) {
    flywheel.write(flywheel_vel);
  } else {
    flywheel.write(0);
  }
  calculateRPM(); // Call once per loop to calculate RPM (if tick occurred)
  Serial.print(rpm, 2);
  Serial.print("\t");
  Serial.print(flywheel_vel);
  Serial.print("\t");
  Serial.print(0);
  Serial.print(" ");
  Serial.println(10000);
}

void hall_ISR() {
  tickTime_isr = micros();  // Capture the current time in microseconds
  tick = 1;  // Set the flag to indicate a tick has occurred
}

void calculateRPM() {
  unsigned long old_tickTime;
  if (tick) {
    tick = 0;  // Reset interrupt flag (single bit, so no need to stop interrupts)
    old_tickTime = tickTime;  // Save the previous tick time to calculate the time difference
    noInterrupts();
    tickTime = tickTime_isr;  // Get new time from interrupt
    interrupts();

    // Calculate the time between two ticks in microseconds
    float timeBetweenTicks = (float)(tickTime - old_tickTime);  // Time in microseconds

    // Convert time to seconds and calculate RPM
    if (timeBetweenTicks > 0) {
      rpm = 60000000.0 / timeBetweenTicks;  // RPM = 60,000,000 / time in microseconds (60 seconds * 1,000,000 microseconds)
    }
  }
}
