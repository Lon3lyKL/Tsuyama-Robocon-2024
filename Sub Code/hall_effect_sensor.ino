int led = 14;
int digitalPin = 12;
int digitalVal;
int prevDigitalVal = LOW;
unsigned long lastPulseTime = 0;
unsigned long currentTime = 0;
float rpm = 0;

void setup() {
  pinMode(led, OUTPUT);
  pinMode(digitalPin, INPUT);
  Serial.begin(115200);
}

void loop() {
  digitalVal = digitalRead(digitalPin);

  // Detect transition from LOW to HIGH
  if (digitalVal == HIGH && prevDigitalVal == LOW) {
    currentTime = millis();  // capture current time

    // Calculate the time difference between pulses (period)
    unsigned long timeDifference = currentTime - lastPulseTime;

    if (timeDifference > 0) {
      // Calculate RPM: 60000 / timeDifference gives RPM
      // (60000 ms per minute, 1 pulse per revolution)
      rpm = 60000.0 / timeDifference;
      Serial.print("RPM: ");
      Serial.println(rpm);
    }

    lastPulseTime = currentTime;  // update last pulse time
  }

  prevDigitalVal = digitalVal;

  delay(10);
}
