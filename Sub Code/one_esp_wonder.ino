#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Bluepad32.h>

Servo liftingServo;
Servo primingServo;
Servo flywheel;
Servo intake;

// PWM pins for Servo Motors
const int pwmPinServoPrime = 32;
const int pwmPinServoLift = 33;
const int pwmPinflywheel = 15;
const int pwmPinintake = 22;
const int irsensor_pin = 35;
const int ledstrip_pin = 22
const int num_pixels = 5;

// Motor Driver 1
// Motor control pins for Motor A
const int dirPinA1 = 4;  // IN3 for Motor A direction control
const int dirPinA2 = 2;  // IN4 for Motor A direction control
const int pwmPinA = 27;  // ENB pin (controls Motor A speed)

// Motor control pins for Motor B
const int dirPinB1 = 21;  // IN1 for Motor B direction control
const int dirPinB2 = 19;   // IN2 for Motor B direction control
const int pwmPinB = 26;   // ENA pin (controls Motor B speed)

// Motor Driver 2
// Motor control pins for Motor C
const int dirPinC1 = 12;  // IN1 for Motor C direction control
const int dirPinC2 = 14;  // IN2 for Motor C direction control
const int pwmPinC = 13;   // ENA pin (controls Motor C speed)

// Motor control pins for Motor D
const int dirPinD1 = 5;  // IN3 for Motor D direction control
const int dirPinD2 = 18;  // IN4 for Motor D direction control
const int pwmPinD = 25;   // ENB pin (controls Motor D speed)

int motorSpeedA = 0;
int motorSpeedB = 0;
int motorSpeedC = 0;
int motorSpeedD = 0;

int RStickX = 0;
int RStickY = 0;
int L1 = 0;
int L2 = 0;
int R1 = 0;
int R2 = 0;
int Cross = 0;
int Circle = 0;
int Triangle = 0;
int Square = 0;
int Left = 0;
int Right = 0;
int Touchpad = 0;

float base_dist = 0;
float dist = 0;
int pucks = 0;

String lift_status = "OFF";
String prime_status = "OFF";
String flywheel_status = "OFF";
String intake_status = "OFF";

int vel_X = 0;
int vel_Y = 0;
int vel_X2 = 0;
int vel_A = 0;
int vel_B = 0;
int vel_C = 0;
int vel_D = 0;
int flywheel_vel = 0;

double theta = 0;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
Adafruit_NeoPixel ws2812b(num_pixels, ledstrip_pin, NEO_GRB + NEO_KHZ800);

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
    
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
      Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

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

  primingServo.attach(pwmPinServoPrime, 500, 2500);
  liftingServo.attach(pwmPinServoLift, 500, 2500);
  flywheel.attach(pwmPinflywheel, 1000, 2000);
  intake.attach(pwmPinintake, 1000, 2000);
  intake.write(0);
  flywheel.write(0);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(true);

  pinMode(dirPinA1, OUTPUT);
  pinMode(dirPinA2, OUTPUT);
  pinMode(dirPinB1, OUTPUT);
  pinMode(dirPinB2, OUTPUT);
  pinMode(dirPinC1, OUTPUT);
  pinMode(dirPinC2, OUTPUT);
  pinMode(dirPinD1, OUTPUT);
  pinMode(dirPinD2, OUTPUT);

  // PWM Channels 0 and 1 for Servo Motors
  // PWM channels 2 and 3 for DC Motor speed control
  ledcSetup(2, 5000, 8);     
  ledcSetup(3, 5000, 8);

  ledcAttachPin(pwmPinA, 2);  
  ledcAttachPin(pwmPinB, 3); 
  ledcAttachPin(pwmPinC, 2);
  ledcAttachPin(pwmPinD, 3); 
}

int* Stop() {
  vel_A = 0;
  vel_B = 0;
  vel_C = 0;
  vel_D = 0;
  static int vel_arr[4] = {vel_A, vel_B, vel_C, vel_D};
  // Motor A: Stop
  digitalWrite(dirPinA1, LOW);
  digitalWrite(dirPinA2, LOW);
  ledcWrite(2, 0);

  // Motor B: Stop
  digitalWrite(dirPinB1, LOW);
  digitalWrite(dirPinB2, LOW);
  ledcWrite(3, 0);

  // Motor C: Stop
  digitalWrite(dirPinC1, LOW);
  digitalWrite(dirPinC2, LOW);
  ledcWrite(2, 0);

  // Motor D: Stop
  digitalWrite(dirPinD1, LOW);
  digitalWrite(dirPinD2, LOW);
  ledcWrite(3, 0);
  return vel_arr;
}

int MotorA(int x, int y, int x2, double theta) {
  int resultant_A = y + x + x2; 
  if (-1024 <= x2 && x2 < 0) {
    digitalWrite(dirPinA1, LOW);
    digitalWrite(dirPinA2, HIGH);
  } else if (0 < x2 && x2 <= 1024) {
    digitalWrite(dirPinA1, HIGH);
    digitalWrite(dirPinA2, LOW);
  } else if (-45 < theta && theta < 135) { // Forward for -45 < theta < 135
    digitalWrite(dirPinA1, HIGH);
    digitalWrite(dirPinA2, LOW);
  } else if (-45 > theta && theta > -180 || 180 >= theta && theta > 135) { // Backward for -45 > theta > 135
    digitalWrite(dirPinA1, LOW);
    digitalWrite(dirPinA2, HIGH);
  } else { // Speed = 0 at -45 and 135
    digitalWrite(dirPinA1, LOW);
    digitalWrite(dirPinA2, LOW);
  }
  ledcWrite(2, abs(resultant_A));
  return resultant_A;
}

int MotorB(int x, int y, int x2, double theta) {
  int resultant_B = y - x - x2;
  
  if (-1024 <= x2 && x2 < 0) {
    digitalWrite(dirPinB1, HIGH);
    digitalWrite(dirPinB2, LOW);
  } else if (0 < x2 && x2 <= 1024) {
    digitalWrite(dirPinB1, LOW);
    digitalWrite(dirPinB2, HIGH);
  } else if (-135 > theta && theta > -180 || 180 >= theta && theta > 45) { // Forward for -135 < theta < 45
    digitalWrite(dirPinB1, HIGH);
    digitalWrite(dirPinB2, LOW);
  } else if (45 > theta && theta > -135) { // Backward for 45 > theta > -135 
    digitalWrite(dirPinB1, LOW);
    digitalWrite(dirPinB2, HIGH);
  } else { // Speed = 0 at 45 and -135
    digitalWrite(dirPinB1, LOW);
    digitalWrite(dirPinB2, LOW);
  }
  ledcWrite(3, abs(resultant_B));
  return resultant_B;
}

int MotorC(int x, int y, int x2, double theta) {
  int resultant_C = y + x - x2;
  if (-1024 <= x2 && x2 < 0) {
    digitalWrite(dirPinC1, HIGH);
    digitalWrite(dirPinC2, LOW);
  } else if (0 < x2 && x2 <= 1024) {
    digitalWrite(dirPinC1, LOW);
    digitalWrite(dirPinC2, HIGH);
  } else if (-45 < theta && theta < 135) { // Forward for -45 < theta < 135
    digitalWrite(dirPinC1, HIGH);
    digitalWrite(dirPinC2, LOW);
  } else if (-45 > theta && theta > -180 || 180 >= theta && theta > 135) { //Backward for -45 > theta > 135
    digitalWrite(dirPinC1, LOW);
    digitalWrite(dirPinC2, HIGH);
  } else { // Speed = 0 at -45 and 135
    digitalWrite(dirPinC1, LOW);
    digitalWrite(dirPinC2, LOW);
  }
  ledcWrite(2, abs(resultant_C));
  return resultant_C;
}

int MotorD(int x, int y, int x2, double theta) {
  int resultant_D = y - x + x2;
  if (-1024 <= x2 && x2 < 0) {
    digitalWrite(dirPinD1, LOW);
    digitalWrite(dirPinD2, HIGH);
  } else if (0 < x2 && x2 <= 1024) {
    digitalWrite(dirPinD1, HIGH);
    digitalWrite(dirPinD2, LOW);
  } else if (-135 > theta && theta > -180 || 180 >= theta && theta > 45) { // Forward for -135 < theta < 45
    digitalWrite(dirPinD1, HIGH);
    digitalWrite(dirPinD2, LOW);
  } else if (45 > theta && theta > -135) { // Backward for 45 > theta > -135 
    digitalWrite(dirPinD1, LOW);
    digitalWrite(dirPinD2, HIGH);
  } else { // Speed = 0 at 45 and -135
    digitalWrite(dirPinD1, LOW);
    digitalWrite(dirPinD2, LOW);
  }
  ledcWrite(3, abs(resultant_D));
  return resultant_D;
}

void loop() {
  bool dataUpdated = BP32.update();
  for (ControllerPtr ctl : myControllers) {
    if (ctl && ctl->isConnected() && ctl->hasData()) {
      if (ctl->isGamepad()) {
        RStickX = map(ctl->axisRX(), -508, 512, -1024, 1024);
        RStickY = map(ctl->axisRY(), 512, -508, -1024, 1024);
        vel_X2 = map(ctl->axisX(), -508, 512, -1024, 1024);
        L1 = ctl->l1();
        L2 = ctl->l2();
        R1 = ctl->r1();
        R2 = ctl->r2();
        Cross = ctl->a();
        Circle = ctl->b();
        Triangle = ctl->y();
        Square = ctl->x();

        if (ctl->dpad() & 0x04) Right = 1; else Right = 0;
        if (ctl->dpad() & 0x08) Left = 1; else Left = 0;
      } else if (ctl->isMouse()) {
        if (ctl->buttons() & 0x01) Touchpad = 1; else Touchpad = 0;
      }
    }
  }

  // Dead zones
  if (-50 <= RStickX && RStickX <= 50) RStickX = 0;

  if (-50 <= RStickY && RStickY <= 50) RStickY = 0;

  if (-50 <= vel_X2 && vel_X2 <= 50) vel_X2 = 0;

  //Theta calculation
  if (RStickX == 0 && RStickY > 0) {
    theta = 90;
  } else if (RStickX == 0 && RStickY < 0) {
    theta = -90;
  } else if (RStickX == 0 && RStickY == 0) {
    theta = 0;
  } else {
    theta = atan(static_cast<float>(RStickY) / RStickX) * 180.0 / PI;

    if (RStickX < 0 && RStickY >= 0) {
      theta += 180;  // Adjust the angle to handle negative X direction
    } else if (RStickX < 0 && RStickY < 0) {
      theta -= 180;  // Third quadrant (X negative, Y negative)
    }
  }

  // Velocity calculations
  if (0 <= theta && theta <= 90) { // 1st  Quadrant
    vel_X = map(theta, 0, 90, RStickX, 0);
    vel_Y = map(theta, 0, 90, 0, RStickY);
  } else if (90 < theta && theta <= 180) {  // 2nd Quadrant
    vel_X = map(theta, 90, 180, 0, RStickX);
    vel_Y = map(theta, 90, 180, RStickY, 0);
  } else if (-180 < theta && theta <= -90) {  // 3rd Quadrant
    vel_X = map(theta, -180, -90, RStickX, 0);
    vel_Y = map(theta, -180, -90, 0, RStickY);
  } else if (-90 <= theta && theta <= 0) {  // 4th Quadrant
    vel_X = map(theta, -90, 0, 0, RStickX);
    vel_Y = map(theta, -90, 0, RStickY, 0);
  }
  
  if (!(RStickX == 0 && RStickY == 0 && vel_X2 == 0)) { // detect joystick input
    vel_A = MotorA(vel_X, vel_Y, vel_X2, theta);
    vel_B = MotorB(vel_X, vel_Y, vel_X2, theta);
    vel_C = MotorC(vel_X, vel_Y, vel_X2, theta);
    vel_D = MotorD(vel_X, vel_Y, vel_X2, theta);
    
  } else { // Stop when controller has no input
    int *arr = Stop();
    vel_A = arr[0];
    vel_B = arr[1];
    vel_C = arr[2];
    vel_D = arr[3];
  }

  if (L1) { // If L2 pressed, servo goes to 180
    liftingServo.writeMicroseconds(map(180, 0, 300, 500, 2500));
    lift_status = "ON";
  } else { // If L2 released, servo goes to 0
    liftingServo.writeMicroseconds(500);
    lift_status = "OFF";
  }
  if (L2) {
    intake.write(50);
    intake_status = "ON";
  } else {
    intake.write(0);
    intake_status = "OFF";
  }
  
  if (R1) { // If R1 pressed, servo goes to 30
    primingServo.writeMicroseconds(map(30, 0, 300, 500, 2500));
    delay(200);
    prime_status = "ON";
  } else { // If R1 released, servo goes to 60
    primingServo.writeMicroseconds(map(60, 0, 300, 500, 2500));
    prime_status = "OFF";
  }
  static bool leftPressed = false;
  static bool rightPressed = false;
  if (Cross) flywheel_vel = 40;
  if (Circle) flywheel_vel = 50;
  if (Triangle) flywheel_vel = 53;
  if (Square) flywheel_vel = 56;
  if (Left) {
    if (!leftPressed) {
        flywheel_vel -= 1;
        leftPressed = true;
    }
  } else leftPressed = false;
  
  if (Right) {
    if (!rightPressed) {
        flywheel_vel += 1;
        rightPressed = true; 
    }
  } else rightPressed = false; 

  if (R2) {
    flywheel.write(flywheel_vel);
    flywheel_status = "ON";
  } else {
    flywheel.write(0);
    flywheel_status = "OFF";
  }

  if (Touchpad) {
    int rawValue = analogRead(irsensor_pin); // Read the raw ADC value (0-4095)
    float voltage = (rawValue / 4095.0) * 5.0; 
    
    // Convert voltage to distance (approximation based on the sensor's datasheet)
    dist = 27.86 / (voltage - 0.42); // Calibration formula
    
    base_dist = 16.40;
    pucks = round((base_dist - dist) / 0.445);
  }
  static bool R1Pressed = false;
  if (R1 && R2) {
    if (!R1Pressed && pucks > 0) {
        pucks--;
        R1Pressed = true;
    }
  } else R1Pressed = false;
  displayBinary(pucks);

  Serial.print("X: ");
  Serial.print(RStickX);
  Serial.print("\tY: ");
  Serial.print(RStickY);
  Serial.print("\ttheta: ");
  Serial.print(theta);
  
  Serial.print("\tvel_A: ");
  Serial.print(vel_A);
  Serial.print("\tvel_B: ");
  Serial.print(vel_B);
  Serial.print("\tvel_C: ");
  Serial.print(vel_C);
  Serial.print("\tvel_D: ");
  Serial.print(vel_D);
  Serial.print("\tvel_X2: ");
  Serial.print(vel_X2);
  Serial.print("\tLift:  ");
  Serial.print(lift_status);
  Serial.print("\tPrime: ");
  Serial.print(prime_status);
  Serial.print("\tIntake:  ");
  Serial.print(intake_status);
  Serial.print("\tFlywheel:  ");
  Serial.print(flywheel_status);
  Serial.print("\tflywheel_vel: ");
  Serial.print(flywheel_vel);
  Serial.print("\tBase Distance: ");
  Serial.print(base_dist);
  Serial.print("\tDistance: ");
  Serial.print(dist, 2);
  Serial.print(" cm\tPucks: ");
  Serial.print(pucks);
  Serial.print("\tTouchpad: ");
  Serial.println(Touchpad);
}