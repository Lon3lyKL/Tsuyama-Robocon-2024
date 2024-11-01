#define S0 22          /*Define S0 Pin Number of ESP32*/
#define S1 23          /*Define S1 Pin Number of ESP32*/
#define S2 18          /*Define S2 Pin Number of ESP32*/
#define S3 19          /*Define S3 Pin Number of ESP32*/
#define sensorOut 21   /*Define Sensor Output Pin Number of ESP32*/

/*Enter the Minimum and Maximum Values which getting from Calibration Code*/
int R_Min = 28;  /*Red minimum value*/
int R_Max = 140; /*Red maximum value*/
int G_Min = 28;  /*Green minimum value*/
int G_Max = 140; /*Green maximum value*/
int B_Min = 28;  /*Blue minimum value*/
int B_Max = 140; /*Blue maximum value*/

/*Define int variables*/
int Red = 0;
int Green = 0;
int Blue = 0;

int redValue;
int greenValue;
int blueValue;
int Frequency;

void setup() {
  pinMode(S0, OUTPUT);        /*Define S0 Pin as OUTPUT*/
  pinMode(S1, OUTPUT);        /*Define S1 Pin as OUTPUT*/
  pinMode(S2, OUTPUT);        /*Define S2 Pin as OUTPUT*/
  pinMode(S3, OUTPUT);        /*Define S3 Pin as OUTPUT*/
  pinMode(sensorOut, INPUT);  /*Define Sensor Output Pin as INPUT*/
  
  Serial.begin(115200);       /*Set the baudrate to 115200*/
  delay(1000);                /*Wait for 1000 mS*/

  // Set S0 and S1 for 20% sensitivity
  digitalWrite(S0, LOW);      /* Set S0 LOW */
  digitalWrite(S1, HIGH);      /* Set S1 LOW */
}

void loop() {
  Red = getRed();
  redValue = map(Red, R_Min, R_Max, 255, 0); /*Map the Red Color Value*/
  delay(200);
 
  Green = getGreen();
  greenValue = map(Green, G_Min, G_Max, 255, 0); /*Map the Green Color Value*/
  delay(200);
 
  Blue = getBlue();
  blueValue = map(Blue, B_Min, B_Max, 255, 0);   /*Map the Blue Color Value*/
  delay(200);

  Serial.print("RAW Red = ");
  Serial.print(Red);
  Serial.print("\tRAW Blue = ");
  Serial.print(Blue);
  Serial.print("\tRAW Green = ");
  Serial.print(Green);
  Serial.print("\t\tRed = ");
  Serial.print(redValue);   
  Serial.print("\tGreen = ");
  Serial.print(greenValue); 
  Serial.print("\tBlue = ");
  Serial.print(blueValue);

  if (redValue > blueValue + 25) {
    Serial.println("\tColor Detected: RED");
  } else if (blueValue > redValue + 15) {
    Serial.println("\tColor Detected: BLUE");
  } else {
    Serial.println("\tColor Detected: Uncertain");
  }
  delay(500);              /*Wait a second*/
}

int getRed() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  Frequency = pulseIn(sensorOut, LOW); /*Get the Red Color Frequency*/
  return Frequency;
}

int getGreen() {
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  Frequency = pulseIn(sensorOut, LOW); /*Get the Green Color Frequency*/
  return Frequency;
}

int getBlue() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  Frequency = pulseIn(sensorOut, LOW); /*Get the Blue Color Frequency*/
  return Frequency;
}
