#include <Adafruit_NeoPixel.h> // Neopixel Library to control the neopixels

// Neopixels
#define PIN 11 // Neopixel pin NI(Input)
#define NUMPIXELS 4 // There are 4 neopixels on the board
void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue);
void startLights();
void finishLights();
void leftLights();
void rightLights();
void neutralLights();

// Distance sensors
const int echoPin = 9; // Echo sensor echo pin
const int triggerPin = 8; // Echo sensor trigger pin
void distanceSensor();
void distanceReader();

// Gripper
const int gripperPin = 12; // Gripper pin GR
void servo(int pulse);

// Motors
const int leftBackwards = 7; // Left backwards A1
const int leftForward = 6; // Left forwards A2
const int rightBackwards = 4; // Right backwards B1
const int rightForward = 5; // Right forwards B2
const int motorPulseLeft = 2; // Left motor pulse R1
const int motorPulseRight = 3; // Right motor pulse R2 
void setMotors(int LFMotor, int LBMotor, int RFMotor, int RBMotor);
void driveForward(int leftSpeed, int rightSpeed);
void driveBackward(int leftSpeed, int rightSpeed);
void driveLeft(int leftSpeed, int rightSpeed);
void driveRight(int leftSpeed, int rightSpeed);
void driveStop();

const int numberOfSensors = 8;
int lineSensor[numberOfSensors] = {A5, A4, A7, A3, A2, A6, A1, A0} ; // Linesensor pins
void defaultLineSensor();
void scanBlackBox_START();
void scanBlackBox_END();
void scanBlackBox(); // Sensors 0 and 7
void fullScan(); // All Sensors

Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_RGB + NEO_KHZ800); // Neopixel needed code from library

 bool startTrigger = false;
 bool startRace = false;
 bool endRace = false;

const int maxDistance = 20;
const int startDistance = 15;
float distance; 
float duration;
const int echoInterval = 100; // Time between seeing the object and calculating the distance

int gripOpen = 1500; // pulse length servo open
int gripClosed = 1050; // pulse length servo closed
int servoInterval = 20; // time between pulse

const int leftSlowSpeed = 138; // Slowest speed of theleft wheel
const int rightSlowSpeed = 138; // Slowest speed of the right wheel
const int backwardsSpeedLeft = 0;// Backwards speed of the left Wheel
const int backwardsSpeedRight = 0;// Backwards speed of the right Wheel

const int speedTurns = 10; // Adding speed for turns
const int speedSharp = 30; // Adding speed for sharp turns
const int speedOneWay = 50; // Adding speed for one direction not turns
const int startSpeed = 40; // Adding speed for start
const int additionalSpeed = 60; // Additional modifiable speed if needed

int lineValues[numberOfSensors];
int maxSensorValue = 0; // Setting Gate
int lineCount = 0; // line counter for the start of the race for it to grab the object

const int MAX_BLACK = 980; // Max value of black reached
const int MIN_BLACK = 950;// The minimum value that reaches black

// ------------------------------------------------------------------------------------------------ setup

void setup() {
  strip.begin(); // Initialize neopixels
  Serial.begin(9600); // Start serial monitoring on 9600 for debugging

  pinMode(leftForward, OUTPUT);  // Specify the leftForward motor
  pinMode(leftBackwards, OUTPUT);  // Specify the leftBackward motor
  pinMode(rightForward, OUTPUT);  // Specify the rightForward motor
  pinMode(rightBackwards, OUTPUT);  // Specify the rightBackward motor

  pinMode(gripperPin, OUTPUT);  // Specify the gripperpin
  pinMode(gripperPin, LOW);

  pinMode(triggerPin, OUTPUT);  // Specify the triggerPin
  pinMode(echoPin, INPUT);  // Specify the echoPin

  pinMode(motorPulseLeft, INPUT); // Specify the motorPulseLeft
  pinMode(motorPulseRight,INPUT); // Specify the motorPulseRight

   for(int i = 0;i<=7;i++){
    pinMode(lineSensor[i], INPUT);
  }

  digitalWrite(gripperPin, LOW); //To open the Gripper
  servo(gripOpen); //Gripper is open
}

// ------------------------------------------------------------------------------------------------ loop

void loop() {
    if (!startTrigger){
    distanceReader();
    startLights();
    for (int i = 0; i < 50; i++){
    servo(gripOpen);
    delayMicroseconds(1000);
    }
    while (distance < startDistance){
      driveStop();
      distanceReader();
      
      if (distance > startDistance){
        break;
      }
      
    } 
    startTrigger = true;
    setMotors(255, 0, 255, 0);
    delay(50);
    setMotors(153, 0, 163, 0);
    delay(1050);
  }
  
    bool lineScanInProgress = false; // Flag to indicate if line scanning is in progress
    unsigned long currentMillis = millis(); // Get the current time
    scanBlackBox();
    static unsigned long timer;
    if (currentMillis > timer){
      if (lineValues[0] >= MIN_BLACK && !lineScanInProgress && lineValues[7] >= MIN_BLACK && !lineScanInProgress){
        lineScanInProgress = true; // Set flag to indicate line scanning is in progress
        lineCount++; // Add to the counter
      }
      timer = currentMillis + 50;
    }
    
    //Start sequence of grabbing the object
    if (lineScanInProgress && lineCount >= 4) {
          if (!startRace){
             scanBlackBox();
            while (lineValues[0] >= MIN_BLACK || lineValues[7] >= MIN_BLACK ){
              scanBlackBox_START();

              if (startRace){
                break;
              }
            }
          }
        lineScanInProgress = false;
    }
    
    defaultLineSensor(); //Reading the line
    distanceSensor(); //Detecting the object and avoiding it

  if (!endRace && startRace){
    fullScan();
    if (lineValues[0] >= MIN_BLACK || lineValues[7] >= MIN_BLACK) {
            setMotors(163, 0, 163, 0);
            pulseIn(2, HIGH, 400UL); // Pin , Pulse , Interval
            pulseIn(3, HIGH, 400UL);
            pulseIn(2, HIGH, 400UL);
            pulseIn(3, HIGH, 400UL);
            pulseIn(2, HIGH, 400UL);
            pulseIn(3, HIGH, 400UL);
            delay(50);
            fullScan();
        }
        if (lineValues[0] >= MAX_BLACK && lineValues[7] >= MAX_BLACK 
            && lineValues[1] >= MAX_BLACK && lineValues[2] >= MAX_BLACK 
            && lineValues[3] >= MAX_BLACK && lineValues[4] >= MAX_BLACK 
            && lineValues[5] >= MAX_BLACK && lineValues[6] >= MAX_BLACK){
              scanBlackBox_END();
        }
    }
}

// ------------------------------------------------------------------------------------------------ motors

void setMotors(int LFMotor, int LBMotor, int RFMotor, int RBMotor) {
  // Sets the speed of all the wheels by entering parameters
  analogWrite(leftForward, LFMotor);
  analogWrite(leftBackwards, LBMotor);
  analogWrite(rightForward, RFMotor);
  analogWrite(rightBackwards, RBMotor);
}

void driveForward(int leftSpeed, int rightSpeed) {
  neutralLights();
  setMotors(leftSpeed, 0, rightSpeed, 0);
}

void driveBackward(int leftSpeed, int rightSpeed) {
  setMotors(0, leftSpeed, 0, rightSpeed);
}

void driveRight(int leftSpeed, int rightSpeed) {
  rightLights();
  setMotors(leftSpeed, 0, 0, rightSpeed);
}

void driveLeft(int leftSpeed, int rightSpeed) {
  leftLights();
  setMotors(0, leftSpeed, rightSpeed, 0);
}

void driveStop() {
  setMotors(0, 0, 0, 0);
}

// ------------------------------------------------------------------------------------------------ line sensor

void defaultLineSensor() {
  fullScan();

  static unsigned long previousTime;

  if ((millis() - previousTime) >= 100UL)
  {
  for (int i = 0; i < numberOfSensors; i++) {
      if (lineValues[3] > maxSensorValue && lineValues[4] > maxSensorValue)
      {
        maxSensorValue = lineValues[3];
      }
    } 
     previousTime = millis();
  }

    if (maxSensorValue >= MAX_BLACK) {

      if (lineValues[3] >= MIN_BLACK || lineValues[4] >= MIN_BLACK ) {
        driveForward(leftSlowSpeed + speedOneWay + additionalSpeed, rightSlowSpeed + speedOneWay + additionalSpeed); 
      }
      else if ( lineValues[2] >= MAX_BLACK) {
        driveRight(leftSlowSpeed + speedTurns + additionalSpeed, backwardsSpeedRight);
      }
      else if (lineValues[5] >= MAX_BLACK) {
        driveLeft(backwardsSpeedLeft, rightSlowSpeed + speedTurns + additionalSpeed);
      }
      else if (lineValues[1] >= MIN_BLACK) {
        driveRight(leftSlowSpeed + speedSharp + additionalSpeed, backwardsSpeedRight);
      }
      else if (lineValues[6] >= MIN_BLACK) {
        driveLeft(backwardsSpeedLeft, rightSlowSpeed + speedSharp + additionalSpeed);
      }    
    } 
}


void scanBlackBox() {
   for (int i = 0; i < 2; i++) {
    lineValues[0] = analogRead(lineSensor[0]); 
    lineValues[7] = analogRead(lineSensor[7]); 
  }  
}

void fullScan() {
  for (int i = 0; i < numberOfSensors; i++) {
    lineValues[i] = analogRead(lineSensor[i]);
  } 
}

void scanBlackBox_START()
{
  if (lineValues[0] >= MAX_BLACK || lineValues[7] >= MAX_BLACK )
  {
    for (int i = 0; i < 50; i++)
    {

      delayMicroseconds(1000);
      servo(gripClosed);
    
    startRace = true;
    }
  }
  driveLeft(leftSlowSpeed, rightSlowSpeed + speedTurns);
  delay(1345);
  driveForward(leftSlowSpeed + startSpeed,rightSlowSpeed + startSpeed);
  defaultLineSensor();
}

void scanBlackBox_END() {
  fullScan();

  if(lineValues[0] >= MAX_BLACK && lineValues[7] >= MAX_BLACK 
     && lineValues[1] >= MAX_BLACK && lineValues[2] >= MAX_BLACK
     && lineValues[3] >= MAX_BLACK && lineValues[4] >= MAX_BLACK
     && lineValues[5] >= MAX_BLACK && lineValues[6] >= MAX_BLACK) {
    driveBackward(leftSlowSpeed + speedOneWay, rightSlowSpeed + speedOneWay);
    delay(125);
    servo(gripOpen);

    endRace = true; 
  }

  if (endRace) {
      driveBackward(leftSlowSpeed + speedOneWay + additionalSpeed,rightSlowSpeed + speedOneWay + additionalSpeed);
      delay(1350);
      driveStop();
      finishLights();
    }

    while (endRace) { // when ending race, bot should go back, stop and end function.
      driveStop();
      finishLights();
    }  
}

// ------------------------------------------------------------------------------------------------ distance sensor

void distanceReader() {
  digitalWrite(triggerPin, LOW); // Reset pin
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH); // High pulses for 10 ms
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  duration = pulseIn(echoPin, HIGH); // Reads pins

  distance = (duration / 2) * 0.034; // 343 m/s per second as speed of sound
}

void distanceSensor() {
static unsigned long timer;
 if (millis() > timer) {

    distanceReader();
    
    if (distance <= maxDistance && lineCount >= 4){ // Condition that it avoided 4 lines (at the start) so it doesnt mix in at the beginning
          //  It will avoid it anything closer at approx. 20CM
          
          driveLeft(backwardsSpeedLeft, rightSlowSpeed + speedTurns + additionalSpeed);
          delay(700);

          driveForward(leftSlowSpeed + speedOneWay + additionalSpeed, rightSlowSpeed + speedOneWay + additionalSpeed);
          delay(500); // 1000 

          driveRight(leftSlowSpeed + speedTurns + additionalSpeed, backwardsSpeedRight);
          delay(700); //850

          driveForward(leftSlowSpeed + speedOneWay + additionalSpeed, rightSlowSpeed + speedOneWay + additionalSpeed);
          delay(650);

          driveRight(leftSlowSpeed + speedTurns + additionalSpeed, backwardsSpeedRight);
          delay(700); //800

          driveForward(leftSlowSpeed + speedOneWay + additionalSpeed, rightSlowSpeed + speedOneWay + additionalSpeed);
          delay(500); //600

          driveLeft(leftSlowSpeed, rightSlowSpeed + speedTurns);
          delay(100);
          
        defaultLineSensor();
      }
    else {
        defaultLineSensor();
      }
       timer = millis() + 100;
    }
}

// ------------------------------------------------------------------------------------------------ gripper

void servo(int pulse) {
 static unsigned long timer;
  static int pulse1;
  if (pulse > 0) {
    pulse1 = pulse;
  }
  if (millis() > timer) {
    digitalWrite(gripperPin, HIGH);
    delayMicroseconds(pulse1);
    digitalWrite(gripperPin, LOW);
    timer = millis() + servoInterval;
  }
}
// ------------------------------------------------------------------------------------------------ neopixels

void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue) {
  strip.setPixelColor(pixel, strip.Color(red,green,blue));
  strip.show(); // Set neopixel on off
}

void startLights() {  // Red
  setPixelColor(0, 255, 0, 0); //left back
  setPixelColor(1, 255, 0, 0); //right back
  setPixelColor(2, 255, 0, 0); //right front
  setPixelColor(3, 255, 0, 0); //left front
}

void finishLights() {  // Cyan
  setPixelColor(0, 0, 255, 255); 
  setPixelColor(1, 0, 255, 255); 
  setPixelColor(2, 0, 255, 255); 
  setPixelColor(3, 0, 255, 255); 
}

void leftLights() {  // Green but purple should appear on the left front LED
  setPixelColor(0, 124, 252, 0); 
  setPixelColor(1, 124, 252, 0); 
  setPixelColor(2, 138, 43, 226); 
  setPixelColor(3, 124, 252, 0); 
}

void rightLights() { // Green but purple should appear on the right front LED
  setPixelColor(0, 124, 252, 0);
  setPixelColor(1, 124,252, 0);
  setPixelColor(2, 124, 252, 0);
  setPixelColor(3, 138, 43, 226);
}

void neutralLights() { // Green
  setPixelColor(0, 124, 252, 0); //left back
  setPixelColor(1, 124, 252, 0); //right back
  setPixelColor(2, 124, 252, 0); //right front
  setPixelColor(3, 124, 252, 0); //left front
}
