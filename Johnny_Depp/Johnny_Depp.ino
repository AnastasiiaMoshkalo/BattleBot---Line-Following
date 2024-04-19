#include <Adafruit_NeoPixel.h> // Neopixel Library to control the neopixels

// neopixels
#define PIN 11 // Neopixel pin NI(Input)
#define NUMPIXELS 4 // There are 4 neopixels on the board
void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue);
void startLights();
void finishLights();
void leftLights();
void rightLights();
void neutralLights();

// distance sensors
const int echoPin = 9; // Echo sensor echo pin
const int triggerPin = 8; // Echo sensor trigger pin
void distanceSensor();
void distanceReader();

// gripper
const int gripperPin = 12; // Gripper pin GR
void servo(int pulse);

// motors
const int leftBackwards = 7; // Left backwards A1
const int leftForward = 6; // Left forwards A2
const int rightBackwards = 4; // Right backwards B1
const int rightForward = 5; // Right forwards B2
const int motorPulseLeft = 2; // Left motor pulse R1
const int motorPulseRight = 3; // Right motor pulse R2 
void setMotors(int LFMotor, int LBMotor, int RFMotor, int RBMotor);
void forward(int leftSpeed, int rightSpeed);
void backwards(int leftSpeed, int rightSpeed);
void left(int leftSpeed, int rightSpeed);
void right(int leftSpeed, int rightSpeed);
void idle();

// line sensors
const int numberOfSensors = 8;
int lineSensor[numberOfSensors] = {A5, A4, A7, A3, A2, A6, A1, A0} ; // Linesensor pins
void followLine();
void scanFirstBlackBox();
void scanSecondBlackBox();
void scanBlackBox(); // scan just sensors 0 and 7
void scanLine(); // scan with all sensors

Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_RGB + NEO_KHZ800); // Neopixel needed code from library

bool startFlag = false;
bool startRace = false;
bool endRace = false;

// ------------------------------------------------------------------------------------------------ adjustments

const int maxDistance = 20;
const int startDistance = 15;
float distance; 
float duration;
const int echoInterval = 100; // time between seeing the object and calculating the distance

int gripOpen = 1500; // pulse length servo open
int gripClosed = 1050; // pulse length servo closed
int servoInterval = 20; // time between pulse

const int leftSlowSpeed = 140; // slowest speed of the left wheel
const int rightSlowSpeed = 140; // slowest speed of the right wheel
const int backwardsSpeedLeft = 0;// backwards speed of the left Wheel
const int backwardsSpeedRight = 0;// backwards speed of the right Wheel

const int speedTurns = 40; // Adding speed for turns
const int speedSharpTurns = 50; // Adding speed for sharp turns
const int speedOneWay = 50; // Adding speed for one direction not turns
const int startSpeed = 50; // Adding speed for start
const int additionalSpeed = 60; // Additional modifiable speed if needed

int lineValues[numberOfSensors];
int maxSensorValue = 0; // setting Gate
int lineCount = 0; // line counter for the start of the race for it to grab the object

const int MAX_BLACK = 950; // Max value of black reached
const int MIN_BLACK = 900;// The minimum value that reaches black

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

   for(int i = 0; i <= 7; i++){
    pinMode(lineSensor[i], INPUT);
  }

  digitalWrite(gripperPin, LOW); //To open the Gripper
  servo(gripOpen); //Gripper is open
}

// ------------------------------------------------------------------------------------------------ loop

void loop() {
  if (!startFlag) {
    distanceReader();
    startLights();
    for (int i = 0; i < 50; i++) {
    servo(gripOpen);
    delayMicroseconds(1000);
    servo(gripOpen);
    }
    while (distance < startDistance) {
      idle();
      distanceReader();
      
      if (distance > startDistance) {
        break;
      }
    }
    startFlag = true;
    setMotors(255, 0, 255, 0);
    delay(50);
    setMotors(150, 0, 165, 0);
    delay(1350);
  } 
  
    bool lineScanInProgress = false; // Flag to indicate if line scanning is in progress
    unsigned long currentMillis = millis(); // Get the current time
    scanBlackBox();
    static unsigned long timer;
    if (currentMillis > timer) {
      if (lineValues[0] >= MAX_BLACK && !lineScanInProgress && lineValues[7] >= MAX_BLACK && !lineScanInProgress) {
        lineScanInProgress = true; // Set flag to indicate line scanning is in progress
        lineCount++; // Add to the counter
      }
      timer = currentMillis + 50;
    }
    
    //Start sequence of grabbing the object
    if (lineScanInProgress && lineCount >= 4)  {
          if (!startRace) {
             scanBlackBox();
            while (lineValues[0] >= MAX_BLACK || lineValues[7] >= MAX_BLACK ) {
              scanFirstBlackBox();

              if (startRace) {
                break;
              }
            }
          }
        lineScanInProgress = false;
    }
    followLine();
    distanceSensor(); //Detecting obstactles and avoid them
  if (!endRace && startRace) {
    scanLine();
    if (lineValues[0] >= MAX_BLACK && lineValues[7] >= MAX_BLACK && 
        lineValues[1] >= MAX_BLACK && lineValues[2] >= MAX_BLACK && 
        lineValues[3] >= MAX_BLACK && lineValues[4] >= MAX_BLACK &&
        lineValues[5] >= MAX_BLACK && lineValues[6] >= MAX_BLACK)  {
          //This code checks twice if it's the black square or not
            setMotors(150, 0, 165, 0);
            pulseIn(2,HIGH,400UL); // Pin , Pulse , Interval
            pulseIn(3,HIGH,400UL); // Calls that measure the duration of a pulse
            pulseIn(2,HIGH,400UL);
            pulseIn(3,HIGH,400UL);
            pulseIn(2,HIGH,400UL);
            pulseIn(3,HIGH,400UL);
            delay(50);
            scanLine();
        }
        if (lineValues[0] >= MAX_BLACK && lineValues[7] >= MAX_BLACK &&
            lineValues[1] >= MAX_BLACK && lineValues[2] >= MAX_BLACK &&
            lineValues[3] >= MAX_BLACK && lineValues[4] >= MAX_BLACK &&
            lineValues[5] >= MAX_BLACK && lineValues[6] >= MAX_BLACK) {
            scanSecondBlackBox();
        }
    }
}

// ------------------------------------------------------------------------------------------------ motors

void setMotors(int LFMotor, int LBMotor, int RFMotor, int RBMotor) {
  // sets the speed of all the wheels by entering parameters
  analogWrite(leftForward, LFMotor);
  analogWrite(leftBackwards, LBMotor);
  analogWrite(rightForward, RFMotor);
  analogWrite(rightBackwards, RBMotor);
}

void forward(int leftSpeed, int rightSpeed) {
  neutralLights();
  setMotors(leftSpeed, 0, rightSpeed, 0);
}

void backwards(int leftSpeed, int rightSpeed) {
  setMotors(0, leftSpeed, 0, rightSpeed);
}

void right(int leftSpeed, int rightSpeed) {
  rightLights();
  setMotors(leftSpeed, 0, 0, rightSpeed);
}

void left(int leftSpeed, int rightSpeed) {
  leftLights();
  setMotors(0, leftSpeed, rightSpeed, 0);
}

void idle() {
  setMotors(0, 0, 0, 0);
}

// ------------------------------------------------------------------------------------------------ line sensor

void followLine() {
  scanLine();
  static unsigned long previousTime;
  if ((millis() - previousTime) >= 100UL) {
  for (int i = 0; i < numberOfSensors; i++) {
      if (lineValues[3] > maxSensorValue && lineValues[4] > maxSensorValue) {
        maxSensorValue = lineValues[3];
      }
    } 
     previousTime = millis();
  }

    // use the thresholds to determine the behavior on the maximum sensor value
    if (maxSensorValue >= MAX_BLACK) {

      if (lineValues[3] >= MIN_BLACK || lineValues[4] >= MIN_BLACK ) {
        //We start with slowest and then modify the speed to a decent speed
        forward(leftSlowSpeed + speedOneWay + additionalSpeed, rightSlowSpeed + speedOneWay + additionalSpeed); 
      }
        else if ( lineValues[2] >= MAX_BLACK) {
        right(leftSlowSpeed + speedTurns + additionalSpeed, backwardsSpeedRight);
      }
      else if (lineValues[5] >= MAX_BLACK) {
        left(backwardsSpeedLeft, rightSlowSpeed + speedTurns + additionalSpeed);
      }
      else if (lineValues[1] >= MIN_BLACK) {
        right(leftSlowSpeed + speedSharpTurns + additionalSpeed, backwardsSpeedRight);
      }
      else if (lineValues[6] >= MIN_BLACK) {
        left(backwardsSpeedLeft, rightSlowSpeed + speedSharpTurns + additionalSpeed);
      }    
   } 
}

void scanBlackBox() {
   for (int i = 0; i < 2; i++) {
    lineValues[0] = analogRead(lineSensor[0]); 
    lineValues[7] = analogRead(lineSensor[7]); 
  }  
}

void scanLine() {
  for (int i = 0; i < numberOfSensors; i++) {
    lineValues[i] = analogRead(lineSensor[i]);
  } 
}

void scanFirstBlackBox() {
  if (lineValues[0] >= MAX_BLACK || lineValues[7] >= MAX_BLACK ) {
    for (int i = 0; i < 50; i++) {
      delayMicroseconds(1000);
      servo(gripClosed);
      startRace = true;
    }
  }
  left(leftSlowSpeed, rightSlowSpeed + speedTurns);
  delay(1595);
  forward(leftSlowSpeed + startSpeed, rightSlowSpeed + startSpeed);
  followLine();
}

void scanSecondBlackBox() {
  scanLine();
  if(lineValues[0] >= MAX_BLACK && lineValues[7] >= MAX_BLACK && 
     lineValues[1] >= MAX_BLACK && lineValues[2] >= MAX_BLACK && 
     lineValues[3] >= MAX_BLACK && lineValues[4] >= MAX_BLACK && 
     lineValues[5] >= MAX_BLACK && lineValues[6] >= MAX_BLACK) {
    backwards(leftSlowSpeed + speedOneWay, rightSlowSpeed + speedOneWay);
    delay(125);
    servo(gripOpen);
    endRace = true; 
  }
  if (endRace) {
      backwards(leftSlowSpeed + speedOneWay + additionalSpeed, rightSlowSpeed + speedOneWay + additionalSpeed);
      delay(1100);
      idle();
      finishLights();
    }
    while (endRace) { // when ending race, bot should go back, stop and end function.
      idle();
      finishLights();
    }  
}

// ------------------------------------------------------------------------------------------------ distance sensor

void distanceReader() {
  digitalWrite(triggerPin, LOW); // reset the pin
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH); // start a high pulses for 10 ms
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  duration = pulseIn(echoPin, HIGH); // read the pins

  distance = (duration / 2) * 0.034; // 343 m/s per second as speed of sound
}

void distanceSensor() {
static unsigned long timer;
 if (millis() > timer) {
    distanceReader();
    if (distance <= maxDistance && lineCount >= 4){ // condition that it avoided 4 lines (at the start) so it doesnt mix in at the beginning
          //  It will avoid it anything closer at approx. 20CM
          
          right(leftSlowSpeed + speedTurns + additionalSpeed, backwardsSpeedRight); 
          delay(500);

          forward(leftSlowSpeed + speedOneWay + additionalSpeed, rightSlowSpeed + speedOneWay + additionalSpeed);
          delay(300);

          left(backwardsSpeedLeft, rightSlowSpeed + speedTurns + additionalSpeed);
          delay(500);

          forward(leftSlowSpeed + speedOneWay + additionalSpeed, rightSlowSpeed + speedOneWay + additionalSpeed);
          delay(550);

          left(backwardsSpeedRight, leftSlowSpeed + speedTurns + additionalSpeed);
          delay(500);

          forward(leftSlowSpeed + speedOneWay + additionalSpeed, rightSlowSpeed + speedOneWay + additionalSpeed);
          delay(350);

          right(rightSlowSpeed + speedTurns, leftSlowSpeed);
          delay(100);
          
        followLine();
      }
    else {
        followLine();
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

// 0 = LEFT BACK, 1 = RIGHT BACK, 2 = RIGHT FRONT, 3 = LEFT FRONT

void startLights() {  // red
  setPixelColor(0, 255, 0, 0);
  setPixelColor(1, 255, 0, 0);
  setPixelColor(2, 255, 0, 0);
  setPixelColor(3, 255, 0, 0);
}

void finishLights() {  // cyan
  setPixelColor(0, 0, 255, 255); 
  setPixelColor(1, 0, 255, 255); 
  setPixelColor(2, 0, 255, 255); 
  setPixelColor(3, 0, 255, 255); 
}

void leftLights() {  // green but purple should appear on the left front LED
  setPixelColor(0, 124, 252, 0); 
  setPixelColor(1, 124, 252, 0); 
  setPixelColor(2, 138, 43, 226); 
  setPixelColor(3, 124, 252, 0); 
}

void rightLights() { // green but purple should appear on the right front LED
  setPixelColor(0, 124, 252, 0);
  setPixelColor(1, 124,252, 0);
  setPixelColor(2, 124, 252, 0);
  setPixelColor(3, 138, 43, 226);
}

void neutralLights() { // green
  setPixelColor(0, 124, 252, 0);
  setPixelColor(1, 124, 252, 0);
  setPixelColor(2, 124, 252, 0);
  setPixelColor(3, 124, 252, 0);
}
