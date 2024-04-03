#define RIGHT_WHEEL_FORWARD 5  //A2 - 4 - left wheel forward - orange
#define RIGHT_WHEEL_BACKWARDS 4  //A1 - 5 - left wheel backwards - orange

#define LEFT_WHEEL_FORWARD 7  //B2 - 3 - right wheel forward - red
#define LEFT_WHEEL_BACKWARDS 6  //B1 - 2 - right wheel backwards - red

#define ROTATION_SENSOR_LEFT 3 //R1 - 7 - left wheel - yellow
#define ROTATION_SENSOR_RIGHT 2 //R2 - 6 - right wheel - yellow

#define TRIGGER_PIN 8  // trig - 8 pin for sending the sound - yellow
#define ECHO_PIN 9 // echo - 9 pin for receiving the sound - green

#define SENSOR_TIME_OUT 100 // gripper needs to be run every 20 ms

const int gripperPin = 12; // GR - 12 - gripper - black
#define OPEN_GRIP 1500
#define CLOSE_GRIP 1050

#include <Adafruit_NeoPixel.h>
#define NUM_PIXELS 4 
#define PIXEL_PIN 11
Adafruit_NeoPixel pixels(NUM_PIXELS, PIXEL_PIN, NEO_RGB + NEO_KHZ800);
const int NO = 10; // NO - 10 - OFF NEOPIXEL - BLUE
const int NI = 11; // NI - 11 - ON NEOPIXEL - BLUE


int sensorPins[] = { A7, A6, A5, A4, A3, A2, A1, A0 };

int sensors[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

volatile int countL = 0;
volatile int countR = 0;

void ISR_L(){
  countL++;
}

void ISR_R(){
  countR++;
}

double distance = 100;

// -------------------------------------------------------------------------------------------------------- SETUP AND LOOP

void setup() {
  pinMode(LEFT_WHEEL_FORWARD, OUTPUT);
  pinMode(LEFT_WHEEL_BACKWARDS, OUTPUT);

  pinMode(RIGHT_WHEEL_FORWARD, OUTPUT);
  pinMode(RIGHT_WHEEL_BACKWARDS, OUTPUT);

  for (int i = 0; i < 8; ++i) {
    pinMode(A0 + i, INPUT);
  }

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(ROTATION_SENSOR_LEFT,INPUT_PULLUP);
  pinMode(ROTATION_SENSOR_RIGHT,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ROTATION_SENSOR_LEFT), ISR_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTATION_SENSOR_RIGHT), ISR_R, CHANGE);

  Serial.begin(9600);
}

void loop() {
  if( distance > 0 && distance < 15){
        avoidObject();
      }
      else{
        followLine();
      }

  distance = getDistance();
}

// -------------------------------------------------------------------------------------------------------- LINE SENSOR

void checkSensorsValues() {
  for (int i = 0; i < sizeof(sensorPins) / sizeof(sensorPins[0]); i++) {
    if (analogRead(sensorPins[i]) > 900) {
      sensors[i] = 1;
    } else {
      sensors[i] = 0;
    }
  }
}

// -------------------------------------------------------------------------------------------------------- SQUARE

//void allBlack(){
//  for(int i=0; i < 8; i++){
//    if()
//      return false;
//      turnYellow();
//    return true;
//  }
//}
//
//void detectSquare(){
//  
//}

// -------------------------------------------------------------------------------------------------------- SOLVE MAZE FUNCTION

void followLine() {
  checkSensorsValues();
  turnGreen;

  if (forwardCondition()) {
    moveForward();
  } 
  else if (turnLeftCondition()) {
    turnLeft();
  }
  else if (turnRightCondition()) {
    turnRight();
  }
  else if (slightRightCondition()) {
    moveSlightlyRight();
  }
  else if (slightLeftCondition()) {
    moveSlightlyLeft();
  }
  else if (hardRightCondition()) {
    moveHardRight();
  }
  else if (hardLeftCondition()) {
    moveHardLeft();
  }
  else {
    moveBackward();
  }
}

// -------------------------------------------------------------------------------------------------------- AVOID OBJECT

int getDistance(){
  digitalWrite(TRIGGER_PIN, LOW); //clear the trig pin
  delay(2);
  
  digitalWrite(TRIGGER_PIN, HIGH); //generate sound
  delay(10); //generate sound for 10ms
  digitalWrite(TRIGGER_PIN, LOW); //stop generating sound

  double distance = (pulseIn(ECHO_PIN, HIGH) * 0.034)/2; //distance calculation in CM, 

  Serial.print("Distance: ");
  Serial.println(distance);

  return distance;
}

void avoidObject(){
  stop();
  delay(500);
  rotateRight();
  delay(500);
  moveForwardSlowly();
  stop();
  rotateLeft();
}

// -------------------------------------------------------------------------------------------------------- NEO LIGHTS

void turnRed(){
  for(int i=0; i<NUM_PIXELS; i++){
    pixels.setPixelColor(i, pixels.Color(255,0,0)); //Red = Stop
  }
  pixels.show();
  delay(1000);
  }

void turnGreen(){
  for(int i=0; i<NUM_PIXELS; i++){
    pixels.setPixelColor(i, pixels.Color(0,100,0)); //Green = Following the line
  }
  pixels.show();
  delay(1000);
  }

void turnLav(){
  for(int i=0; i<NUM_PIXELS; i++){
    pixels.setPixelColor(i, pixels.Color(181,126,220)); //Lavender = Slight Left
  }
  pixels.show();
  delay(1000);
  }

void turnRas(){
  for(int i=0; i<NUM_PIXELS; i++){
    pixels.setPixelColor(i, pixels.Color(227, 11, 93)); //Rasberry = Slight Right
  }
  pixels.show();
  delay(1000);
  }

void turnJeans(){
  for(int i=0; i<NUM_PIXELS; i++){
    pixels.setPixelColor(i, pixels.Color(92, 178, 197)); //Acid Wash Jeans Blue = Avoiding object
  }
  pixels.show();
  delay(1000);
  }

void turnYellow(){
  for(int i=0; i<NUM_PIXELS; i++){
    pixels.setPixelColor(i, pixels.Color(240,230,140)); //Khaki = Seeing all black
  }
  pixels.show();
  delay(1000);
  }

// -------------------------------------------------------------------------------------------------------- MISC. FUNCTIONS

void stop(){
    analogWrite(LEFT_WHEEL_FORWARD, 0);
    analogWrite(LEFT_WHEEL_BACKWARDS, 0);
    analogWrite(RIGHT_WHEEL_FORWARD, 0);
    analogWrite(RIGHT_WHEEL_BACKWARDS, 0);
}

void forward(){
    analogWrite(LEFT_WHEEL_FORWARD, 120);
    analogWrite(LEFT_WHEEL_BACKWARDS, 0);
    analogWrite(RIGHT_WHEEL_FORWARD, 120);
    analogWrite(RIGHT_WHEEL_BACKWARDS, 0);
}

void back(){
    analogWrite(LEFT_WHEEL_FORWARD, 0);
    analogWrite(LEFT_WHEEL_BACKWARDS, 200);
    analogWrite(RIGHT_WHEEL_FORWARD, 0);
    analogWrite(RIGHT_WHEEL_BACKWARDS, 200);
}

// -------------------------------------------------------------------------------------------------------- MOVEMENTS

void turnRight() {
  while (!(sensors[4] || sensors[3])) 
  {
    checkSensorsValues();
    analogWrite(LEFT_WHEEL_FORWARD, 255);
    analogWrite(LEFT_WHEEL_BACKWARDS, 0);
    analogWrite(RIGHT_WHEEL_FORWARD, 0);
    analogWrite(RIGHT_WHEEL_BACKWARDS, 0);
  }
}

void turnLeft() {
  while (!(sensors[4] || sensors[3])) 
  {
    checkSensorsValues();
    analogWrite(LEFT_WHEEL_FORWARD, 0);
    analogWrite(LEFT_WHEEL_BACKWARDS, 0);
    analogWrite(RIGHT_WHEEL_FORWARD, 255);
    analogWrite(RIGHT_WHEEL_BACKWARDS, 0);
  }
}

bool forwardCondition() {
  return sensors[3] && sensors[4] && (sensors[2] || sensors[5]);
}

bool turnLeftCondition() {
  return sensors[0] || sensors[1];
}

bool turnRightCondition() {
  return sensors[7] || sensors[6];
}

bool slightRightCondition() {
  return sensors[5];
}

bool slightLeftCondition() {
  return sensors[2];
}

bool hardRightCondition() {
  return sensors[4] && !sensors[3];
}

bool hardLeftCondition() {
  return sensors[3] && !sensors[4];
}

void moveForward() {
  analogWrite(LEFT_WHEEL_FORWARD, 255);
  analogWrite(LEFT_WHEEL_BACKWARDS, 0);
  analogWrite(RIGHT_WHEEL_FORWARD, 255);
  analogWrite(RIGHT_WHEEL_BACKWARDS, 0);
}

void moveForwardSlowly() {
  analogWrite(LEFT_WHEEL_FORWARD, 210);
  analogWrite(LEFT_WHEEL_BACKWARDS, 0);
  analogWrite(RIGHT_WHEEL_FORWARD, 210);
  analogWrite(RIGHT_WHEEL_BACKWARDS, 0);
}

void moveSlightlyRight() {
  analogWrite(LEFT_WHEEL_FORWARD, 255);
  analogWrite(LEFT_WHEEL_BACKWARDS, 0);
  analogWrite(RIGHT_WHEEL_FORWARD, 210);
  analogWrite(RIGHT_WHEEL_BACKWARDS, 0);
}

void moveSlightlyLeft() {
  analogWrite(LEFT_WHEEL_FORWARD, 210);
  analogWrite(LEFT_WHEEL_BACKWARDS, 0);
  analogWrite(RIGHT_WHEEL_FORWARD, 255);
  analogWrite(RIGHT_WHEEL_BACKWARDS, 0);
}

void moveHardRight() {
  analogWrite(LEFT_WHEEL_FORWARD, 255);
  analogWrite(LEFT_WHEEL_BACKWARDS, 0);
  analogWrite(RIGHT_WHEEL_FORWARD, 245);
  analogWrite(RIGHT_WHEEL_BACKWARDS, 0);
}

void moveHardLeft() {
  analogWrite(LEFT_WHEEL_FORWARD, 245);
  analogWrite(LEFT_WHEEL_BACKWARDS, 0);
  analogWrite(RIGHT_WHEEL_FORWARD, 255);
  analogWrite(RIGHT_WHEEL_BACKWARDS, 0);
}

void moveBackward() {
  analogWrite(LEFT_WHEEL_FORWARD, 0);
  analogWrite(LEFT_WHEEL_BACKWARDS, 255);
  analogWrite(RIGHT_WHEEL_FORWARD, 0);
  analogWrite(RIGHT_WHEEL_BACKWARDS, 255);
}

void rotateLeft() {
  analogWrite(LEFT_WHEEL_FORWARD, 0);
  analogWrite(LEFT_WHEEL_BACKWARDS, 0);
  analogWrite(RIGHT_WHEEL_FORWARD, 225);
  analogWrite(RIGHT_WHEEL_BACKWARDS, 0);
}

void rotateRight() {
  analogWrite(LEFT_WHEEL_FORWARD, 225);
  analogWrite(LEFT_WHEEL_BACKWARDS, 0);
  analogWrite(RIGHT_WHEEL_FORWARD, 0);
  analogWrite(RIGHT_WHEEL_BACKWARDS, 0);
}
