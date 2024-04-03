// Defining the pins connected to the motor driver module
const int motorLeftF = 3; // Left motor forward
const int motorRightF = 5; // Right motor forward
const int motorLeftB = 2; // Left motor backwards
const int motorRightB = 4; // Right motor backwards

// Defining the pins for the rotary sensors
const int rotaryLeft = 7; // Left rotary sensor
const int rotaryRight = 6; // Right rotary sensor
int ticksLeft;
int ticksRight;

// Defining pins for echo and trigger of HC-SR04
const int echoPin = 9;
const int trigPin = 8;
long duration; // Variable to store the time taken to the pulse to reach the receiver
long distance; // Variable to store the distance calculated using the distance formula

// Integers for adjusting the speed
int valueRight;
int valueLeft;

// Defining NEOPixels
#include <Adafruit_NeoPixel.h>

#define NUM_PIXELS 4
#define PIXEL_PIN 11
Adafruit_NeoPixel pixels(NUM_PIXELS, PIXEL_PIN, NEO_RGB + NEO_KHZ800);
const int NO = 10;
const int NI = 11;

// Defining the gripper's pin
const int gripperPin = 12;
#define GRIPPER_CLOSE 1050
#define GRIPPER_OPEN 1500

// Line sensors
int LS[8];

boolean startRace = false;
boolean finishRace = false;

void idle();
void readLineSensors();
void readDistance();
void avoidObject();

void tickLeft();
void tickRight();

// -------------------------------------------------------------------------------------------------------- READINGS 

void gripper(int pulse) {
   for (int i = 0; i < 10; i++) {
     digitalWrite(gripperPin, HIGH);
     delayMicroseconds(pulse);
     digitalWrite(gripperPin, LOW);
     delay(20);
   }
 }
 
void readLineSensors(){
  for(int i = 0; i < 8; i++){
    LS[i] = analogRead(A0 + i);
  }
}

// -------------------------------------------------------------------------------------------------------- CALIBRATION

//bool detectAllBlack(){
//  for(int i = 0; i < 8; i++){
//    if (LS[i] <= 900)
//      return true;
//      turnYellow();
//  }
//  return true;
//}

int isBlack(){
  return 900; 
}

bool allBlack(){
  for(int i=0; i < 8; i++){
    if(!isBlack(LS[i]))
      return false;
      turnYellow();
    return true;
  }
}

bool allWhite(){
  for(int i=0; i < 8; i++){
    if(isBlack(LS[i]))
      return false;
    return true;
  }
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


// -------------------------------------------------------------------------------------------------------- MOTOR FUNCTION

void turnLeft(int ticksRemaining, int speed){
  ticksLeft = 0;
  if(ticksLeft > ticksRemaining){
    idle();
  }else{
    Serial.println(ticksLeft);
    //setting right wheel on speed so robot will turn going forward, not backwards
    analogWrite(motorLeftF, 0);
    analogWrite(motorRightF, speed);
  }
}

void turnRight(int ticksRemaining, int speed){
  ticksRight = 0;
  if(ticksRight > ticksRemaining){
    idle();
  }else{
    Serial.println(ticksRight);
    //setting left wheel on speed so robot will turn going forward, not backwards
    analogWrite(motorLeftF, speed);
    analogWrite(motorRightF, 0);
  }
}

void backwards(){
  analogWrite(motorLeftB, 200);
  digitalWrite(motorLeftF, LOW);
  analogWrite(motorRightB, 200);
  digitalWrite(motorRightF, LOW);
}

void forward(){
  analogWrite(motorLeftF, 150);
  digitalWrite(motorLeftB, LOW);
  analogWrite(motorRightF, 150);
  digitalWrite(motorRightB, LOW);
}

void distanceForward(int ticksRemaining, int speed){
  if(ticksLeft > ticksRemaining){
    idle();
  }else{
    analogWrite(motorLeftF, speed);
    analogWrite(motorRightF, speed);
  }
}

void adjustLeft(){  // Adjusts the left motor
  analogWrite(motorLeftF, 170);
  analogWrite(motorRightF, 150);
}

void adjustRight(){  // Adjusts the right motor
  analogWrite(motorRightF, 170);
  analogWrite(motorLeftF, 150);
}

void lineFollower(){
//    turnGreen();
//    readLineSensors();
//    for(int i = 0; i < 8; i++){
//      if (LS[i] <= 900){
//      int valueLeft = (LS[4] + LS[5] + LS[6]) / 3;
//      int valueRight = (LS[1] + LS[2] + LS[3]) / 3;
//  
//      if (allBlack()){
//         idle();
//         forward();
//         delay(150);
//         idle();
//         readLineSensors();
//            if (allBlack()){
//              finishRace = true;
//        }
//      }
//    }
//    if(valueLeft > valueRight){ 
//      turnRas();
//      adjustRight();
//      // If the values to the left are bigger than the values on the right, go sligthly to the right
//    }else{
//      turnLav();
//      adjustLeft(); 
//      // vice versa if the right value is bigger, go slightly to the left
//    }
//  }

      if (allBlack()){
        idle();
        forward();
        delay(150);
        idle();
        // check line sensor again?
          if(allBlack()){
            backwards();
            delay(450);
            idle();
            delay(500);
            gripper(GRIPPER_OPEN);
            finishRace = true;
            idle();
          }
      }
}

// -------------------------------------------------------------------------------------------------------- AVOID OBJECT FUNCTION

void readDistance(){
  //send a 10ms pulse to HC-SR04
  digitalWrite(trigPin, LOW); //initialize trigPin on low
  delayMicroseconds(2); // wait 2ms before sending pulse
  digitalWrite(trigPin,HIGH); //send pulse to trigPin to send UltraSonic wave for detection
  delayMicroseconds(10); //keep trigPin HIGH for 10ms
  digitalWrite(trigPin,LOW); //stop sending pulse
  //calculating distance
  duration = pulseIn(echoPin, HIGH); //storing how much time it took to get the pulse back
  distance = duration * 0.034 / 2; //calculating distance based on time from pulseIn
  delay(500); // waiting before taking another measurement
}

void avoidObject(){
    turnJeans();
    boolean turningLeft = true;
    boolean turningRight = false;
    boolean changedPosition1 = false;
    boolean changedPosition2 = false;

    if(turningLeft){
      idle(); //stopping the motors so the robot will not move
      turnLeft(20, 255);// turning left to avoid object
      turningLeft = false;
      changedPosition1 = true;
    }
    if(changedPosition1){
      idle(); //stopping the motors so the robot will not move
      distanceForward(50, 200); // going forward - on the lateral side of the object
      changedPosition1 = false;
      turningRight = true;
    }
    if(turningRight){
      idle(); //stopping the motors so the robot will not move
      turnRight(20, 255); //moving right so robot will be headed back for the line
      delay(900);
      changedPosition2 = true;
      turningRight = false;
    }
    if(changedPosition2){
      idle(); // stopping the motor so the robot will not move
      distanceForward(50, 200); // going forward to get to the line -- object avoided
      delay(1000);
      changedPosition1 = false;
      changedPosition2 = false;
      turningRight = false;
      turningLeft = false;
    }
  }

void getDistance(){
  //send a 10ms pulse to HC-SR04
  digitalWrite(trigPin, LOW); //initialize trigPin on low
  delayMicroseconds(2); // wait 2ms before sending pulse
  digitalWrite(trigPin, HIGH); //send pulse to trigPin to send UltraSonic wave for detection
  delayMicroseconds(10); //keep trigPin HIGH for 10ms
  digitalWrite(trigPin,LOW); //stop sending pulse
  //calculating distance
  duration = pulseIn(echoPin, HIGH); //storing how much time it took to get the pulse back
  distance = duration * 0.034 / 2; //calculating distance based on time from pulseIn
  Serial.println(distance);
   // waiting before taking another measurement
}

void idle(){
  analogWrite(motorLeftF, LOW);
  analogWrite(motorRightF, LOW);
  digitalWrite(motorLeftB, LOW);
  digitalWrite(motorRightB, LOW);
  turnRed();
}

void tickLeft(){
  ticksLeft++;
  interrupts();
}

void tickRight(){
  ticksRight++;
  interrupts();
}

// -------------------------------------------------------------------------------------------------------- SETUP AND LOOP

void setup(){
  Serial.begin(9600);

    pinMode(motorRightF, OUTPUT); // Setup motors
    pinMode(motorLeftF, OUTPUT);
    
    for(int i = 0; i < 8; i++){ // Setup the line sensors
    pinMode(A0 + i, INPUT);
    }

    pinMode(rotaryLeft, INPUT); // Setup rotary sensors
    pinMode(rotaryRight, INPUT);
    pinMode(motorRightF, OUTPUT);
    pinMode(motorLeftF, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(rotaryLeft), tickLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rotaryRight), tickRight, CHANGE); 
    // Rotary sensor's purpose is to detect whether one of the wheels are going faster than the other
    
    pinMode(echoPin, INPUT); // Setup distance sensors
    pinMode(trigPin, OUTPUT);

    pixels.begin(); // Setup light pixels
    pinMode(NO, OUTPUT);
    pinMode(NI, OUTPUT);

    pinMode(gripperPin, OUTPUT); // Setup gripper

    if(startRace == false){
      readLineSensors();
    if(allBlack){
      gripper(GRIPPER_OPEN);
    }
      distanceForward(40, 255);
      turnLeft(20, 255);
      idle();
      startRace = true;
      finishRace = false;
    }
  }


void loop(){
  if(startRace){
    readDistance();
    if(distance <= 2){
      avoidObject();
    }else{
      lineFollower();
    }
    if(finishRace){
      gripper(GRIPPER_OPEN);
      idle();
      backwards();
      idle();
    }

  readLineSensors();
  digitalRead(Serial.println(LS[4]));
  delay(200);
  }
}
