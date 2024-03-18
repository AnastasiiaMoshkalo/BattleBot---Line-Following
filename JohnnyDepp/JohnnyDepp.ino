#include <QYFSensors.h>

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

// Integers for adjusting the speed
int speedRight;
int speedLeft;
int valueRight;
int valueLeft;


// Line sensors
int LS1;
int LS2;
int LS3;
int LS4;
int LS5;
int LS6;
int LS7;
int LS8;

boolean startRace = false;
boolean finishRace = false;

void setup(){
  Serial.begin(9600);
    setupMotors();
    setupLineSensor();
    setupRotarySensor();

    if(startRace == false){
    readLineSensors();
    distanceForward(40, 255);
    idle();
    startRace = true;
    finishRace = false;
    }
  }

void loop(){
  readLineSensors();
  int sensorValue = analogRead(LS1);
  delay(500);

  if(startRace){
      followLine();
    }  
}

// ---------------------------------------------------- SETUPS 

void setupMotors(){
    pinMode(motorRightF, OUTPUT);
    pinMode(motorLeftF, OUTPUT);
  }

void setupLineSensor(){
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
}

void readLineSensors(){
  LS1 = analogRead(A0);
  LS2 = analogRead(A1);
  LS3 = analogRead(A2);
  LS4 = analogRead(A3);
  LS5 = analogRead(A4);
  LS6 = analogRead(A5);
  LS7 = analogRead(A6);
  LS8 = analogRead(A7);
}

void setupRotarySensor(){
  pinMode(rotaryLeft, INPUT);
  pinMode(rotaryRight, INPUT);
  pinMode(motorRightF, OUTPUT);
  pinMode(motorLeftF, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(rotaryLeft), tickLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotaryRight), tickRight, CHANGE); 
  // Rotary sensor's purpose is to detect whether one of the wheels are going faster than the other
}

boolean detectedFinishSquare(){
  if (LS1 > 300 && LS2 > 300 && LS3 > 300 && LS4 > 300 && LS5 > 300 && LS6 > 300 && LS7 > 300 && LS8 > 300){
    return true;
  }
}

// ---------------------------------------------------- MOTOR FUNCTION

void idle(){
  digitalWrite(motorLeftF, LOW);
  digitalWrite(motorRightF, LOW);
  digitalWrite(motorLeftB, LOW);
  digitalWrite(motorRightB, LOW);
}

void turnLeft(int ticksRemaining, int speed){
  ticksLeft = 0;
  if(ticksLeft > ticksRemaining){
    idle();
  }else{
    Serial.println(ticksLeft);
    //setting right wheel on speed so robot will turn going forward, not backwards
    analogWrite(motorLeftF, LOW);
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
    analogWrite(motorRightF, LOW);
  }
}

void backwards(){
  analogWrite(motorLeftB, 200);
  digitalWrite(motorLeftF, LOW);
  analogWrite(motorRightB, 200);
  digitalWrite(motorRightF, LOW);
}

void forward(){
  analogWrite(motorLeftF, 200);
  digitalWrite(motorLeftB, LOW);
  analogWrite(motorRightF, 200);
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
  analogWrite(motorLeftF, 255);
  analogWrite(motorRightF, speedRight);
}

void adjustRight(){  // Adjusts the right motor
  analogWrite(motorRightF, 255);
  analogWrite(motorLeftF, speedLeft);
}

void tickLeft(){
  ticksLeft++;
  interrupts();
}

void tickRight(){
  ticksRight++;
  interrupts();
}

void followLine(){
  readLineSensors();
    if (LS2 > 300 || LS3 > 300 || LS4 > 300 || LS5 > 300 || LS6 > 300 || LS7 > 300){
      int valueLeft = (LS5 + LS6 + LS7) / 3;
      int valueRight = (LS2 + LS3 + LS4) / 3;

      if (detectedFinishSquare){
        finishRace = true;
      }
  }
  if(valueLeft > valueRight){ 
    // If the values to the left are bigger than the values on the right, go sligthly to the right
    adjustRight();
  }else{
    adjustLeft(); 
    // vice versa if the right value is bigger, go slightly to the left
  }
}
