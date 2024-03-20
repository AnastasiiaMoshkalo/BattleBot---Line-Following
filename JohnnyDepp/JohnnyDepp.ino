#include <Servo.h>
Servo gripper;
int openPos = 48;
int closePos = 125;

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
  Serial.println(LS2);
  Serial.println(LS3);
  Serial.println(LS4);
  delay(200);

//  if(startRace){
  followLine();
    
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
  if (LS1 > 900 && LS2 > 900 && LS3 > 900 && LS4 > 900 && LS5 > 900 && LS6 > 900 && LS7 > 900 && LS8 > 900){
    return true;
  }
}

void openGripper(){
    gripper.write(openPos);
  }

  void closeGripper(){
    gripper.write(closePos);
  }

// ---------------------------------------------------- MOTOR FUNCTION

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

void followLine(){
  readLineSensors();
    if (LS2 > 970 || LS3 > 960 || LS4 > 960 || LS5 > 970 || LS6 > 980 || LS7 > 980){
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

// ---------------------------------------------------- AVOID OBJECT FUNCTION

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
    boolean turningLeft = true;
    boolean turningRight = false;
    boolean changedPosition1 = false;
    boolean changedPosition2 = false;

    if(turningLeft){
      idle(); //stopping the motors so the robot will not move
      turnLeft(30, 200);// turning left to avoid object
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
      turnRight(30, 200); //moving right so robot will be headed back for the line
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
  digitalWrite(trigPin,HIGH); //send pulse to trigPin to send UltraSonic wave for detection
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
}

void tickLeft(){
  noInterrupts();
  ticksLeft++;
  interrupts();
}

void tickRight(){
  noInterrupts();
  ticksRight++;
  interrupts();
}
