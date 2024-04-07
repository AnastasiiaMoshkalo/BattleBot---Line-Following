
const int leftSensorPin = 6; 
const int rightSensorPin = 7; 
const int leftMotorPin = 5; 
const int rightMotorPin = 3;

int leftSensorValue = 0;
int rightSensorValue = 0;

const int threshold = 500;

void setup() {
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
}

void loop() {
  leftSensorValue = analogRead(leftSensorPin);
  rightSensorValue = analogRead(rightSensorPin);

  if (leftSensorValue < threshold && rightSensorValue < threshold) {
    moveForward();
  } else if (leftSensorValue < threshold) {
    turnRight();
  } else if (rightSensorValue < threshold) {
    turnLeft();
  }
}

void moveForward() {
  digitalWrite(leftMotorPin, HIGH); 
  digitalWrite(rightMotorPin, HIGH);
}

void turnLeft() {
  digitalWrite(leftMotorPin, LOW); 
  digitalWrite(rightMotorPin, HIGH);
}

void turnRight() {
    digitalWrite(leftMotorPin, HIGH);
    digitalWrite(rightMotorPin, LOW); 
