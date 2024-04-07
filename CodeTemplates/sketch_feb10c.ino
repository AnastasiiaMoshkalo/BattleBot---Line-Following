const int button = 2;
const int red = 13;
const int yellow = 12;
const int green = 11;
int buttonState = 0;

void setup() {
  pinMode(red, OUTPUT);
  pinMode(yellow, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(button, INPUT);
}

void loop() {
  buttonState = digitalRead(button);

  if(buttonState == LOW) {
      digitalWrite(red, LOW);        
      delay(3000); 
      digitalWrite(red, HIGH);
    
      digitalWrite(yellow, LOW);
      delay(1000);
      digitalWrite(yellow, HIGH);
     
      digitalWrite(green, LOW);
      delay(4000);
      digitalWrite(green, HIGH);
    
      digitalWrite(yellow, LOW);
      delay(1000);
      digitalWrite(yellow, HIGH);
    } else{
      digitalWrite(red, HIGH);
      digitalWrite(yellow, HIGH);
      digitalWrite(green, HIGH);
    }

}
