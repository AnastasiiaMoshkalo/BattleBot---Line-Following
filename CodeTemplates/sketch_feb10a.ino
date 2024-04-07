int red = 13;
int yellow = 12;
int green = 11;


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(red, OUTPUT);
  pinMode(yellow, OUTPUT);
  pinMode(green, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
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

          
}
