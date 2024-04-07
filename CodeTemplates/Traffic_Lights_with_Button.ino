int ledRed = 13;
 int ledYellow = 12;
 int ledGreen = 11;
 int buttonPin = 2;
 int buttonState = 0;

 void setup() {
   // put your setup code here, to run once:
   pinMode(ledRed, OUTPUT);
   pinMode(ledYellow, OUTPUT);
   pinMode(ledGreen, OUTPUT);
   pinMode(buttonPin, INPUT);
 }

 void loop() {
   // put your main code here, to run repeatedly:
   buttonState = digitalRead(buttonPin);

   if(buttonState == LOW){
  
   digitalWrite(ledRed, LOW);
   delay(3000);
   digitalWrite(ledRed, HIGH);
  
   digitalWrite(ledGreen, LOW);
   delay(4000);
   digitalWrite(ledGreen, HIGH);

   digitalWrite(ledYellow, LOW);
   delay(1000);
   digitalWrite(ledYellow, HIGH);
   }
 }
