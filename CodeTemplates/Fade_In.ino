int ledRed = 11;
 int brightness = 0;

 void setup() {
   // put your setup code here, to run once:
   pinMode(ledRed, OUTPUT);
 }

 void loop() {
   // put your main code here, to run repeatedly:
   for (brightness = 0; brightness < 255; brightness += 5){
  analogWrite(ledRed, brightness);
  delay(30);
   }
   for (brightness = 255; brightness > 0; brightness -= 5){
  analogWrite(ledRed, brightness);
  delay(30);
   }
 } int ledRed = 11;
 int brightness = 0;

 void setup() {
   // put your setup code here, to run once:
   pinMode(ledRed, OUTPUT);
 }

 void loop() {
   // put your main code here, to run repeatedly:
   for (brightness = 0; brightness < 255; brightness += 5){
  analogWrite(ledRed, brightness);
  delay(30);
   }
   for (brightness = 255; brightness > 0; brightness -= 5){
  analogWrite(ledRed, brightness);
  delay(30);
   }
 }
