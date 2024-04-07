int ledGreen = 11;
int brightness = 0;

void setup() {
  pinMode(ledGreen, OUTPUT);
}

void loop() {
  // Fade out
  for (brightness = 255; brightness >= 0; brightness -= 5) {
    analogWrite(ledGreen, brightness);
    delay(30);
  }

  // Fade in
  for (brightness = 0; brightness <= 255; brightness += 5) {
    analogWrite(ledGreen, brightness);
    delay(30);
  }
}
