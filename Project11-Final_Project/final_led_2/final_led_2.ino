void setup() {
  pinMode(6, OUTPUT);  // Set pin 6 as output
}

void loop() {
  digitalWrite(6, HIGH);  // Turn the buzzer on
  delay(1000);  // Wait for 1 second
  digitalWrite(6, LOW);  // Turn the buzzer off
  delay(1000);  // Wait for 1 second
}
