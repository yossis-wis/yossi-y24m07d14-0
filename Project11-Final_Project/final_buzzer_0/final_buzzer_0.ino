#define BUZZER_PIN 6

void setup() {
  pinMode(BUZZER_PIN, OUTPUT); // Set buzzer pin as output
}

void loop() {
  tone(BUZZER_PIN, 1000); // Generate a 1000 Hz tone
  delay(1000); // Wait for 1 second
  noTone(BUZZER_PIN); // Stop the tone
  delay(1000); // Wait for 1 second
}
