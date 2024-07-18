#include <Servo.h>

Servo myservo;  // Create a Servo object

#define SERVO_PIN 7  // Define the pin connected to the servo signal

void setup() {
  myservo.attach(SERVO_PIN);  // Attach the servo to pin 3
}

void loop() {
  myservo.write(0);    // Move to 0 degrees
  delay(1000);         // Wait for 1 second
  myservo.write(90);   // Move to 90 degrees
  delay(1000);         // Wait for 1 second
  myservo.write(150);  // Move to 180 degrees
  delay(1000);         // Wait for 1 second
}
