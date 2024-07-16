#include <Servo.h>

// Define the pin numbers
const int rotary0 = A0;
// Define the pin for the servo
const int servo0 = 7;

int rotary1 = 0;

// Create a Servo object
Servo myServo;

void setup() {
  // put your setup code here, to run once:
  pinMode(rotary0, INPUT);
  // Attach the Servo object to pin 7
  myServo.attach(servo0);

}

void loop() {
  rotary1 = analogRead(rotary0);
  myServo.write(rotary1/6.5);
}
