#include <Servo.h>

#include <MD_PWM.h>

// Define the pin numbers
const int rotary0 = A0;
// Define the pin for the Grove LED
const int ledPin = 4;

int rotary1 = 0;

// Create an MD_PWM object and pass the pin number to the constructor
MD_PWM pwmControl(ledPin);

void setup() {

  pinMode(rotary0, INPUT);

  // Start serial communication at 9600 baud
  Serial.begin(9600);

  // Initialize the MD_PWM library on pin 4
  pwmControl.begin(50);

  // Set an initial PWM value
  // pwmControl.write(50); // Set PWM to 50% duty cycle

}

void loop() {

  // Serial statements to monitor the loop
  rotary1 = analogRead(rotary0);
  pwmControl.write(rotary1/4); // Set PWM to 50% duty cycle

}

