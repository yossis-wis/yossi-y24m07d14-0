#include <Servo.h>
#include "LIS3DHTR.h"
#include <Wire.h>

// Create objects for the servo motor and accelerometer
Servo myservo;  
LIS3DHTR<TwoWire> LIS; 

// Define the pins
#define SERVO_PIN 7  
#define WIRE Wire

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
  };

  // Initialize the accelerometer
  LIS.begin(WIRE, 0x19); // IIC init
  LIS.closeTemp();
  delay(100);
  LIS.setFullScaleRange(LIS3DHTR_RANGE_2G);
  LIS.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);

  // Initialize the servo motor
  myservo.attach(SERVO_PIN);  // Attach the servo to the defined pin
}

void loop() {
  // Read accelerometer data
  float x = LIS.getAccelerationX();

  // Map the X-axis acceleration to servo angle (0 to 180 degrees)
  int angle = map(x * 100, -100, 100, 0, 150);  // Assuming x ranges from -2g to 2g

  // Move the servo based on the mapped angle
  //myservo.write(angle);

  // Print the readings to the Serial Monitor
  Serial.print("x:"); Serial.println(x);
  Serial.print("Angle: "); Serial.println(angle);

  delay(50);  // Small delay for stability
}
