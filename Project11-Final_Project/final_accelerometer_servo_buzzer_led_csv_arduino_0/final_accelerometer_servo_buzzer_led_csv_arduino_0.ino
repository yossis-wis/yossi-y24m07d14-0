#include <Servo.h>
#include "LIS3DHTR.h"
#include <Wire.h>
#include <U8g2lib.h>

// Create objects for the servo motor and accelerometer
Servo myservo;  
LIS3DHTR<TwoWire> LIS; 

// Create object for the OLED display
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);  // High speed I2C

// Define the pins
#define SERVO_PIN 7  
#define BUZZER_PIN 5
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

  // Initialize the buzzer pin as output
  pinMode(BUZZER_PIN, OUTPUT); // Set buzzer pin as output

  // Initialize the OLED display
  u8g2.begin();
}

void loop() {
  // Read accelerometer data
  float x = LIS.getAccelerationX();

  // Map the X-axis acceleration to servo angle (0 to 150 degrees)
  int angle = map(x * 100, -100, 100, 0, 150);  // Assuming x ranges from -2g to 2g

  // Move the servo based on the mapped angle
  myservo.write(angle);

  // Determine the buzzer state
  bool buzzerState = (angle > 120 || angle < 30);
  if (buzzerState) {
    tone(BUZZER_PIN, 1000);  // Generate a 1000 Hz tone
  } else {
    noTone(BUZZER_PIN);  // Stop the tone
  }

  // Print the readings to the Serial Monitor in CSV format
  Serial.print(x);
  Serial.print(", ");
  Serial.print(angle);
  Serial.print(", ");
  Serial.println(buzzerState ? "ON" : "OFF");

  // Display the angle and buzzer state on the OLED
  u8g2.clearBuffer();                   // Clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr);   // Choose a suitable font
  u8g2.setCursor(0, 20);                // Set the cursor position for the angle
  u8g2.print("Angle: ");
  u8g2.print(angle);
  u8g2.setCursor(0, 40);                // Set the cursor position for the buzzer state
  u8g2.print("Buzzer: ");
  u8g2.print(buzzerState ? "ON" : "OFF");
  u8g2.sendBuffer();                    // Transfer internal memory to the display

  delay(50);  // Small delay for stability
}
