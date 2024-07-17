#include <MsTimer2.h>

// Define constant for LED pin
const int buttonPin = 6;
const int ledPin = 4;
const int interruptPin = 2;  // Pin 2 is commonly used for interrupts on many Arduino boards

// Variable to store the button state
int buttonState = 0;

const byte numChars = 32;
char receivedChars[numChars];  // an array to store the received data

boolean newData = false;

int dataNumber = 0;  // new for this version

void setup() {
  // Initialize the LED pin as an output
  pinMode(ledPin, OUTPUT);
  // Initialize the button pin as an input
  pinMode(buttonPin, INPUT);

  // Initialize interrupt pin and attach interrupt handler
  attachInterrupt(digitalPinToInterrupt(interruptPin), buttonInterrupt, CHANGE);

  // Initialize Serial communication (for printing messages)
  Serial.begin(9600);

  Serial.println("<Arduino is ready>");
  MsTimer2::set(1000, timerInterrupt);  // set the timer initial interrupt period 1000 ms
}

void loop() {
  recvWithEndMarker();  //recive input from the user
  showNewNumber();      //use it to set a timer for the LED

  // Read the state of the button
  int buttonState = digitalRead(buttonPin);

  // Read the state of the LED
  int ledState = digitalRead(ledPin);

  // Determine and print the current state
  if (ledState == LOW) {
    // LED off
    Serial.println("0");
  } else if (buttonState == HIGH && ledState == HIGH) {
    // Button and LED on
    Serial.println("1");
  } else if (buttonState == LOW) {
    // Button off
    Serial.println("2");
  }
}

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  if (Serial.available() > 0) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    } else {
      receivedChars[ndx] = '\0';  // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

void showNewNumber() {
  if (newData == true) {
    dataNumber = 0;                    // new for this version
    dataNumber = atoi(receivedChars);  // new for this version
    Serial.print("This just in ... ");
    Serial.println(receivedChars);
    Serial.print("I received: ");                          // new for this version
    Serial.println(dataNumber);                            // new for this version
    MsTimer2::set(dataNumber * 1000 + 1, timerInterrupt);  // set the timer interrupt period "dataNumber" ms
    newData = false;
  }
}

// Interrupt service routine for the button press
void buttonInterrupt() {
  // Read the state of the button
  buttonState = digitalRead(buttonPin);

  // Update LED state based on button press
  if (buttonState == HIGH) {
    // Turn on the LED
    digitalWrite(ledPin, HIGH);
    // Start the timer (MsTimer2 starts automatically on set)
    MsTimer2::start();
  }
}

void timerInterrupt() {
  // Turn off the LED after "dataNumber" ms
  digitalWrite(ledPin, LOW);
}