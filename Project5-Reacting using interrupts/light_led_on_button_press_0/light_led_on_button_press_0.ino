// Define the pin numbers
const int buttonPin = 6;
const int ledPin = 4;

// Variable to store the button state
int buttonState = 0;

void setup() {
  // Initialize the button pin as an input
  pinMode(buttonPin, INPUT);

  // Initialize the LED pin as an output
  pinMode(ledPin, OUTPUT);

  // Start serial communication at 9600 baud
  Serial.begin(9600);
}

void loop() {
  // Read the state of the button
  buttonState = digitalRead(buttonPin);

  // Check if the button is pressed
  if (buttonState == HIGH) {
    // Turn the LED on
    digitalWrite(ledPin, HIGH);
  } else {
    // Turn the LED off
    digitalWrite(ledPin, LOW);
  }

  // Serial statements before the delay
  Serial.println("Before delay");

  // Add a delay of 1000 milliseconds (1 second)
  delay(1000);

  // Serial statements after the delay
  Serial.println("After delay");
}
