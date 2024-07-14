// Define the pin numbers
const int buttonPin = 6;
const int ledPin = 4;
const int interruptPin = 2; // Use pin 2 for interrupt (most Arduinos support interrupts on pin 2 and 3)

// Variable to store the LED state
volatile bool ledState = false;

void setup() {
  // Initialize the button pin as an input
  pinMode(buttonPin, INPUT);

  // Initialize the LED pin as an output
  pinMode(ledPin, OUTPUT);

  // Start serial communication at 9600 baud
  Serial.begin(9600);

  // Attach interrupt to the interrupt pin, trigger on CHANGE
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, CHANGE);
}

void loop() {
  // Update the LED state based on the interrupt
  if (ledState) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }

  // Serial statements to monitor the loop
  Serial.println("In loop");
  delay(1000);
}

// Interrupt service routine to handle the button press
void handleInterrupt() {
  // Read the button state
  int buttonState = digitalRead(buttonPin);

  // Toggle the LED state based on the button state
  if (buttonState == HIGH) {
    ledState = true;
  } else {
    ledState = false;
  }
}
