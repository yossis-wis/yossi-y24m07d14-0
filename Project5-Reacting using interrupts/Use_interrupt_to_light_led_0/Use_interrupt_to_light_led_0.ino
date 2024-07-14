// Define the pin numbers
const int buttonPin = 6;
const int ledPin = 4;
const int interruptPin = 2; // Use pin 2 for interrupt

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
  digitalWrite(ledPin, ledState ? HIGH : LOW);

  // Serial statements to monitor the loop
  Serial.println("In loop");
  delay(1000);
}

// Interrupt service routine to toggle the LED state
void handleInterrupt() {
  // Toggle the LED state
  ledState = !ledState;
  digitalWrite(ledPin, ledState ? HIGH : LOW);
}
