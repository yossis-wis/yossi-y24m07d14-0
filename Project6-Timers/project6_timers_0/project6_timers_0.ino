// Define the pin numbers
const int buttonPin = 6;
const int ledPin = 4;
const int interruptPin = 2;  // Use pin 2 for interrupt

// Variable to store the LED state
volatile bool ledState = false;
unsigned long myTime;

void setup() {
  // Initialize the button pin as an input
  // pinMode(buttonPin, INPUT);

  // Initialize the LED pin as an output
  pinMode(ledPin, OUTPUT);

  // Start serial communication at 9600 baud
  // Serial.begin(9600);

  // Attach interrupt to the interrupt pin, trigger on CHANGE
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, CHANGE);
}

void loop() {
  // Update the LED state based on the interrupt
  // digitalWrite(ledPin, ledState ? HIGH : LOW);

  int my Time = millis();

  // Serial statements to monitor the loop
  // Serial.println("In loop");
  // delay(1000);
}

// Interrupt service routine to toggle the LED state
void handleInterrupt() {
  // Toggle the LED state
  // ledState = !ledState;
  // Serial.println("entered interrupt");
  // digitalWrite(ledPin, HIGH);  // Turn the LED on
  interruptFlag = true; // Set the flag
  // Serial.print("Time: ");
  
  // Serial.println(myTime); // prints time since program started

  for (int i = 0; i < 10000; i++) {}
  // delay(500000);

  // digitalWrite(ledPin, LOW);

  // Serial.println("exited interrupt");

  // } else {
  //   digitalWrite(ledPin, LOW);  // Turn the LED off
  // }
}
