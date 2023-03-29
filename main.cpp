#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Define the LED pin (usually GPIO 2 for the onboard LED on ESP32)
const int ledPin = 2;

// Define the capacitive touch sensor pin
const int touchSensorPin = 4; // Replace '4' with the GPIO pin you connected the touch sensor to

// Store the current state of the LED
bool ledState = false;

// Store the last state of the touch sensor
int lastTouchSensorState = HIGH;

int x=0;


// Initialize the I2C LCD (adjust the address if needed, usually 0x27 or 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address to 0x27 for a 16 chars and 2 line display

void setup() {
// Initialize the LED pin as an output
  pinMode(ledPin, OUTPUT);

  // Initialize the touch sensor pin as an input
 pinMode(touchSensorPin, INPUT);

  // Initialize the serial communication
  Serial.begin(9600);
    // Initialize the LCD
  lcd.init();

}

void loop() {
  // Read the capacitive touch sensor value
  int touchSensorValue = digitalRead(touchSensorPin);

  // Print the touch sensor value to the serial monitor
  Serial.print("Touch Sensor Value: ");
  Serial.println(touchSensorValue);

  if (touchSensorValue == 1) {
    x += 1;
     Serial.println(x);
       // Turn on the backlight
 
  lcd.backlight();

     // Print a message on the first line
  lcd.setCursor(0, 0);
  lcd.print("Hello, ESP32!");
  }
  else {
    Serial.println("not touching button");
}

  // Check if the touch sensor state has changed from not touched to touched

  // Update the last touch sensor state
  lastTouchSensorState = touchSensorValue;

  // Add a small delay to debounce the touch sensor
  delay(1000);
}
