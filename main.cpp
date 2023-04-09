#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include "Adafruit_CCS811.h"
#include <SPI.h>

// Define the LED pin (usually GPIO 2 for the onboard LED on ESP32)
const int ledPin = 2;

// Define the capacitive touch sensor pin
const int touchSensorPin = 4; // Replace '4' with the GPIO pin you connected the touch sensor to

// Store the current state of the LED
bool ledState = false;

// Store the last state of the tobuch sensor
int lastTouchSensorState = LOW;
int x = 0;


Adafruit_CCS811 ccs;
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
   

  if (!ccs.begin()) {
    Serial.println("Failed to start CCS811 sensor! Please check your wiring.");
    while (1);
  }

  // Calibrate temperature sensor
  while (!ccs.available());
  float temp = ccs.calculateTemperature();
  ccs.setTempOffset(temp - 25.0);
}

// Store the current state of the LCD backlight
bool lcdBacklightState = false;

void loop() {
  // Read the capacitive touch sensor value
  int touchSensorValue = digitalRead(touchSensorPin);

 

  // Check if the touch sensor state has changed from not touched to touched
  if (lastTouchSensorState == LOW && touchSensorValue == HIGH) {
    // Toggle the LED state
    ledState = !ledState;

    // Set the LED to the new state
    digitalWrite(ledPin, ledState ? HIGH : LOW);

    // Toggle the LCD backlight state
    lcdBacklightState = !lcdBacklightState;
  
    if (lcdBacklightState) {
      // Turn on the LCD backlight
      lcd.backlight();

      // Print a message on the first line
      lcd.setCursor(0, 0);
      lcd.print("testing...");
      delay(100);
      lcd.clear();
      lcd.setCursor(0, 0);
        if(ccs.available()){
    
      float temp = ccs.calculateTemperature();
      if(ccs.readData()){
        lcd.print("eCO2: ");
        float eCO2 = ccs.geteCO2();
        lcd.print(eCO2);
      }
    }
    } else {
      // Turn off the LCD backlight
      lcd.noBacklight();

      // Clear the LCD
      lcd.clear();
    }
  } else {
    Serial.println("not touching button");
  }

  // Update the last touch sensor state
  lastTouchSensorState = touchSensorValue;

  // Add a small delay to debounce the touch sensor
  delay(100);
} 
