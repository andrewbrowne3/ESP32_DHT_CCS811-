#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_GFX.h>
#include "Adafruit_CCS811.h"
#include <SPI.h>
#include <DHT.h> // Include the DHT library

// Define the LED pin (usually GPIO 2 for the onboard LED on ESP32)
const int ledPin = 2;
float temp;

// Define the capacitive touch sensor pin
const int touchSensorPin = 4; // Replace '4' with the GPIO pin you connected the touch sensor to

// Define the DHT pin and type
const int dhtPin = 5; // Replace '5' with the GPIO pin you connected the DHT11 to
#define DHTTYPE DHT11 // Define the DHT type as DHT11

// Store the current state of the LED
bool ledState = false;

// Store the last state of the touch sensor
int lastTouchSensorState = LOW;
int x = 0;

DHT dht(dhtPin, DHTTYPE); // Initialize the DHT object

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

  // Initialize the DHT sensor
  dht.begin();

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
  }

  if (lcdBacklightState) {
    // Turn on the LCD backlight
    lcd.backlight();

    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    // Print humidity values on the first line
    lcd.setCursor(0, 0);
    lcd.print("Humidity: ");
    if (isnan(humidity)) {
      lcd.print("Err");
    } else {
      lcd.print(humidity);
      lcd.print("%");
    }
    delay(2000);
    lcd.clear();

    // Print temperature values
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    if (isnan(temperature)) {
      lcd.print("Err");
    } else {
      lcd.print(temperature);
      lcd.print((char)223);
      lcd.print("C");
    }
    delay(2000);
    lcd.clear();

    // Calculate and print PH2O values
    const float A = 6.11; // hPa
    const float B = 5310.0; // K
    const float t0 = 273.0; // K
    float x = B * (1 / t0 - 1 / (temperature + t0));
    float PH2Osat = A * exp(x);
    float PH2O = (humidity * PH2Osat) / 100;

    lcd.setCursor(0, 0);
    lcd.print("P H2O: ");
    lcd.print(PH2O);
    delay(2000);
    lcd.clear();

    if (ccs.available()) {
      if (!ccs.readData()) {
        // Print CO2 values
        lcd.setCursor(0, 0);
        lcd.print("CO2: ");
        lcd.print(ccs.geteCO2());
        lcd.print("ppm");
        delay(2000);
        lcd.clear();

        // Print TVOC values
        lcd.setCursor(0, 0);
        lcd.print("TVOC: ");
        lcd.print(ccs.getTVOC());
        delay(2000);
        lcd.clear();
      }
    }
  } else {
    // Turn off the LCD backlight
    lcd.noBacklight();

    // Clear the LCD
    lcd.clear();
  }

  // Update the last touch sensor state
  lastTouchSensorState = touchSensorValue;

  // Add a small delay to debounce the touch sensor
  delay(100);
}
