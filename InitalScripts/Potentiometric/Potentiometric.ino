//using 4 analogue pins - No good, no current applied
#include <Arduino.h>

const int ELECTRODES[4] = {32, 33, 34, 35}; // analog pins
const int REFERENCE_RESISTOR = 510; // Resistance value in Ohms
const float OPERATING VOLTAGE = 5;
const float RESOLUTION = 1023.0;
void setup() {
  Serial.begin(9600);
  Serial.println("Starting Conductivity Sensor...");

  // Set electrode pins as input
  for (int pin : electrodes) {
    pinMode(pin, INPUT);
  }
}

void loop() {
  for (int pin : ELECTRODES) {
    int sensorValue = analogRead(pin);
    float voltage = sensorValue * (OPERATING VOLTAGE / RESOLUTION); // Convert ADC reading to voltage (assuming 5V reference)
    float resistance = REFERENCE_RESISTOR * ((OPERATING VOLTAGE - voltage) / voltage); // Calculate resistance using voltage divider formula

    Serial.print("Electrode ");
    Serial.print(pin - ELECTRODES[0] + 1); // Print electrode value
    Serial.print(": Resistance = ");
    Serial.print(resistance);
    Serial.println(" Ohms");
  }
  Serial.println("-------------------------");
  delay(1000); //  delay between readings 
}