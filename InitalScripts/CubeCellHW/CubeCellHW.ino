//based on Dallas example

#include <DallasTemperature.h>

// DS18B20 pin
#define ONE_WIRE_BUS GPIO1

// Setup a oneWire instance
OneWire oneWire(ONE_WIRE_BUS);

// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(115200);
  sensors.begin(); // Start up the library
}

void loop() {
  sensors.requestTemperatures(); // Send command to get temperatures
  float temp = sensors.getTempCByIndex(0); // Get temperature in Celsius

  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println(" C");

  delay(1000); // Wait 1 second
}

