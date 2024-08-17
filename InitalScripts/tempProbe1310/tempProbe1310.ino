//editted from Dallas example

#include <DallasTemperature.h>

// GPIO where the DS18B20 signal pin is connected
#define ONE_WIRE_BUS 5 //using D5

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

/*
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is conntec to the Arduino digital pin 4
#define ONE_WIRE_BUS 6

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Start up the library
  sensors.begin();  
}

void loop() {

  // Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
  sensors.requestTemperatures(); 

  Serial.print("Celsius temperature: ");
  // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
  Serial.print(sensors.getTempCByIndex(0)); 
  Serial.print(" - Fahrenheit temperature: ");
  Serial.println(sensors.getTempFByIndex(0));

  // put your main code here, to run repeatedly:
  Serial.print(".");
  delay(1000);
}
*/