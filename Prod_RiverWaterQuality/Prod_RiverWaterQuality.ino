/////////////////////////////////////////////////////////////////////////////////////
/// This sketch is designed for the CE final project
/// It is intended for use with 3 sensors - DS18B20, TCS34725 and a home made 4 pole conductivity sensor
/// It intermittently performs reads on the sesnors and sends the data over LoRa. 
/// Conductivity and RGB readings are taken multiple times, topped and tailed to remove outliers, leaving the core readings which are averaged before sending
/// It is based on examples from Adafruit, Dallas, Arduino LoRa
/// The 4 pole design is based on the idea in //https://thecavepearlproject.org/2017/08/12/measuring-electrical-conductivity-with-an-arduino-part1-overview/
/// Coded by mfoster
/////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <DallasTemperature.h>
#include "ArduinoLowPower.h"
#include <MKRWAN.h>
#include "secrets.h" //get char *appKey and char *appEui

LoRaModem modem;


/* Adapted from example code for the Adafruit TCS34725 breakout library */

/* For Nano set up
   Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground 
   TCS LED to D2/PIN 2
   */

 /* For MKR1310 set up
   Connect SCL    D12/SCL
   Connect SDA    D11/SDA
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground 
   TCS LED to D10.....D2/PIN 2
   External LED to D8
   */    

/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with int time and gain values */
//https://adafruit.github.io/Adafruit_TCS34725/html/_adafruit___t_c_s34725_8h.html
//gain 1X, 4X, 16X, 60X
//int time 2?4, 24, 50, 60, 101, 120, 154, 180, 199, 240, 300, 360, 401, 420, 480, 499, 540, 600, 614
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_480MS, TCS34725_GAIN_60X); // if changed, update message in setup()
const int INTERVAL = 614; //copying longest integration time that the sensor would use for the built in LED

const int READS = 8;
const int LED_LIGHTUP_TIME = 1;

/*
const int currentPin1 = 2; //electrode 1 via 100k restistor
const int currentPin2 = 3; //electrode 2 via 100k restistor
const int voltagePin1 = A0; //ADC 1 read
const int voltagePin2 = A1; //ADC 2 read
const int voltagePin3 = A2; //ADC 3 read current over R2

float upperBound = 1023.0;
*/
const int OUTER_ELECTRODE1 = 0;
const int OUTER_ELECTRODE2 = 1;
const int INNER_ELECTRODE1 = A0;
const int INNER_ELECTRODE2 = A1;
const int RESISTOR_READ = A2;
const int TEMP_SENSOR_PIN = 5; 

// GPIO where the DS18B20 signal pin is connected
#define ONE_WIRE_BUS TEMP_SENSOR_PIN 
#define TCS_LED_PIN 10
#define EXT_LED_PIN 8

// Setup a oneWire instance
OneWire oneWire(ONE_WIRE_BUS);

// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

//for use with conductivity sensor
const int OSCILLATION_FREQUENCY = 1000; // Hz
const int SAMPLE_DELAY = 100; // ms
const int HALF_SEC_MICROS = 500000; //half a second in micros

float TEMP_COEF = 0.02; // Typical for conductivity calcs - 2%/degree
float BOARD_VOLTAGE = 3.3; // MKR1310 logic voltage
float STANDARD_REF_TEMP = 25.0; // standardised temperature for conductivity measurements
const float UPPER_BOUND = 4095.0; //12 bit resolution, if using 10 bit change to 1023.0

uint16_t Temperature = 0;
uint16_t Conductivity = 0;
uint16_t Red = 0;
uint16_t Green = 0;
uint16_t Blue = 0;


void setup(void) {
  //delay to allow interaction with board on startup before going to deep sleep
  delay(20000);

  Serial.begin(9600);

  //switch to 12 bit resolution
  analogReadResolution(12); //if using 10 bit change to 1023.0

  pinMode(OUTER_ELECTRODE1, OUTPUT);
  pinMode(OUTER_ELECTRODE2, OUTPUT);
  pinMode(INNER_ELECTRODE1, INPUT);
  pinMode(INNER_ELECTRODE2, INPUT);
  pinMode(RESISTOR_READ, INPUT);
  pinMode(TEMP_SENSOR_PIN, INPUT);
  /*
  pinMode(currentPin1, OUTPUT);
  pinMode(currentPin2, OUTPUT);
  pinMode(voltagePin3, INPUT);  
  pinMode(voltagePin2, INPUT);  
  pinMode(voltagePin1, INPUT); 
  */  
  pinMode(TCS_LED_PIN, OUTPUT);
  digitalWrite(TCS_LED_PIN, LOW);  
  pinMode(EXT_LED_PIN, OUTPUT);
  digitalWrite(EXT_LED_PIN, LOW);   

  delay(1000);
  Serial.println("Starting");

  delay(1000);
  //start up sensor


  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  Serial.print("READS:");
  Serial.println(READS);
  Serial.print("LED_LIGHTUP_TIME:");
  Serial.println(LED_LIGHTUP_TIME);
  
  Serial.println("TCS Integration Time: TCS34725_INTEGRATIONTIME_480MS:");


  //startup LoRa modem instance set regional band 
  if (!modem.begin(EU868)) {
    Serial.println("Failed to start module");
  }

  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());

  int connected = modem.joinOTAA(appEui, appKey);
  if (!connected) {
    Serial.println("Unable to connect");
  }
}

void loop(void) {
  USBDevice.detach(); //possibly detach in production version

  //get colour values
  readRGBValues();
  
  //get conductivity and temperature values
  processConductivity();

  //send data via LoRa
  sendLora();

  delay(1000);

  //go into sleep mode
  LowPower.deepSleep(600000);
 
}

void sendLora(){
    // Split readings (16 bits) into 2 bytes of 8
  int payloadLen = 10;
  Serial.println(payloadLen);
  
  byte payload[payloadLen];
  payload[0] = highByte(Temperature);
  payload[1] = lowByte(Temperature);
  payload[2] = highByte(Conductivity);
  payload[3] = lowByte(Conductivity);
  
  payload[4] = highByte(Red);
  payload[5] = lowByte(Red);
  payload[6] = highByte(Green);
  payload[7] = lowByte(Green);
  payload[8] = highByte(Blue);
  payload[9] = lowByte(Blue);

  modem.beginPacket();
  modem.write(payload, sizeof(payload));
  int err = modem.endPacket(true);
  if (err > 0) {
    Serial.println("Message sent correctly");
  } else {
    Serial.println("Error sending message");
  }
}

//function to get conductivity values
//to the code takes multiple readings
//the highest and lowest values are dropped to help reduce the opportunity for erroneous values to be included
//the central values are then averaged 
void processConductivity(){
  // Generate oscillating current and read voltages
  std::vector<int> readings1;
  std::vector<int> readings2;
  std::vector<int> resistorReadings;
  
  unsigned long startTime = millis();
  int sampleCount = 0;

  //loop for specified time taking multiple readings
  while (millis() - startTime < SAMPLE_DELAY) {
    digitalWrite(OUTER_ELECTRODE1, HIGH);
    digitalWrite(OUTER_ELECTRODE2, LOW);
    delayMicroseconds(500000 / OSCILLATION_FREQUENCY);
    
    readings1.push_back(analogRead(INNER_ELECTRODE1));
    readings2.push_back(analogRead(INNER_ELECTRODE2));
    resistorReadings.push_back(analogRead(RESISTOR_READ));
    sampleCount++;
    
    digitalWrite(OUTER_ELECTRODE1, LOW);
    digitalWrite(OUTER_ELECTRODE2, HIGH);
    delayMicroseconds(500000 / OSCILLATION_FREQUENCY);
    
    readings1.push_back(analogRead(INNER_ELECTRODE1));
    readings2.push_back(analogRead(INNER_ELECTRODE2));
    resistorReadings.push_back(analogRead(RESISTOR_READ));
    sampleCount++;
  }
  
  //process the arrays, dropping highest and lowest values before averaging
  float avgVoltage1 = trimAndAverageReadings(readings1, sampleCount);
  float avgVoltage2 = trimAndAverageReadings(readings2, sampleCount);
  float avgResistorVoltage = trimAndAverageReadings(resistorReadings, sampleCount);
  
  //read temperature
  sensors.requestTemperatures(); // get temperature
  float temperature = sensors.getTempCByIndex(0); // Get temperature in Celsius
  Temperature = (uint16_t)(temperature * 100.0);
  Serial.print("temperature: ");
  Serial.println(temperature,3);
  Serial.println(Temperature);

  //convert to voltages
  float voltage1 = (avgVoltage1 / UPPER_BOUND) * BOARD_VOLTAGE;
  float voltage2 = (avgVoltage2 / UPPER_BOUND) * BOARD_VOLTAGE;
  float measuredVoltage = (avgResistorVoltage / UPPER_BOUND) * BOARD_VOLTAGE;
  Conductivity = (uint16_t)(measuredVoltage * 100);
  Serial.println(Conductivity);


  // Print raw and converted values
  Serial.print("Raw1: ");
  Serial.print(avgVoltage1);
  Serial.print(" Raw2: ");
  Serial.print(avgVoltage2);
  Serial.print(" RawMeasured: ");
  Serial.print(avgResistorVoltage);
  Serial.print(" V1: ");
  Serial.print(voltage1, 3);
  Serial.print(" V2: ");
  Serial.print(voltage2, 3);
  Serial.print(" VMeasured: ");
  Serial.println(measuredVoltage, 3);

}

//drop highest and lowest n% of values to reduce noise
float trimAndAverageReadings(std::vector<int>& readings, int numReadings) {
    if (readings.empty() || numReadings < 3) {
        return 0.0f;  // return 0 if the list is empty or numReadings is too small
    }

    //sort the vector
    std::sort(readings.begin(), readings.end());

    // calc number of elements to trim
    int trimCount = std::round(numReadings * 0.2);  // 20% from top and bottom
    
    // don't trim more than half the values
    trimCount = std::min(trimCount, ((int)readings.size() / 2) - 1);

    // sum up
    float sum = 0;
    for (int i = trimCount; i < readings.size() - trimCount; ++i) {
        sum += readings[i];
    }

    // retrun average
    int remainingElements = readings.size() - (2 * trimCount); //2x - 1 at each end
    return sum / remainingElements;
}



//function to get RGB values
//multiple reads are taken to improve accuracy
//high and low values are dropped and the core values averaged
void readRGBValues(){
  uint16_t r, g, b, c; //, colorTemp, lux;
  std::vector<int> rVals;
  std::vector<int> gVals;
  std::vector<int> bVals;
  std::vector<int> cVals;

  //switch on external LED
  digitalWrite(EXT_LED_PIN, HIGH);  

  delay(LED_LIGHTUP_TIME);
  //waitForMillis(INTERVAL);

  for (int i=0; i<READS; i++){
    tcs.getRawData(&r, &g, &b, &c);
    rVals.push_back(r);
    gVals.push_back(g);
    bVals.push_back(b);
    cVals.push_back(c);
    delay(5);
    //waitForMillis(5);
  }
  
  //switch off external LED
  digitalWrite(EXT_LED_PIN, LOW); 

  r = (uint16_t)trimAndAverageReadings(rVals, READS);
  g = (uint16_t)trimAndAverageReadings(gVals, READS);
  b = (uint16_t)trimAndAverageReadings(bVals, READS);
  c = (uint16_t)trimAndAverageReadings(cVals, READS);
  
  Red = r;
  Green = g;
  Blue = b;
  Serial.println(Red);
  Serial.println(Green);
  Serial.println(Blue);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  //colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  //lux = tcs.calculateLux(r, g, b);

  //Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  //Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");
}

void waitForMillis(int period){
  unsigned long currentMillis = millis();
  unsigned long waitPeriod = currentMillis + period;
  while (waitPeriod > currentMillis) {
    currentMillis = millis();
    yield();
  }
}

