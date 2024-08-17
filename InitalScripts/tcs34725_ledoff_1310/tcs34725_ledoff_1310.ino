#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <vector>
#include <algorithm>
#include <cmath>

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

#define TCS_LED_PIN 10
#define EXT_LED_PIN 8

void setup(void) {
  Serial.begin(9600);

  delay(1000);
  Serial.println("Starting");
  
  pinMode(TCS_LED_PIN, OUTPUT);
  digitalWrite(TCS_LED_PIN, LOW);  
  pinMode(EXT_LED_PIN, OUTPUT);
  digitalWrite(EXT_LED_PIN, HIGH);  ////////////8888888888888888888888888

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

}

void loop(void) {
  
  readRGBValues();
  
  waitForMillis(4000);
}

void readRGBValues(){
  uint16_t r, g, b, c; //, colorTemp, lux;
  std::vector<int> rVals;
  std::vector<int> gVals;
  std::vector<int> bVals;
  std::vector<int> cVals;
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
  
  //digitalWrite(EXT_LED_PIN, LOW); //////////////8888888888888888888888888

  r = (uint16_t)trimAndAverageReadings(rVals, READS);
  g = (uint16_t)trimAndAverageReadings(gVals, READS);
  b = (uint16_t)trimAndAverageReadings(bVals, READS);
  c = (uint16_t)trimAndAverageReadings(cVals, READS);
  
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

//drop highest and lowest n% of values to reduce noise
float trimAndAverageReadings(std::vector<int>& readings, int numReadings) {
    if (readings.empty() || numReadings < 3) {
        return 0.0f;  // Return 0 if the vector is empty or numReadings is invalid
    }

    // Sort the vector
    std::sort(readings.begin(), readings.end());

    // Calc number of elements to trim
    int trimCount = std::round(numReadings * 0.2);  // 20% from top and bottom
    
    // don't trim more than half the vector
    trimCount = std::min(trimCount, (int)readings.size() / 2 - 1);

    // Sum up
    float sum = 0;
    for (int i = trimCount; i < readings.size() - trimCount; ++i) {
        sum += readings[i];
    }

    // retrun average
    int remainingElements = readings.size() - (2 * trimCount); //2x - 1 at each end
    return sum / remainingElements;
}