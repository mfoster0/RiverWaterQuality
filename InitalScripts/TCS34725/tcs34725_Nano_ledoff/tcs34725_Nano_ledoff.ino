#include <Wire.h>
#include "Adafruit_TCS34725.h"

/* Example code for the Adafruit TCS34725 breakout library */

/* For Nano set up
   Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground 
   LED to D2/PIN 2
   */
            
/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

#define LED_PIN 13
#define EXT_LED_PIN 4

void setup(void) {
  Serial.begin(9600);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  
/*
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
*/
  // Now we're ready to get readings!
}

void loop(void) {
  delay(2000);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(EXT_LED_PIN, HIGH);
  delay(1000);
  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  long total = (long)(r + g + b);

  int rPix = (r * 32) / total;
  int gPix = (g * 32) / total;
  int bPix = (b * 32) / total;


  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");
  Serial.print("Ratio Pixels: "); Serial.print(rPix); Serial.print(" "); Serial.print(gPix); Serial.print(" "); Serial.print(bPix); Serial.println(" ");
  digitalWrite(LED_PIN, LOW);
  digitalWrite(EXT_LED_PIN, LOW);

}
