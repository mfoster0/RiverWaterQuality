/////////////////////////////////////////////////////////////////////////////////////
/// This sketch is designed for the CE Exhibition
/// It is intended for use with LED matrix and TCS34725 to take RGB readings
/// The LED matrix is used to display user instructions and readings
/// Code is based on the example in matrixtest in Adafruit_NeoMatrix library with
/// changes to display more text and interupts to handle user interaction. 
/// Example from cyaninfinite.com/scrolling-text-with-flexible-32x8-rgb-led-matrix also used
/// Coded by mfoster
/////////////////////////////////////////////////////////////////////////////////////


/* Example code for the Adafruit TCS34725 breakout library */

/* For Nano set up
   Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground 
   LED to D2/PIN 2... now using 13
   */

#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

#ifndef PSTR
 #define PSTR // Make Arduino Due happy
#endif

#define PIN 6

// MATRIX DECLARATION:
// Parameter 1 = width of NeoPixel matrix
// Parameter 2 = height of matrix
// Parameter 3 = pin number (most are valid)
// Parameter 4 = matrix layout flags, add together as needed:
//   NEO_MATRIX_TOP, NEO_MATRIX_BOTTOM, NEO_MATRIX_LEFT, NEO_MATRIX_RIGHT:
//     Position of the FIRST LED in the matrix; pick two, e.g.
//     NEO_MATRIX_TOP + NEO_MATRIX_LEFT for the top-left corner.
//   NEO_MATRIX_ROWS, NEO_MATRIX_COLUMNS: LEDs are arranged in horizontal
//     rows or in vertical columns, respectively; pick one or the other.
//   NEO_MATRIX_PROGRESSIVE, NEO_MATRIX_ZIGZAG: all rows/columns proceed
//     in the same order, or alternate lines reverse direction; pick one.
//   See example below for these values in action.
// Parameter 5 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_GRBW    Pixels are wired for GRBW bitstream (RGB+W NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)


// Example for NeoPixel Shield.  In this application we'd like to use it
// as a 5x8 tall matrix, with the USB port positioned at the top of the
// Arduino.  When held that way, the first pixel is at the top right, and
// lines are arranged in columns, progressive order.  The shield uses
// 800 KHz (v2) pixels that expect GRB color data.
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(32, 8, PIN,
  NEO_MATRIX_BOTTOM    + NEO_MATRIX_RIGHT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG,
  NEO_GRB            + NEO_KHZ800);
  
const uint16_t colors[] = {
  matrix.Color(255, 0, 0), matrix.Color(0, 255, 0), matrix.Color(0, 0, 255) };


#include <avr/sleep.h>
const int awakePeriod = 30000;
const int wakeUpPin = 3;
const int actionPin = 2;

volatile bool sleepFlag = false;
volatile bool actionFlag = false;
volatile bool promptFlag = true;

unsigned long lastInteraction = millis();

/* Initialise TCS with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_240MS, TCS34725_GAIN_1X);

#define LED_PIN 13
//#define EXT_LED_PIN 4

//global colour values
 uint16_t r, g, b, c, colorTemp, lux;

void setup() {
  
  Serial.begin(9600);
  pinMode(wakeUpPin, INPUT_PULLUP);
  
  pinMode(actionPin, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, FALLING);
  
  attachInterrupt(digitalPinToInterrupt(actionPin), doAction, FALLING);

  Serial.begin(9600);
  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setBrightness(30);
  matrix.setTextColor(colors[0]);

  const __FlashStringHelper* text = F("Hi there!");
  displayText(text,2,30);
  

  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  actionFlag = false;

}

//on each loop check if it is time to sleep and if the user has pressed a button then react
void loop() {
  checkTime();
  if (sleepFlag) {
    Serial.println("Going to sleep...");
    delay(1000);
    sleep();
  } 
  
  digitalWrite(LED_PIN, LOW);

  delay(10); //slow things down
  Serial.println("Awake!");

  if (promptFlag){
    Serial.println("Prompting user");
    const __FlashStringHelper* text = F("Insert a sample and press 'Read' to examine it");
    displayText(text, 1, 40);

    lastInteraction = millis();
    promptFlag = false;
  }

  if (actionFlag){
    Serial.println("It's action time!");
    readSample();
    actionFlag = false;
  }

}
int x    = matrix.width();
int pass = 0;

void displayText(const __FlashStringHelper* text, int colour){
  displayText(text, colour, 50);
}
// function to simplify the call to display text on the pixel matrix
// textSpeed will speed up (decrease the delay) or slow down (increase the delay) the text (pixel) progression across the matrix
//    the overloaded function above uses 50 as the default value
void displayText(const __FlashStringHelper* text, int colour, int textSpeed) {
  
  int length = static_cast<int>(strlen_P((const char*)text));
  int i = 0;

  for (i=0; i < (length * 6)+33; i++){
    matrix.fillScreen(0);
    matrix.setCursor(x, 0);
    matrix.setTextColor(colors[colour]);

    matrix.print(text);
    if(--x < -1 * (length * 6)) {
      x = matrix.width();
      //if(++pass >= 3) pass = 0;
      //matrix.setTextColor(colors[pass]);
    }
    matrix.show();
    delay(textSpeed);
  }
  matrix.fillScreen(0);
  matrix.show();
}

   // get user to interact
    //sleepFlag = true;

//read from TCS sensor
void readSample(){
  
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
  
  displayColourRatios();

  digitalWrite(LED_PIN, LOW);
  //digitalWrite(EXT_LED_PIN, LOW);



}
void displayColourRatios(){

  //clear screen
  matrix.fillScreen(0);
  matrix.setCursor(0, 0);
  int x,y; //,r,g,b;
  //0,0 is top left, x is the column, y is row

/*
  // test with so fixed RGB values
  //convert to 32 pixel representation
  r=10;
  g=8;
  b=22;
*/

  //light 2 rows for each colour
  for (x = 0; x < 32; x++){
    for (y = 0; y < 8; y++){
      switch (y) {
        case 0:
        case 1:
          if (x<=r) matrix.drawPixel(x,y,colors[0]);
          break;
        case 3:
        case 4:
          if (x<=g) matrix.drawPixel(x,y,colors[1]);
          break;
        case 6:
        case 7:
          if (x<=b) matrix.drawPixel(x,y,colors[2]);
          break;
      }
    }    
  }

  //*************matrix.drawPixel(x,y,colors[1]);
  matrix.show();


}

//user has woken the device
/*
void promptUser(){

  Serial.println("In promptUser");
  
  sleepFlag = false;
  actionFlag = false;
  promptFlag = true;
  lastInteraction = millis();

}
*/
//check if time to go back to sleep based on awakePeriod
void checkTime(){
  if ((millis() - lastInteraction) > awakePeriod){
    sleepFlag = true;
  }
}

//go to sleep
void sleep() {
  Serial.println("Going to sleep");
  const __FlashStringHelper* text = F("Going to sleep");
  displayText(text,2,30);

  matrix.fillScreen(0);
  matrix.setCursor(0, 0);
  matrix.show();

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();
  // wake up point ***********
  sleep_disable();
  Serial.println("Waking up");
}

//user has pressed the read button
void doAction(){
  Serial.println("In do action");
  
  promptFlag = false;
  sleepFlag = false;
  actionFlag = true;
  lastInteraction = millis();
}

//user has woken the device
void wakeUp() {
  promptFlag = true;
  actionFlag = false;
  sleepFlag = false;
  lastInteraction = millis();
}