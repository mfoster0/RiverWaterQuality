//////////////////////////////////////////////////////////////////////////////////////////////////
/// code based on example from cyaninfinite.com/scrolling-text-with-flexible-32x8-rgb-led-matrix
////////////////////////////////////////////////////////////////////////////////////////////////// 
#include <iostream>
#include <string>

#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>

#define PIN 3

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
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)

Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(32, 8, PIN,
  NEO_MATRIX_BOTTOM    + NEO_MATRIX_RIGHT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG,
  NEO_GRB            + NEO_KHZ800);

const uint16_t colors[] = {
  matrix.Color(255, 0, 0), matrix.Color(0, 255, 0), matrix.Color(255, 255, 0),matrix.Color(0, 0, 255), matrix.Color(255, 0, 255), matrix.Color(0, 255, 255), matrix.Color(255, 255, 255)};

void setup() {
  matrix.begin();
  matrix.setTextWrap(true);
  matrix.setBrightness(30);
  matrix.setTextColor(colors[0]);
}

int x    = matrix.width();
int pass = 0;

void loop(){
  displayText("Press Test");
}

void tester() {
  matrix.fillScreen(0);    //Turn off all the LEDs
  matrix.setCursor(x, 0);
  matrix.print(F("Hello. Press 'Process' to check the colour"));

  if( --x < -30 ) {
    x = matrix.width();

    if(++pass >= 8) pass = 0;
    matrix.setTextColor(colors[pass]);
  }
  matrix.show();
  delay(30);
}

void displayText(const char*  text){
  int i= 0;
  x    = matrix.width();
  
  for (i=0; i<31; i++){
    matrix.fillScreen(0);    //Turn off all the LEDs
    matrix.setCursor(x, 0);
    matrix.print(F(text));

    if( --x < -30 ) {
      x = matrix.width();

      matrix.setTextColor(colors[0]);
    }
    matrix.show();
    delay(60);
  }
}

void hello(){
  matrix.fillScreen(0);    //Turn off all the LEDs
  matrix.setCursor(x, 0);
  matrix.print(F("Hello"));

  if( --x < -30 ) {
    x = matrix.width();

    if(++pass >= 8) pass = 0;
    matrix.setTextColor(colors[0]);
  }
  matrix.show();
  delay(60);
}