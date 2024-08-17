#include "ArduinoLowPower.h"


void setup() {
  //delay to allow interaction with board on startup
  delay(10000);

  for (int i=0; i < 15; i++) {
    pinMode(i, INPUT_PULLUP);
  }

  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(3000);
  digitalWrite(LED_BUILTIN, LOW);

}

void loop() {
  USBDevice.detach();

  digitalWrite(LED_BUILTIN, HIGH);
  delay(1500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1500);

  LowPower.deepSleep(900000);

}
