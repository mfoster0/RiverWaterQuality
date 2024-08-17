/////////////////////////////////////////////////////////////////////////////////////
/// This sketch is designed for the CE Exhibition
/// It is intended for use with LED matrix and TCS34725 to take RGB readings
/// The LED matrix is used to display user instructions and readings
/// Code is based on the example in matrixtest in Adafruit_NeoMatrix library with
/// changes to display more text and interupts to handle user interaction.
/// Coded by mfoster
/////////////////////////////////////////////////////////////////////////////////////

#include <avr/sleep.h>

const int wakeUpPin = 3;
const int actionPin = 2;

volatile bool sleepFlag = false;
volatile bool actionFlag = false;
volatile bool promptFlag = true;

unsigned long lastInteraction = millis();

void setup() {
  
  Serial.begin(9600);
  pinMode(wakeUpPin, INPUT_PULLUP);
  
  pinMode(actionPin, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, FALLING);
  
  attachInterrupt(digitalPinToInterrupt(actionPin), doAction, FALLING);
}

void loop() {
  checkTime();
  if (sleepFlag) {
    Serial.println("Going to sleep...");
    delay(1000);
    sleep();
  } 
  delay(50); //slow things down
  Serial.println("Awake!");

  if (promptFlag){
    Serial.println("Prompting user");
    promptUser();
  }

  if (actionFlag){
    Serial.println("It's action time!");

    actionFlag = false;
  }

    // get user to interact
    //sleepFlag = true;

  
}

void promptUser(){

  Serial.println("In prompt user");
  
  sleepFlag = false;
  actionFlag = false;
  promptFlag = false;
  lastInteraction = millis();

}

void checkTime(){
  if ((millis() - lastInteraction) > 10000){
    sleepFlag = true;
  }
}

void sleep() {
  Serial.println("Going to sleep");
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();
  // wake up point ***********
  sleep_disable();
  Serial.println("Waking up");
}

void doAction(){
  Serial.println("In do action");
  
  sleepFlag = false;
  actionFlag = true;
  promptFlag = false;
  lastInteraction = millis();
}

void wakeUp() {
  promptFlag = true;
  sleepFlag = false;
  lastInteraction = millis();
}