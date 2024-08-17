#include <MKRWAN.h>
#include "secrets.h" //char *appKey and char *appEui

LoRaModem modem;


#define debugSerial Serial

int changeBy = 0;
uint16_t temperature = 100*19.6;
uint16_t humidity = 100*65.4;

void setup(){

  debugSerial.begin(9600);

  // Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 15000)
    ;

  // set regional band 
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


void loop(){
  debugSerial.println("-- LOOP");
  changeBy++;

  //float humidity = 65.4;


  humidity += changeBy;

  if (humidity >= 10000) {
    humidity = 100*65.4;
  }

  // false: Celsius (default)
  // true: Farenheit
  //float temperature = 19.6;
  

  temperature -= changeBy;

  if (temperature <= 100) {
    temperature = 100*19.6;
  }
  debugSerial.print("Temperature: ");
  debugSerial.println(temperature);
  debugSerial.print("Humidity: ");
  debugSerial.println(humidity);

  // Split readings (16 bits) into 2 bytes of 8
  int payloadLen = 9;
  Serial.println(payloadLen);
  
  byte payload[payloadLen];
  payload[0] = highByte(temperature);
  payload[1] = lowByte(temperature);
  payload[2] = highByte(humidity);
  payload[3] = lowByte(humidity);
  
  payload[4] = highByte(temperature);
  payload[5] = lowByte(temperature);
  payload[6] = highByte(humidity);
  payload[7] = lowByte(humidity);
  payload[8] = lowByte(humidity);
  

  modem.beginPacket();
  modem.write(payload, sizeof(payload));
  int err = modem.endPacket(true);
  if (err > 0) {
    Serial.println("Message sent correctly");
  } else {
    Serial.println("Error sending message");
  }

  //due to fair use, keep updates infrequent > 1 min
  delay(300000);
}