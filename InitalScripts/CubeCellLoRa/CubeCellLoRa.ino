#include "LoRaWan_APP.h"
#include "Arduino.h"

// OTAA parameters
uint8_t devEui[] = { ......... };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { ........ };


// ABP parameters (not used in OTAA, but required for compilation)
uint32_t devAddr = 0x260111FD;
uint8_t nwkSKey[] = { 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88 };
uint8_t appSKey[] = { 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88 };

// LoRaWAN parameters
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };
LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_EU868; // Set region
DeviceClass_t loraWanClass = CLASS_A;
uint32_t appTxDutyCycle = 61000;
bool overTheAirActivation = true;
bool loraWanAdr = true;
bool keepNet = false;
bool isTxConfirmed = false;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("CubeCell AB01 OTAA Test");
  
  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();
}

void loop() {
  switch( deviceState )
  {
    case DEVICE_STATE_INIT:
    {
      printDevEui();
      LoRaWAN.init(loraWanClass, loraWanRegion);
      deviceState = DEVICE_STATE_JOIN;
      Serial.println("DEVICE_STATE_INIT");
      break;
    }
    case DEVICE_STATE_JOIN:
    {
      Serial.println("DEVICE_STATE_JOIN");
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND:
    {
      Serial.println("DEVICE_STATE_SEND");
      prepareTxFrame( appPort );
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      Serial.println("DEVICE_STATE_CYCLE");
      // Schedule next packet transmission
      txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
      Serial.println("DEVICE_STATE_SLEEP");
      LoRaWAN.sleep();
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}

void prepareTxFrame( uint8_t port )
{
  appDataSize = 4;
  appData[0] = 0x00;
  appData[1] = 0x01;
  appData[2] = 0x02;
  appData[3] = 0x03;
}

void printDevEui()
{
  Serial.print("DevEui: ");
  for (int i = 0; i < 8; i++) {
    if (devEui[i] < 0x10)
      Serial.print("0");
    Serial.print(devEui[i], HEX);
  }
  Serial.println();
}