//https://forum.arduino.cc/t/sensor-readings-not-working-in-full-code-but-works-in-individual-function/413801/3
//https://arduino.stackexchange.com/questions/49895/how-to-measure-electrical-conductivity-using-arduino
//https://github.com/DFRobot/Analog-Electrical-Conductivity-Sensor/blob/master/EC_calibration.ino
//https://thecavepearlproject.org/2017/08/12/measuring-electrical-conductivity-with-an-arduino-part1-overview/

#include <DallasTemperature.h>

const int OUTER_ELECTRODE1 = 2;
const int OUTER_ELECTRODE2 = 3;
const int INNER_ELECTRODE1 = A0;
const int INNER_ELECTRODE2 = A1;
const int RESISTOR_READ = A2;
const int TEMP_SENSOR_PIN = A3; 

// DS18B20 pin
#define ONE_WIRE_BUS TEMP_SENSOR_PIN 

// Setup a oneWire instance
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

const int OSCILLATION_FREQUENCY = 1000; // Hz
const int SAMPLE_DELAY = 100; // ms
const int HALF_SEC_MICROS = 500000; //half a second in micros

float TEMP_COEF = 0.02;  //for conductivity calcs 2%
float BOARD_VOLTAGE = 3.3; // MKR1310 logic voltage
float STANDARD_REF_TEMP = 25.0; // standardised temp for conductivity measurements
float RESISTOR_VALUE = 10000.0; // resistor value in Ohms
float KNOWN_COND1 = 1413.0; // µS/cm from solution 1
float KNOWN_COND2 = 12880.0; // µS/cm from solution 2
float calibrationFactor1, calibrationFactor2;

void setup() {
  Serial.begin(9600);
  pinMode(OUTER_ELECTRODE1, OUTPUT);
  pinMode(OUTER_ELECTRODE2, OUTPUT);
  pinMode(INNER_ELECTRODE1, INPUT);
  pinMode(INNER_ELECTRODE2, INPUT);
  pinMode(RESISTOR_READ, INPUT);
  pinMode(TEMP_SENSOR_PIN, INPUT);
}

void loop() {

  processConductivity();

  delay(4000);
}

void processConductivity(){
    // Generate oscillating current and read voltages
  long sum1 = 0, sum2 = 0, sumR = 0;
  int samples = 0;
  
  unsigned long startTime = millis();
  while (millis() - startTime < SAMPLE_DELAY) {
    digitalWrite(OUTER_ELECTRODE1, HIGH);
    digitalWrite(OUTER_ELECTRODE2, LOW);
    delayMicroseconds(500000 / OSCILLATION_FREQUENCY);
    
    sum1 += analogRead(INNER_ELECTRODE1);
    sum2 += analogRead(INNER_ELECTRODE2);
    sumR += analogRead(RESISTOR_READ);
    samples++;
    
    digitalWrite(OUTER_ELECTRODE1, LOW);
    digitalWrite(OUTER_ELECTRODE2, HIGH);
    delayMicroseconds(500000 / OSCILLATION_FREQUENCY);
    
    sum1 += analogRead(INNER_ELECTRODE1);
    sum2 += analogRead(INNER_ELECTRODE2);
    sumR += analogRead(RESISTOR_READ);
    samples++;
  }
  
  float avgVoltage1 = (float)sum1 / samples;
  float avgVoltage2 = (float)sum2 / samples;
  float avgResistorVoltage = (float)sumR / samples;
  
  // Read temperature
  sensors.requestTemperatures(); 
  float temperature = sensors.getTempCByIndex(0); // get temp
  
  // Calculate conductivity
  float conductivity = calculateConductivity(avgVoltage1, avgVoltage2, avgResistorVoltage, temperature);
  
  Serial.print("Conductivity: ");
  Serial.print(conductivity, 4);
  Serial.print(" µS/cm, Temperature: ");
  Serial.print(temperature, 2);
  Serial.println(" °C");
}

float calculateConductivity(float v1, float v2, float vR, float temperature) {
  // Convert analog readings to voltages
  float voltage1 = v1 * (BOARD_VOLTAGE / 1023.0);
  float voltage2 = v2 * (BOARD_VOLTAGE / 1023.0);
  //across R2
  float resistorVoltage = vR * (BOARD_VOLTAGE / 1023.0);
  
  // Calc voltage drop in solution
  float solutionVoltage = voltage2 - voltage1;
  
  // Calc resistance of solution
  float resistance = resistorVoltage / (solutionVoltage / RESISTOR_VALUE); 
  
  // Calc raw conductivity
  float rawConductivity = 1.0 / resistance; // Siemens
  
  // temperature compensation
  float compensatedConductivity = rawConductivity / (1 + TEMP_COEF * (temperature - STANDARD_REF_TEMP));
  
  // apply calibration factors
  float calibratedConductivity = compensatedConductivity * calibrationFactor1 + calibrationFactor2;
  
  return calibratedConductivity * 1000000; // Convert to µS/cm
}

void calibrateSensor() {
  // Calibration using known solutions
  // First calibration solution
  float avgVoltage1 = measureAverageVoltage(INNER_ELECTRODE1);
  float avgVoltage2 = measureAverageVoltage(INNER_ELECTRODE2);
  float avgResistorVoltage = measureAverageVoltage(RESISTOR_READ);
  float temperature = analogRead(TEMP_SENSOR_PIN) * (BOARD_VOLTAGE / 1023.0) * 100.0;
  float rawConductivity1 = calculateConductivity(avgVoltage1, avgVoltage2, avgResistorVoltage, temperature);
  
  // second calibration solution
  avgVoltage1 = measureAverageVoltage(INNER_ELECTRODE1);
  avgVoltage2 = measureAverageVoltage(INNER_ELECTRODE2);
  avgResistorVoltage = measureAverageVoltage(RESISTOR_READ);
  temperature = analogRead(TEMP_SENSOR_PIN) * (3.3 / 1023.0) * 100.0;
  float rawConductivity2 = calculateConductivity(avgVoltage1, avgVoltage2, avgResistorVoltage, temperature);
  
  // Calculate calibration factors
  calibrationFactor1 = (KNOWN_COND2 - KNOWN_COND1) / (rawConductivity2 - rawConductivity1);
  calibrationFactor2 = KNOWN_COND1 - calibrationFactor1 * rawConductivity1;
}

float measureAverageVoltage(int pin) {
  long sum = 0;
  int samples = 100; // Number of samples to average
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  return (float)sum / samples;
}
