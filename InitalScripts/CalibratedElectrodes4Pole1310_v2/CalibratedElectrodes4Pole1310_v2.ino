//https://forum.arduino.cc/t/sensor-readings-not-working-in-full-code-but-works-in-individual-function/413801/3
//https://arduino.stackexchange.com/questions/49895/how-to-measure-electrical-conductivity-using-arduino
//https://github.com/DFRobot/Analog-Electrical-Conductivity-Sensor/blob/master/EC_calibration.ino
//https://thecavepearlproject.org/2017/08/12/measuring-electrical-conductivity-with-an-arduino-part1-overview/

#include <DallasTemperature.h>
#include <vector>
#include <algorithm>
#include <cmath>

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

float TEMP_COEF = 0.02; // for conductivity calcs
float BOARD_VOLTAGE = 3.3; // MKR1310 logic voltage
float STANDARD_REF_TEMP = 25.0; // standardised temp for conductivity measurements
float RESISTOR_VALUE = 10000.0; // resistor value in Ohms
float KNOWN_COND1 = 1413.0; // µS/cm from solution 1
float KNOWN_COND2 = 9580.0; // µS/cm from solution 2
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

  //calibrateSensor();
  processConductivity();

  delay(4000);
}

void processConductivity(){
    // Generate oscillating current and read voltages
  std::vector<int> readings1;
  std::vector<int> readings2;
  std::vector<int> resistorReadings;
  
  unsigned long startTime = millis();
  int sampleCount = 0;
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
  
  float avgVoltage1 = trimAndAverageReadings(readings1, sampleCount);
  float avgVoltage2 = trimAndAverageReadings(readings2, sampleCount);
  float avgResistorVoltage = trimAndAverageReadings(resistorReadings, sampleCount);
  
  // Read temperature
  sensors.requestTemperatures(); 
  float temperature = sensors.getTempCByIndex(0); // get temp
  
  // Calculate conductivity
  //float conductivity = calculateConductivity(avgVoltage1, avgVoltage2, avgResistorVoltage, temperature);
  
  Serial.print("Conductivity: ");
  Serial.print(conductivity, 4);
  Serial.print(" µS/cm, Temperature: ");
  Serial.print(temperature, 2);
  Serial.println(" °C");
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
/*
//drop highest and lowest values to reduce noise
float trimAndAverageReadings(std::vector<int>& readings, int numReadings) {
    if (numReadings < 3) {
        return 0.0;  // Not enough readings to trim
    }

    // Sort  readings 
    for (int i = 0; i < numReadings - 1; i++) {
        for (int j = i + 1; j < numReadings; j++) {
            if (readings[i] > readings[j]) {
                int temp = readings[i];
                readings[i] = readings[j];
                readings[j] = temp;
            }
        }
    }

    // Trim the first and last
    int sum = 0;
    for (int i = 1; i < numReadings - 1; i++) {
        sum += readings[i];
    }

    // Calculate the average of the remaining readings
    float average = (float)sum / (numReadings - 2);
    return average;
}
*/

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
    // Read temperature
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0); // get temp

  Serial.print("Known cond for solution 1: ");
  Serial.println(KNOWN_COND1);
  Serial.print("Known cond for solution 2: ");
  Serial.println(KNOWN_COND2);

  Serial.println("Reading first solution in 10 seconds");
  delay(10000);
  float rawConductivity1 = calculateConductivity(avgVoltage1, avgVoltage2, avgResistorVoltage, temperature);

  Serial.print("rawConductivity1: ");
  Serial.println(rawConductivity1, 4);
  Serial.println("Switch to next solution within 20 seconds"); 
  delay(20000);

  // second calibration solution
  avgVoltage1 = measureAverageVoltage(INNER_ELECTRODE1);
  avgVoltage2 = measureAverageVoltage(INNER_ELECTRODE2);
  avgResistorVoltage = measureAverageVoltage(RESISTOR_READ);

  float rawConductivity2 = calculateConductivity(avgVoltage1, avgVoltage2, avgResistorVoltage, temperature);
  Serial.print("rawConductivity2: ");
  Serial.println(rawConductivity2, 4);

  // Calculate calibration factors
  calibrationFactor1 = (KNOWN_COND2 - KNOWN_COND1) / (rawConductivity2 - rawConductivity1);
  calibrationFactor2 = KNOWN_COND1 - calibrationFactor1 * rawConductivity1;
  Serial.print("calibrationFactor1: ");
  Serial.println(KNOWN_COND1);
  Serial.print("Known cond for solution 2: ");
  Serial.println(KNOWN_COND2);

}

float measureAverageVoltage(int pin) {
  long sum = 0;
  int samples = 100; // Number of samples to average
  for (int i = 0; i < samples; i++) {
    digitalWrite(OUTER_ELECTRODE1, HIGH);
    digitalWrite(OUTER_ELECTRODE2, LOW);
    sum += analogRead(pin);
    delayMicroseconds(500000 / OSCILLATION_FREQUENCY);
    i++;
    digitalWrite(OUTER_ELECTRODE1, LOW);
    digitalWrite(OUTER_ELECTRODE2, HIGH);
    sum += analogRead(pin);
    delayMicroseconds(500000 / OSCILLATION_FREQUENCY);
  }
  return (float)sum / samples;
}
