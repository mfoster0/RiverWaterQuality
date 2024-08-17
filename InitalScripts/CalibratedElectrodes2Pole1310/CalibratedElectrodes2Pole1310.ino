#include <DallasTemperature.h>

const int OUTER_ELECTRODE1 = 2;
const int OUTER_ELECTRODE2 = 3;
const int INNER_ELECTRODE = A0;
const int TEMP_SENSOR_PIN = A1; 

// DS18B20 pin
#define ONE_WIRE_BUS TEMP_SENSOR_PIN 

// Setup a oneWire instance
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

const int OSCILLATION_FREQUENCY = 1000; // Hz
const int SAMPLE_DELAY = 100; // ms
const int HALF_SEC_MICROS = 500000; //half a second in micros

float TEMP_COEF = 0.02; // for conductivity calcs 2%
float BOARD_VOLTAGE = 3.3; // MKR1310 logic voltage
float STANDARD_REF_TEMP = 25.0; // standardised temp for conductivity measurements

void setup() {
  Serial.begin(9600);
  pinMode(OUTER_ELECTRODE1, OUTPUT);
  pinMode(OUTER_ELECTRODE2, OUTPUT);
  pinMode(INNER_ELECTRODE, INPUT);
  pinMode(TEMP_SENSOR_PIN, INPUT);
}

void loop() {

  readConductivityAndTemp();

  delay(4000);
}

//read temperature and conductivity together to perform temperature adjustment for conductivity
void readConductivityAndTemp() {
  long sum = 0;
  int samples = 0;
  
  unsigned long startTime = millis();
  //loop to take readings for desiggnated period at specified frequency (Hz)
  while (millis() - startTime < SAMPLE_DELAY) {
    // Generate oscillating current
    digitalWrite(OUTER_ELECTRODE1, HIGH);
    digitalWrite(OUTER_ELECTRODE2, LOW);
    delayMicroseconds(HALF_SEC_MICROS / OSCILLATION_FREQUENCY);
    
    // get voltage
    sum += analogRead(INNER_ELECTRODE);
    samples++;
    
    // flip polarity
    digitalWrite(OUTER_ELECTRODE1, LOW);
    digitalWrite(OUTER_ELECTRODE2, HIGH);
    delayMicroseconds(HALF_SEC_MICROS / OSCILLATION_FREQUENCY);
    
    // Read voltage again
    sum += analogRead(INNER_ELECTRODE);
    samples++;
  }
  
  // Calculate average reading
  float averageReading = (float)sum / samples;
  
  // Convert to conductivity
  float conductivity = averageReading / 1023.0 * BOARD_VOLTAGE;
  
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0); // Get temp

  // Apply temperature compensation
  float compensatedConductivity = conductivity / (1 + TEMP_COEF * (temperature - STANDARD_REF_TEMP));
  
  Serial.print("Conductivity: ");
  Serial.print(compensatedConductivity, 4);
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