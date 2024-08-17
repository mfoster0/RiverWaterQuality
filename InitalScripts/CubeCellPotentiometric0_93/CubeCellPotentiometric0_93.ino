//https://thecavepearlproject.org/2017/08/12/measuring-electrical-conductivity-with-an-arduino-part1-overview/

//updated version with correct voltage for cubecell board

// using config suggested in thecavepearl =project
const int currentPin1 = GPIO2; //electrode 1 via 100k restistor
const int currentPin2 = GPIO3; //electrode 2 via 100k restistor
const int voltagePin1 = GPIO1; //ADC 1 read
const int voltagePin2 = ADC; //ADC 2 read
const int voltagePin3 = GPIO4; //ADC 3 read current over R2

void setup() {
  pinMode(currentPin1, OUTPUT);
  pinMode(currentPin2, OUTPUT);
  pinMode(voltagePin3, INPUT);  
  pinMode(voltagePin2, INPUT);  
  pinMode(voltagePin1, INPUT);  
  Serial.begin(9600);

pinMode(voltagePin1, INPUT_PULLUP);
pinMode(voltagePin2, INPUT_PULLUP);
pinMode(voltagePin3, INPUT_PULLUP);

  //analogReadResolution(12);  // Set ADC resolution to 12 bits
  //analogSetAttenuation(ADC_11db);  // Set attenuation for full 3.3V range
}

void loop() {
  // Generate AC current
  digitalWrite(currentPin1, HIGH);
  digitalWrite(currentPin2, LOW);
  delay(500);

  // Read raw ADC values
  int raw1 = analogRead(voltagePin1);
  delay(100);
  int raw2 = analogRead(voltagePin2);
  delay(100);
  int rawMeasured = analogRead(voltagePin3);

  // Convert to voltages
  float voltage1 = (raw1 / 4095.0) * 2.4;
  float voltage2 = (raw2 / 4095.0) * 2.4;
  float measuredVoltage = (rawMeasured / 4095.0) * 2.4;

  // Print raw and converted values
  Serial.print("Raw1: ");
  Serial.print(raw1);
  Serial.print(" Raw2: ");
  Serial.print(raw2);
  Serial.print(" RawMeasured: ");
  Serial.print(rawMeasured);
  Serial.print(" V1: ");
  Serial.print(voltage1, 3);
  Serial.print(" V2: ");
  Serial.print(voltage2, 3);
  Serial.print(" VMeasured: ");
  Serial.println(measuredVoltage, 3);

  // Reverse current direction
  digitalWrite(currentPin1, LOW);
  digitalWrite(currentPin2, HIGH);
  delay(500);
}

/*

void loop() {
  // Generate AC current
  digitalWrite(currentPin1, HIGH);
  digitalWrite(currentPin2, LOW);
  delay(500);

  // Read raw ADC values
  int raw1 = analogRead(voltagePin1);
  int raw2 = analogRead(voltagePin2);
  int rawMeasured = analogRead(voltagePin3);

  // Convert to voltages
  float voltage1 = (raw1 / 4095.0) * 2.4;
  float voltage2 = (raw2 / 4095.0) * 2.4;
  float measuredVoltage = (rawMeasured / 4095.0) * 2.4;

  // Print raw and converted values
  Serial.print("Raw1: ");
  Serial.print(raw1);
  Serial.print(" Raw2: ");
  Serial.print(raw2);
  Serial.print(" RawMeasured: ");
  Serial.print(rawMeasured);
  Serial.print(" V1: ");
  Serial.print(voltage1, 3);
  Serial.print(" V2: ");
  Serial.print(voltage2, 3);
  Serial.print(" VMeasured: ");
  Serial.println(measuredVoltage, 3);

  // Reverse current direction
  digitalWrite(currentPin1, LOW);
  digitalWrite(currentPin2, HIGH);
  delay(500);
}
*/
/*
void loop() {
  // Generate AC current
  digitalWrite(currentPin1, HIGH);
  digitalWrite(currentPin2, LOW);
  delay(500); // Half-period for 1Hz square wave

  // Read voltages
  float voltage1 = (analogRead(voltagePin1) / 4095.0) * 2.4;
  float voltage2 = (analogRead(voltagePin2) / 4095.0) * 2.4;
  float measuredVoltage = (analogRead(voltagePin3) / 4095.0) * 2.4;

  // Calculate resistance and conductivity (with error checking)
  float resistance = (measuredVoltage > 0) ? ((voltage1 - voltage2) / measuredVoltage) : 0;
  float conductivity = (resistance > 0) ? (1.0 / resistance) : 0;

  // Print values
  Serial.print("Voltage1: ");
  Serial.print(voltage1);
  Serial.print(" Voltage2: ");
  Serial.print(voltage2);
  Serial.print(" Measured voltage: ");
  Serial.print(measuredVoltage);
  Serial.print(" Resistance: ");
  Serial.print(resistance);
  Serial.print(" Conductivity: ");
  Serial.println(conductivity);

  // Reverse current direction
  digitalWrite(currentPin1, LOW);
  digitalWrite(currentPin2, HIGH);
  delay(500); // Half-period for 1Hz square wave
}
*/