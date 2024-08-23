//Tests on MKR 1010

const int pwmPin = 9; // output pin
const int analogPin = A0; // input pin

void setup() {
  pinMode(pwmPin, OUTPUT);
  Serial.begin(9600);
  analogReadResolution(12);
}

void loop() {
  // Generate a PWM signal
  analogWrite(pwmPin, 128); // 50% duty cycle for a square wave

  // induced voltage
  int induced = analogRead(analogPin);

  // calc voltage
  float voltage = induced * (3.3 / 4095.0);

  Serial.print("Induced Voltage: ");
  Serial.println(voltage);

  delay(500); 
}
