// Test to bypass the PCA9685 and check for correct functioning of servos and current distirbution
int servoPin = 18;

void setup() {
  pinMode(servoPin, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // PWM impulse generation (without use of libraries)
  for(int i=0; i<50; i++) {
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(1500); // neutral impulse (center)
    digitalWrite(servoPin, LOW);
    delay(20);
  }
  delay(1000);
}
