#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000); 
  pwm.setPWMFreq(50);  // 50Hz for servos
}

void loop() {
  // Servo motor test on all PCA board channels (shotgun debugging)
  for (uint16_t p = 150; p < 600; p += 5) {
    for (uint8_t i = 0; i < 16; i++) {
      pwm.setPWM(i, 0, p); 
    }
    delay(10); 
  }
  delay(500); 
  for (uint16_t p = 600; p > 150; p -= 5) {
    for (uint8_t i = 0; i < 16; i++) {
      pwm.setPWM(i, 0, p); 
    }
    delay(10);
  }
  delay(500); 
}
