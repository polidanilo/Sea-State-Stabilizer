// Sea-State Stabilizer - Main Control Logic (WIP)
// Integrates MPU6050 sensor readings and PCA9685 actuation with PID loop

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_MPU6050 mpu;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Placeholder PID variables
float kp = 1.5, ki = 0.05, kd = 0.5;
float error, last_error, integral;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  
  // MPU6050 initialization
  if (!mpu.begin()) {
    Serial.println("MPU6050 sensor not found");
  }
  
  // PCA9685 initialization
  pwm.begin();
  pwm.setPWMFreq(50);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Error calculation (Y angle / Pitch)
  error = a.acceleration.y; // Semplification for tests
  
  // PID base logic
  integral += error;
  float derivative = error - last_error;
  float output = (kp * error) + (ki * integral) + (kd * derivative);
  last_error = error;

  // Output mapping on PWM signal (indicative values)
  int pwm_val = map(output, -10, 10, 150, 600);
  pwm_val = constrain(pwm_val, 150, 600);
  
  // Channel 0 actuation (Tilt)
  pwm.setPWM(0, 0, pwm_val);
  
  delay(20); // 50Hz loop
}
