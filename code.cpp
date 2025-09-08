
#include <Wire.h>

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// L298N Motor Driver Pins
#define ENA 5   // PWM pin for motor A speed
#define IN1 6   // Motor A direction pin 1
#define IN2 7   // Motor A direction pin 2
#define IN3 8   // Motor B direction pin 1
#define IN4 9   // Motor B direction pin 2
#define ENB 10  // PWM pin for motor B speed

// MPU6050 Register addresses
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47
#define PWR_MGMT_1 0x6B

// Robot parameters
float targetAngle = 0.0;  // Desired balance angle (0 = upright)
float currentAngle = 0.0; // Current tilt angle from vertical

// Complementary filter variables
float gyroAngle = 0.0;
float accelAngle = 0.0;
float alpha = 0.96;  // Complementary filter coefficient (0.96 = 96% gyro, 4% accel)

// PID Controller variables (tuned for 300 RPM motors)
float kp = 25.0;   // Proportional gain - reduced for faster motors
float ki = 0.1;    // Integral gain - reduced to prevent windup
float kd = 0.6;    // Derivative gain - reduced for smoother response

float pidOutput = 0.0;
float previousError = 0.0;
float integral = 0.0;
unsigned long previousTime = 0;

// Motor speed limits
const int maxSpeed = 255;
const int minSpeed = -255;

// Calibration offsets (adjust these after calibration)
float gyroXOffset = 0.0;
float gyroYOffset = 0.0;
float accelXOffset = 0.0;
float accelYOffset = 0.0;

void setup() {
  Serial.begin(9600);
  
  // Initialize I2C communication
  Wire.begin();
  
  // Initialize MPU6050
  initMPU6050();
  
  // Initialize motor driver pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Stop motors initially
  stopMotors();
  
  // Calibrate sensors
  Serial.println(F("Calibrating sensors..."));
  calibrateSensors();
  Serial.println(F("Calibration complete!"));
  
  delay(3000); // Give more time to position robot upright for slower motors
  Serial.println(F("Starting balance control..."));
}

void loop() {
  // Read sensor data and calculate angle
  readSensors();
  
  // Calculate PID output
  calculatePID();
  
  // Control motors based on PID output
  controlMotors();
  
  // Debug output (comment out for better performance)
  if (millis() % 100 == 0) { // Print every 100ms
    Serial.print("Angle: ");
    Serial.print(currentAngle);
    Serial.print(" | PID: ");
    Serial.println(pidOutput);
  }
  
  delay(10); // Main loop delay for stability
}

void initMPU6050() {
  // Wake up MPU6050 (it starts in sleep mode)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00); // Wake up device
  Wire.endTransmission();
  
  // Configure accelerometer range (+/- 2g)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();
  
  // Configure gyroscope range (+/- 250 degrees/sec)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void readSensors() {
  // Read raw accelerometer and gyroscope data
  int16_t accelX, accelY, accelZ;
  int16_t gyroX, gyroY, gyroZ;
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6);
  
  accelX = (Wire.read() << 8) | Wire.read();
  accelY = (Wire.read() << 8) | Wire.read();
  accelZ = (Wire.read() << 8) | Wire.read();
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6);
  
  gyroX = (Wire.read() << 8) | Wire.read();
  gyroY = (Wire.read() << 8) | Wire.read();
  gyroZ = (Wire.read() << 8) | Wire.read();
  
  // Convert raw values to meaningful units
  float accelX_g = (accelX - accelXOffset) / 16384.0; // +/- 2g range
  float accelZ_g = (accelZ) / 16384.0;
  float gyroY_dps = (gyroY - gyroYOffset) / 131.0;    // +/- 250°/s range
  
  // Calculate angle from accelerometer (in degrees)
  accelAngle = atan2(accelX_g, accelZ_g) * 180.0 / PI;
  
  // Calculate time delta for gyroscope integration
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;
  
  // Integrate gyroscope to get angle (in degrees)
  gyroAngle += gyroY_dps * dt;
  
  // Apply complementary filter
  currentAngle = alpha * (currentAngle + gyroY_dps * dt) + (1.0 - alpha) * accelAngle;
}

void calculatePID() {
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  
  // Calculate error
  float error = targetAngle - currentAngle;
  
  // Proportional term
  float proportional = kp * error;
  
  // Integral term (with windup protection)
  integral += error * dt;
  if (integral > 100) integral = 100;
  if (integral < -100) integral = -100;
  float integralTerm = ki * integral;
  
  // Derivative term
  float derivative = (error - previousError) / dt;
  float derivativeTerm = kd * derivative;
  
  // Calculate total PID output
  pidOutput = proportional + integralTerm + derivativeTerm;
  
  // Check for nan values and reset if found
  if (isnan(pidOutput) || isnan(integral)) {
    pidOutput = 0.0;
    integral = 0.0;
    Serial.println("PID reset due to nan values");
  }
  
  // Limit output to motor speed range
  if (pidOutput > maxSpeed) pidOutput = maxSpeed;
  if (pidOutput < minSpeed) pidOutput = minSpeed;
  
  previousError = error;
  previousTime = currentTime; // Update previousTime here!
}
}

void controlMotors() {
  int motorSpeed = (int)pidOutput;
  
  // If robot is too tilted (more than 45 degrees), stop motors for safety
  if (abs(currentAngle) > 45) {
    stopMotors();
    return;
  }
  
  if (motorSpeed > 0) {
    // Move forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, abs(motorSpeed));
    analogWrite(ENB, abs(motorSpeed));
  } else if (motorSpeed < 0) {
    // Move backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, abs(motorSpeed));
    analogWrite(ENB, abs(motorSpeed));
  } else {
    // Stop motors
    stopMotors();
  }
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void calibrateSensors() {
  float sumGyroX = 0, sumGyroY = 0;
  float sumAccelX = 0, sumAccelY = 0;
  int samples = 1000;
  
  for (int i = 0; i < samples; i++) {
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
    
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6);
    
    accelX = (Wire.read() << 8) | Wire.read();
    accelY = (Wire.read() << 8) | Wire.read();
    accelZ = (Wire.read() << 8) | Wire.read();
    
    // Read gyroscope
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6);
    
    gyroX = (Wire.read() << 8) | Wire.read();
    gyroY = (Wire.read() << 8) | Wire.read();
    gyroZ = (Wire.read() << 8) | Wire.read();
    
    sumGyroX += gyroX;
    sumGyroY += gyroY;
    sumAccelX += accelX;
    sumAccelY += accelY;
    
    delay(3);
  }
  
  // Calculate average offsets
  gyroXOffset = sumGyroX / samples;
  gyroYOffset = sumGyroY / samples;
  accelXOffset = sumAccelX / samples;
  accelYOffset = sumAccelY / samples;
}
