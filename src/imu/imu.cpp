#include "IMU.h"
#include <Arduino.h>
#include <math.h>

IMU::IMU() : pitch(0), roll(0) {}

bool IMU::begin(const bool calibrate) {
  mpu.initialize(ACCEL_FS::A8G, GYRO_FS::G500DPS);
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);

  // Hardcoded for now
  mpu.setXAccelOffset(167);
  mpu.setYAccelOffset(863);
  mpu.setZAccelOffset(1246);
  mpu.setXGyroOffset(40);
  mpu.setYGyroOffset(-65);
  mpu.setZGyroOffset(-5);

  const bool connected = mpu.testConnection();

  if (connected) {
    if (calibrate)
      calibrateGyro();
    setStartAngles();

    Serial.println("IMU initialized successfully");
    Serial.print("Gyro X Bias: ");
    Serial.println(calibrationData.gyroXBias);
    Serial.print("Gyro Y Bias: ");
    Serial.println(calibrationData.gyroYBias);
    Serial.print("Gyro Z Bias: ");
    Serial.println(calibrationData.gyroZBias);
    Serial.print("Initial Pitch: ");
    Serial.println(pitch);
    Serial.print("Initial Roll: ");
    Serial.println(roll);
  }

  return connected;
}

void IMU::update(float dt) {
  readSensor();
  convert();
  computeAngles(dt);
}

void IMU::readSensor() {
  mpu.getMotion6(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);
}

void IMU::convert() {
  // Convert to physical units
  ax = rawAx / ACCEL_SCALE;
  ay = rawAy / ACCEL_SCALE;
  az = rawAz / ACCEL_SCALE;

  gx = rawGx / GYRO_SCALE - calibrationData.gyroXBias; // deg/s
  gy = rawGy / GYRO_SCALE - calibrationData.gyroYBias; // deg/s
  gz = rawGz / GYRO_SCALE - calibrationData.gyroZBias; // deg/s
}

void IMU::computeAngles(float dt) {
  roll += gx * dt;
  pitch += gy * dt;

  // --- Accelerometer angles (only if accel is valid) ---
  float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);
  float accelRoll = 0, accelPitch = 0;

  // Only trust accel if magnitude is close to 1g (0.6 to 1.4g range)
  // This rejects high-G maneuvers and centripetal acceleration
  if (accelMagnitude > 0.6f && accelMagnitude < 1.4f) {
    // Use proper 2-axis atan2 for more accurate accel angles
    // Roll: rotation about X-axis (use Y and Z)
    accelRoll = atan2(ay, az) * 180.0f / M_PI;
    // Pitch: rotation about Y-axis (use X and Z)
    accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;
  } else {
    // High-G detected, trust gyro only
    accelPitch = pitch;
    accelRoll = roll;
  }

  const float alpha = 0.996f;
  pitch = alpha * pitch + (1.0f - alpha) * accelPitch;
  roll = alpha * roll + (1.0f - alpha) * accelRoll;
}

float IMU::getPitch() { return pitch; }

float IMU::getRoll() { return roll; }

void IMU::calibrateGyro() {
  long sumX = 0, sumY = 0, sumZ = 0;
  const int samples = 1000;

  for (int i = 0; i < samples; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);

    sumX += gx;
    sumY += gy;
    sumZ += gz;

    delay(2);
  }

  // Convert AFTER averaging
  calibrationData.gyroXBias = (sumX / (float)samples) / GYRO_SCALE;
  calibrationData.gyroYBias = (sumY / (float)samples) / GYRO_SCALE;
  calibrationData.gyroZBias = (sumZ / (float)samples) / GYRO_SCALE;
}

void IMU::setStartAngles() {
  // Read raw data and convert to physical units
  readSensor();
  convert();

  // Set initial angles based on accelerometer
  roll = atan2(ay, az) * 180.0f / M_PI;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;
}