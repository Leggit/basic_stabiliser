#include "IMU.h"
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
  // --- Accelerometer angles ---
  float accelRoll = atan2(ay, az) * 180.0f / M_PI;
  float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;

  // --- Gyro integration ---
  roll += gx * dt;
  pitch += gy * dt;

  // --- Complementary filter ---
  const float alpha = 0.98f;

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