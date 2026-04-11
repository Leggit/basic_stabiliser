#pragma once

#include <MPU6050.h>

struct CalibrationData {
  float gyroXBias = 0;
  float gyroYBias = 0;
  float gyroZBias = 0;
};

class IMU {
public:
  IMU();

  bool begin(bool calibrate = true);
  void update(float dt); // dt in seconds

  float getPitch();
  float getRoll();

  CalibrationData getCalibrationData() const { return calibrationData; }

private:
  static constexpr float GYRO_SCALE = 65.5f;
  static constexpr float ACCEL_SCALE = 4096.0f;

  MPU6050 mpu;

  // Raw data
  int16_t rawAx, rawAy, rawAz;
  int16_t rawGx, rawGy, rawGz;

  // Converted
  float ax, ay, az;
  float gx, gy, gz;

  // Orientation
  float pitch;
  float roll;

  CalibrationData calibrationData;

  void readSensor();
  void convert();
  void computeAngles(float dt);
  void calibrateGyro();
  void setStartAngles();
};