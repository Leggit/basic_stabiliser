#include "config.h"
#include "imu/imu.h"
#include "pid/pid.h"
#include "pid_tuner.h"
#include <Arduino.h>
#include <ESP32Servo.h>
#include <FlyskyIBUS.h>
#include <WiFi.h>
#include <Wire.h>

FlyskyIBUS ibus(Serial2, RC_IN);
IMU imu;

PID pitchPID(KP_PITCH, KI_PITCH, KD_PITCH);
PID rollPID(KP_ROLL, KI_ROLL, KD_ROLL);

WebControlPanel webControlPanel;

Servo elevatorServo;
Servo aileron1Servo;
Servo aileron2Servo;

long timer;
long loopTime;
int loopCounter = 0;

void showError();
void showSetupInProgress();
void showSetupComplete();
void setupImu();
void setupRc();
void setupServos();
float calculateDesiredAngle(uint16_t msValue);
long convertPIDOutputToMicroseconds(float pidOutput);

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  WiFi.mode(WIFI_OFF);
  btStop();
  Wire.begin();

  webControlPanel.init(&pitchPID, &rollPID);
  webControlPanel.loadPIDGains(KP_PITCH, KI_PITCH, KD_PITCH, KP_ROLL, KI_ROLL, KD_ROLL);

  showSetupInProgress();
  setupRc();
  setupImu();
  setupServos();
  showSetupComplete();
}

void loop() {
  unsigned long now = micros();
  float dt = (now - timer) / 1000000.0f; // seconds
  timer = now;

  imu.update(dt);

  float desiredPitch = calculateDesiredAngle(ibus.getChannel(ELEVATOR_CHANNEL));
  float desiredRoll = calculateDesiredAngle(ibus.getChannel(AILERON_CHANNEL));

  float correctedPitchOutput = pitchPID.update(desiredPitch, imu.getPitch());
  float correctedRollOutput = rollPID.update(desiredRoll * -1, imu.getRoll());

  uint16_t stabilizationSwitch = ibus.getChannel(SWITCH_STABILIZATION);
  uint16_t configModeSwitch = ibus.getChannel(SWITCH_CONFIG_MODE);
  
  webControlPanel.update(configModeSwitch);

  if (loopCounter % 5 == 0) {
    loopCounter = 0;
    bool stabilizationEnabled = stabilizationSwitch > 1500;
    
    if (stabilizationEnabled) {
      elevatorServo.writeMicroseconds(
          convertPIDOutputToMicroseconds(correctedPitchOutput));
      aileron1Servo.writeMicroseconds(
          convertPIDOutputToMicroseconds(correctedRollOutput * -1));
      aileron2Servo.writeMicroseconds(
          convertPIDOutputToMicroseconds(correctedRollOutput * -1));

    } else {
      // manual
      elevatorServo.writeMicroseconds(
          constrain(ibus.getChannel(ELEVATOR_CHANNEL), 1000, 2000));
      aileron1Servo.writeMicroseconds(
          constrain(ibus.getChannel(AILERON_CHANNEL), 1000, 2000));
      aileron2Servo.writeMicroseconds(
          constrain(ibus.getChannel(AILERON_CHANNEL), 1000, 2000));
    }
  }

  loopCounter = loopCounter + 1;

  if (micros() - now < 4000) {
    while (micros() - now < 4000)
      ;
  } else {
    Serial.println("Loop is taking too long!");
  }
}

float calculateDesiredAngle(uint16_t msValue) {
  if (msValue > 1450 && msValue < 1550)
    return 0;
  else
    return (msValue - 1500) / 9.0f * -1;
}

long convertPIDOutputToMicroseconds(float pidOutput) {
  return constrain(1500 + pidOutput * -8, 1000, 2000);
}

void setupRc() {
  ibus.begin();
  Serial.println("Receiver initialized");
}

void setupImu() {
  if (!imu.begin(CALIBRATE_GYRO_ON_STARTUP)) {
    Serial.println("Failed to initialize IMU!");
    showError();
    while (1)
      ;
  } else {
    Serial.println("IMU initialized successfully");
    CalibrationData calibData = imu.getCalibrationData();
    Serial.print("Gyro X Bias: ");
    Serial.println(calibData.gyroXBias);
    Serial.print("Gyro Y Bias: ");
    Serial.println(calibData.gyroYBias);
    Serial.print("Gyro Z Bias: ");
    Serial.println(calibData.gyroZBias);
    Serial.print("Initial Pitch: ");
    Serial.println(imu.getPitch());
    Serial.print("Initial Roll: ");
    Serial.println(imu.getRoll());
  }
}
void showSetupInProgress() {
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}

void showSetupComplete() { digitalWrite(LED_PIN, HIGH); }

void showError() {
  while (true) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

void setupServos() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  elevatorServo.setPeriodHertz(50);
  aileron1Servo.setPeriodHertz(50);
  aileron2Servo.setPeriodHertz(50);

  elevatorServo.attach(ELEVATOR_SERVO_PIN, 1000, 2000);
  aileron1Servo.attach(AILERON_1_SERVO_PIN, 1000, 2000);
  aileron2Servo.attach(AILERON_2_SERVO_PIN, 1000, 2000);
}