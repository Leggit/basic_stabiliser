#pragma once

#include <stdint.h>

constexpr uint8_t RC_IN = 32;

constexpr uint8_t ELEVATOR_SERVO_PIN = 33;
constexpr uint8_t RUDDER_SERVO_PIN = 25;
constexpr uint8_t AILERON_1_SERVO_PIN = 26;
constexpr uint8_t AILERON_2_SERVO_PIN = 27;

constexpr uint8_t ELEVATOR_CHANNEL = 1;
constexpr uint8_t RUDDER_CHANNEL = 3;
constexpr uint8_t AILERON_CHANNEL = 0;
constexpr uint8_t SWITCH_STABILIZATION = 5;
constexpr uint8_t SWITCH_CONFIG_MODE = 6;

constexpr bool CALIBRATE_GYRO_ON_STARTUP = false;

constexpr uint8_t LED_PIN = 15;

// constexpr int8_t PITCH_SIGN_CORRECTION = 1;
// constexpr int8_t ROLL_SIGN_CORRECTION = -1;

constexpr float KP_ROLL = 1;
constexpr float KI_ROLL = 0;
constexpr float KD_ROLL = 0;
constexpr float KP_PITCH = 1;
constexpr float KI_PITCH = 0;
constexpr float KD_PITCH = 0;