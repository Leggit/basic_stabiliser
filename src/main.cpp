#include "airframe/airframe.h"
#include "config.h"
#include "control_input/control_input.h"
#include "imu/imu.h"
#include "pid/pid.h"
#include "status_indicator/status_indicator.h"
#include "web_control_panel/web_control_panel.h"
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>

IMU imu;

PID pitchPID(KP_PITCH, KI_PITCH, KD_PITCH);
PID rollPID(KP_ROLL, KI_ROLL, KD_ROLL);

WebControlPanel webControlPanel;
SingleLED singleLED(LED_PIN);
StatusIndicator &statusIndicator = singleLED;
ControlInput controlInput(Serial2, RC_IN);

StandardAirframe standardAirframe;
ElevonAirframe elevonAirframe(ELEVON_1_PITCH_SIGN, ELEVON_1_ROLL_SIGN,
                              ELEVON_2_PITCH_SIGN, ELEVON_2_ROLL_SIGN);
Airframe &airframe = ENABLE_ELEVONS ? static_cast<Airframe &>(elevonAirframe)
                                    : static_cast<Airframe &>(standardAirframe);

int loopCounter = 0;

void setup() {
  Serial.begin(115200);
  statusIndicator.begin();
  WiFi.mode(WIFI_OFF);
  btStop();
  Wire.begin();

  webControlPanel.init(&pitchPID, &rollPID);
  webControlPanel.loadPIDGains(KP_PITCH, KI_PITCH, KD_PITCH, KP_ROLL, KI_ROLL,
                               KD_ROLL);

  statusIndicator.showSetupInProgress();
  controlInput.begin();
  if (!imu.begin(CALIBRATE_GYRO_ON_STARTUP)) {
    Serial.println("Failed to initialize IMU!");
    statusIndicator.showError();
  }
  airframe.begin();
  statusIndicator.showSetupComplete();
}
void loop() {
  const unsigned long now = micros();
  const float dt = 0.004f;

  imu.update(dt);

  float desiredPitch = controlInput.desiredPitch();
  float desiredRoll = controlInput.desiredRoll();

  float correctedPitchOutput =
      pitchPID.update(desiredPitch, imu.getPitch(), dt);
  float correctedRollOutput = rollPID.update(desiredRoll, imu.getRoll(), dt);

  AirframeInputs airframeInputs{controlInput.elevator(), controlInput.aileron(),
                                controlInput.flaps()};

  webControlPanel.update(controlInput.configModeEnabled());

  if (loopCounter % 5 == 0) {
    loopCounter = 0;
    StabilizationOutput stabilizationOutput{correctedPitchOutput,
                                            correctedRollOutput};
    if (controlInput.stabilizationEnabled()) {
      airframe.writeStabilized(stabilizationOutput);
    } else {
      airframe.writeManual(airframeInputs);
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
