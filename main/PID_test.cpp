#include <Bluepad32.h>
#include <Arduino.h>
#include <QTRSensors.h>
#include <ESP32Servo.h>
#include <sdkconfig.h>

#define rightServoPin 12
#define leftServoPin 13
#define NUM_SAMPLES_PER_SENSOR 4 // average 4 analog samples per sensor reading
#define QTR_NO_EMITTER_PIN // emitter is controlled by digital pin 2
#define QTRSENSOR1 36
#define QTRSENSOR2 39
#define QTRSENSOR3 34
#define QTRSENSOR4 35
#define QTRSENSOR5 32
#define QTRSENSOR6 33
#define QTRSENSOR7 25
#define QTRSENSOR8 26
#define LED_BUILTIN 2

//initializing objects
GamepadPtr myGamepads[BP32_MAX_GAMEPADS];
QTRSensors qtr;
Servo rightServo;
Servo leftServo;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];

//PID CONTROLLER VARIABLES
    //lfr means line follower
uint16_t LFR_SensorValue[SensorCount];       /**< Array to Save Raw IR Sensor values of QTR-8RC */
uint16_t LFR_Position = 0;         /**< Variable to Save the QTR-8RC Sensor Position */
int16_t  LFR_Proportional = 0;     /**< Variable to Save the Proportional Output of PID Control Algorithm */
int16_t  LFR_LastProportional = 0; /**< Variable to Save the Previous Proportional Output of PID Control Algorithm */
int16_t  LFR_Derivative = 0;       /**< Variable to Save the Derivative Output of PID Control Algorithm */
int64_t  LFR_Integral = 0;         /**< Variable to Save the Integral Output of PID Control Algorithm */
int16_t  LFR_ControlOutput = 0;    /**< Variable to Save the Final Control Output of PID Control Algorithm */

//edit these to change the behavior of LFR
uint8_t  Kd = 14; // derivative constant
uint8_t  Kp = 1/2; //proportional constant
uint16_t Ki = 1/10000; //integral constant

uint16_t Speed = LFR_MAX_MOTOR_SPEED;

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == nullptr) {
      Serial.printf("CALLBACK: Gamepad is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      GamepadProperties properties = gp->getProperties();
      Serial.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n",
                    gp->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myGamepads[i] = gp;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println(
        "CALLBACK: Gamepad connected, but could not found empty slot");
  }
}

void onDisconnectedGamepad(GamepadPtr gp) {
  bool foundGamepad = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == gp) {
      Serial.printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
      myGamepads[i] = nullptr;
      foundGamepad = true;
      break;
    }
  }

  if (!foundGamepad) {
    Serial.println(
        "CALLBACK: Gamepad disconnected, but not found in myGamepads");
  }
}



void setup(){
  delay(500);

  //SERIAL MONITOR
  Serial.begin(115200);
  Serial.print("Serial monitor check");

  //CALIBRATION MODE
  calibrateQTR();

  delay(1000);
}




void loop(){
  LFR_Position = QTR_ReadLine(LFR_SensorValue, QTR_EMITTERS_ON);    /**< Reads the QTR-8RC Line Sensor to Get the Line Position */

  LFR_Proportional = LFR_Position - QTR_LINE_MID_VALUE;             /**< Computes the Proportional Output of PID Control Algorithm */
  LFR_Derivative = LFR_Proportional - LFR_LastProportional;         /**< Computes the Derivative Output of PID Control Algorithm */
  LFR_Integral += LFR_Proportional;                                 /**< Computes the Integral Output of PID Control Algorithm */
  LFR_LastProportional = LFR_Proportional;                          /**< Saves the Old Proportional Output of PID Control Algorithm */

  LFR_ControlOutput = LFR_Proportional / Kp + LFR_Integral / Ki + LFR_Derivative * Kd; /**< Computes the Final Control Output of PID Control Algorithm 300RPM*/

  if (LFR_ControlOutput > Speed)
  {
    LFR_ControlOutput = Speed;    /**< Keeps The Motor Speed in Limit */
  }
  if (LFR_ControlOutput < -Speed)
  {
    LFR_ControlOutput = -Speed;    /**< Keeps The Motor Speed in Limit */
  }

  if (LFR_ControlOutput < 0)
  {
    Motor_SetSpeed(Speed + LFR_ControlOutput, Speed);    /**< Drives the Motor According to the Control Output */
  }
  else
  {
    Motor_SetSpeed(Speed, Speed - LFR_ControlOutput);    /**< Drives the Motor According to the Control Output */
  }
}