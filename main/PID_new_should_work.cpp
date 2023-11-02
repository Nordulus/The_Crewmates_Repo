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
#define LFR_MAX_MOTOR_SPEED 90
#define QTR_LINE_MID_VALUE 2500

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
// uint16_t LFR_SensorValue[SensorCount];       /**< Array to Save Raw IR Sensor values of QTR-8RC */

uint16_t LFR_Position = 0;         /**< Variable to Save the QTR-8RC Sensor Position */
int16_t  LFR_Proportional = 0;     /**< Variable to Save the Proportional Output of PID Control Algorithm */
int16_t  LFR_LastProportional = 0; /**< Variable to Save the Previous Proportional Output of PID Control Algorithm */
int16_t  LFR_Derivative = 0;       /**< Variable to Save the Derivative Output of PID Control Algorithm */
int64_t  LFR_Integral = 0;         /**< Variable to Save the Integral Output of PID Control Algorithm */
int16_t  LFR_ControlOutput = 0;    /**< Variable to Save the Final Control Output of PID Control Algorithm */

//edit these to change the behavior of LFR
const double  Kd = 14; // derivative constant
const double  Kp = 0.5; //proportional constant
const float Ki = 0.0001; //integral constant

int16_t Speed = LFR_MAX_MOTOR_SPEED;

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

void loopGamepadControl(){
  // This call fetches all the gamepad info from the NINA (ESP32) module.
  // Just call this function in your main loop.
  // The gamepads pointer (the ones received in the callbacks) gets updated
  // automatically.
  BP32.update();

  // It is safe to always do this before using the gamepad API.
  // This guarantees that the gamepad is valid and connected.
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    GamepadPtr myGamepad = myGamepads[i];

    if (myGamepad && myGamepad->isConnected()) {
      // There are different ways to query whether a button is pressed.
      // By query each button individually:
      //  a(), b(), x(), y(), l1(), etc...

      // write to the servo motors
      rightServo.write(((((float) myGamepad->axisY()) / 512.0f) * -500) + 1500 );
      leftServo.write(((((float) myGamepad->axisRY()) / 512.0f) * 500) + 1500 );
      
      if (myGamepad->a()) {
        static int colorIdx = 0;
        // Some gamepads like DS4 and DualSense support changing the color LED.
        // It is possible to change it by calling:
        switch (colorIdx % 3) {
        case 0:
          // Red
          myGamepad->setColorLED(255, 0, 0);
          break;
        case 1:
          // Green
          myGamepad->setColorLED(0, 255, 0);
          break;
        case 2:
          // Blue
          myGamepad->setColorLED(0, 0, 255);
          break;
        }
        colorIdx++;
      }

      if (myGamepad->b()) {
        // Turn on the 4 LED. Each bit represents one LED.
        static int led = 0;
        led++;
        // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
        // support changing the "Player LEDs": those 4 LEDs that usually
        // indicate the "gamepad seat". It is possible to change them by
        // calling:
        myGamepad->setPlayerLEDs(led & 0x0f);
      }

      if (myGamepad->x()) {
        // Duration: 255 is ~2 seconds
        // force: intensity
        // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S support
        // rumble.
        // It is possible to set it by calling:
        myGamepad->setRumble(0xc0 /* force */, 0xc0 /* duration */);
      }
      // Another way to query the buttons, is by calling buttons(), or
      // miscButtons() which return a bitmask.
      // Some gamepads also have DPAD, axis and more.
      Serial.printf(
          "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: "
          "%4d, %4d, brake: %4d, throttle: %4d, misc: 0x%02x, gyro x:%6d y:%6d "
          "z:%6d, accel x:%6d y:%6d z:%6d\n",
          i,                        // Gamepad Index
          myGamepad->dpad(),        // DPAD
          myGamepad->buttons(),     // bitmask of pressed buttons
          myGamepad->axisX(),       // (-511 - 512) left X Axis
          myGamepad->axisY(),       // (-511 - 512) left Y axis
          myGamepad->axisRX(),      // (-511 - 512) right X axis
          myGamepad->axisRY(),      // (-511 - 512) right Y axis
          myGamepad->brake(),       // (0 - 1023): brake button
          myGamepad->throttle(),    // (0 - 1023): throttle (AKA gas) button
          myGamepad->miscButtons(), // bitmak of pressed "misc" buttons
          myGamepad->gyroX(),       // Gyro X
          myGamepad->gyroY(),       // Gyro Y
          myGamepad->gyroZ(),       // Gyro Z
          myGamepad->accelX(),      // Accelerometer X
          myGamepad->accelY(),      // Accelerometer Y
          myGamepad->accelZ()       // Accelerometer Z
      );

      // You can query the axis and other properties as well. See Gamepad.h
      // For all the available functions.
    }
  }
  vTaskDelay(1);
}



void printQTR(){
  uint16_t position = qtr.readLineBlack(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();
  delay(10);
}
void calibrateQTR(){
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){QTRSENSOR1, QTRSENSOR2, QTRSENSOR3, QTRSENSOR4, QTRSENSOR5, QTRSENSOR6, QTRSENSOR7, QTRSENSOR8}, SensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Calibrating, place sensor over white floor.");
  for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
    }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println();


  //print threshold values
  for (uint8_t i = 0; i < SensorCount; i++){
    threshold[i] = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i])/2;
    Serial.print(threshold[i]);
    Serial.print("  ");
  }
  Serial.println();
  delay(1000);
}

void setup(){
  delay(500);

  //SERIAL MONITOR
  Serial.begin(115200);
  Serial.print("Serial monitor check");
  Serial.printf("%.2f Kd \n", Kd);
  Serial.printf("%f Ki \n", Ki);
  Serial.printf("%.2f Kp \n", Kp);

  //Connect to Gamepad
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad); //setup Bluepad32 Callbacks


    // motor setup
        // turn on all allocation timers
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  rightServo.setPeriodHertz(50);
  rightServo.attach(rightServoPin, 1000, 2000);
  rightServo.write(180); //write 180 to briefly test servos
  delay(500);
  rightServo.write(90);
  leftServo.setPeriodHertz(50);
  leftServo.attach(leftServoPin, 1000, 2000);
  leftServo.write(-180);
  delay(500);
  leftServo.write(90); //servos are left stationary

  //CALIBRATION MODE
  calibrateQTR();
  

  delay(1000);


}

void loop(){
    Serial.println('beforeprintQTR');
    printQTR();
    Serial.println('beforeloopgpcontrol');
    //loopGamepadControl();

    Serial.println("afterloopgpcontrol");
// This call fetches all the gamepad info from the NINA (ESP32) module.
  // Just call this function in your main loop.
  // The gamepads pointer (the ones received in the callbacks) gets updated
  // automatically.
  BP32.update();

  // It is safe to always do this before using the gamepad API.
  // This guarantees that the gamepad is valid and connected.
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    GamepadPtr myGamepad = myGamepads[i];

    if (myGamepad && myGamepad->isConnected()) {
      // There are different ways to query whether a button is pressed.
      // By query each button individually:
      //  a(), b(), x(), y(), l1(), etc...

      // write to the servo motors
      rightServo.write(((((float) myGamepad->axisY()) / 512.0f) * -500) + 1500 );
      leftServo.write(((((float) myGamepad->axisRY()) / 512.0f) * 500) + 1500 );
      
      if (myGamepad->a()) {
        static int colorIdx = 0;
        // Some gamepads like DS4 and DualSense support changing the color LED.
        // It is possible to change it by calling:
        switch (colorIdx % 3) {
        case 0:
          // Red
          myGamepad->setColorLED(255, 0, 0);
          break;
        case 1:
          // Green
          myGamepad->setColorLED(0, 255, 0);
          break;
        case 2:
          // Blue
          myGamepad->setColorLED(0, 0, 255);
          break;
        }
        colorIdx++;
      }

      if (myGamepad->b()) {
        // Turn on the 4 LED. Each bit represents one LED.
        static int led = 0;
        led++;
        // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
        // support changing the "Player LEDs": those 4 LEDs that usually
        // indicate the "gamepad seat". It is possible to change them by
        // calling:
        myGamepad->setPlayerLEDs(led & 0x0f);
      }

      if (myGamepad->x()) {
        // Duration: 255 is ~2 seconds
        // force: intensity
        // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S support
        // rumble.
        // It is possible to set it by calling:
        myGamepad->setRumble(0xc0 /* force */, 0xc0 /* duration */);
      }
      // Another way to query the buttons, is by calling buttons(), or
      // miscButtons() which return a bitmask.
      // Some gamepads also have DPAD, axis and more.
      Serial.printf(
          "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: "
          "%4d, %4d, brake: %4d, throttle: %4d, misc: 0x%02x, gyro x:%6d y:%6d "
          "z:%6d, accel x:%6d y:%6d z:%6d\n",
          i,                        // Gamepad Index
          myGamepad->dpad(),        // DPAD
          myGamepad->buttons(),     // bitmask of pressed buttons
          myGamepad->axisX(),       // (-511 - 512) left X Axis
          myGamepad->axisY(),       // (-511 - 512) left Y axis
          myGamepad->axisRX(),      // (-511 - 512) right X axis
          myGamepad->axisRY(),      // (-511 - 512) right Y axis
          myGamepad->brake(),       // (0 - 1023): brake button
          myGamepad->throttle(),    // (0 - 1023): throttle (AKA gas) button
          myGamepad->miscButtons(), // bitmak of pressed "misc" buttons
          myGamepad->gyroX(),       // Gyro X
          myGamepad->gyroY(),       // Gyro Y
          myGamepad->gyroZ(),       // Gyro Z
          myGamepad->accelX(),      // Accelerometer X
          myGamepad->accelY(),      // Accelerometer Y
          myGamepad->accelZ()       // Accelerometer Z
      );

      // You can query the axis and other properties as well. See Gamepad.h
      // For all the available functions.
    }
  }
  vTaskDelay(1);
    /**< Reads the QTR-8RC Line Sensor to Get the Line Position */
    LFR_Position = qtr.readLineBlack(sensorValues);

    LFR_Proportional = LFR_Position - QTR_LINE_MID_VALUE;             /**< Computes the Proportional Output of PID Control Algorithm */
    LFR_Derivative = LFR_Proportional - LFR_LastProportional;         /**< Computes the Derivative Output of PID Control Algorithm */
    LFR_Integral += LFR_Proportional;                                 /**< Computes the Integral Output of PID Control Algorithm */
    LFR_LastProportional = LFR_Proportional;                          /**< Saves the Old Proportional Output of PID Control Algorithm */
    Serial.printf("%u position \n", LFR_Position);
    Serial.printf("%d proportional \n", LFR_Proportional);
    Serial.printf("%d derivative \n", LFR_Derivative);
    Serial.printf("%d lfr integral \n", LFR_Integral);
    Serial.printf("%d lft last proportional \n", LFR_LastProportional);
    Serial.printf("%.2f kp \n", Kp);
    Serial.printf("%f ki \n", Ki);
    Serial.printf("%.2f kd \n", Kd);
    
    LFR_ControlOutput = (LFR_Proportional / Kp + LFR_Integral / Ki + LFR_Derivative * Kd); /**< Computes the Final Control Output of PID Control Algorithm 300RPM*/
    Serial.printf("%d\n", LFR_ControlOutput);

    if (LFR_ControlOutput > Speed){
        LFR_ControlOutput = Speed;    /**< Keeps The Motor Speed in Limit */
    }
    if (LFR_ControlOutput < -Speed){
        LFR_ControlOutput = -Speed;    /**< Keeps The Motor Speed in Limit */
        Serial.println("");
    }

    if (LFR_ControlOutput < 0){
        leftServo.write(-Speed - LFR_ControlOutput - 90);
        rightServo.write(Speed + 90);
    }
    else{
        leftServo.write(-Speed - 90);
        rightServo.write(Speed - LFR_ControlOutput + 90);
    }
}