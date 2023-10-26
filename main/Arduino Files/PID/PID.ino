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

const int offsetA = 1;
const int offsetB = 1;

//initializing objects
GamepadPtr myGamepads[BP32_MAX_GAMEPADS];
QTRSensors qtr;
Servo rightServo;
Servo leftServo;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];

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

// PID CONTROLLER VARIABLES
float Kp = 0;
float Ki = 0;
float Kd = 0;

uint8_t multiP = 1;
uint8_t multiI = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
float Pvalue;
float Ivalue;
float Dvalue;

int val, cnt = 0, v[3];

uint16_t position;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 230;

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
  leftServo.write(180);
  delay(500);
  leftServo.write(90); //servos are left stationary

  //CALIBRATION MODE
  calibrateQTR();
  

  delay(1000);


}

void loop(){
  printQTR();
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
      rightServo.write( ((((float) myGamepad->axisY()) / 512.0f) * -500) + 1500 );
      leftServo.write( ((((float) myGamepad->axisRY()) / 512.0f) * 500) + 1500 );
      
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
      /* Serial.printf(
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
      ); */

      // You can query the axis and other properties as well. See Gamepad.h
      // For all the available functions.
    }
  }

  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  // vTaskDelay(1);
  delay(150);

  // robot_control();
}
void robot_control(){
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 4000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);
  error = 2000 - position;
  while(sensorValues[0]>=980 && sensorValues[1]>=980 && sensorValues[2]>=980 && sensorValues[3]>=980 && sensorValues[4]>=980 && sensorValues[5]>=980 && sensorValues [6]>=980){ // A case when the line follower leaves the line
    if(previousError>0){       //Turn left if the line was to the left before
      motor_drive(-180,180);
    }
    else{
      motor_drive(180,-180); // Else turn right
    }
    position = qtr.readLineBlack(sensorValues);
  }
  
  PID_Linefollow(error);
  //PID_Linefollow(error);
}
void PID_Linefollow(int error){
    P = error;
    I = I + error;
    D = error - previousError;
    
    Pvalue = (Kp/pow(10,multiP))*P;
    Ivalue = (Ki/pow(10,multiI))*I;
    Dvalue = (Kd/pow(10,multiD))*D; 

    float PIDvalue = Pvalue + Ivalue + Dvalue;
    previousError = error;

    lsp = lfspeed - PIDvalue;
    Serial.println(lsp);
    rsp = lfspeed + PIDvalue;
    Serial.println(rsp);

    if (lsp > 255) {
      lsp = 255;
    }
    if (lsp < -255) {
      lsp = -255;
    }
    if (rsp > 255) {
      rsp = 255;
    }
    if (rsp < -255) {
      rsp = -255;
    }
    motor_drive(lsp,rsp);
}

//In this void the the 2 read values are assigned.
void  processing() {
  int a = v[1];
  if (a == 1) {
    Kp = v[2];
  }
  if (a == 2) {
    multiP = v[2];
  }
  if (a == 3) {
    Ki = v[2];
  }
  if (a == 4) {
    multiI = v[2];
  }
  if (a == 5) {
    Kd  = v[2];
  }
  if (a == 6) {
    multiD = v[2];
  }
}
void motor_drive(int left, int right){
  
  if(right>0){
    rightServo.write(right);
  }
  else 
  {
    rightServo.write(right);
  }
  
 
  if(left>0){
    leftServo.write(left);
  }
  else {
    leftServo.write(left);
  }
}