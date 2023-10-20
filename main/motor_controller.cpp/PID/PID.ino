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

int pos1 = 0;
int pos2 = 0;
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



// PID CONTROLLER VARIABLES
float Kp = 0;
float Ki = 0;
float Kd = 0;

uint8_t multiP = 1;
uint8_t multiI  = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
float Pvalue;
float Ivalue;
float Dvalue;

boolean onoff = false;

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
  Serial.println("Sensor calibrating. Hold sensor on nonconstrasting floor.");
  pinMode(LED_BUILTIN, OUTPUT);
  // turn on Arduino's LED to indicate we are in calibration mode
  digitalWrite(LED_BUILTIN, HIGH);
  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 8 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.print("printing minimum values:");
  for (uint8_t i = 0; i < SensorCount; i++){
      Serial.print(qtr.calibrationOn.minimum[i]);
      Serial.print(' ');
  }
  Serial.println();

  // print max calibration values
  Serial.print("printing max values");
  for (uint8_t i = 0; i < SensorCount; i++){
      Serial.print(qtr.calibrationOn.maximum[i]);
      Serial.print(' ');
    }
  Serial.println();
  Serial.println();
  delay(1000);

  //print threshold values
  for (uint8_t i = 0; i < SensorCount; i++){
    threshold[i] = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i])/2;
    Serial.print(threshold[i]);
    Serial.print("  ");
  }
  Serial.println();
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

  if (onoff == true){
    robot_control();
  }

  else if(onoff == false){
    motor1.stop();
    motor2.stop();
  }
}
void robot_control(){
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 4000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);
  error = 2000 - position;
  while(sensorValues[0]>=980 && sensorValues[1]>=980 && sensorValues[2]>=980 && sensorValues[3]>=980 && sensorValues[4]>=980){ // A case when the line follower leaves the line
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
  if (a == 7)  {
    onoff = v[2];
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