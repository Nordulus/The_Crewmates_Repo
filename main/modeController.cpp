#include <Bluepad32.h>
#include <Arduino.h>
#include <QTRSensors.h>
#include <ESP32Servo.h>
#include <sdkconfig.h>

#define rightServoPin 12
#define leftServoPin 13
#define launcherPin 17
Servo rightServo;
Servo leftServo;
Servo launcher;

int P;
int I;
int D;
int baseSpeedA = 1750; //left
int baseSpeedB = 1190; //right
float Kp = 0.047;
float Ki = 0.000015;
float Kd = 0.8;

int lastError = 0;

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

int buttonB = 0;
int buttonA = 0;
int buttonX = 0;
int buttonY = 0;
int dpadUp = 0;
int dpadDown = 0;
int dpadRight = 0;
int dpadLeft = 0;

void setup() {
    pinMode(2, OUTPUT);
    Serial.begin(115200);
    Serial.println("");
    Serial.print("Serial monitor check");
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){36, 39, 34, 35, 32, 33, 25, 26}, SensorCount);
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    rightServo.setPeriodHertz(50);
    rightServo.attach(rightServoPin, 1000, 2000);
    leftServo.setPeriodHertz(50);
    leftServo.attach(leftServoPin, 1000, 2000);
    launcher.setPeriodHertz(50);
    launcher.attach(launcherPin, 1000, 2000);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t *addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();
}

void loop() {
  BP32.update();
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    GamepadPtr myGamepad = myGamepads[i];

      if (myGamepad && myGamepad->isConnected()) {
        if (myGamepad->a()) {
          dpadUp = 0;
          buttonB = 0;
          buttonA = 1;
          buttonX = 0;
          buttonY = 0;
          dpadDown = 0;
          dpadRight = 0;
          dpadLeft = 0;
        }
        if (myGamepad->b()) {
          dpadUp = 0;
          buttonB = 1;
          buttonA = 0;
          buttonX = 0;
          buttonY = 0;
          dpadDown = 0;
          dpadRight = 0;
          dpadLeft = 0;
        }
        if (myGamepad->y()) {
          dpadUp = 0;
          buttonB = 0;
          buttonA = 0;
          buttonX = 1;
          buttonY = 0;
          dpadDown = 0;
          dpadRight = 0;
          dpadLeft = 0;
        }
        if (myGamepad->x()) {
          dpadUp = 0;
          buttonB = 0;
          buttonA = 0;
          buttonX = 0;
          buttonY = 1;
          dpadDown = 0;
          dpadRight = 0;
          dpadLeft = 0;
        }
        if ((myGamepad->dpad()) == 1){
          dpadUp = 1;
          buttonB = 0;
          buttonA = 0;
          buttonX = 0;
          buttonY = 0;
          dpadDown = 0;
          dpadRight = 0;
          dpadLeft = 0;
        }
        if ((myGamepad->dpad()) == 2){
          dpadUp = 0;
          buttonB = 0;
          buttonA = 0;
          buttonX = 0;
          buttonY = 0;
          dpadDown = 1;
          dpadRight = 0;
          dpadLeft = 0;
        }
        if (buttonA == 0 && buttonB == 0 && buttonX == 0) {
            leftServo.write(1440);
        }
        if (dpadUp == 1){
          launcher.write(1750);
          delay(1600);
          launcher.write(1500);
          dpadUp = 0;
          buttonB = 0;
          buttonA = 0;
          buttonX = 0;
          buttonY = 0;
          dpadDown = 0;
          dpadRight = 0;
          dpadLeft = 0;
        }
        if (dpadDown == 1){
          launcher.write(1250);
          delay(1600);
          launcher.write(1500);
          dpadUp = 0;
          buttonB = 0;
          buttonA = 0;
          buttonX = 0;
          buttonY = 0;
          dpadDown = 0;
          dpadRight = 0;
          dpadLeft = 0;
        }

        if (buttonY == 1){
          printQTR();
          Serial.println();
        }

        if (buttonA == 1){
          //btMotorControl();
          Serial.print("btMotorControl");
          Serial.println();
          rightServo.write( ((((float) myGamepad->axisY()) / 512.0f) * -500) + 1500 );
          leftServo.write( ((((float) myGamepad->axisRY()) / 512.0f) * 500) + 1440 );
        }
        if (buttonB == 1){
          calibrateQTR();
          Serial.print("printQTR for now");
          Serial.println();
          buttonB = 0;
        }
        if (buttonX == 1){
          PID_control();
          Serial.print("Linefollow");
          Serial.println();
        }
        //dpad down = 2
        //dpad up = 1
        //dpad right = 4
        //dpad left = 8
    }
  }
  vTaskDelay(1);
}

void PID_control() {
  uint16_t positionLine = qtr.readLineBlack(sensorValues);
  int error = 3500 - positionLine;

  P = error;
  I = error + I;
  D = error - lastError;
  lastError = error; 

  int motorSpeedChange = P*Kp + I*Ki + D*Kd;

  /*apparently "keep in mind the way the servos work is that they are 
  centered around 1500; anything greater rotates one direction and 
  anything less rotates in the other. typically servos will a range 
  from around 500-2500, so anything past that likely will not have 
  any visible changes" 
  change baseSpeed to reflect this in a test at some point*/
  int motorSpeedA = baseSpeedA + motorSpeedChange;
  int motorSpeedB = baseSpeedB + motorSpeedChange;

  if (motorSpeedA > 2000) {
    motorSpeedA = 2000;
  }
  if (motorSpeedB > 2000) {
    motorSpeedB = 2000;
  }
  if (motorSpeedA < 1000) {
    motorSpeedA = 1000;
  }
  if (motorSpeedB < 1000) {
    motorSpeedB = 1000;
  }
  if (motorSpeedB & motorSpeedA == 1500){
    motorSpeedA = 2000;
    motorSpeedB = 2000;
  }
  leftServo.write(motorSpeedA);
  Serial.printf("Left: %u ", motorSpeedA);
  Serial.print("");
  rightServo.write(motorSpeedB);
  Serial.printf("Right: %u ", motorSpeedB);
  Serial.print("");
  Serial.printf("Error: %u", error);
  Serial.println();
}
void btMotorControl(){

}

void printQTR(){
  uint16_t position = qtr.readLineBlack(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.print(position);
  delay(10);
}
void launcherControl(){

}
void calibrateQTR(){
    digitalWrite(2, HIGH);
    Serial.print("Calibrating");
    //10 seconds
    for (uint16_t i = 0; i < 400; i++){
        qtr.calibrate();
    }
    digitalWrite(2, LOW);
}
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