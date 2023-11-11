#include <Bluepad32.h>
#include <Arduino.h>
#include <QTRSensors.h>
#include <ESP32Servo.h>
#include <sdkconfig.h>
#include <ESP32SharpIR.h>
#include <Wire.h>
#include <Arduino_APDS9960.h>
#include <bits/stdc++.h>

#define rightServoPin 12
#define leftServoPin 13
#define launcherPin 17
#define APDS9960_INT 2
#define I2C_SDA 21
#define I2C_SCL 22                                                                                                                                      
#define I2C_FREQ 100000
TwoWire I2C_0 = TwoWire(0);
APDS9960 apds = APDS9960(I2C_0, APDS9960_INT);
Servo rightServo;
Servo leftServo;
Servo launcher;
ESP32SharpIR left(ESP32SharpIR::GP2Y0A21YK0F, 4);
ESP32SharpIR center(ESP32SharpIR::GP2Y0A21YK0F, 15);
ESP32SharpIR right(ESP32SharpIR::GP2Y0A21YK0F, 14);
const int IRbasespeedright = 1500;
const int IRbasespeedleft = 1440;

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

    left.setFilterRate(0.1f);//IR follower setup
    center.setFilterRate(0.1f);
    right.setFilterRate(0.1f);
    I2C_0.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    //apds.setInterruptPin(APDS9960_INT);
    apds.begin();
    Serial.begin(115200);
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
        if ((myGamepad->dpad()) == 4){
          dpadUp = 0;
          buttonB = 0;
          buttonA = 0;
          buttonX = 0;
          buttonY = 0;
          dpadDown = 0;
          dpadRight = 1;
          dpadLeft = 0;
        }
        if ((myGamepad->dpad()) == 8){
          dpadUp = 0;
          buttonB = 0;
          buttonA = 0;
          buttonX = 0;
          buttonY = 0;
          dpadDown = 0;
          dpadRight = 0;
          dpadLeft = 1;
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
        if (dpadRight == 1){
          wallfollower();
        }
        if (dpadLeft == 1){
          getInitColor();
        }
        //dpad down = 2
        //dpad up = 1
        //dpad right = 4 wall follow
        //dpad left = 8 color
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

void colorLoop(int val) {
    int r, g, b, a;
    int checker = val;
    rightServo.write(IRbasespeedright - 300);//go straight
    leftServo.write(IRbasespeedleft + 300);
    delay(1000);
    while(checker != 0)
    {

        while (!apds.colorAvailable())
        {
            delay(5);
        }
        apds.readColor(r, g, b, a);
        if (r >= 20 && g >= 20 && b >= 20)
        {
            //white
        }
        else if (r > g && r > b)
        {
        if (checker == 1)
        {
            rightServo.write(IRbasespeedright);//stop
            leftServo.write(IRbasespeedleft);
            checker=0;
        }
    }
    else if (g > r && g > b)
    {
        if (checker == 2)
        {
            rightServo.write(IRbasespeedright);//stop
            leftServo.write(IRbasespeedleft);
            checker=0;
        }
    }
    else if ((b <= 10 && b == g) || (b > 10 && (b - g) < 10))
    {
        if (checker == 3)
        {
            rightServo.write(IRbasespeedright);//stop
            leftServo.write(IRbasespeedleft);
            checker=0;
        }
    }
    else
    {
        //black
    }
    
    //red = r>30
    //blue = b & g >=15
    //g>20
    //white everything>30
    delay(100);
    }
//red 1 flash
//green 2 flash
//blue 3 flash
}

void getInitColor(){
    int r, g, b, a;
    while (!apds.colorAvailable())
    {
            delay(5);
    }

    apds.readColor(r, g, b, a);
    if (r >= 20 && g >= 20 && b >= 20)
    {
        
    }
    else if (r > g && r > b)
    {
        blinkLED();
        Serial.println("save red");
        colorLoop(1);
        
    }
    else if (g > r && g > b)
    {
        blinkLED();
        blinkLED();
        Serial.println("save green");
        colorLoop(2);
    }
    else if ((b <= 10 && b == g) || (b > 10 && (b - g) < 10))
    {
        blinkLED();
        blinkLED();
        blinkLED();
        Serial.println("save blue");
        colorLoop(3);
    }
    
    
    //red = r>30
    //blue = b & g >=15
    //g>20
    //white everything>30
   /* 
    Serial.print("R: ");
    Serial.println(r);
    Serial.print("G: ");
    Serial.println(g);
    Serial.print("B: ");
    Serial.println(b);
    Serial.print("Ambient");
    Serial.println(a);
    */
}

  void blinkLED(){
    // blinker code
    digitalWrite(LED, LOW);
    Serial.println("LED is on");
    delay(1000);
    digitalWrite(LED, HIGH);
    Serial.println("LED is off");
    delay(1000);
}

  void wallfollower(){
    
    //int diff
    //Serial.print("right");
    //Serial.println(right.getDistanceFloat());
    //Serial.print("left");
    //Serial.println(left.getDistanceFloat());
    //rightServo.write(IRbasespeedright);//right moter 1000-1500 is forward
    //leftServo.write(IRbasespeedleft);//left with lag 1440-1940 is forward
    //delay(3000);

    //rightServo.write(IRbasespeedright + 250); //90 degrees right
    //leftServo.write(IRbasespeedleft + 250); 
    //delay(800);

    //rightServo.write(IRbasespeedright - 250); //90 degrees left
    //leftServo.write(IRbasespeedleft - 250); 
    //delay(800);
    
    //Wall sensor code
    if(center.getDistanceFloat() <= 10) //stops at value 10
    {
        rightServo.write(IRbasespeedright);//stop
        leftServo.write(IRbasespeedleft);
        delay(100);
        if(left.getDistanceFloat() < right.getDistanceFloat())
        {
            rightServo.write(IRbasespeedright + 300);//turn right
            leftServo.write(IRbasespeedleft + 300);
            delay(800);
        }else
        if(left.getDistanceFloat() > right.getDistanceFloat())
        {
            rightServo.write(IRbasespeedright - 300);//turn left
            leftServo.write(IRbasespeedleft - 300);
            delay(800);
        }
    rightServo.write(IRbasespeedright - 300);//go straight
    leftServo.write(IRbasespeedleft + 300);
    }
}

}