/****************************************************************************
Copyright 2021 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
****************************************************************************/

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <ESP32SharpIR.h>
#include <QTRSensors.h>
#include <ESP32SharpIR.h>

#define LED 2
#define servoPin1 12
#define servoPin2 13
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
int pos1 = 0;
int pos2 = 0;

//
// README FIRST, README FIRST, README FIRST
//
// Bluepad32 has a built-in interactive console.
// By default it is enabled (hey, this is a great feature!).
// But it is incompatible with Arduino "Serial" class.
//
// Instead of using "Serial" you can use Bluepad32 "Console" class instead.
// It is somewhat similar to Serial but not exactly the same.
//
// Should you want to still use "Serial", you have to disable the Bluepad32's console
// from "sdkconfig.defaults" with:
//    CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE=n

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

Servo rightServo;
Servo leftServo;
ESP32SharpIR left(ESP32SharpIR::GP2Y0A21YK0F, 2);
ESP32SharpIR center(ESP32SharpIR::GP2Y0A21YK0F, 15);
ESP32SharpIR right(ESP32SharpIR::GP2Y0A21YK0F, 0);
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == nullptr) {
            // Console.printf("CALLBACK: Gamepad is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            // GamepadProperties properties = gp->getProperties();
            // Console.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n", gp->getModelName(), properties.vendor_id,
            //                properties.product_id);
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        // Console.println("CALLBACK: Gamepad connected, but could not found empty slot");
    }
}

void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == gp) {
            // Console.printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }

    if (!foundGamepad) {
        // Console.println("CALLBACK: Gamepad disconnected, but not found in myGamepads");
    }
}

void calibrateQTR(){

        // Calibrate sensor
    Serial.print("Calibration mode begin");
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH); //LED blue to indicate calibration mode
    for (uint16_t i = 0; i < 400; i++){
        qtr.calibrate();
    }
    Serial.println();
    // print the calibration minimum values measured when emitters were on
    Serial.print("printing minimum values:");
    for (uint8_t i = 0; i < SensorCount; i++)
        {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
        }
    Serial.println();
    digitalWrite(LED, LOW);

    // print max calibration values
    Serial.print("printing max values");
    for (uint8_t i = 0; i < SensorCount; i++)
        {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
        }
    Serial.println();
    Serial.println();
    delay(1000);

}

int readandprintQTR(){
    uint16_t position = qtr.readLineBlack(sensorValues);
    int sensor_array_values[8];
    for (uint8_t i = 0; i < SensorCount; i++){
        Serial.print(sensorValues[i]);
        Serial.print('\t');
        sensor_array_values[i] = sensorValues[i];
    }
    Serial.println();
    delay(250);
    return 0;
}

void blinkLED(){
    // blinker code
    digitalWrite(LED, HIGH);
    Serial.println("LED is on");
    delay(50);
    digitalWrite(LED, LOW);
    Serial.println("LED is off");
    delay(50);
}

void wallfollower(){
    rightServo.write(1500);//right moter 1000-1500 is forward
    leftServo.write(1440);//left with lag 1440-1940 is forward
    delay(3000);

    //rightServo.write(1750); //90 degrees right
    //leftServo.write(1690); 
    //delay(800);

    rightServo.write(1250); //90 degrees left
    leftServo.write(1190); 
    delay(800);
    
    //Wall sensor code
    /*if(center.getDistanceFloat() <= 10.50) //stops at value 10.50
    {
        Serial.println("stop");
        rightServo.write(1500);
        leftServo.write(1440);
        delay(500);

        if(left.getDistanceFloat() <= right.getDistanceFloat())
        {
            Serial.println("turn right");
            rightServo.write(1750);
            leftServo.write(1690);
            delay(1000);
            
        }else if(right.getDistanceFloat() <= left.getDistanceFloat())
        {
            Serial.println("turn left");
            rightServo.write(1250);
            leftServo.write(1190);
            delay(1000);
        }
        //inch forward
        rightServo.write(1750);
        leftServo.write(1190);
        delay(1000);
    }else{
    Serial.println("Go straight");
    rightServo.write(1750);
    leftServo.write(1190);
    }
 */
}

// Arduino setup function. Runs in CPU 1
void setup() {
    
    //delay(500);
   Serial.begin(115200);
    Serial.print("Serial monitor check");
    // Console.printf("Firmware: %s\n", BP32.firmwareVersion());

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But might also fix some connection / re-connection issues.
    //BP32.forgetBluetoothKeys();

    // motor setup
        // turn on all allocation timers     
    ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
    rightServo.setPeriodHertz(50);
    rightServo.attach(servoPin1, 1000, 2000);
    rightServo.write(90);
    leftServo.setPeriodHertz(50);
    leftServo.attach(servoPin2, 1000, 2000);
    leftServo.write(90); 
    //servos are stationary
    

    // QTR SENSOR
    //qtr.setTypeAnalog();
    //qtr.setSensorPins((const uint8_t[]){QTRSENSOR1, QTRSENSOR2, QTRSENSOR3, QTRSENSOR4, QTRSENSOR5, QTRSENSOR6, QTRSENSOR7, QTRSENSOR8}, SensorCount);
    //calibrateQTR();

    // qtr.setTypeRC(); // or setTypeAnalog()
    // qtr.setSensorPins((const uint8_t[]) {12,13,14}, 3);
    // for (uint8_t i = 0; i < 250; i++)
    // {
    //     Serial.println("calibrating");
    //     qtr.calibrate();
    //     delay(20);
    // }
    // qtr.calibrate();

    //IR sensor setup
    //left.setFilterRate(0.1f);
    center.setFilterRate(0.1f);
    //right.setFilterRate(0.1f);


}

// ESP32 loop function. Runs in CPU 1
void loop() {
    //Serial.println("not read");
    // This call fetches all the gamepad info from the NINA (ESP32) module.
    // Just call this function in your main loop.
    // The gamepads pointer (the ones received in the callbacks) gets updated
    // automatically.
    //BP32.update();
    // It is safe to always do this before using the gamepad API.
    // This guarantees that the gamepad is valid and connected.
    /* for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        GamepadPtr myGamepad = myGamepads[i];

        if (myGamepad && myGamepad->isConnected()) {

            rightServo.write( ((((float) myGamepad->axisY()) / 512.0f) * 500) + 1500 );
            leftServo.write( ((((float) myGamepad->axisY()) / 512.0f) * 500) + 1500 );
            // Another way to query the buttons, is by calling buttons(), or
            // miscButtons() which return a bitmask.
            // Some gamepads also have DPAD, axis and more.
            // Console.printf(
            //     "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, "
            //     "%4d, brake: %4d, throttle: %4d, misc: 0x%02x\n",
            //     i,                        // Gamepad Index
            //     myGamepad->dpad(),        // DPAD
            //     myGamepad->buttons(),     // bitmask of pressed buttons
            //     myGamepad->axisX(),       // (-511 - 512) left X Axis
            //     myGamepad->axisY(),       // (-511 - 512) left Y axis
            //     myGamepad->axisRX(),      // (-511 - 512) right X axis
            //     myGamepad->axisRY(),      // (-511 - 512) right Y axis
            //     myGamepad->brake(),       // (0 - 1023): brake button
            //     myGamepad->throttle(),    // (0 - 1023): throttle (AKA gas) button
            //     myGamepad->miscButtons()  // bitmak of pressed "misc" buttons
            // );

            // You can query the axis and other properties as well. See Gamepad.h
            // For all the available functions.
        }
    }  */ 

    // Serial.println(sensor1.getDistanceFloat());

    // uint16_t sensors[3];
    // int16_t position = qtr.readLineBlack(sensors);
    // int16_t error = position - 1000;
    // if (error < 0)
    // {
    //     Serial.println("On the left");
    // }
    // if (error > 0)
    // {
    //     Serial.println("On the right");
    // }
    // if(error == 0){
    //     Serial.println("Straight Ahead");  
    // }
    wallfollower();  
    vTaskDelay(1);
    // delay(100);    
}

