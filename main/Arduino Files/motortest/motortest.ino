#include <Arduino.h>
#include <ESP32Servo.h>
#include <sdkconfig.h>

#define rightServoPin 12
#define leftServoPin 13


Servo rightServo;
Servo leftServo;
int motorSpeedA;
int motorSpeedB;

void setup() {
    Serial.begin(115200);
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    rightServo.setPeriodHertz(50);
    rightServo.attach(rightServoPin, 1000, 2000);
    leftServo.setPeriodHertz(50);
    leftServo.attach(leftServoPin, 1000, 2000);

}

void loop() {
  motorSpeedA = 1500 + 250;
  motorSpeedB = 1440 - 250;

  leftServo.write(motorSpeedA);
  Serial.print(motorSpeedA);
  Serial.print(", ");
  rightServo.write(motorSpeedB);
  Serial.print(motorSpeedB);
  Serial.println();
  
}
