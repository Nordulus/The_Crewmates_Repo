#include <Bluepad32.h>
#include <Arduino.h>
#include <QTRSensors.h>
#include <ESP32Servo.h>
#include <sdkconfig.h>

#define rightServoPin 12
#define leftServoPin 13

int P;
int I;
int D;

int baseSpeed = 1500;

float Kp = 0.05;
float Ki = 0.00001;
float Kd = 0.8;

int lastError = 0;

QTRSensors qtr;
Servo rightServo;
Servo leftServo;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup() {
    pinMode(2, OUTPUT);
    Serial.begin(115200);
    Serial.println("");
    Serial.print("Serial monitor check");
  // put your setup code here, to run once:
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){36, 39, 34, 35, 32, 33, 25, 26}, SensorCount);
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

    digitalWrite(2, HIGH);
    Serial.print("Calibrating");
    //10 seconds
    for (uint16_t i = 0; i < 400; i++){
        qtr.calibrate();
    }
    digitalWrite(2, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  PID_control();
  printQTR();
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
  int motorSpeedA = baseSpeed + motorSpeedChange;
  int motorSpeedB = baseSpeed + motorSpeedChange;

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
  leftServo.write(motorSpeedA);
  rightServo.write(motorSpeedB);
}

void printQTR(){
  uint16_t position = qtr.readLineBlack(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.print(position);
  Serial.println();
  delay(10);
}