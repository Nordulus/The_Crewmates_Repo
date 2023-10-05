#include <QTRSensors.h>
#include <Arduino.h>

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
#define LED 2

QTRSensors qtra;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


void setup(){
delay(500);

qtra.setTypeAnalog();
qtra.setSensorPins((const uint8_t[]){QTRSENSOR1, QTRSENSOR2, QTRSENSOR3, QTRSENSOR4, QTRSENSOR5, QTRSENSOR6, QTRSENSOR7, QTRSENSOR8}, SensorCount);

delay(500);
//serial monitor
Serial.begin(9600);
Serial.print("Serial monitor check");

Serial.print("Calibration mode begin");
pinMode(LED, OUTPUT);
digitalWrite(LED, HIGH); //LED blue to indicate calibration mode
for (uint16_t i = 0; i < 400; i++){
    qtra.calibrate();
}
Serial.println();


// print the calibration minimum values measured when emitters were on
Serial.print("printing minimum values:");
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtra.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
digitalWrite(LED, LOW);

// print max calibration values
Serial.print("printing max values");
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtra.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
delay(1000);

}

void loop(){

uint16_t position = qtra.readLineBlack(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

delay(250);


}