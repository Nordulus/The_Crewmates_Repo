#include <ESP32SharpIR.h>


ESP32SharpIR left(ESP32SharpIR::GP2Y0A21YK0F, 2);
ESP32SharpIR center(ESP32SharpIR::GP2Y0A21YK0F, 15);
ESP32SharpIR right(ESP32SharpIR::GP2Y0A21YK0F, 0);

void setup(){
    delay(500);
    left.setFilterRate(0.1f);
    center.setFilterRate(0.1f);
    right.setFilterRate(0.1f);
}

void loop(){
    Serial.println(left.getDistanceFloat());
    delay(500);
}
