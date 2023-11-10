#include <ESP32Servo.h>
#define servoPin 12
#define led 2

Servo myservo;
int pos = 0;

void setup() {
  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(54);
	myservo.attach(servoPin,1000, 2000);
  Serial.begin(115200);
  Serial.print("Serial monitor check");
  Serial.println();
  pinMode(led, OUTPUT);

}

void loop() {

  // digitalWrite(led, HIGH);
  // Serial.print("starting move");
  // Serial.println();
  // delay(5000);

	// for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
	// 	// in steps of 1 degree
	// 	myservo.write(pos);    // tell servo to go to position in variable 'pos'
  //   Serial.print(pos);
  //   Serial.println();
	// 	delay(1.5);             // waits 15ms for the servo to reach the position
    
	// }

  // digitalWrite(led, LOW);
  // Serial.print("starting move");
  // Serial.println();
  // delay(5000);

	// for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
	// 	myservo.write(pos);    // tell servo to go to position in variable 'pos'
  //   Serial.print(pos);
  //   Serial.println();
	// 	delay(1.5);             // waits 15ms for the servo to reach the position
    
	// }

  myservo.write(0);

  
}
