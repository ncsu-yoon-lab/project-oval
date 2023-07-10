#include <Servo.h>

int message;

Servo myServo , myDC;

int list[2];

void setup() {
  // put your setup code here, to run once:
  // Setting rate for serial
  Serial.begin(9600);
  // Wait for serial to get ready
  while(!Serial);

  myServo.attach(18);
  
  myDC.attach(19);

}

void loop() {
  if (Serial.available()>0){
    for (int i = 0; i < 2; i++){
      list[i] = Serial.read();
    }
    int throttle = list[0];
    int steering = list[1];

    myDC.writeMicroseconds(10 * throttle);
    myServo.write(steering);
    
  }
}