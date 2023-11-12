#include <FlexCAN_T4.h>
#include <Servo.h>

// Initializing the servo and DC motor
Servo myServo , myDC;

// Preset the initial position and throttle and the center position.
// The center is slightly offset from 90 to account for the servo center not being aligned with the wheels facing straight.
int position = 95;
int throttle = 1500;
int center = 95;


// Setting the maximum and minimum PWM values
// <1500 == Backwards
// >1500 == Forwards
int pwm_lowerlimit = 1000; 
int pwm_center_value = 1500;
int pwm_upperlimit = 2000;


// Initializing the CAN bus(PIN 22 , 23) and calling it can1
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// Freq test
int count = 0;

void setup() {
  Serial.begin(100000);

  // Servo in PIN 18
  myServo.attach(18);

  // DC Motor in PIN 19
  myDC.attach(19);

  // Initialize CAN controllers
  can1.begin();

  // Set baud rate
  can1.setBaudRate(250000);

  // Used to check that any new uploads are complete
  Serial.println("Upload Complete.");
}

void loop() {

  // Creating the two can message objects
  CAN_message_t msg;
  CAN_message_t msg2;

  // Defining msg2
  // The buf will need to be modified to communicate the voltage to the orin along with other values that what to be sent back.
  msg2.id = 0x123;
  msg2.len = 2;
  msg2.buf[0] = 1;

  // Writing msg2 through the can bus to the ORIN
  can1.write(msg2);

  // If there is a message to be read in can1
  if (can1.read(msg)) {

    // Receiving throttle and position values
    // The values need to be modified to account for any modifications made before communicating over the can bus
    // throttle: -28 to 28
    // position: -100 to 100
    throttle = word(msg.buf[0] , msg.buf[1]) - 28;
    position = word(msg.buf[2] , msg.buf[3]) - 100;

    // The values need to be converted into pwm values that correspond to the separate motors
    // throttle: 1300 to 1700
    // position: 40 to 140 (Steering is unable to reach any degrees beyond these two)
    throttle = floor((7.14)*float(throttle)) + 1500;
    position = floor(0.5 * float(position)) + center;

    // Writing the new positions and throttles received and creating a delay to ensure that the motors have time to adjust before next message is read
    myServo.write(position);
    delay(15);
    myDC.writeMicroseconds(throttle);
    delay(15);
  }
}
