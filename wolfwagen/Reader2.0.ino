#include <FlexCAN_T4.h>
#include <Servo.h>

// Initializing the servo and DC motor
Servo myServo , myDC;
int position = 90;
int throttle = 1500;

// Setting the maximum and minimum PWM values
// int pwm_lowerlimit = 1000; 
// int pwm_center_value = 1500;
// int pwm_upperlimit = 2000;
// int pwm_lowerlimit = 6554; 
// int pwm_center_value = 9830;
// int pwm_upperlimit = 13108;


// Initializing the CAN (PIN 22 , 23)
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// Freq test
// int count = 0;

void setup() {
  Serial.begin(100000);
  while (!Serial);

  // Servo in PIN 18
  myServo.attach(18);

  // DC Motor in PIN 19
  myDC.attach(19);

  // Initialize CAN controllers
  can1.begin();

  // Set baud rate
  can1.setBaudRate(250000);
}

// void turn_servo(float cmd_pos){
//   // Turning servo based on cmd_pos
//   // Converting 6554 - 13108 to -16 - 184 values for the servo centered at 84
//   // Can further be adjusted to reach a maximum and a minimum in the future, currently exceeds the possible values
//   //int cmd_position = floor((100.0 / 3277.0) * cmd_pos - 216.0);

//   // Commanding servo to move to new position
//   //myServo.write(cmd_position);
// }

// void change_throttle(int cmd_throt){
//   // Changing throttle based on cmd_throt
//   // Converting 6554 - 13108 to 1000 - 2000 values for the throttle centered at 1500
//   int cmd_throttle = floor((500.0 / 3277.0) * cmd_throt);

//   Serial.print("Throttle found");

//   // Commanding DC motor to change throttle
//   myDC.writeMicroseconds(cmd_throttle);
// }

// void pwm_received(int received_throttle , int received_position){
//   // Series of if and else to make sure pwm values never exceed the limits
//   if(received_throttle < pwm_lowerlimit)
//     throttle = pwm_lowerlimit;
//   else if(received_throttle > pwm_upperlimit)
//     throttle = pwm_upperlimit;
//   else
//     throttle = received_throttle;
    
//   if(received_position < pwm_lowerlimit)
//     position = pwm_lowerlimit;
//   else if(received_position > pwm_upperlimit)
//     position = pwm_upperlimit;
//   else
//     position = received_position;

//   // Calling the servo and throttle functions
//   turn_servo(position);
//   change_throttle(throttle);

// }

void loop() {
  // Delay needs to be double writer to process
  // Check for incoming CAN messages on can1
  // Serial.println("Outside CAN Loop");
  //count = count + 1;
  CAN_message_t msg;
  //Serial.println(can1.read(msg));
  //Serial.print("Looking for CAN... "); Serial.println(count);
  if (can1.read(msg)) {
    // Process received message
    // Access message fields
    // Serial.print("Received message ID: 0x");
    // Serial.print(msg.id, HEX);
    // Serial.print(" Length: ");
    // Serial.println(msg.len);
    //Serial.print("Data: ");

    // Receiving throttle and position values
    throttle = word(msg.buf[0] , msg.buf[1]);
    position = word(msg.buf[2] , msg.buf[3]);

    // Serial.println(throttle);
    // Serial.println(position);

    // Calling pwm_received with throttle and position values
    //pwm_received(throttle , position);
    myServo.write(position);
    myDC.write(throttle);
  }
  delay(15);
  // if (count == 10){
  //   count = 0;
  // }

}
