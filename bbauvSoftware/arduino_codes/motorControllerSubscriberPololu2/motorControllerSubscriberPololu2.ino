#include <thruster.h>
#include <ros.h>
#include <SoftwareSerial.h>

#define txPin1 4
#define rxPin1 5

#define txPin2 8 
#define rxPin2 9

#define txPin3 17 
#define rxPin3 16

ros::NodeHandle nh;

void motorControl(const bbauv_msgs::thruster &msg);

ros::Subscriber<bbauv_msgs::thruster> control_sub("motor_controller",&motorControl);

SoftwareSerial smcSerial1 = SoftwareSerial(rxPin1, txPin1);
SoftwareSerial smcSerial2 = SoftwareSerial(rxPin2, txPin2);
SoftwareSerial smcSerial3 = SoftwareSerial(rxPin3, txPin3);
//SoftwareSerial smcSerial4 = SoftwareSerial(rxPin4, txPin4);
//SoftwareSerial smcSerial5 = SoftwareSerial(rxPin5, txPin5);

// required to allow motors to move
// must be called when controller restarts and after any error

SoftwareSerial getSmc (uint8_t motor_index)
{
  switch(motor_index)
  {
    case 1: return smcSerial1;
    case 2: return smcSerial2;
    case 3: return smcSerial3;
//    case 4: return smcSerial4;
//    case 5: return smcSerial5;
  }
}

void exitSafeStart(uint8_t motor_index)
{
  SoftwareSerial currSmc= getSmc(motor_index);
  currSmc.write(0x83);
}
 
// speed should be a number from -3200 to 3200
void setMotorSpeed(uint8_t motor_index,int speed)
{
  SoftwareSerial currSmc= getSmc(motor_index);
  if (speed < 0)
  {
    currSmc.write(0x86);  // motor reverse command
    speed = -speed;  // make speed positive
  }
  else
  {
    currSmc.write(0x85);  // motor forward command
  }
  currSmc.write(speed & 0x1F);
  currSmc.write(speed >> 5);
}

void setup()
{
  // initialize software serial object with baud rate of 19.2 kbps
  smcSerial1.begin(19200);
  smcSerial2.begin(19200);
  smcSerial3.begin(19200);
//  smcSerial4.begin(19200);
//  smcSerial5.begin(19200);
  
  // the Simple Motor Controller must be running for at least 1 ms
  // before we try to send serial data, so we delay here for 5 ms
  delay(5);
 
  // if the Simple Motor Controller has automatic baud detection
  // enabled, we first need to send it the byte 0xAA (170 in decimal)
  // so that it can learn the baud rate
  smcSerial1.write(0xAA);  // send baud-indicator byte
  smcSerial2.write(0xAA);
  smcSerial3.write(0xAA);
//  smcSerial4.write(0xAA);
//  smcSerial5.write(0xAA);
  
  // next we need to send the Exit Safe Start command, which
  // clears the safe-start violation and lets the motor run
  exitSafeStart(1);  // clear the safe-start violation and let the motor run
  exitSafeStart(2);
  exitSafeStart(3);
//  exitSafeStart(4);
//  exitSafeStart(5);
  nh.initNode();
  nh.subscribe(control_sub);
}
 
void loop()
{
  nh.spinOnce();
  delay(1);
}


void motorControl(const bbauv_msgs::thruster &msg)
{
  setMotorSpeed(1,msg.speed1);
  setMotorSpeed(2,msg.speed2);
  setMotorSpeed(3,msg.speed3);
  
}
