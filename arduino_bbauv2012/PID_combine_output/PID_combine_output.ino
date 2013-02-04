//using for 3rd integration
#include <ros.h>
#include <smcDriver.h>
#include <PID_v1.h> //based on http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/ 
#include <controller_input.h>
#include <controller_translational_constants.h>
#include <controller_rotational_constants.h>
#include <controller_onoff.h>
#include <thruster.h>
#include <SoftwareSerial.h>
#include <ArduinoHardware.h>

//ROS initialize

ros::NodeHandle nh;

int manual_speed[6];
void teleopControl(const bbauv_msgs::thruster &msg);
ros::Subscriber<bbauv_msgs::thruster> teleopcontrol_sub("teleop_controller",&teleopControl);

void updateControllerMode (const bbauv_msgs::controller_onoff &msg);
ros::Subscriber<bbauv_msgs::controller_onoff> controller_mode("controller_mode", &updateControllerMode);

void updateTranslationalControllerConstants(const bbauv_msgs::controller_translational_constants &msg);
ros::Subscriber<bbauv_msgs::controller_translational_constants> pidconst_trans_sub("translational_constants", &updateTranslationalControllerConstants);
void updateRotationalControllerConstants(const bbauv_msgs::controller_rotational_constants &msg);
ros::Subscriber<bbauv_msgs::controller_rotational_constants> pidconst_rot_sub("rotational_constants", &updateRotationalControllerConstants);

void updateControllerInput(const bbauv_msgs::controller_input &msg);
ros::Subscriber<bbauv_msgs::controller_input> controller_sub("controller_input",&updateControllerInput);

bbauv_msgs::thruster thrusterSpeed;
ros::Publisher thruster_pub("thruster_feedback",&thrusterSpeed);


//smcDriver initialize
#define rxPin 36 // Orange wire <-- receive from the 1st SMC Tx pin
#define txPin 37 // Red wire --> transmit to all SMCs Rx pin
smcDriver mDriver= smcDriver(rxPin,txPin);

//PID initialize
bool inDepthPID, inHeadingPID, inForwardPID, inBackwardPID, inSidemovePID, inTeleop, resetPID;

double depth_setpoint,depth_input,depth_output;
PID depthPID(&depth_input, &depth_output, &depth_setpoint,1,0,0, DIRECT);

double heading_setpoint,heading_input,heading_output;
PID headingPID(&heading_input, &heading_output, &heading_setpoint,1,0,0, DIRECT);

double forward_setpoint,forward_input,forward_output;
PID forwardPID(&forward_input, &forward_output, &forward_setpoint,1,0,0, DIRECT);

double backward_setpoint,backward_input,backward_output;
PID backwardPID(&backward_input, &backward_output, &backward_setpoint,1,0,0, DIRECT);

double sidemove_setpoint,sidemove_input,sidemove_output;
PID sidemovePID(&sidemove_input, &sidemove_output, &sidemove_setpoint,1,0,0, DIRECT);

void setup()
{

  //initialize value for variables
  inDepthPID= false;
  inHeadingPID= false;
  inForwardPID=false;
  inBackwardPID=false;
  inSidemovePID=false;
  inTeleop=false;
  
  resetPID=true;
  
  //Initialize thruster ratio to 1:1:1:1:1:1
  float ratio[6]={1, 1, 1, 1, 1, 1};
                          
  for(int i=0;i<6;i++)
    manual_speed[i]=0;
    
  //initialize Motor driver and ROS
  mDriver.init();
  nh.initNode();
  nh.subscribe(controller_mode);
  nh.subscribe(controller_sub);
  nh.subscribe(pidconst_trans_sub);
  nh.subscribe(pidconst_rot_sub);
  nh.subscribe(teleopcontrol_sub);
  nh.advertise(thruster_pub);
  mDriver.setThrusterRatio(ratio);
  
  //Note the sample time here is 50ms. ETS SONIA's control loop runs at 70ms.
  //If Sample Time is set too low, will cause 
  //lost sync issues when transferring data back and forth with ROS
  depthPID.SetMode(AUTOMATIC);
  depthPID.SetSampleTime(20);
  depthPID.SetOutputLimits(-2560,2560);
  depthPID.SetControllerDirection(REVERSE);
  
  headingPID.SetMode(AUTOMATIC);
  headingPID.SetSampleTime(20);
//  headingPID.SetOutputLimits(-2000,2000);
  headingPID.SetOutputLimits(-1280,1280);
  headingPID.SetControllerDirection(DIRECT);
  
  forwardPID.SetMode(AUTOMATIC);
  forwardPID.SetSampleTime(20);
  forwardPID.SetOutputLimits(-500,1280); //if lower limit of forwardPID is too high (i.e -1280),
                                         //AUV will move back a lot when going from 
                                         //positive velocity to zero velocity
  forwardPID.SetControllerDirection(DIRECT);
  
  backwardPID.SetMode(AUTOMATIC);
  backwardPID.SetSampleTime(20);
  backwardPID.SetOutputLimits(-2560,2560);
  backwardPID.SetControllerDirection(DIRECT);
  
  sidemovePID.SetMode(AUTOMATIC);
  sidemovePID.SetSampleTime(20);
  sidemovePID.SetOutputLimits(-2560,2560);
  sidemovePID.SetControllerDirection(DIRECT);
  
  pinMode(13, OUTPUT); 
}

void runThruster()
{

  mDriver.setMotorSpeed(1,thrusterSpeed.speed1);
  mDriver.setMotorSpeed(2,thrusterSpeed.speed2);
  mDriver.setMotorSpeed(3,thrusterSpeed.speed3);
  mDriver.setMotorSpeed(4,thrusterSpeed.speed4);
  mDriver.setMotorSpeed(5,thrusterSpeed.speed5);
  mDriver.setMotorSpeed(6,thrusterSpeed.speed6);  
}

void getTeleopControllerUpdate()
{

}

void getDepthPIDUpdate()
{
  depthPID.SetMode(inDepthPID);
  if(inDepthPID)
  {
    depthPID.Compute();
  }
  else
  {
    depth_output=0;
  }
}

void getHeadingPIDUpdate()
{
  headingPID.SetMode(inHeadingPID);
  if(inHeadingPID)
  {
    headingPID.Compute();
  }
  else
  {
    heading_output=0;
  }
}

void getForwardPIDUpdate()
{
  forwardPID.SetMode(inForwardPID);
  if(inForwardPID && !inBackwardPID)
  {

    forwardPID.Compute();
  }
  else
  {
    forward_output=0;
  }
}

void getBackwardPIDUpdate()
{
  backwardPID.SetMode(inBackwardPID);
  if(inBackwardPID && !inForwardPID)
  {
    backwardPID.Compute();
  }
  else
  {
    backward_output=0;
  }
}

void getSidemovePIDUpdate()
{
  sidemovePID.SetMode(inSidemovePID);
  if(inSidemovePID)
  {
    sidemovePID.Compute();
  }
  else
  {
    sidemove_output=0;
  }
  
}

void calculateThrusterSpeed()
{
  
  if(inTeleop)
  {
  thrusterSpeed.speed1=manual_speed[0];
  thrusterSpeed.speed2=manual_speed[1];
  thrusterSpeed.speed3=manual_speed[2];
  thrusterSpeed.speed4=manual_speed[3];   
  thrusterSpeed.speed5=manual_speed[4];
  thrusterSpeed.speed6=manual_speed[5];  
  }
  else
  {
  getDepthPIDUpdate();
  getForwardPIDUpdate();
  getBackwardPIDUpdate();
  getHeadingPIDUpdate();
  getSidemovePIDUpdate();
  
  //side move not implemented yet
  
  //Uncomment if in simulation mode
  heading_output *= -1;
  sidemove_output *= -1;
  //forward_output *= -1;

  thrusterSpeed.speed1=heading_output-forward_output+backward_output;//+sidemove_output;
  thrusterSpeed.speed2=-heading_output-forward_output+backward_output;//sidemove_output;
  thrusterSpeed.speed3=heading_output+forward_output-backward_output;//sidemove_output;
  thrusterSpeed.speed4=-heading_output+forward_output-backward_output;//-sidemove_output;
  thrusterSpeed.speed5=depth_output;
  thrusterSpeed.speed6=depth_output;
  
  }

}

//rosserial has been tested succesfully for message sizes (topic which are pushed to Arduino) 
//to be 14 fields of float32
//If a message type exceeds 14 fields of float32, testing shows that it will not work.\
// www.ros.org/wiki/rosserial/Overview/Limitations

void loop()
{

  //calculate final M.Speed for each thruster
  calculateThrusterSpeed();
  
  //execute the calculated thruster speed
  //runThruster();
  
  //publish thruster speed info
  thruster_pub.publish(&thrusterSpeed);
  
  nh.spinOnce();
  //if delay is too low, will also cause lost sync issues
  delay(50);
}    

void updateControllerMode (const bbauv_msgs::controller_onoff &msg)
{
  inDepthPID= msg.depth_PID;
  inForwardPID= msg.forward_PID;
  inBackwardPID= msg.backward_PID;
  inSidemovePID= msg.sidemove_PID;
  inHeadingPID= msg.heading_PID;
  inTeleop=msg.teleop;
  resetPID=msg.reset;
}

void updateTranslationalControllerConstants(const bbauv_msgs::controller_translational_constants &msg)
{
  /*
  float ratio[6]={msg.ratio_t1, msg.ratio_t2, msg.ratio_t3, 
                           msg.ratio_t4, msg.ratio_t5,msg.ratio_t6}; */

  depthPID.SetTunings(msg.depth_kp,msg.depth_ki,msg.depth_kd);
  forwardPID.SetTunings(msg.forward_kp,msg.forward_ki,msg.forward_kd);
  backwardPID.SetTunings(msg.backward_kp,msg.backward_ki,msg.backward_kd);
  sidemovePID.SetTunings(msg.sidemove_kp,msg.sidemove_ki,msg.sidemove_kd);

}

void updateRotationalControllerConstants(const bbauv_msgs::controller_rotational_constants &msg)
{
  headingPID.SetTunings(msg.heading_kp,msg.heading_ki,msg.heading_kd);

}


void updateControllerInput(const bbauv_msgs::controller_input &msg)
{
  depth_input=msg.depth_input;
  depth_setpoint=msg.depth_setpoint;
  
  heading_input=msg.heading_input;
  heading_setpoint=msg.heading_setpoint;
  
  //nh.loginfo("getting forward input and setpoint");

  forward_input=msg.forward_input;
  forward_setpoint=msg.forward_setpoint;
  
  backward_input=msg.backward_input;
  backward_setpoint=msg.backward_setpoint;
  
  sidemove_input=msg.sidemove_input;
  sidemove_setpoint=msg.sidemove_setpoint;
 
}

void teleopControl(const bbauv_msgs::thruster &msg)
{
  manual_speed[0]=msg.speed1;
  manual_speed[1]=msg.speed2;
  manual_speed[2]=msg.speed3;
  manual_speed[3]=msg.speed4;
  manual_speed[4]=msg.speed5;
  manual_speed[5]=msg.speed6;
}
