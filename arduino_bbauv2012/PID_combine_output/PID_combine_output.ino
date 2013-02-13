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

//initialize subscribers and publishers in ROS

ros::NodeHandle nh;

int manual_speed[6];

//teleopControl
void teleopControl(const bbauv_msgs::thruster &msg);
ros::Subscriber<bbauv_msgs::thruster> teleopcontrol_sub("teleop_controller",&teleopControl);

//Controller Mode
void updateControllerMode (const bbauv_msgs::controller_onoff &msg);
ros::Subscriber<bbauv_msgs::controller_onoff> controller_mode("controller_mode", &updateControllerMode);

//update PID Controller Constants; seperated into Rotation and Translational;
//Reason for separation: msg file cannot be too bit, restriction from rosserial client
void updateTranslationalControllerConstants(const bbauv_msgs::controller_translational_constants &msg);
ros::Subscriber<bbauv_msgs::controller_translational_constants> pidconst_trans_sub("translational_constants", &updateTranslationalControllerConstants);
void updateRotationalControllerConstants(const bbauv_msgs::controller_rotational_constants &msg);
ros::Subscriber<bbauv_msgs::controller_rotational_constants> pidconst_rot_sub("rotational_constants", &updateRotationalControllerConstants);

// Update Controller Input; contains both the setpoitn and sensor feedback
void updateControllerInput(const bbauv_msgs::controller_input &msg);
ros::Subscriber<bbauv_msgs::controller_input> controller_sub("controller_input",&updateControllerInput);

//Publish thruster speed
bbauv_msgs::thruster thrusterSpeed;
ros::Publisher thruster_pub("thruster_feedback",&thrusterSpeed);


//smcDriver initialize
#define rxPin 36 // Orange wire <-- receive from the 1st SMC Tx pin
#define txPin 37 // Red wire --> transmit to all SMCs Rx pin
smcDriver mDriver= smcDriver(rxPin,txPin);

//PID initialize
bool inDepthPID, inHeadingPID, inForwardPID, inBackwardPID, inSidemovePID, inTopside, inSuperimpose, inTeleop, resetPID;

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
  inSuperimpose=true;
  
  resetPID=true;
  
  //Initialize thruster ratio to 1:1:1:1:1:1
  float ratio[6]={1, 1, 1, 1, 1, 0.92};
                          
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
// too high a limit will result in too much overshoot.
  headingPID.SetOutputLimits(-1000,1000);
  headingPID.SetControllerDirection(DIRECT);
  
  forwardPID.SetMode(AUTOMATIC);
  forwardPID.SetSampleTime(20);
  forwardPID.SetOutputLimits(-1000,1280); //if lower limit of forwardPID is too high (i.e -1280),
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

/************************************************************/

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
  if(!inTeleop)
  {
  manual_speed[0]=0;
  manual_speed[1]=0;
  manual_speed[2]=0;
  manual_speed[3]=0;
  manual_speed[4]=0;
  manual_speed[5]=0;
  }
}

void getDepthPIDUpdate()
{
  depthPID.SetMode(inDepthPID);
  if(inDepthPID)
  {
    depthPID.Compute();
      /*
    if(inDepthPID && depth_setpoint == float(int(depth_input*10))/10) // && depth_setpoint-depth_input>-0.01)
    {
      
      thrusterSpeed.speed5=-1725;
      thrusterSpeed.speed6=-1725;
    }
    else
    {
      thrusterSpeed.speed5=depth_output+manual_speed[4];
      thrusterSpeed.speed6=depth_output+manual_speed[5];
    }*/
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

void setZeroHorizThrust()
{

  thrusterSpeed.speed1=0;
  thrusterSpeed.speed2=0;
  thrusterSpeed.speed3=0;
  thrusterSpeed.speed4=0;
  
}

void setZeroVertThrust()
{
  thrusterSpeed.speed5=0;
  thrusterSpeed.speed6=0;

}

void setHorizThrustSpeed()
{
  thrusterSpeed.speed1=heading_output-forward_output+manual_speed[0];
  thrusterSpeed.speed2=-heading_output-forward_output+manual_speed[1];
  thrusterSpeed.speed3=heading_output+forward_output+manual_speed[2];
  thrusterSpeed.speed4=-heading_output+forward_output+manual_speed[3];   
}

void setVertThrustSpeed()
{
  thrusterSpeed.speed5=depth_output+manual_speed[4];
  thrusterSpeed.speed6=depth_output+manual_speed[5];
}

void calculateThrusterSpeed()
{
  getTeleopControllerUpdate();
  getDepthPIDUpdate();
  getHeadingPIDUpdate();
  getForwardPIDUpdate();
  getBackwardPIDUpdate();
  getHeadingPIDUpdate();
  getSidemovePIDUpdate();
}

/* How are we sending same set of thrusters multiple PID outputs? */

void superImposePIDoutput()
{
  calculateThrusterSpeed();
  setHorizThrustSpeed();
  setVertThrustSpeed();
  //execute the calculated thruster speed
  nh.loginfo("Running thrusters");
  runThruster();
}

void rotatePIDoutput()
{
  nh.loginfo("Running AUTONOMOUSLY!");
  //Vertical
  inDepthPID=true;
  getDepthPIDUpdate();
  setVertThrustSpeed();
  //Horizontal
  inHeadingPID=true; inForwardPID=false; inSidemovePID=false;
  getHeadingPIDUpdate();
  setHorizThrustSpeed();
  runThruster();
  setZeroHorizThrust();
  inHeadingPID=false; inForwardPID=true; inSidemovePID=false;
  getForwardPIDUpdate();
  setHorizThrustSpeed();
  runThruster();
  setZeroHorizThrust();
  inHeadingPID=false; inForwardPID=false; inSidemovePID=true;
  getForwardPIDUpdate();
  setHorizThrustSpeed();
  runThruster();
  setZeroHorizThrust();  
}

/************************************************************/
// MAIN LOOP
//rosserial has been tested succesfully for message sizes (topic which are pushed to Arduino) 
//to be 14 fields of float32
//If a message type exceeds 14 fields of float32, testing shows that it will not work.\
// www.ros.org/wiki/rosserial/Overview/Limitations

void loop()
{
  if(inSuperimpose)
  {
    superImposePIDoutput();
  }
  else
  {
    rotatePIDoutput();
  }
  
  //publish thruster speed info
  thruster_pub.publish(&thrusterSpeed);
  
  nh.spinOnce();
  //if delay is too low, will also cause lost sync issues
  delay(50);
}    

/****** ROS Subsriber Call Back Functions *********/

void updateControllerMode (const bbauv_msgs::controller_onoff &msg)
{
  inTopside=msg.topside;
  
  if(inTopside)
  {
  inDepthPID= msg.depth_PID;
  inForwardPID= msg.forward_PID;
  inBackwardPID= msg.backward_PID;
  inSidemovePID= msg.sidemove_PID;
  inHeadingPID= msg.heading_PID;
  inTeleop=msg.teleop;
  inSuperimpose=msg.superimpose;
  }
  else
  {
    inSuperimpose=true;
  }
  
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
  /*setpoint and input of a PID must be of the same units */
  
  depth_input=msg.depth_input;
  depth_setpoint=msg.depth_setpoint;
  
  heading_input=msg.heading_input;
  heading_setpoint=msg.heading_setpoint;

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

