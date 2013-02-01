//using for 3rd integration
#include <smcDriver.h>
#include <PID_v1.h>
#include <controller_input.h>
#include <controller_param.h>
#include <controller_param_depth.h>
#include <thruster.h>
#include <ros.h>
#include <SoftwareSerial.h>
#include <ArduinoHardware.h>

//ROS initialize

ros::NodeHandle nh;

int manual_speed[6];
void teleopControl(const bbauv_msgs::thruster &msg);
ros::Subscriber<bbauv_msgs::thruster> teleopcontrol_sub("teleop_controller",&teleopControl);

void updateControllerParam(const bbauv_msgs::controller_param &msg);
ros::Subscriber<bbauv_msgs::controller_param> pidconst_sub("controller_config",&updateControllerParam);
void updateControllerParamDepth(const bbauv_msgs::controller_param_depth &msg);
ros::Subscriber<bbauv_msgs::controller_param_depth> pidconst_depth_sub("controller_config_depth",&updateControllerParamDepth);

void updateControllerInput(const bbauv_msgs::controller_input &msg);
ros::Subscriber<bbauv_msgs::controller_input> controller_sub("controller_input",&updateControllerInput);

bbauv_msgs::thruster thrusterSpeed;
ros::Publisher thruster_pub("thruster_feedback",&thrusterSpeed);


//smcDriver initialize
#define rxPin 36 // Orange wire <-- receive from the 1st SMC Tx pin
#define txPin 37 // Red wire --> transmit to all SMCs Rx pin
smcDriver mDriver= smcDriver(rxPin,txPin);

//PID initialize
bool inDepthPID, inHeadingPID, inForwardPID, inBackwardPID, inSidemovePID, resetPID;

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
  inDepthPID= true;
  inHeadingPID= true;
  inForwardPID=true;
  inBackwardPID=true;
  inSidemovePID=true;
  
  resetPID=false;
  
  float ratio[6]={1, 1, 1, 1, 1, 1};
                          
  for(int i=0;i<6;i++)
    manual_speed[i]=0;
    
  //initialize Motor driver and ROS
  mDriver.init();
  nh.initNode();
  nh.subscribe(controller_sub);
  nh.subscribe(pidconst_sub);
  nh.subscribe(pidconst_depth_sub);
  nh.subscribe(teleopcontrol_sub);
  nh.advertise(thruster_pub);
  mDriver.setThrusterRatio(ratio);
  
  //Note the sample time here is 50ms. ETS SONIA's control loop runs at 70ms.
  //If Sample Time is set too low, will cause 
  //lost sync issues when transferring data back and forth with ROS
  depthPID.SetMode(AUTOMATIC);
  depthPID.SetSampleTime(50);
  depthPID.SetOutputLimits(-2560,2560);
  depthPID.SetControllerDirection(REVERSE);
  
  headingPID.SetMode(AUTOMATIC);
  headingPID.SetSampleTime(50);
  headingPID.SetOutputLimits(-1280,1280);
  headingPID.SetControllerDirection(DIRECT);
  
  forwardPID.SetMode(AUTOMATIC);
  forwardPID.SetSampleTime(50);
  forwardPID.SetOutputLimits(0,2560);
  forwardPID.SetControllerDirection(DIRECT);
  
  backwardPID.SetMode(AUTOMATIC);
  backwardPID.SetSampleTime(50);
  backwardPID.SetOutputLimits(0,2560);
  backwardPID.SetControllerDirection(DIRECT);
  
  sidemovePID.SetMode(AUTOMATIC);
  sidemovePID.SetSampleTime(50);
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
  thrusterSpeed.speed1=manual_speed[0];
  thrusterSpeed.speed2=manual_speed[1];
  thrusterSpeed.speed3=manual_speed[2];
  thrusterSpeed.speed4=manual_speed[3];   
  thrusterSpeed.speed5=manual_speed[4];
  thrusterSpeed.speed6=manual_speed[5];    
}

void getDepthPIDUpdate()
{
  //depthPID.SetMode(inDepthPID);
  //if(inDepthPID)
  {
    depthPID.Compute();
  }
}

void getHeadingPIDUpdate()
{
  //headingPID.SetMode(inHeadingPID);
  //if(inHeadingPID)
  {
    headingPID.Compute();
  }
}

void getForwardPIDUpdate()
{
  //forwardPID.SetMode(inForwardPID);
  //if(inForwardPID)
  {
    forwardPID.Compute();
  }
}

void getBackwardPIDUpdate()
{
  //backwardPID.SetMode(inBackwardPID);
  //if(inBackwardPID)
  {
    backwardPID.Compute();
  }
}

void getSidemovePIDUpdate()
{
  //sidemovePID.SetMode(inSidemovePID);
  //if(inSidemovePID)
  {
    sidemovePID.Compute();
  }
}

void calculateThrusterSpeed()
{
  getTeleopControllerUpdate();
  getDepthPIDUpdate();
  getForwardPIDUpdate();
  getBackwardPIDUpdate();
  getHeadingPIDUpdate();
  getSidemovePIDUpdate();
  
  //side move not implemented yet
  thrusterSpeed.speed1=heading_output-forward_output+backward_output+manual_speed[0];
  thrusterSpeed.speed2=-heading_output-forward_output+backward_output+manual_speed[1];
  thrusterSpeed.speed3=heading_output+forward_output-backward_output+manual_speed[2];
  thrusterSpeed.speed4=-heading_output+forward_output-backward_output+manual_speed[3];
  thrusterSpeed.speed5=depth_output+manual_speed[4];
  thrusterSpeed.speed6=depth_output+manual_speed[5];
}

void loop()
{

  //calculate final M.Speed for each thruster
  calculateThrusterSpeed();
  
  //execute the calculated thruster speed
  runThruster();
  
  //publish thruster speed info
  thruster_pub.publish(&thrusterSpeed);
  
  nh.spinOnce();
  //if delay is too low, will also cause lost sync issues
  delay(55);
}    


void updateControllerParam(const bbauv_msgs::controller_param &msg)
{
  /*
  float ratio[6]={msg.ratio_t1, msg.ratio_t2, msg.ratio_t3, 
                           msg.ratio_t4, msg.ratio_t5,msg.ratio_t6}; */

  headingPID.SetTunings(msg.heading_kp,msg.heading_ki,msg.heading_kd);
  //inHeadingPID= msg.heading_PID;
  
  forwardPID.SetTunings(msg.forward_kp,msg.forward_ki,msg.forward_kd);
  //inForwardPID= msg.forward_PID;

  backwardPID.SetTunings(msg.backward_kp,msg.backward_ki,msg.backward_kd);
  //inBackwardPID= msg.backward_PID;

  sidemovePID.SetTunings(msg.sidemove_kp,msg.sidemove_ki,msg.sidemove_kd);
  //inSidemovePID= msg.sidemove_PID;
  
  //resetPID=msg.reset;
}

void updateControllerParamDepth(const bbauv_msgs::controller_param_depth &msg)
{
  depthPID.SetTunings(msg.depth_kp,msg.depth_ki,msg.depth_kd);
  //inDepthPID= msg.depth_PID;
  
  //resetPID=msg.reset;  
}


void updateControllerInput(const bbauv_msgs::controller_input &msg)
{
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
