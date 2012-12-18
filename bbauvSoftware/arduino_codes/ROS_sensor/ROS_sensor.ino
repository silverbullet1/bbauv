#include <thruster.h>
#include <env_data.h>

#include <ros.h>

ros::NodeHandle  nh;
bbauv_msgs::env_data env_msg;
ros::Publisher env_pub("env_data", &env_msg);



//Initialise Pins -------------------------------------------------------

const int TempPin0 = A0;
const int TempPin1 = A1;
const int TempPin2 = A2;
float Temp0 = 0;
float Temp1 = 0;
float Temp2 = 0;

const int DepthPin = A5;
float Depth = 0;

const int WaterPin2 = 2; // White
const int WaterPin3 = 3; // Grey
const int WaterPin4 = 4; // Pink

float WaterDetA = 0;
float WaterDetB = 0;
float WaterDetC = 0;


const int RedLedPin = 8;

int i = 0;

// Functions ---------------------------------------------------------
float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



// Setup -------------------------------------------------------------
void setup()
{
  nh.initNode();
  nh.advertise(env_pub);
  
  pinMode(RedLedPin, OUTPUT);
  
  // 5V and GND for Water Sensors
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, INPUT); // Inner
  pinMode(3, INPUT); // Middle
  pinMode(4, INPUT); // Outer
  digitalWrite(0,HIGH); // Brown
  digitalWrite(1,LOW); // Black
}



// Loop ---------------------------------------------------------------
void loop()
{
  //Temp-------------
  
  Temp0 = (analogRead(TempPin0)-102.3)/2.046;
  Temp1 = (analogRead(TempPin1)-102.3)/2.046;
  Temp2 = (analogRead(TempPin2)-102.3)/2.046;
  
  //Depth = fmap(analogRead(DepthPin),DataRead of 0.5V, DataRead of 4.5V, 0 Depth, 70.280m Depth at 500psi)
  
  Depth = fmap(analogRead(DepthPin),102.300, 902.700,0.000, 70.280);
  
 digitalWrite(RedLedPin, HIGH);
 
 // Water -----------
 
 WaterDetA = digitalRead(WaterPin2);
 WaterDetB = digitalRead(WaterPin3);
 WaterDetC = digitalRead(WaterPin4);
 
  // ROS Code
  env_msg.Temp0 = Temp0;
  env_msg.Temp1 = Temp1;
  env_msg.Temp2 = Temp2;
  env_msg.Depth = Depth;
  env_msg.WaterDetA = WaterDetA;
  env_msg.WaterDetB = WaterDetB;
  env_msg.WaterDetC = WaterDetC;
  
  env_pub.publish( &env_msg );
  nh.spinOnce();
  delay(100);
}
