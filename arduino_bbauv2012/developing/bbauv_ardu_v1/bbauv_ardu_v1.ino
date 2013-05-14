//bbauv_ardu_v1 - Combine Thruster and Sensor arduino
//***************************************************
//Include Libraries
#include <ros.h>
#include <smcDriver_v2.h> //Simple Motor Controller from Pololu Robotics and Electronics
#include <Servo.h> //Manipulator
#include <Adafruit_ADS1015.h> //Display
#include <Wire.h> //For I2C
//Messages to communicate with ROS
#include <thruster.h>   //thruster speed
#include <manipulator.h> //For servos control
#include <openups.h>    //battery capacity
#include <hull_status.h> //Temperature, Water Sensor
#include <std_msgs.h>
//Constant declaration
#include "defines.h"
//Constants definition
#define PRESSURE_TYPE PRESSURE_TYPE_GAUGE_30
#define DEBUG_MODE DEBUG_BB

//Timming variables - to ensure the loop run at correct frequency
static uint32_t currentTime,loopTime, fast_loop,time_elapsed, medium_loop, slow_loop;

uint32_t slow_loop_ctr;

//Declare Subscribers, Publishers & Call back functions in ROS
    ros::NodeHandle nh;

    //Thruster Controller
    void getThrusterSpeed(const bbauv_msgs::thruster &msg);
    ros::Subscriber<bbauv_msgs::thruster> thruster_sub("thruster_speed",&getThrusterSpeed);

    //Manipulator Controller
    void getManipulator(const bbauv_msgs::manipulator &msg){}
    ros::Subscriber<bbauv_msgs::manipulator> manipulator_sub("manipulator",&getManipulator);

    //Battery reading - openups
    void getBatteryReading(const bbauv_msgs::hull_status &msg){}
    ros::Subscriber<bbauv_msgs::hull_status> battery_sub("battery_reading",&getBatteryReading);

    //Hull Status Publishers - Temperature, Water Sensor
    bbauv_msgs::hull_status env_msg;
    ros::Publisher env_pub("hull_status", &env_msg);

    //Pressure publisher
    std_msgs::Int16 pressure_msg;
    ros::Publisher pressure_pub("pressure",&pressure_msg);

//Motor Driver definitions
  bbauv_msgs::thruster thrusterSpeed;
  smcDriver mDriver(&Serial1); //Use Serial1 to handle UART communication with motor controllers

//Manipulators definitions

    Servo myservo;

//ADC definitions
    Adafruit_ADS1115 ads1115;

//Pressure Sensor Definitions

    static int16_t depth;

//Temperature Sensor Definitions
    float temp1 = 0;
    float temp2 = 0;
    float temp3 = 0;

void setup()
{
//Initialize ROS: publishers, subscribers
    nh.initNode();
    nh.subscribe(thruster_sub);
    nh.subscribe(manipulator_sub);
    nh.subscribe(battery_sub);
    nh.advertise(env_pub);
    nh.advertise(depth_pub);

//Initialize Motor Driver:
    //Set Baud rate for Serial1 (UART communication)
    Serial1.begin(115200);
    mDriver.init();
    //Set Thruster Ratio:
        //float ratio[6]={0.8471, 0.9715, 0.9229, 0.9708, 0.8858, 1};
        //mDriver.setThrusterRatio(ratio);

//Initialize Manipulators
    myservo.attach(9);
    myservo.write(0);

//Initialize I2C bus:

    Wire.begin();

//Initialize MainLoop Timming variables
    time_elapsed=0;
    currentTime=millis();
    loopTime=currentTime;

//Debug Mode: to be removed by the compiler if not in debug mode.
    #if DEBUG_MODE == DEBUG_BB
      Serial2.begin(9600);
      Serial2.println("Debug Mode");
    #endif
}

void loop()
{
  currentTime=millis();
  
  //100 Hertz Loop for Pressure Sensor filtering
  if( currentTime >= (fast_loop + 10))
  {
    readPressureFilter();
    #if DEBUG_MODE == DEBUG_BB
      Serial2.println(currentTime - fast_loop);
    #endif
    fast_loop = currentTime;
  }
   
  //20 Hertz Loop for Control Loop
  currentTime=millis();
  
  if( currentTime >= (medium_loop + 50))
  {
     #if DEBUG_MODE == DEBUG_BB
      Serial2.println(currentTime - medium_loop);
    #endif
    runThruster();
    depth_msg.depth = depth;
    depth_pub.publish(&depth_msg);
    nh.spinOnce();
    medium_loop = currentTime;
  }
  
 //1Hertz Loop for critical sensors
 currentTime=millis();
  
 if(currentTime >=(slow_loop + 333))
 {
   switch(slow_loop_ctr)
   {
     case 0:
       readTemperature();
       break;
     case 1:
       readWater();
       env_pub.publish(&env_msg);
       break;
     case 2:
       //LCD code here
       break;
   }
   if(slow_loop_ctr != 3)  slow_loop_ctr++;
   else  slow_loop_ctr = 0;
    #if DEBUG_MODE == DEBUG_BB
    Serial2.println(currentTime - slow_loop);
    #endif
   slow_loop = currentTime;
 } 
}

//Supporting functions:
int32_t fmap(int32_t input, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max){
  return (input- in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


//Sensor Tasks Callbacks ---------------------------------------------------------
int16_t readPressure()
{
    //int32_t pressure;
    //float temp_depth;
    int16_t adc;
 #if PRESSURE_TYPE == PRESSURE_TYPE_GAUGE_30
    adc = ads1115.readADC_SingleEnded(0);
    //pressure = fmap(adc, 5340,26698,ATM,PSI30);
    //temp_depth = (float) pressure*100/(1000*9.81); //In centimetres
 #elif PRESSURE_TYPE == PRESSURE_TYPE_ABSOLUTE_100
    adc = ads1115.readADC_SingleEnded(1);
    //pressure = fmap(adc, 3277,29491,0, PSI100);
    //temp_depth = (float) pressure*100/(1000*9.81);
 #endif

 #if DEBUG_MODE == DEBUG_BB
    //Serial2.println(temp_depth);
 #endif
 
 return adc;
}

/* Discrete Low Pass Filter to reduce noise in signal */
void readPressureFilter()
{
   int16_t temp = readPressure();
   depth = depth + LPF_CONSTANT*(float)(temp -depth);
}

float readTempSensor(int8_t addr)
{
    int16_t reading = 0;
    int8_t store = 0;
    float temp = 0 ;
    Wire.beginTransmission(addr);
    Wire.requestFrom(addr,2);
    if(2 <= Wire.available())    // if two bytes were received
    {
        reading = Wire.read();  // receive high byte (overwrites previous reading)
        reading = reading << 4;    // shift high byte to be high 8 bits
        store |= Wire.read() >> 4; // receive low byte as lower 8 bits
        reading |= store;
        temp = (float) reading*0.0625;
 #if DEBUG_MODE == DEBUG_BB
        //reading >>= 4;
      //  Serial2.println(temp,4);   // print the reading
       #endif
        return temp;
      } else return 0;
}

void readTemperature()
{
    temp1 = readTempSensor(TempAddr1);
    temp2 = readTempSensor(TempAddr2);
    temp3 = readTempSensor(TempAddr3);

    env_msg.Temp0 = temp1;
    env_msg.Temp1 = temp2;
    env_msg.Temp2 = temp3;
 #if DEBUG_MODE == DEBUG_BB
        Serial2.println();
 #endif
}

void readWater()
{
    env_msg.WaterDetA = 1-digitalRead(WaterPin1);
    env_msg.WaterDetB = 1-digitalRead(WaterPin2);
    env_msg.WaterDetC = 1-digitalRead(WaterPin3);
    //turn-on LED when water is detected
    if (env_msg.WaterDetA==1 || env_msg.WaterDetB==1 || env_msg.WaterDetC==1)
        digitalWrite(waterLedPin,HIGH);
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

void getThrusterSpeed(const bbauv_msgs::thruster &msg)
{
    thrusterSpeed=msg;
}
