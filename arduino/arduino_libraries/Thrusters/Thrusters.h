#ifndef Thrusters_h
#define Thrusters_h

#include "Arduino.h"
#include <Servo.h>

class Thrusters
{
 public:
 Thrusters(int pin1, int pin2);
 void mov(int input);
 

 private: 
 void acc(int input1, int input2);
 int thrusterForward(int input);
 int thrusterReverse(int input);
 int thrusterStop(void);
 Servo esc1, esc2;
 int motor1, motor2;
};

#endif