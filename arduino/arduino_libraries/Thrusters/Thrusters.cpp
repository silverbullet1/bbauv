#include <Arduino.h>
#include <Servo.h>
#include <Thrusters.h>

Thrusters::Thrusters(int pin1,int pin2)
{
  motor1 = pin1;
  motor2 = pin2;
}

void Thrusters::acc(int throttleOld, int throttle)
{
  
  int i;
  int difference = throttle - throttleOld;
  if (difference > 0)
  {
	
	for(i=throttleOld;i <= throttle - (throttle%10) - 10; i=i+10)
	{
          esc1.writeMicroseconds(i);
          esc2.writeMicroseconds(i);
  	      delay(10);
	        Serial.println(i);
	}
	for(i;i <= throttle; i++)
	{
          Serial.println(i);
          esc1.writeMicroseconds(i);
          esc2.writeMicroseconds(i);
  	      delay(10);
	}
   }
  else if (difference < 0)
  {
	for(i=throttleOld;i >= throttle - (throttle%10) + 20; i=i-10)
	{
          Serial.println(i);
	  esc1.writeMicroseconds(i);
    esc2.writeMicroseconds(i);
  	  delay(10);
	}
	for(i;i >= throttle; i=i-1)
	{
          Serial.println(i);
          esc1.writeMicroseconds(i);
          esc2.writeMicroseconds(i);
  	  delay(10);
	}
   }


   return;
}

void Thrusters::mov(int input)
{
  esc1.attach(motor1);
  esc2.attach(motor2);
  int throttle, throttleOld;

  throttleOld = esc1.readMicroseconds();
  if(input > 0)    throttle = thrusterForward(input);
  if(input < 0)    throttle = thrusterReverse(input);
  if(input == 0)   throttle = thrusterStop();
  
  acc(throttleOld, throttle);
  return;
}

int Thrusters::thrusterForward(int input)
{
  int throttle = map(input, 1, 3200, 1570, 2000);
  return throttle;
}

int Thrusters::thrusterReverse(int input)
{
  int throttle = map(input, -3200, -1, 1000, 1430);
  return throttle;
}

int Thrusters::thrusterStop(void)
{
  return 1500;
}