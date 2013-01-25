#ifndef _ROS_bbauv_msgs_log_h
#define _ROS_bbauv_msgs_log_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bbauv_msgs
{

  class log : public ros::Msg
  {
    public:
      float cmdX;
      float cmdY;
      float cmdZ;
      float cmdYaw;
      float yaw;
      float pitch;
      float roll;
      float depth;
      float Ax;
      float Ay;
      float Az;
      int16_t speed1;
      int16_t speed2;
      int16_t speed3;
      int16_t speed4;
      int16_t speed5;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_cmdX;
      u_cmdX.real = this->cmdX;
      *(outbuffer + offset + 0) = (u_cmdX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmdX.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmdX.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmdX.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmdX);
      union {
        float real;
        uint32_t base;
      } u_cmdY;
      u_cmdY.real = this->cmdY;
      *(outbuffer + offset + 0) = (u_cmdY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmdY.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmdY.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmdY.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmdY);
      union {
        float real;
        uint32_t base;
      } u_cmdZ;
      u_cmdZ.real = this->cmdZ;
      *(outbuffer + offset + 0) = (u_cmdZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmdZ.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmdZ.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmdZ.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmdZ);
      union {
        float real;
        uint32_t base;
      } u_cmdYaw;
      u_cmdYaw.real = this->cmdYaw;
      *(outbuffer + offset + 0) = (u_cmdYaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmdYaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmdYaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmdYaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmdYaw);
      union {
        float real;
        uint32_t base;
      } u_yaw;
      u_yaw.real = this->yaw;
      *(outbuffer + offset + 0) = (u_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw);
      union {
        float real;
        uint32_t base;
      } u_pitch;
      u_pitch.real = this->pitch;
      *(outbuffer + offset + 0) = (u_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch);
      union {
        float real;
        uint32_t base;
      } u_roll;
      u_roll.real = this->roll;
      *(outbuffer + offset + 0) = (u_roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll);
      union {
        float real;
        uint32_t base;
      } u_depth;
      u_depth.real = this->depth;
      *(outbuffer + offset + 0) = (u_depth.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_depth.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_depth.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_depth.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->depth);
      union {
        float real;
        uint32_t base;
      } u_Ax;
      u_Ax.real = this->Ax;
      *(outbuffer + offset + 0) = (u_Ax.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Ax.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Ax.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Ax.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Ax);
      union {
        float real;
        uint32_t base;
      } u_Ay;
      u_Ay.real = this->Ay;
      *(outbuffer + offset + 0) = (u_Ay.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Ay.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Ay.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Ay.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Ay);
      union {
        float real;
        uint32_t base;
      } u_Az;
      u_Az.real = this->Az;
      *(outbuffer + offset + 0) = (u_Az.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Az.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Az.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Az.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Az);
      union {
        int16_t real;
        uint16_t base;
      } u_speed1;
      u_speed1.real = this->speed1;
      *(outbuffer + offset + 0) = (u_speed1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->speed1);
      union {
        int16_t real;
        uint16_t base;
      } u_speed2;
      u_speed2.real = this->speed2;
      *(outbuffer + offset + 0) = (u_speed2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->speed2);
      union {
        int16_t real;
        uint16_t base;
      } u_speed3;
      u_speed3.real = this->speed3;
      *(outbuffer + offset + 0) = (u_speed3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed3.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->speed3);
      union {
        int16_t real;
        uint16_t base;
      } u_speed4;
      u_speed4.real = this->speed4;
      *(outbuffer + offset + 0) = (u_speed4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed4.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->speed4);
      union {
        int16_t real;
        uint16_t base;
      } u_speed5;
      u_speed5.real = this->speed5;
      *(outbuffer + offset + 0) = (u_speed5.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed5.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->speed5);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_cmdX;
      u_cmdX.base = 0;
      u_cmdX.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmdX.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmdX.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmdX.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmdX = u_cmdX.real;
      offset += sizeof(this->cmdX);
      union {
        float real;
        uint32_t base;
      } u_cmdY;
      u_cmdY.base = 0;
      u_cmdY.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmdY.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmdY.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmdY.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmdY = u_cmdY.real;
      offset += sizeof(this->cmdY);
      union {
        float real;
        uint32_t base;
      } u_cmdZ;
      u_cmdZ.base = 0;
      u_cmdZ.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmdZ.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmdZ.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmdZ.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmdZ = u_cmdZ.real;
      offset += sizeof(this->cmdZ);
      union {
        float real;
        uint32_t base;
      } u_cmdYaw;
      u_cmdYaw.base = 0;
      u_cmdYaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmdYaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmdYaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmdYaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmdYaw = u_cmdYaw.real;
      offset += sizeof(this->cmdYaw);
      union {
        float real;
        uint32_t base;
      } u_yaw;
      u_yaw.base = 0;
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw = u_yaw.real;
      offset += sizeof(this->yaw);
      union {
        float real;
        uint32_t base;
      } u_pitch;
      u_pitch.base = 0;
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pitch = u_pitch.real;
      offset += sizeof(this->pitch);
      union {
        float real;
        uint32_t base;
      } u_roll;
      u_roll.base = 0;
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll = u_roll.real;
      offset += sizeof(this->roll);
      union {
        float real;
        uint32_t base;
      } u_depth;
      u_depth.base = 0;
      u_depth.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_depth.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_depth.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_depth.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->depth = u_depth.real;
      offset += sizeof(this->depth);
      union {
        float real;
        uint32_t base;
      } u_Ax;
      u_Ax.base = 0;
      u_Ax.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Ax.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Ax.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Ax.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Ax = u_Ax.real;
      offset += sizeof(this->Ax);
      union {
        float real;
        uint32_t base;
      } u_Ay;
      u_Ay.base = 0;
      u_Ay.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Ay.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Ay.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Ay.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Ay = u_Ay.real;
      offset += sizeof(this->Ay);
      union {
        float real;
        uint32_t base;
      } u_Az;
      u_Az.base = 0;
      u_Az.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Az.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Az.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Az.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Az = u_Az.real;
      offset += sizeof(this->Az);
      union {
        int16_t real;
        uint16_t base;
      } u_speed1;
      u_speed1.base = 0;
      u_speed1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->speed1 = u_speed1.real;
      offset += sizeof(this->speed1);
      union {
        int16_t real;
        uint16_t base;
      } u_speed2;
      u_speed2.base = 0;
      u_speed2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->speed2 = u_speed2.real;
      offset += sizeof(this->speed2);
      union {
        int16_t real;
        uint16_t base;
      } u_speed3;
      u_speed3.base = 0;
      u_speed3.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed3.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->speed3 = u_speed3.real;
      offset += sizeof(this->speed3);
      union {
        int16_t real;
        uint16_t base;
      } u_speed4;
      u_speed4.base = 0;
      u_speed4.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed4.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->speed4 = u_speed4.real;
      offset += sizeof(this->speed4);
      union {
        int16_t real;
        uint16_t base;
      } u_speed5;
      u_speed5.base = 0;
      u_speed5.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed5.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->speed5 = u_speed5.real;
      offset += sizeof(this->speed5);
     return offset;
    }

    const char * getType(){ return "bbauv_msgs/log"; };
    const char * getMD5(){ return "e7b988fcc49d16f9272324462efde8cb"; };

  };

}
#endif