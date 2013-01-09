#ifndef _ROS_bbauv_msgs_compass_data_h
#define _ROS_bbauv_msgs_compass_data_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bbauv_msgs
{

  class compass_data : public ros::Msg
  {
    public:
      float yaw;
      float pitch;
      float roll;
      float temperature;
      float Ax;
      float Ay;
      float Az;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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
      } u_temperature;
      u_temperature.real = this->temperature;
      *(outbuffer + offset + 0) = (u_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temperature);
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
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
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
      } u_temperature;
      u_temperature.base = 0;
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temperature = u_temperature.real;
      offset += sizeof(this->temperature);
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
     return offset;
    }

    const char * getType(){ return "bbauv_msgs/compass_data"; };
    const char * getMD5(){ return "b93be8a36a8380f5876e9c37f8493a2f"; };

  };

}
#endif