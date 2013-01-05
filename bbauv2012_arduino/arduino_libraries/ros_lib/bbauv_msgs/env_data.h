#ifndef _ROS_bbauv_msgs_env_data_h
#define _ROS_bbauv_msgs_env_data_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bbauv_msgs
{

  class env_data : public ros::Msg
  {
    public:
      float Temp0;
      float Temp1;
      float Temp2;
      float Depth;
      float WaterDetA;
      float WaterDetB;
      float WaterDetC;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_Temp0;
      u_Temp0.real = this->Temp0;
      *(outbuffer + offset + 0) = (u_Temp0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Temp0.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Temp0.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Temp0.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Temp0);
      union {
        float real;
        uint32_t base;
      } u_Temp1;
      u_Temp1.real = this->Temp1;
      *(outbuffer + offset + 0) = (u_Temp1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Temp1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Temp1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Temp1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Temp1);
      union {
        float real;
        uint32_t base;
      } u_Temp2;
      u_Temp2.real = this->Temp2;
      *(outbuffer + offset + 0) = (u_Temp2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Temp2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Temp2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Temp2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Temp2);
      union {
        float real;
        uint32_t base;
      } u_Depth;
      u_Depth.real = this->Depth;
      *(outbuffer + offset + 0) = (u_Depth.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Depth.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Depth.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Depth.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Depth);
      union {
        float real;
        uint32_t base;
      } u_WaterDetA;
      u_WaterDetA.real = this->WaterDetA;
      *(outbuffer + offset + 0) = (u_WaterDetA.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_WaterDetA.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_WaterDetA.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_WaterDetA.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->WaterDetA);
      union {
        float real;
        uint32_t base;
      } u_WaterDetB;
      u_WaterDetB.real = this->WaterDetB;
      *(outbuffer + offset + 0) = (u_WaterDetB.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_WaterDetB.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_WaterDetB.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_WaterDetB.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->WaterDetB);
      union {
        float real;
        uint32_t base;
      } u_WaterDetC;
      u_WaterDetC.real = this->WaterDetC;
      *(outbuffer + offset + 0) = (u_WaterDetC.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_WaterDetC.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_WaterDetC.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_WaterDetC.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->WaterDetC);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_Temp0;
      u_Temp0.base = 0;
      u_Temp0.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Temp0.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Temp0.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Temp0.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Temp0 = u_Temp0.real;
      offset += sizeof(this->Temp0);
      union {
        float real;
        uint32_t base;
      } u_Temp1;
      u_Temp1.base = 0;
      u_Temp1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Temp1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Temp1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Temp1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Temp1 = u_Temp1.real;
      offset += sizeof(this->Temp1);
      union {
        float real;
        uint32_t base;
      } u_Temp2;
      u_Temp2.base = 0;
      u_Temp2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Temp2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Temp2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Temp2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Temp2 = u_Temp2.real;
      offset += sizeof(this->Temp2);
      union {
        float real;
        uint32_t base;
      } u_Depth;
      u_Depth.base = 0;
      u_Depth.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Depth.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Depth.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Depth.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Depth = u_Depth.real;
      offset += sizeof(this->Depth);
      union {
        float real;
        uint32_t base;
      } u_WaterDetA;
      u_WaterDetA.base = 0;
      u_WaterDetA.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_WaterDetA.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_WaterDetA.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_WaterDetA.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->WaterDetA = u_WaterDetA.real;
      offset += sizeof(this->WaterDetA);
      union {
        float real;
        uint32_t base;
      } u_WaterDetB;
      u_WaterDetB.base = 0;
      u_WaterDetB.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_WaterDetB.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_WaterDetB.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_WaterDetB.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->WaterDetB = u_WaterDetB.real;
      offset += sizeof(this->WaterDetB);
      union {
        float real;
        uint32_t base;
      } u_WaterDetC;
      u_WaterDetC.base = 0;
      u_WaterDetC.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_WaterDetC.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_WaterDetC.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_WaterDetC.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->WaterDetC = u_WaterDetC.real;
      offset += sizeof(this->WaterDetC);
     return offset;
    }

    const char * getType(){ return "bbauv_msgs/env_data"; };
    const char * getMD5(){ return "5ea35f20e21f1863e1e58052359d3145"; };

  };

}
#endif