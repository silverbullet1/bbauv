#ifndef _ROS_bbauv_msgs_pid_const_h
#define _ROS_bbauv_msgs_pid_const_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bbauv_msgs
{

  class pid_const : public ros::Msg
  {
    public:
      float depth_kp;
      float depth_ki;
      float depth_kd;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_depth_kp;
      u_depth_kp.real = this->depth_kp;
      *(outbuffer + offset + 0) = (u_depth_kp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_depth_kp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_depth_kp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_depth_kp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->depth_kp);
      union {
        float real;
        uint32_t base;
      } u_depth_ki;
      u_depth_ki.real = this->depth_ki;
      *(outbuffer + offset + 0) = (u_depth_ki.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_depth_ki.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_depth_ki.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_depth_ki.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->depth_ki);
      union {
        float real;
        uint32_t base;
      } u_depth_kd;
      u_depth_kd.real = this->depth_kd;
      *(outbuffer + offset + 0) = (u_depth_kd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_depth_kd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_depth_kd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_depth_kd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->depth_kd);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_depth_kp;
      u_depth_kp.base = 0;
      u_depth_kp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_depth_kp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_depth_kp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_depth_kp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->depth_kp = u_depth_kp.real;
      offset += sizeof(this->depth_kp);
      union {
        float real;
        uint32_t base;
      } u_depth_ki;
      u_depth_ki.base = 0;
      u_depth_ki.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_depth_ki.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_depth_ki.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_depth_ki.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->depth_ki = u_depth_ki.real;
      offset += sizeof(this->depth_ki);
      union {
        float real;
        uint32_t base;
      } u_depth_kd;
      u_depth_kd.base = 0;
      u_depth_kd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_depth_kd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_depth_kd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_depth_kd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->depth_kd = u_depth_kd.real;
      offset += sizeof(this->depth_kd);
     return offset;
    }

    const char * getType(){ return "bbauv_msgs/pid_const"; };
    const char * getMD5(){ return "afe3e535320fff747efc397871d1729a"; };

  };

}
#endif