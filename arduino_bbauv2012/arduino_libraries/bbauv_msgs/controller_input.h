#ifndef _ROS_bbauv_msgs_controller_input_h
#define _ROS_bbauv_msgs_controller_input_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bbauv_msgs
{

  class controller_input : public ros::Msg
  {
    public:
      float depth_input;
      float depth_setpoint;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_depth_input;
      u_depth_input.real = this->depth_input;
      *(outbuffer + offset + 0) = (u_depth_input.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_depth_input.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_depth_input.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_depth_input.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->depth_input);
      union {
        float real;
        uint32_t base;
      } u_depth_setpoint;
      u_depth_setpoint.real = this->depth_setpoint;
      *(outbuffer + offset + 0) = (u_depth_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_depth_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_depth_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_depth_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->depth_setpoint);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_depth_input;
      u_depth_input.base = 0;
      u_depth_input.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_depth_input.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_depth_input.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_depth_input.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->depth_input = u_depth_input.real;
      offset += sizeof(this->depth_input);
      union {
        float real;
        uint32_t base;
      } u_depth_setpoint;
      u_depth_setpoint.base = 0;
      u_depth_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_depth_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_depth_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_depth_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->depth_setpoint = u_depth_setpoint.real;
      offset += sizeof(this->depth_setpoint);
     return offset;
    }

    const char * getType(){ return "bbauv_msgs/controller_input"; };
    const char * getMD5(){ return "0a17330d0f814cf49b912511de7be02a"; };

  };

}
#endif