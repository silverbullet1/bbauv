#ifndef _ROS_bbauv_msgs_controller_param_h
#define _ROS_bbauv_msgs_controller_param_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bbauv_msgs
{

  class controller_param : public ros::Msg
  {
    public:
      float depth_kp;
      float depth_ki;
      float depth_kd;
      float heading_kp;
      float heading_ki;
      float heading_kd;
      float ratio_t1;
      float ratio_t2;
      float ratio_t3;
      float ratio_t4;
      float ratio_t5;
      float ratio_t6;
      bool mode;

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
      union {
        float real;
        uint32_t base;
      } u_heading_kp;
      u_heading_kp.real = this->heading_kp;
      *(outbuffer + offset + 0) = (u_heading_kp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heading_kp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heading_kp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heading_kp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->heading_kp);
      union {
        float real;
        uint32_t base;
      } u_heading_ki;
      u_heading_ki.real = this->heading_ki;
      *(outbuffer + offset + 0) = (u_heading_ki.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heading_ki.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heading_ki.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heading_ki.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->heading_ki);
      union {
        float real;
        uint32_t base;
      } u_heading_kd;
      u_heading_kd.real = this->heading_kd;
      *(outbuffer + offset + 0) = (u_heading_kd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heading_kd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heading_kd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heading_kd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->heading_kd);
      union {
        float real;
        uint32_t base;
      } u_ratio_t1;
      u_ratio_t1.real = this->ratio_t1;
      *(outbuffer + offset + 0) = (u_ratio_t1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ratio_t1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ratio_t1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ratio_t1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ratio_t1);
      union {
        float real;
        uint32_t base;
      } u_ratio_t2;
      u_ratio_t2.real = this->ratio_t2;
      *(outbuffer + offset + 0) = (u_ratio_t2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ratio_t2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ratio_t2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ratio_t2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ratio_t2);
      union {
        float real;
        uint32_t base;
      } u_ratio_t3;
      u_ratio_t3.real = this->ratio_t3;
      *(outbuffer + offset + 0) = (u_ratio_t3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ratio_t3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ratio_t3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ratio_t3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ratio_t3);
      union {
        float real;
        uint32_t base;
      } u_ratio_t4;
      u_ratio_t4.real = this->ratio_t4;
      *(outbuffer + offset + 0) = (u_ratio_t4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ratio_t4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ratio_t4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ratio_t4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ratio_t4);
      union {
        float real;
        uint32_t base;
      } u_ratio_t5;
      u_ratio_t5.real = this->ratio_t5;
      *(outbuffer + offset + 0) = (u_ratio_t5.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ratio_t5.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ratio_t5.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ratio_t5.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ratio_t5);
      union {
        float real;
        uint32_t base;
      } u_ratio_t6;
      u_ratio_t6.real = this->ratio_t6;
      *(outbuffer + offset + 0) = (u_ratio_t6.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ratio_t6.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ratio_t6.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ratio_t6.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ratio_t6);
      union {
        bool real;
        uint8_t base;
      } u_mode;
      u_mode.real = this->mode;
      *(outbuffer + offset + 0) = (u_mode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
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
      union {
        float real;
        uint32_t base;
      } u_heading_kp;
      u_heading_kp.base = 0;
      u_heading_kp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_heading_kp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_heading_kp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_heading_kp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->heading_kp = u_heading_kp.real;
      offset += sizeof(this->heading_kp);
      union {
        float real;
        uint32_t base;
      } u_heading_ki;
      u_heading_ki.base = 0;
      u_heading_ki.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_heading_ki.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_heading_ki.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_heading_ki.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->heading_ki = u_heading_ki.real;
      offset += sizeof(this->heading_ki);
      union {
        float real;
        uint32_t base;
      } u_heading_kd;
      u_heading_kd.base = 0;
      u_heading_kd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_heading_kd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_heading_kd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_heading_kd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->heading_kd = u_heading_kd.real;
      offset += sizeof(this->heading_kd);
      union {
        float real;
        uint32_t base;
      } u_ratio_t1;
      u_ratio_t1.base = 0;
      u_ratio_t1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ratio_t1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ratio_t1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ratio_t1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ratio_t1 = u_ratio_t1.real;
      offset += sizeof(this->ratio_t1);
      union {
        float real;
        uint32_t base;
      } u_ratio_t2;
      u_ratio_t2.base = 0;
      u_ratio_t2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ratio_t2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ratio_t2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ratio_t2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ratio_t2 = u_ratio_t2.real;
      offset += sizeof(this->ratio_t2);
      union {
        float real;
        uint32_t base;
      } u_ratio_t3;
      u_ratio_t3.base = 0;
      u_ratio_t3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ratio_t3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ratio_t3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ratio_t3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ratio_t3 = u_ratio_t3.real;
      offset += sizeof(this->ratio_t3);
      union {
        float real;
        uint32_t base;
      } u_ratio_t4;
      u_ratio_t4.base = 0;
      u_ratio_t4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ratio_t4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ratio_t4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ratio_t4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ratio_t4 = u_ratio_t4.real;
      offset += sizeof(this->ratio_t4);
      union {
        float real;
        uint32_t base;
      } u_ratio_t5;
      u_ratio_t5.base = 0;
      u_ratio_t5.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ratio_t5.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ratio_t5.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ratio_t5.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ratio_t5 = u_ratio_t5.real;
      offset += sizeof(this->ratio_t5);
      union {
        float real;
        uint32_t base;
      } u_ratio_t6;
      u_ratio_t6.base = 0;
      u_ratio_t6.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ratio_t6.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ratio_t6.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ratio_t6.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ratio_t6 = u_ratio_t6.real;
      offset += sizeof(this->ratio_t6);
      union {
        bool real;
        uint8_t base;
      } u_mode;
      u_mode.base = 0;
      u_mode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mode = u_mode.real;
      offset += sizeof(this->mode);
     return offset;
    }

    const char * getType(){ return "bbauv_msgs/controller_param"; };
    const char * getMD5(){ return "5b4c0cd5c36bc74b6d65a62816f0c85f"; };

  };

}
#endif
