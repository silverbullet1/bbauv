#ifndef _ROS_bbauv_msgs_thruster_h
#define _ROS_bbauv_msgs_thruster_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bbauv_msgs
{

  class thruster : public ros::Msg
  {
    public:
      int16_t speed1;
      int16_t speed2;
      int16_t speed3;
      int16_t speed4;
      int16_t speed5;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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

    const char * getType(){ return "bbauv_msgs/thruster"; };
    const char * getMD5(){ return "53a024ea886fdbb1d3759712c38532e7"; };

  };

}
#endif