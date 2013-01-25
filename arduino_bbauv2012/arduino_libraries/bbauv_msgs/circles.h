#ifndef _ROS_bbauv_msgs_circles_h
#define _ROS_bbauv_msgs_circles_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bbauv_msgs
{

  class circles : public ros::Msg
  {
    public:
      uint8_t x_length;
      float st_x;
      float * x;
      uint8_t y_length;
      float st_y;
      float * y;
      uint8_t radius_length;
      float st_radius;
      float * radius;
      uint32_t size;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = x_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < x_length; i++){
      union {
        float real;
        uint32_t base;
      } u_xi;
      u_xi.real = this->x[i];
      *(outbuffer + offset + 0) = (u_xi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x[i]);
      }
      *(outbuffer + offset++) = y_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < y_length; i++){
      union {
        float real;
        uint32_t base;
      } u_yi;
      u_yi.real = this->y[i];
      *(outbuffer + offset + 0) = (u_yi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y[i]);
      }
      *(outbuffer + offset++) = radius_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < radius_length; i++){
      union {
        float real;
        uint32_t base;
      } u_radiusi;
      u_radiusi.real = this->radius[i];
      *(outbuffer + offset + 0) = (u_radiusi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_radiusi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_radiusi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_radiusi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->radius[i]);
      }
      *(outbuffer + offset + 0) = (this->size >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->size >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->size >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->size >> (8 * 3)) & 0xFF;
      offset += sizeof(this->size);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t x_lengthT = *(inbuffer + offset++);
      if(x_lengthT > x_length)
        this->x = (float*)realloc(this->x, x_lengthT * sizeof(float));
      offset += 3;
      x_length = x_lengthT;
      for( uint8_t i = 0; i < x_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_x;
      u_st_x.base = 0;
      u_st_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_x = u_st_x.real;
      offset += sizeof(this->st_x);
        memcpy( &(this->x[i]), &(this->st_x), sizeof(float));
      }
      uint8_t y_lengthT = *(inbuffer + offset++);
      if(y_lengthT > y_length)
        this->y = (float*)realloc(this->y, y_lengthT * sizeof(float));
      offset += 3;
      y_length = y_lengthT;
      for( uint8_t i = 0; i < y_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_y;
      u_st_y.base = 0;
      u_st_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_y = u_st_y.real;
      offset += sizeof(this->st_y);
        memcpy( &(this->y[i]), &(this->st_y), sizeof(float));
      }
      uint8_t radius_lengthT = *(inbuffer + offset++);
      if(radius_lengthT > radius_length)
        this->radius = (float*)realloc(this->radius, radius_lengthT * sizeof(float));
      offset += 3;
      radius_length = radius_lengthT;
      for( uint8_t i = 0; i < radius_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_radius;
      u_st_radius.base = 0;
      u_st_radius.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_radius.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_radius.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_radius.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_radius = u_st_radius.real;
      offset += sizeof(this->st_radius);
        memcpy( &(this->radius[i]), &(this->st_radius), sizeof(float));
      }
      this->size =  ((uint32_t) (*(inbuffer + offset)));
      this->size |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->size |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->size |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->size);
     return offset;
    }

    const char * getType(){ return "bbauv_msgs/circles"; };
    const char * getMD5(){ return "0405909a37e53d3658a36ae2e1dee5fb"; };

  };

}
#endif