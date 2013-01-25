#ifndef _ROS_SERVICE_PID_h
#define _ROS_SERVICE_PID_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bbauv_msgs
{

static const char PID[] = "bbauv_msgs/PID";

  class PIDRequest : public ros::Msg
  {
    public:
      float gain_p;
      float gain_i;
      float gain_d;
      float current_val;
      float target_val;
      float previous_error;
      float previous_integrator_val;
      float integral_term_min;
      float integral_term_max;
      float dt;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      int32_t * val_gain_p = (long *) &(this->gain_p);
      int32_t exp_gain_p = (((*val_gain_p)>>23)&255);
      if(exp_gain_p != 0)
        exp_gain_p += 1023-127;
      int32_t sig_gain_p = *val_gain_p;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_gain_p<<5) & 0xff;
      *(outbuffer + offset++) = (sig_gain_p>>3) & 0xff;
      *(outbuffer + offset++) = (sig_gain_p>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_gain_p<<4) & 0xF0) | ((sig_gain_p>>19)&0x0F);
      *(outbuffer + offset++) = (exp_gain_p>>4) & 0x7F;
      if(this->gain_p < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_gain_i = (long *) &(this->gain_i);
      int32_t exp_gain_i = (((*val_gain_i)>>23)&255);
      if(exp_gain_i != 0)
        exp_gain_i += 1023-127;
      int32_t sig_gain_i = *val_gain_i;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_gain_i<<5) & 0xff;
      *(outbuffer + offset++) = (sig_gain_i>>3) & 0xff;
      *(outbuffer + offset++) = (sig_gain_i>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_gain_i<<4) & 0xF0) | ((sig_gain_i>>19)&0x0F);
      *(outbuffer + offset++) = (exp_gain_i>>4) & 0x7F;
      if(this->gain_i < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_gain_d = (long *) &(this->gain_d);
      int32_t exp_gain_d = (((*val_gain_d)>>23)&255);
      if(exp_gain_d != 0)
        exp_gain_d += 1023-127;
      int32_t sig_gain_d = *val_gain_d;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_gain_d<<5) & 0xff;
      *(outbuffer + offset++) = (sig_gain_d>>3) & 0xff;
      *(outbuffer + offset++) = (sig_gain_d>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_gain_d<<4) & 0xF0) | ((sig_gain_d>>19)&0x0F);
      *(outbuffer + offset++) = (exp_gain_d>>4) & 0x7F;
      if(this->gain_d < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_current_val = (long *) &(this->current_val);
      int32_t exp_current_val = (((*val_current_val)>>23)&255);
      if(exp_current_val != 0)
        exp_current_val += 1023-127;
      int32_t sig_current_val = *val_current_val;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_current_val<<5) & 0xff;
      *(outbuffer + offset++) = (sig_current_val>>3) & 0xff;
      *(outbuffer + offset++) = (sig_current_val>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_current_val<<4) & 0xF0) | ((sig_current_val>>19)&0x0F);
      *(outbuffer + offset++) = (exp_current_val>>4) & 0x7F;
      if(this->current_val < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_target_val = (long *) &(this->target_val);
      int32_t exp_target_val = (((*val_target_val)>>23)&255);
      if(exp_target_val != 0)
        exp_target_val += 1023-127;
      int32_t sig_target_val = *val_target_val;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_target_val<<5) & 0xff;
      *(outbuffer + offset++) = (sig_target_val>>3) & 0xff;
      *(outbuffer + offset++) = (sig_target_val>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_target_val<<4) & 0xF0) | ((sig_target_val>>19)&0x0F);
      *(outbuffer + offset++) = (exp_target_val>>4) & 0x7F;
      if(this->target_val < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_previous_error = (long *) &(this->previous_error);
      int32_t exp_previous_error = (((*val_previous_error)>>23)&255);
      if(exp_previous_error != 0)
        exp_previous_error += 1023-127;
      int32_t sig_previous_error = *val_previous_error;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_previous_error<<5) & 0xff;
      *(outbuffer + offset++) = (sig_previous_error>>3) & 0xff;
      *(outbuffer + offset++) = (sig_previous_error>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_previous_error<<4) & 0xF0) | ((sig_previous_error>>19)&0x0F);
      *(outbuffer + offset++) = (exp_previous_error>>4) & 0x7F;
      if(this->previous_error < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_previous_integrator_val = (long *) &(this->previous_integrator_val);
      int32_t exp_previous_integrator_val = (((*val_previous_integrator_val)>>23)&255);
      if(exp_previous_integrator_val != 0)
        exp_previous_integrator_val += 1023-127;
      int32_t sig_previous_integrator_val = *val_previous_integrator_val;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_previous_integrator_val<<5) & 0xff;
      *(outbuffer + offset++) = (sig_previous_integrator_val>>3) & 0xff;
      *(outbuffer + offset++) = (sig_previous_integrator_val>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_previous_integrator_val<<4) & 0xF0) | ((sig_previous_integrator_val>>19)&0x0F);
      *(outbuffer + offset++) = (exp_previous_integrator_val>>4) & 0x7F;
      if(this->previous_integrator_val < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_integral_term_min = (long *) &(this->integral_term_min);
      int32_t exp_integral_term_min = (((*val_integral_term_min)>>23)&255);
      if(exp_integral_term_min != 0)
        exp_integral_term_min += 1023-127;
      int32_t sig_integral_term_min = *val_integral_term_min;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_integral_term_min<<5) & 0xff;
      *(outbuffer + offset++) = (sig_integral_term_min>>3) & 0xff;
      *(outbuffer + offset++) = (sig_integral_term_min>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_integral_term_min<<4) & 0xF0) | ((sig_integral_term_min>>19)&0x0F);
      *(outbuffer + offset++) = (exp_integral_term_min>>4) & 0x7F;
      if(this->integral_term_min < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_integral_term_max = (long *) &(this->integral_term_max);
      int32_t exp_integral_term_max = (((*val_integral_term_max)>>23)&255);
      if(exp_integral_term_max != 0)
        exp_integral_term_max += 1023-127;
      int32_t sig_integral_term_max = *val_integral_term_max;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_integral_term_max<<5) & 0xff;
      *(outbuffer + offset++) = (sig_integral_term_max>>3) & 0xff;
      *(outbuffer + offset++) = (sig_integral_term_max>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_integral_term_max<<4) & 0xF0) | ((sig_integral_term_max>>19)&0x0F);
      *(outbuffer + offset++) = (exp_integral_term_max>>4) & 0x7F;
      if(this->integral_term_max < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_dt = (long *) &(this->dt);
      int32_t exp_dt = (((*val_dt)>>23)&255);
      if(exp_dt != 0)
        exp_dt += 1023-127;
      int32_t sig_dt = *val_dt;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_dt<<5) & 0xff;
      *(outbuffer + offset++) = (sig_dt>>3) & 0xff;
      *(outbuffer + offset++) = (sig_dt>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_dt<<4) & 0xF0) | ((sig_dt>>19)&0x0F);
      *(outbuffer + offset++) = (exp_dt>>4) & 0x7F;
      if(this->dt < 0) *(outbuffer + offset -1) |= 0x80;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t * val_gain_p = (uint32_t*) &(this->gain_p);
      offset += 3;
      *val_gain_p = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_gain_p |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_gain_p |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_gain_p |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_gain_p = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_gain_p |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_gain_p !=0)
        *val_gain_p |= ((exp_gain_p)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->gain_p = -this->gain_p;
      uint32_t * val_gain_i = (uint32_t*) &(this->gain_i);
      offset += 3;
      *val_gain_i = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_gain_i |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_gain_i |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_gain_i |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_gain_i = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_gain_i |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_gain_i !=0)
        *val_gain_i |= ((exp_gain_i)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->gain_i = -this->gain_i;
      uint32_t * val_gain_d = (uint32_t*) &(this->gain_d);
      offset += 3;
      *val_gain_d = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_gain_d |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_gain_d |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_gain_d |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_gain_d = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_gain_d |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_gain_d !=0)
        *val_gain_d |= ((exp_gain_d)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->gain_d = -this->gain_d;
      uint32_t * val_current_val = (uint32_t*) &(this->current_val);
      offset += 3;
      *val_current_val = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_current_val |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_current_val |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_current_val |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_current_val = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_current_val |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_current_val !=0)
        *val_current_val |= ((exp_current_val)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->current_val = -this->current_val;
      uint32_t * val_target_val = (uint32_t*) &(this->target_val);
      offset += 3;
      *val_target_val = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_target_val |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_target_val |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_target_val |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_target_val = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_target_val |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_target_val !=0)
        *val_target_val |= ((exp_target_val)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->target_val = -this->target_val;
      uint32_t * val_previous_error = (uint32_t*) &(this->previous_error);
      offset += 3;
      *val_previous_error = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_previous_error |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_previous_error |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_previous_error |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_previous_error = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_previous_error |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_previous_error !=0)
        *val_previous_error |= ((exp_previous_error)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->previous_error = -this->previous_error;
      uint32_t * val_previous_integrator_val = (uint32_t*) &(this->previous_integrator_val);
      offset += 3;
      *val_previous_integrator_val = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_previous_integrator_val |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_previous_integrator_val |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_previous_integrator_val |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_previous_integrator_val = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_previous_integrator_val |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_previous_integrator_val !=0)
        *val_previous_integrator_val |= ((exp_previous_integrator_val)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->previous_integrator_val = -this->previous_integrator_val;
      uint32_t * val_integral_term_min = (uint32_t*) &(this->integral_term_min);
      offset += 3;
      *val_integral_term_min = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_integral_term_min |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_integral_term_min |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_integral_term_min |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_integral_term_min = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_integral_term_min |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_integral_term_min !=0)
        *val_integral_term_min |= ((exp_integral_term_min)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->integral_term_min = -this->integral_term_min;
      uint32_t * val_integral_term_max = (uint32_t*) &(this->integral_term_max);
      offset += 3;
      *val_integral_term_max = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_integral_term_max |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_integral_term_max |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_integral_term_max |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_integral_term_max = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_integral_term_max |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_integral_term_max !=0)
        *val_integral_term_max |= ((exp_integral_term_max)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->integral_term_max = -this->integral_term_max;
      uint32_t * val_dt = (uint32_t*) &(this->dt);
      offset += 3;
      *val_dt = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_dt |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_dt |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_dt |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_dt = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_dt |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_dt !=0)
        *val_dt |= ((exp_dt)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->dt = -this->dt;
     return offset;
    }

    const char * getType(){ return PID; };
    const char * getMD5(){ return "4d3f19ad468f6225a1cf11a4c82adf57"; };

  };

  class PIDResponse : public ros::Msg
  {
    public:
      float current_integrator_val;
      float current_error;
      float u;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      int32_t * val_current_integrator_val = (long *) &(this->current_integrator_val);
      int32_t exp_current_integrator_val = (((*val_current_integrator_val)>>23)&255);
      if(exp_current_integrator_val != 0)
        exp_current_integrator_val += 1023-127;
      int32_t sig_current_integrator_val = *val_current_integrator_val;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_current_integrator_val<<5) & 0xff;
      *(outbuffer + offset++) = (sig_current_integrator_val>>3) & 0xff;
      *(outbuffer + offset++) = (sig_current_integrator_val>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_current_integrator_val<<4) & 0xF0) | ((sig_current_integrator_val>>19)&0x0F);
      *(outbuffer + offset++) = (exp_current_integrator_val>>4) & 0x7F;
      if(this->current_integrator_val < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_current_error = (long *) &(this->current_error);
      int32_t exp_current_error = (((*val_current_error)>>23)&255);
      if(exp_current_error != 0)
        exp_current_error += 1023-127;
      int32_t sig_current_error = *val_current_error;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_current_error<<5) & 0xff;
      *(outbuffer + offset++) = (sig_current_error>>3) & 0xff;
      *(outbuffer + offset++) = (sig_current_error>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_current_error<<4) & 0xF0) | ((sig_current_error>>19)&0x0F);
      *(outbuffer + offset++) = (exp_current_error>>4) & 0x7F;
      if(this->current_error < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_u = (long *) &(this->u);
      int32_t exp_u = (((*val_u)>>23)&255);
      if(exp_u != 0)
        exp_u += 1023-127;
      int32_t sig_u = *val_u;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_u<<5) & 0xff;
      *(outbuffer + offset++) = (sig_u>>3) & 0xff;
      *(outbuffer + offset++) = (sig_u>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_u<<4) & 0xF0) | ((sig_u>>19)&0x0F);
      *(outbuffer + offset++) = (exp_u>>4) & 0x7F;
      if(this->u < 0) *(outbuffer + offset -1) |= 0x80;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t * val_current_integrator_val = (uint32_t*) &(this->current_integrator_val);
      offset += 3;
      *val_current_integrator_val = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_current_integrator_val |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_current_integrator_val |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_current_integrator_val |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_current_integrator_val = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_current_integrator_val |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_current_integrator_val !=0)
        *val_current_integrator_val |= ((exp_current_integrator_val)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->current_integrator_val = -this->current_integrator_val;
      uint32_t * val_current_error = (uint32_t*) &(this->current_error);
      offset += 3;
      *val_current_error = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_current_error |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_current_error |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_current_error |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_current_error = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_current_error |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_current_error !=0)
        *val_current_error |= ((exp_current_error)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->current_error = -this->current_error;
      uint32_t * val_u = (uint32_t*) &(this->u);
      offset += 3;
      *val_u = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_u |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_u |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_u |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_u = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_u |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_u !=0)
        *val_u |= ((exp_u)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->u = -this->u;
     return offset;
    }

    const char * getType(){ return PID; };
    const char * getMD5(){ return "12d2a7003b0a8e0851c9897028086206"; };

  };

  class PID {
    public:
    typedef PIDRequest Request;
    typedef PIDResponse Response;
  };

}
#endif