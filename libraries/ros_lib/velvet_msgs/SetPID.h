#ifndef _ROS_SERVICE_SetPID_h
#define _ROS_SERVICE_SetPID_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace velvet_msgs
{

static const char SETPID[] = "velvet_msgs/SetPID";

  class SetPIDRequest : public ros::Msg
  {
    public:
      int16_t id;
      uint8_t mode;
      float pval;
      float ival;
      float dval;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->id);
      *(outbuffer + offset + 0) = (this->mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
      union {
        float real;
        uint32_t base;
      } u_pval;
      u_pval.real = this->pval;
      *(outbuffer + offset + 0) = (u_pval.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pval.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pval.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pval.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pval);
      union {
        float real;
        uint32_t base;
      } u_ival;
      u_ival.real = this->ival;
      *(outbuffer + offset + 0) = (u_ival.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ival.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ival.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ival.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ival);
      union {
        float real;
        uint32_t base;
      } u_dval;
      u_dval.real = this->dval;
      *(outbuffer + offset + 0) = (u_dval.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dval.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dval.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dval.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dval);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id = u_id.real;
      offset += sizeof(this->id);
      this->mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mode);
      union {
        float real;
        uint32_t base;
      } u_pval;
      u_pval.base = 0;
      u_pval.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pval.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pval.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pval.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pval = u_pval.real;
      offset += sizeof(this->pval);
      union {
        float real;
        uint32_t base;
      } u_ival;
      u_ival.base = 0;
      u_ival.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ival.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ival.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ival.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ival = u_ival.real;
      offset += sizeof(this->ival);
      union {
        float real;
        uint32_t base;
      } u_dval;
      u_dval.base = 0;
      u_dval.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dval.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dval.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dval.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dval = u_dval.real;
      offset += sizeof(this->dval);
     return offset;
    }

    const char * getType(){ return SETPID; };
    const char * getMD5(){ return "b72c923fb365f1a519560482985ef3b7"; };

  };

  class SetPIDResponse : public ros::Msg
  {
    public:
      bool success;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return SETPID; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetPID {
    public:
    typedef SetPIDRequest Request;
    typedef SetPIDResponse Response;
  };

}
#endif
