#ifndef _ROS_SERVICE_SetCur_h
#define _ROS_SERVICE_SetCur_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace velvet_msgs
{

static const char SETCUR[] = "velvet_msgs/SetCur";

  class SetCurRequest : public ros::Msg
  {
    public:
      int16_t id;
      float curr;
      float time;

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
      union {
        float real;
        uint32_t base;
      } u_curr;
      u_curr.real = this->curr;
      *(outbuffer + offset + 0) = (u_curr.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_curr.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_curr.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_curr.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->curr);
      union {
        float real;
        uint32_t base;
      } u_time;
      u_time.real = this->time;
      *(outbuffer + offset + 0) = (u_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time);
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
      union {
        float real;
        uint32_t base;
      } u_curr;
      u_curr.base = 0;
      u_curr.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_curr.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_curr.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_curr.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->curr = u_curr.real;
      offset += sizeof(this->curr);
      union {
        float real;
        uint32_t base;
      } u_time;
      u_time.base = 0;
      u_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->time = u_time.real;
      offset += sizeof(this->time);
     return offset;
    }

    const char * getType(){ return SETCUR; };
    const char * getMD5(){ return "a28d6fe5046ed93c94709e512446e6a8"; };

  };

  class SetCurResponse : public ros::Msg
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

    const char * getType(){ return SETCUR; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetCur {
    public:
    typedef SetCurRequest Request;
    typedef SetCurResponse Response;
  };

}
#endif
