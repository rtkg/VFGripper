#ifndef _ROS_velvet_msgs_CurrentState_h
#define _ROS_velvet_msgs_CurrentState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace velvet_msgs
{

  class CurrentState : public ros::Msg
  {
    public:
      int16_t id;
      float val;
      float target;

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
      } u_val;
      u_val.real = this->val;
      *(outbuffer + offset + 0) = (u_val.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_val.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_val.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_val.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->val);
      union {
        float real;
        uint32_t base;
      } u_target;
      u_target.real = this->target;
      *(outbuffer + offset + 0) = (u_target.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target);
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
      } u_val;
      u_val.base = 0;
      u_val.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_val.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_val.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_val.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->val = u_val.real;
      offset += sizeof(this->val);
      union {
        float real;
        uint32_t base;
      } u_target;
      u_target.base = 0;
      u_target.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target = u_target.real;
      offset += sizeof(this->target);
     return offset;
    }

    const char * getType(){ return "velvet_msgs/CurrentState"; };
    const char * getMD5(){ return "b83c23363c6ae4abfa4d7ab9d877dab3"; };

  };

}
#endif