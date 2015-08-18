#ifndef _ROS_velvet_msgs_GripperState_h
#define _ROS_velvet_msgs_GripperState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "velvet_msgs/EncoderState.h"
#include "velvet_msgs/CurrentState.h"

namespace velvet_msgs
{

  class GripperState : public ros::Msg
  {
    public:
      velvet_msgs::EncoderState oc;
      velvet_msgs::EncoderState blf;
      velvet_msgs::EncoderState blb;
      velvet_msgs::EncoderState brf;
      velvet_msgs::EncoderState brb;
      velvet_msgs::EncoderState pl;
      velvet_msgs::EncoderState pr;
      velvet_msgs::CurrentState c_oc;
      velvet_msgs::CurrentState c_blf;
      velvet_msgs::CurrentState c_blb;
      velvet_msgs::CurrentState c_brf;
      velvet_msgs::CurrentState c_brb;
      uint8_t oc_mode;
      uint8_t b_mode;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->oc.serialize(outbuffer + offset);
      offset += this->blf.serialize(outbuffer + offset);
      offset += this->blb.serialize(outbuffer + offset);
      offset += this->brf.serialize(outbuffer + offset);
      offset += this->brb.serialize(outbuffer + offset);
      offset += this->pl.serialize(outbuffer + offset);
      offset += this->pr.serialize(outbuffer + offset);
      offset += this->c_oc.serialize(outbuffer + offset);
      offset += this->c_blf.serialize(outbuffer + offset);
      offset += this->c_blb.serialize(outbuffer + offset);
      offset += this->c_brf.serialize(outbuffer + offset);
      offset += this->c_brb.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->oc_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->oc_mode);
      *(outbuffer + offset + 0) = (this->b_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->b_mode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->oc.deserialize(inbuffer + offset);
      offset += this->blf.deserialize(inbuffer + offset);
      offset += this->blb.deserialize(inbuffer + offset);
      offset += this->brf.deserialize(inbuffer + offset);
      offset += this->brb.deserialize(inbuffer + offset);
      offset += this->pl.deserialize(inbuffer + offset);
      offset += this->pr.deserialize(inbuffer + offset);
      offset += this->c_oc.deserialize(inbuffer + offset);
      offset += this->c_blf.deserialize(inbuffer + offset);
      offset += this->c_blb.deserialize(inbuffer + offset);
      offset += this->c_brf.deserialize(inbuffer + offset);
      offset += this->c_brb.deserialize(inbuffer + offset);
      this->oc_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->oc_mode);
      this->b_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->b_mode);
     return offset;
    }

    const char * getType(){ return "velvet_msgs/GripperState"; };
    const char * getMD5(){ return "8bc9b5d0115a904ba1a035d326878073"; };

  };

}
#endif