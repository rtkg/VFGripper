#ifndef _ROS_SERVICE_PlanGrasp_h
#define _ROS_SERVICE_PlanGrasp_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Header.h"

namespace grasp_planner
{

static const char PLANGRASP[] = "grasp_planner/PlanGrasp";

  class PlanGraspRequest : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Pose objectPose;
      float object_radius;
      float object_height;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->objectPose.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_object_radius;
      u_object_radius.real = this->object_radius;
      *(outbuffer + offset + 0) = (u_object_radius.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_object_radius.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_object_radius.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_object_radius.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->object_radius);
      union {
        float real;
        uint32_t base;
      } u_object_height;
      u_object_height.real = this->object_height;
      *(outbuffer + offset + 0) = (u_object_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_object_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_object_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_object_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->object_height);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->objectPose.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_object_radius;
      u_object_radius.base = 0;
      u_object_radius.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_object_radius.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_object_radius.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_object_radius.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->object_radius = u_object_radius.real;
      offset += sizeof(this->object_radius);
      union {
        float real;
        uint32_t base;
      } u_object_height;
      u_object_height.base = 0;
      u_object_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_object_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_object_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_object_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->object_height = u_object_height.real;
      offset += sizeof(this->object_height);
     return offset;
    }

    const char * getType(){ return PLANGRASP; };
    const char * getMD5(){ return "e9dc1d01513864dcf58084ce6eef7a4e"; };

  };

  class PlanGraspResponse : public ros::Msg
  {
    public:
      float minD;
      float maxD;
      float minA;
      float maxA;
      float minH;
      float maxH;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_minD;
      u_minD.real = this->minD;
      *(outbuffer + offset + 0) = (u_minD.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_minD.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_minD.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_minD.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->minD);
      union {
        float real;
        uint32_t base;
      } u_maxD;
      u_maxD.real = this->maxD;
      *(outbuffer + offset + 0) = (u_maxD.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_maxD.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_maxD.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_maxD.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->maxD);
      union {
        float real;
        uint32_t base;
      } u_minA;
      u_minA.real = this->minA;
      *(outbuffer + offset + 0) = (u_minA.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_minA.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_minA.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_minA.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->minA);
      union {
        float real;
        uint32_t base;
      } u_maxA;
      u_maxA.real = this->maxA;
      *(outbuffer + offset + 0) = (u_maxA.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_maxA.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_maxA.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_maxA.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->maxA);
      union {
        float real;
        uint32_t base;
      } u_minH;
      u_minH.real = this->minH;
      *(outbuffer + offset + 0) = (u_minH.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_minH.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_minH.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_minH.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->minH);
      union {
        float real;
        uint32_t base;
      } u_maxH;
      u_maxH.real = this->maxH;
      *(outbuffer + offset + 0) = (u_maxH.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_maxH.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_maxH.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_maxH.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->maxH);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_minD;
      u_minD.base = 0;
      u_minD.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_minD.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_minD.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_minD.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->minD = u_minD.real;
      offset += sizeof(this->minD);
      union {
        float real;
        uint32_t base;
      } u_maxD;
      u_maxD.base = 0;
      u_maxD.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_maxD.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_maxD.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_maxD.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->maxD = u_maxD.real;
      offset += sizeof(this->maxD);
      union {
        float real;
        uint32_t base;
      } u_minA;
      u_minA.base = 0;
      u_minA.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_minA.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_minA.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_minA.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->minA = u_minA.real;
      offset += sizeof(this->minA);
      union {
        float real;
        uint32_t base;
      } u_maxA;
      u_maxA.base = 0;
      u_maxA.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_maxA.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_maxA.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_maxA.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->maxA = u_maxA.real;
      offset += sizeof(this->maxA);
      union {
        float real;
        uint32_t base;
      } u_minH;
      u_minH.base = 0;
      u_minH.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_minH.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_minH.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_minH.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->minH = u_minH.real;
      offset += sizeof(this->minH);
      union {
        float real;
        uint32_t base;
      } u_maxH;
      u_maxH.base = 0;
      u_maxH.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_maxH.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_maxH.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_maxH.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->maxH = u_maxH.real;
      offset += sizeof(this->maxH);
     return offset;
    }

    const char * getType(){ return PLANGRASP; };
    const char * getMD5(){ return "c86ec4d9e8f1b271b414023be89d8d6a"; };

  };

  class PlanGrasp {
    public:
    typedef PlanGraspRequest Request;
    typedef PlanGraspResponse Response;
  };

}
#endif
