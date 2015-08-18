#ifndef _ROS_ft_msgs_FTData_h
#define _ROS_ft_msgs_FTData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ft_msgs
{

  class FTData : public ros::Msg
  {
    public:
      int16_t id;
      float fx;
      float fy;
      float fz;
      float tx;
      float ty;
      float tz;
      float cfx;
      float cfy;
      float cfz;
      float ctx;
      float cty;
      float ctz;

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
      } u_fx;
      u_fx.real = this->fx;
      *(outbuffer + offset + 0) = (u_fx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fx);
      union {
        float real;
        uint32_t base;
      } u_fy;
      u_fy.real = this->fy;
      *(outbuffer + offset + 0) = (u_fy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fy);
      union {
        float real;
        uint32_t base;
      } u_fz;
      u_fz.real = this->fz;
      *(outbuffer + offset + 0) = (u_fz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fz);
      union {
        float real;
        uint32_t base;
      } u_tx;
      u_tx.real = this->tx;
      *(outbuffer + offset + 0) = (u_tx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tx);
      union {
        float real;
        uint32_t base;
      } u_ty;
      u_ty.real = this->ty;
      *(outbuffer + offset + 0) = (u_ty.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ty.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ty.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ty.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ty);
      union {
        float real;
        uint32_t base;
      } u_tz;
      u_tz.real = this->tz;
      *(outbuffer + offset + 0) = (u_tz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tz);
      union {
        float real;
        uint32_t base;
      } u_cfx;
      u_cfx.real = this->cfx;
      *(outbuffer + offset + 0) = (u_cfx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cfx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cfx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cfx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cfx);
      union {
        float real;
        uint32_t base;
      } u_cfy;
      u_cfy.real = this->cfy;
      *(outbuffer + offset + 0) = (u_cfy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cfy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cfy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cfy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cfy);
      union {
        float real;
        uint32_t base;
      } u_cfz;
      u_cfz.real = this->cfz;
      *(outbuffer + offset + 0) = (u_cfz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cfz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cfz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cfz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cfz);
      union {
        float real;
        uint32_t base;
      } u_ctx;
      u_ctx.real = this->ctx;
      *(outbuffer + offset + 0) = (u_ctx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ctx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ctx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ctx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ctx);
      union {
        float real;
        uint32_t base;
      } u_cty;
      u_cty.real = this->cty;
      *(outbuffer + offset + 0) = (u_cty.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cty.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cty.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cty.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cty);
      union {
        float real;
        uint32_t base;
      } u_ctz;
      u_ctz.real = this->ctz;
      *(outbuffer + offset + 0) = (u_ctz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ctz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ctz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ctz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ctz);
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
      } u_fx;
      u_fx.base = 0;
      u_fx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fx = u_fx.real;
      offset += sizeof(this->fx);
      union {
        float real;
        uint32_t base;
      } u_fy;
      u_fy.base = 0;
      u_fy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fy = u_fy.real;
      offset += sizeof(this->fy);
      union {
        float real;
        uint32_t base;
      } u_fz;
      u_fz.base = 0;
      u_fz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fz = u_fz.real;
      offset += sizeof(this->fz);
      union {
        float real;
        uint32_t base;
      } u_tx;
      u_tx.base = 0;
      u_tx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tx = u_tx.real;
      offset += sizeof(this->tx);
      union {
        float real;
        uint32_t base;
      } u_ty;
      u_ty.base = 0;
      u_ty.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ty.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ty.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ty.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ty = u_ty.real;
      offset += sizeof(this->ty);
      union {
        float real;
        uint32_t base;
      } u_tz;
      u_tz.base = 0;
      u_tz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tz = u_tz.real;
      offset += sizeof(this->tz);
      union {
        float real;
        uint32_t base;
      } u_cfx;
      u_cfx.base = 0;
      u_cfx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cfx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cfx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cfx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cfx = u_cfx.real;
      offset += sizeof(this->cfx);
      union {
        float real;
        uint32_t base;
      } u_cfy;
      u_cfy.base = 0;
      u_cfy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cfy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cfy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cfy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cfy = u_cfy.real;
      offset += sizeof(this->cfy);
      union {
        float real;
        uint32_t base;
      } u_cfz;
      u_cfz.base = 0;
      u_cfz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cfz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cfz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cfz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cfz = u_cfz.real;
      offset += sizeof(this->cfz);
      union {
        float real;
        uint32_t base;
      } u_ctx;
      u_ctx.base = 0;
      u_ctx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ctx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ctx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ctx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ctx = u_ctx.real;
      offset += sizeof(this->ctx);
      union {
        float real;
        uint32_t base;
      } u_cty;
      u_cty.base = 0;
      u_cty.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cty.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cty.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cty.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cty = u_cty.real;
      offset += sizeof(this->cty);
      union {
        float real;
        uint32_t base;
      } u_ctz;
      u_ctz.base = 0;
      u_ctz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ctz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ctz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ctz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ctz = u_ctz.real;
      offset += sizeof(this->ctz);
     return offset;
    }

    const char * getType(){ return "ft_msgs/FTData"; };
    const char * getMD5(){ return "33979c6b7e7ef3f649dc1bfe080948c0"; };

  };

}
#endif