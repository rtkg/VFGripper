#ifndef _ROS_constraint_map_SimpleOccMapMsg_h
#define _ROS_constraint_map_SimpleOccMapMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace constraint_map
{

  class SimpleOccMapMsg : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float cell_size;
      float x_cen;
      float y_cen;
      float z_cen;
      int32_t x_size;
      int32_t y_size;
      int32_t z_size;
      uint8_t data_length;
      int32_t st_data;
      int32_t * data;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_cell_size;
      u_cell_size.real = this->cell_size;
      *(outbuffer + offset + 0) = (u_cell_size.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cell_size.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cell_size.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cell_size.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cell_size);
      union {
        float real;
        uint32_t base;
      } u_x_cen;
      u_x_cen.real = this->x_cen;
      *(outbuffer + offset + 0) = (u_x_cen.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_cen.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x_cen.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x_cen.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_cen);
      union {
        float real;
        uint32_t base;
      } u_y_cen;
      u_y_cen.real = this->y_cen;
      *(outbuffer + offset + 0) = (u_y_cen.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y_cen.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y_cen.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y_cen.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y_cen);
      union {
        float real;
        uint32_t base;
      } u_z_cen;
      u_z_cen.real = this->z_cen;
      *(outbuffer + offset + 0) = (u_z_cen.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z_cen.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_z_cen.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_z_cen.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z_cen);
      union {
        int32_t real;
        uint32_t base;
      } u_x_size;
      u_x_size.real = this->x_size;
      *(outbuffer + offset + 0) = (u_x_size.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_size.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x_size.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x_size.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_size);
      union {
        int32_t real;
        uint32_t base;
      } u_y_size;
      u_y_size.real = this->y_size;
      *(outbuffer + offset + 0) = (u_y_size.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y_size.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y_size.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y_size.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y_size);
      union {
        int32_t real;
        uint32_t base;
      } u_z_size;
      u_z_size.real = this->z_size;
      *(outbuffer + offset + 0) = (u_z_size.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z_size.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_z_size.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_z_size.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z_size);
      *(outbuffer + offset++) = data_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < data_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_datai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_datai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_datai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_cell_size;
      u_cell_size.base = 0;
      u_cell_size.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cell_size.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cell_size.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cell_size.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cell_size = u_cell_size.real;
      offset += sizeof(this->cell_size);
      union {
        float real;
        uint32_t base;
      } u_x_cen;
      u_x_cen.base = 0;
      u_x_cen.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_cen.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x_cen.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x_cen.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x_cen = u_x_cen.real;
      offset += sizeof(this->x_cen);
      union {
        float real;
        uint32_t base;
      } u_y_cen;
      u_y_cen.base = 0;
      u_y_cen.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y_cen.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y_cen.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y_cen.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y_cen = u_y_cen.real;
      offset += sizeof(this->y_cen);
      union {
        float real;
        uint32_t base;
      } u_z_cen;
      u_z_cen.base = 0;
      u_z_cen.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z_cen.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_z_cen.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_z_cen.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->z_cen = u_z_cen.real;
      offset += sizeof(this->z_cen);
      union {
        int32_t real;
        uint32_t base;
      } u_x_size;
      u_x_size.base = 0;
      u_x_size.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_size.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x_size.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x_size.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x_size = u_x_size.real;
      offset += sizeof(this->x_size);
      union {
        int32_t real;
        uint32_t base;
      } u_y_size;
      u_y_size.base = 0;
      u_y_size.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y_size.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y_size.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y_size.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y_size = u_y_size.real;
      offset += sizeof(this->y_size);
      union {
        int32_t real;
        uint32_t base;
      } u_z_size;
      u_z_size.base = 0;
      u_z_size.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z_size.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_z_size.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_z_size.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->z_size = u_z_size.real;
      offset += sizeof(this->z_size);
      uint8_t data_lengthT = *(inbuffer + offset++);
      if(data_lengthT > data_length)
        this->data = (int32_t*)realloc(this->data, data_lengthT * sizeof(int32_t));
      offset += 3;
      data_length = data_lengthT;
      for( uint8_t i = 0; i < data_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_data;
      u_st_data.base = 0;
      u_st_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_data = u_st_data.real;
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(int32_t));
      }
     return offset;
    }

    const char * getType(){ return "constraint_map/SimpleOccMapMsg"; };
    const char * getMD5(){ return "6f28770d397777b04da3e80e23845c66"; };

  };

}
#endif