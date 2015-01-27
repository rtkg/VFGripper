#ifndef _ROS_ft_msgs_FTArray_h
#define _ROS_ft_msgs_FTArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ft_msgs/FTData.h"

namespace ft_msgs
{

  class FTArray : public ros::Msg
  {
    public:
      uint8_t sensor_data_length;
      ft_msgs::FTData st_sensor_data;
      ft_msgs::FTData * sensor_data;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = sensor_data_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < sensor_data_length; i++){
      offset += this->sensor_data[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t sensor_data_lengthT = *(inbuffer + offset++);
      if(sensor_data_lengthT > sensor_data_length)
        this->sensor_data = (ft_msgs::FTData*)realloc(this->sensor_data, sensor_data_lengthT * sizeof(ft_msgs::FTData));
      offset += 3;
      sensor_data_length = sensor_data_lengthT;
      for( uint8_t i = 0; i < sensor_data_length; i++){
      offset += this->st_sensor_data.deserialize(inbuffer + offset);
        memcpy( &(this->sensor_data[i]), &(this->st_sensor_data), sizeof(ft_msgs::FTData));
      }
     return offset;
    }

    const char * getType(){ return "ft_msgs/FTArray"; };
    const char * getMD5(){ return "fe0882e7d3da209948c5524044934b59"; };

  };

}
#endif