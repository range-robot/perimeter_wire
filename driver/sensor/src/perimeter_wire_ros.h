
#ifndef SRC_PERIMETER_WIRE_ROS_H_
#define SRC_PERIMETER_WIRE_ROS_H_

#include <perimeter_wire_sensor_driver/driver.h>
#include <ros/node_handle.h>
#include <std_msgs/Float32MultiArray.h>
#include <array>

namespace perimeter_wire_sensor {

struct ChannelMapping
{
  std::array<int, 3> map;
  std::array<bool, 3> negate;
};

class PerimeterWireRos
{
private:
  PerimeterWireDriver driver_;
  ros::Publisher pub_;
  const int code_weight_;
  const ChannelMapping channel_map_;

public:
  /*
   * Create new Perimeter Wire Ros
   * channel_map: channel_map[channel_index] = axis_index
   */
  PerimeterWireRos(ros::NodeHandle nh,
    PerimeterWireDriver& driver, int code_weight, const ChannelMapping& channel_map) :
    driver_(driver),
    code_weight_(code_weight),
    channel_map_(channel_map)
  {
    pub_ = nh.advertise<std_msgs::Float32MultiArray>("perimeter_wire", 1);
  }

  void cycle()
  {
    float value;
    float quality;
    std_msgs::Float32MultiArray array;
    array.data.resize(6);

    for (int i = 0; i < 3; i++)
    {
      if (!driver_.getChannel(i, value))
      {
        ROS_WARN("Reading channel %d failed.", i);
        return;
      }
      if (!driver_.getQuality(i, quality))
      {
        ROS_WARN("Reading quality %d failed.", i);
        return;
      }
      bool negate = channel_map_.negate[i];
      if (negate)
        value = -value;
      int channel = channel_map_.map[i];
      assert(channel >= 0 && channel < 3);
      array.data[channel * 2] = value / code_weight_;
      array.data[channel * 2 + 1] = quality;
    }

    pub_.publish(array);
  }
};
}  // namespace perimeter_wire_driver

#endif /* SRC_PERIMETER_WIRE_ROS_H_ */
