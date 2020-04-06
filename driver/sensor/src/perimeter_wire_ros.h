
#ifndef SRC_PERIMETER_WIRE_ROS_H_
#define SRC_PERIMETER_WIRE_ROS_H_

#include <perimeter_wire_sensor/driver.h>
#include <ros/node_handle.h>
#include <std_msgs/Float32MultiArray.h>

namespace perimeter_wire_sensor {

class PerimeterWireRos
{
private:
  PerimeterWireDriver driver_;
  ros::Publisher pub_;
  const int code_weight_;

public:
  PerimeterWireRos(ros::NodeHandle nh,
    PerimeterWireDriver& driver, int code_weight) :
    driver_(driver),
    code_weight_(code_weight)
  {
    pub_ = nh.advertise<std_msgs::Float32MultiArray>("perimeter_wire", 1);
  }

  void cycle()
  {
    float value;
    float quality;
    std_msgs::Float32MultiArray array;

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
      array.data.push_back(value / code_weight_);
      array.data.push_back(quality);
    }

    pub_.publish(array);
  }
};
}  // namespace perimeter_wire_driver

#endif /* SRC_PERIMETER_WIRE_ROS_H_ */
