
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
  float last_[4];
  float filter_;

public:
  PerimeterWireRos(ros::NodeHandle nh,
    PerimeterWireDriver& driver, float filter) :
    driver_(driver),
    filter_(filter),
    last_{}
  {
    pub_ = nh.advertise<std_msgs::Float32MultiArray>("perimeter_wire", 1);
  }

  void cycle()
  {
    float value[4];
    std_msgs::Float32MultiArray array;

    for (int i = 0; i < 4; i++)
    {
      if (!driver_.getChannel(i, value[i]))
      {
        ROS_WARN("Reading channel %d failed.", i);
        return;
      }
      last_[i] = (1.0 - filter_) * value[i] + filter_ * last_[i];
      array.data.push_back(value[i]);
      array.data.push_back(last_[i]);
    }

    array.data.push_back(sqrt(value[0]*value[0] + value[1]*value[1]));
    pub_.publish(array);
  }
};
}  // namespace perimeter_wire_driver

#endif /* SRC_PERIMETER_WIRE_ROS_H_ */
