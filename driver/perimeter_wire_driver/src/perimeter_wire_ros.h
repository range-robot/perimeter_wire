
#ifndef SRC_PERIMETER_WIRE_ROS_H_
#define SRC_PERIMETER_WIRE_ROS_H_

#include <perimeter_wire_driver/perimeter_wire_driver.h>
#include <ros/node_handle.h>
#include <std_msgs/Float32MultiArray.h>

namespace perimeter_wire_driver {

class PerimeterWireRos
{
private:
  PerimeterWireDriver driver_;
  ros::Publisher pub_;

public:
  PerimeterWireRos(ros::NodeHandle nh,
    PerimeterWireDriver& driver) :
    driver_(driver)
  {
    pub_ = nh.advertise<std_msgs::Float32MultiArray>("perimeter_wire", 1);

    driver_.reset();
    driver_.setEnabled(0x0f);
    driver_.setControl(true);
  }

  void cycle()
  {
    float a, b, c, d;
    if (!driver_.getChannel(0, a))
    {
      ROS_WARN("Reading channel a failed.");
      return;
    }
    if (!driver_.getChannel(1, b))
    {
      ROS_WARN("Reading channel b failed.");
      return;
    }
    if (!driver_.getChannel(2, c))
    {
      ROS_WARN("Reading channel c failed.");
      return;
    }
    if (!driver_.getChannel(3, d))
    {
      ROS_WARN("Reading channel d failed.");
      return;
    }
    std_msgs::Float32MultiArray array;
    array.data.push_back(a);
    array.data.push_back(b);
    array.data.push_back(c);
    array.data.push_back(d);
    pub_.publish(array);
  }
};
}  // namespace perimeter_wire_driver

#endif /* SRC_PERIMETER_WIRE_ROS_H_ */
