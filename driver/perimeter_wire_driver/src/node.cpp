
#include <string>
#include <thread>
#include <ros/node_handle.h>
#include <perimeter_wire_driver/perimeter_wire_driver.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include "perimeter_wire_ros.h"

namespace perimeter_wire_driver {

struct PerimeterWireDriverConfig {
  std::string com_port;

  void fromParam(const ros::NodeHandle& privateNh)
  {
    privateNh.param("port", com_port, std::string("/dev/ttyO2"));
  }
};

}  // namespace perimeter_wire_driver

using namespace perimeter_wire_driver;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "perimeter_wire");
  ros::NodeHandle nh, privateNh("~");

  PerimeterWireDriverConfig config;
  config.fromParam(privateNh);

  ROS_INFO("Init perimeter wire driver using port %s", config.com_port.c_str());
  PerimeterWireDriver driver(config.com_port);
  std::thread thread([&driver](){driver.run();});

  PerimeterWireRos rosDrv(nh, driver);
  
  diagnostic_updater::Updater updater;
  updater.setHardwareID(config.com_port);
  
  ros::Rate rate(10.0);
  int state = 0;
  while (privateNh.ok())
  {
    // process
    rosDrv.cycle();
    ros::spinOnce();

    updater.update();
    rate.sleep();
  }

  // stop serial thread
  driver.stop();
  thread.join();
}
