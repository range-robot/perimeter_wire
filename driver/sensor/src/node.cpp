
#include <string>
#include <thread>
#include <ros/node_handle.h>
#include <perimeter_wire_sensor/driver.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include "perimeter_wire_ros.h"

namespace perimeter_wire_sensor {

struct PerimeterWireDriverConfig {
  std::string com_port;
  int divider;
  int code;

  void fromParam(const ros::NodeHandle& privateNh)
  {
    privateNh.param("port", com_port, std::string("/dev/ttyO2"));
    privateNh.param("divider", divider, 10);
    privateNh.param("code", code, 0x5555);
  }
};

}  // namespace perimeter_wire_driver

using namespace perimeter_wire_sensor;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "perimeter_wire");
  ros::NodeHandle nh, privateNh("~");

  PerimeterWireDriverConfig config;
  config.fromParam(privateNh);

  ROS_INFO("Init perimeter wire driver using port %s", config.com_port.c_str());
  PerimeterWireDriver driver(config.com_port);
  std::thread thread([&driver](){driver.run();});

  for (int i = 0; i < 4; i++)
  {
    if (!driver.setDivider(i, config.divider))
    {
      ROS_ERROR("Failed to set divider for channel %d", i);
      return EXIT_FAILURE;
    }
    // avoid overflow
    usleep(10000);
    if (!driver.setCode(i, config.code))
    {
      ROS_ERROR("Failed to set code for channel %d", i);
      return EXIT_FAILURE;
    }
    // avoid overflow
    usleep(10000);
  }

  driver.setEnabled(0x0f);
  usleep(10000);
  driver.setControl(true);
  usleep(10000);
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
  return EXIT_SUCCESS;
}
