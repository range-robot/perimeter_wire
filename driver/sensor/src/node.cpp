
#include <string>
#include <thread>
#include <regex>
#include <ros/node_handle.h>
#include <perimeter_wire_sensor_driver/driver.h>
#include <perimeter_wire_sensor_firmware/registers.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include "perimeter_wire_ros.h"

namespace perimeter_wire_sensor
{

struct PerimeterWireDriverConfig
{
  std::string com_port;
  int divider;
  int code;
  int repeat;
  bool sync;
  bool differential;
  int filter;

  /*
   * orientation of the sensor
   * default is 'xyz' any order of x, y, z and -x, -y, -z is possible.
   * First letter ist which axis sensor channel 0 is on and so on.
   */
  std::string orientation;

  void fromParam(const ros::NodeHandle &privateNh)
  {
    privateNh.param("port", com_port, std::string("/dev/ttyO2"));
    privateNh.param("divider", divider, 10);
    privateNh.param("code", code, 0x5555);
    privateNh.param("repeat", repeat, 1);
    privateNh.param("sync", sync, true);
    privateNh.param("differential", differential, true);
    privateNh.param("filter", filter, 0);
    privateNh.param("orientation", orientation, std::string("xyz"));
    ROS_INFO("perimeter_wire configuration: divider: %d, code: 0x%x, repeat: %d, diff: %d", divider, code, repeat, differential);
  }

  void getChannelMap(ChannelMapping& map)
  {
    const std::regex base_regex("^(-?[xyz])(-?[xyz])(-?[xyz])$");
    std::smatch base_match;
    if (!std::regex_match(orientation, base_match, base_regex))
      throw std::invalid_argument("invalid orientation (format: xyz / x-yz)");

    // The first sub_match is the whole string; the next
    // sub_match is the first parenthesized expression.
    if (base_match.size() != 4)
      throw std::invalid_argument("invalid orientation (need 3 axes)");

    for (int i = 0; i < 3; i++)
    {
      std::string axis = base_match[i + 1].str();
      if (axis.length() < 1 || axis.length() > 2)
        throw std::invalid_argument("invalid orientation");

      char a = axis.back();
      switch (a)
      {
      case 'x':
        map.map[i] = 0;
        break;
      case 'y':
        map.map[i] = 1;
        break;
      case 'z':
        map.map[i] = 2;
        break;
      default:
        throw std::invalid_argument("invalid orientation");
      }
      if (axis.front() == '-')
        map.negate[i] = true;
      else
        map.negate[i] = false;
    }
  }
};

} // namespace perimeter_wire_sensor

using namespace perimeter_wire_sensor;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "perimeter_wire");
  ros::NodeHandle nh, privateNh("~");

  PerimeterWireDriverConfig config;
  config.fromParam(privateNh);

  ROS_INFO("Init perimeter wire driver using port %s", config.com_port.c_str());
  PerimeterWireDriver driver(config.com_port);
  std::thread thread([&driver]() { driver.run(); });

  if (!driver.setFlags(0))
  {
    ROS_ERROR("Failed to stop device.");
    return EXIT_FAILURE;
  }
  // avoid overflow
  usleep(10000);
  if (!driver.setDivider(config.divider))
  {
    ROS_ERROR("Failed to set divider");
    return EXIT_FAILURE;
  }
  // avoid overflow
  usleep(10000);
  if (!driver.setCode(config.code))
  {
    ROS_ERROR("Failed to set code");
    return EXIT_FAILURE;
  }
  // avoid overflow
  usleep(10000);
  if (!driver.setRepeat(config.repeat))
  {
    ROS_ERROR("Failed to set repeat");
    return EXIT_FAILURE;
  }
  // avoid overflow
  usleep(10000);
  if (!driver.setFilterSize(config.filter))
  {
    ROS_ERROR("Failed to set filter");
    return EXIT_FAILURE;
  }
  // avoid overflow
  usleep(10000);

  if (!driver.setFlags(PWSENS_FLAGS_START | PWSENS_FLAGS_CONTINUOUS | PWSENS_FLAGS_ENABLE | (config.differential ? PWSENS_FLAGS_DIFFERENTIAL : 0) | (config.sync ? PWSENS_FLAGS_SYNC_MODE : 0)))
  {
    ROS_ERROR("Failed to enable device");
    return EXIT_FAILURE;
  }
  usleep(10000);
  driver.setControl(true);
  usleep(10000);
  bool control;
  if (!driver.getControl(control))
  {
    ROS_ERROR("Failed to read control.");
    return EXIT_FAILURE;
  }
  else if (!control)
  {
    ROS_ERROR("Failed to enable device.");
    return EXIT_FAILURE;
  }

  int code_weight = PerimeterWireDriver::getCodeWeight(config.code, config.differential);
  ChannelMapping channel_map;
  config.getChannelMap(channel_map);
  PerimeterWireRos rosDrv(nh, driver, code_weight, channel_map);

  diagnostic_updater::Updater updater;
  updater.setHardwareID(config.com_port);

  ros::Rate rate(10.0);
  int state = 0;
  while (privateNh.ok())
  {
    // check
    auto error = driver.getLastError();
    if (error)
    {
      if (error.value() == boost::system::errc::no_such_file_or_directory)
      {
        ROS_ERROR("Interface does not exist any more (device detached)");
      }
      else
      {
        ROS_ERROR("IO error (%d): %s", error.value(), error.message().c_str());
      }
      break;
    }

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
