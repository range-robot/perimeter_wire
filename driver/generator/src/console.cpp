#include <getopt.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <perimeter_wire_generator/driver.h>
#include "console.h"

using namespace perimeter_wire_generator;

void usage()
{
  printf("Usage: console [-h] [-abs] [-m mode] [-d div] [-c code] [-p port]\n");
  printf("Options:\n");
  printf("-h\thelp\n");
  printf("-p\tuse serial port\n");
  printf("-m\tset mode\n");
  printf("-d\tset divider\n");
  printf("-c\tset code\n");
  printf("-a\tconfigure channel a\n");
  printf("-b\tconfigure channel b\n");
  printf("-s\tsave configuration to nv storage\n");
}

int main(int argc, char **argv)
{
  bool chA = false, chB = false, save = false;
  bool setDiv = false, setMode = false, setCode = false;
  uint8_t div = 0;
  uint8_t mode = 0;
  uint16_t code = 0;
  int opt;
  std::string port("/dev/ttyUSB0");
  while ((opt = getopt(argc, argv, "habsp:m:d:c:")) != -1)
  {
    switch (opt)
    {
      case 'h':
        usage();
        exit(EXIT_SUCCESS);
        break;
      case 'p':
        port = optarg;
        break;
      case 'a':
        chA = true;
        break;
      case 'b':
        chB = true;
        break;
      case 'm':
        setMode = true;
        mode = std::stoi(optarg);
        break;
      case 'd':
        setDiv = true;
        div = std::stoi(optarg);
        break;
      case 'c':
        setCode = true;
        code = std::stoi(optarg);
        break;
      case 's':
        save = true;
        break;
      default: /* '?' */
        usage();
        exit(EXIT_FAILURE);
    }
  }

  ROS_INFO("Perimeter wire generator configuration tool");
  GeneratorDriver driver(port);
  // Spin thread for serial (required for sync api)
  std::thread thread([&driver]() { ROS_INFO("Spin thread"); driver.run(); });

  uint16_t fw;
  if (!driver.getFWVersion(fw))
    ROS_WARN("Failed to get firmware version");
  else
    ROS_INFO("Firmware version: %d", (int)fw);

  if ((setDiv || setCode))
  {
    // modifying div or code required the generator to be disabled.
    if (chA)
    {
      uint8_t curMode;
      if (!driver.getChannelAMode(curMode))
        ROS_WARN("Failed to read channel A mode.");
      if (curMode != 0)
      {
        if (!driver.setChannelAMode(0))
          ROS_WARN("Setting mode for channel A failed.");
        else
          ROS_INFO("Disabled channel A");
        if (!setMode)
        {
          setMode = true;
          mode = curMode;
        }
      }
      usleep(10000);
    }
    if (chB)
    {
      uint8_t curMode;
      if (!driver.getChannelBMode(curMode))
        ROS_WARN("Failed to read channel B mode.");
      if (curMode != 0)
      {
        if (!driver.setChannelBMode(0))
          ROS_WARN("Setting mode for channel B failed.");
        else
          ROS_INFO("Disabled channel B");
        if (!setMode)
        {
          setMode = true;
          mode = curMode;
        }
      }
    }
    usleep(10000);
  }

  if (setDiv)
  {
    if (chA)
    {
      if (!driver.setChannelADivider(div))
        ROS_WARN("Setting divider for channel A failed.");
      else
        ROS_INFO("Divider set for channel A");
    }
    if (chB)
    {
      if (!driver.setChannelBDivider(div))
        ROS_WARN("Setting divider for channel B failed.");
      else
        ROS_INFO("Divider set for channel B");
    }
    usleep(10000);
  }

  if (setCode)
  {
    if (chA)
    {
      if (!driver.setChannelACode(code))
        ROS_WARN("Setting code for channel A failed.");
      else
        ROS_INFO("Code set for channel A");
    }
    if (chB)
    {
      if (!driver.setChannelBCode(code))
        ROS_WARN("Setting code for channel B failed.");
      else
        ROS_INFO("Code set for channel B");
    }
    usleep(10000);
  }

  if (setMode)
  {
    if (chA)
    {
      if (!driver.setChannelAMode(mode))
        ROS_WARN("Setting mode for channel A failed.");
      else
        ROS_INFO("Set mode for channel A to 0x%x", mode);
    }
    if (chB)
    {
      if (!driver.setChannelBMode(mode))
        ROS_WARN("Setting mode for channel B failed.");
      else
        ROS_INFO("Set mode for channel B to 0x%x", mode);
    }
    usleep(10000);
  }

  if (save) {
    if (!driver.saveConfiguration())
      ROS_WARN("Failed to save configuration");
    else
      ROS_INFO("Configuration saved.");
  }

  float voltage;
  if (!driver.getVoltage(voltage))
    ROS_WARN("Failed to read voltage");
  else
    ROS_INFO("Voltage: %fV", voltage);

  float temp;
  if (!driver.getTemperature(temp))
    ROS_WARN("Failed to read temperature");
  else
    ROS_INFO("Temperature: %f°C", temp);

  // read info
  ROS_INFO("Channel A:");
  if (!driver.getChannelAMode(mode))
    ROS_WARN("Failed to read channel A mode.");
  else
    ROS_INFO("Mode:\t0x%x", mode);

  if (!driver.getChannelADivider(div))
    ROS_WARN("Failed to read channel A divider.");
  else
    ROS_INFO("Divider:\t%d", div);

  if (!driver.getChannelACode(code))
    ROS_WARN("Failed to read channel A code.");
  else
    ROS_INFO("Code:\t0x%x", code);

  ROS_INFO("Channel B:");
  if (!driver.getChannelBMode(mode))
    ROS_WARN("Failed to read channel B mode.");
  else
    ROS_INFO("Mode:\t0x%x", mode);

  if (!driver.getChannelBDivider(div))
    ROS_WARN("Failed to read channel B divider.");
  else
    ROS_INFO("Divider:\t%d", div);

  if (!driver.getChannelBCode(code))
    ROS_WARN("Failed to read channel B code.");
  else
    ROS_INFO("Code:\t0x%x", code);

  driver.stop();
  thread.join();

  exit(EXIT_SUCCESS);
}



