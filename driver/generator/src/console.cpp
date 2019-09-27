#include <getopt.h>
#include <string>
#include <thread>
#include <perimeter_wire_generator/driver.h>
#include "console.h"

using namespace perimeter_wire_generator;

void usage()
{
  printf("Usage: console [-h] [-ab] [-m mode] [-f freq] [-p port]\n");
  printf("Options:\n");
  printf("-h\thelp\n");
  printf("-p\tuse serial port\n");
  printf("-m\tset mode\n");
  printf("-f\tset frequency\n");
  printf("-a\tconfigure channel a\n");
  printf("-b\tconfigure channel b\n");
}

int main(int argc, char **argv)
{
  bool chA = false, chB = false;
  bool setFreq = false, setMode = false;
  uint16_t freq = 0;
  uint8_t mode = 0;
  int opt;
  std::string port("/dev/ttyUSB0");
  while ((opt = getopt(argc, argv, "habp:m:f:")) != -1)
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
      case 'f':
        setFreq = true;
        freq = std::stoi(optarg);
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

  if (setFreq)
  {
    if (chA)
    {
      if (!driver.setChannelAFrequency(freq))
        ROS_WARN("Setting frequency for channel A failed.");
      else
        ROS_INFO("Frequency set for channel A");
    }
    if (chB)
    {
      if (!driver.setChannelBFrequency(freq))
        ROS_WARN("Setting frequency for channel B failed.");
      else
        ROS_INFO("Frequency set for channel B");
    }
  }

  if (setMode)
  {
    if (chA)
    {
      if (!driver.setChannelAMode(mode))
        ROS_WARN("Setting mode for channel A failed.");
      else
        ROS_INFO("Mode set for channel A");
    }
    if (chB)
    {
      if (!driver.setChannelBMode(mode))
        ROS_WARN("Setting mode for channel B failed.");
      else
        ROS_INFO("Mode set for channel B");
    }
  }

  float temp;
  if (!driver.getTemperature(temp))
    ROS_WARN("Failed to read temperature");
  else
    ROS_INFO("Temperature: %f", temp);

  // read info
  ROS_INFO("Channel A:");
  if (!driver.getChannelAMode(mode))
    ROS_WARN("Failed to read channel A mode.");
  else
    ROS_INFO("Mode:\t0x%x", mode);

  if (!driver.getChannelAFrequency(freq))
    ROS_WARN("Failed to read channel A frequency.");
  else
    ROS_INFO("Frequency:\t%d", freq);

  ROS_INFO("Channel B:");
  if (!driver.getChannelBMode(mode))
    ROS_WARN("Failed to read channel B mode.");
  else
    ROS_INFO("Mode:\t0x%x", mode);

  if (!driver.getChannelBFrequency(freq))
    ROS_WARN("Failed to read channel B frequency.");
  else
    ROS_INFO("Frequency:\t%d", freq);

  driver.stop();
  thread.join();

  exit(EXIT_SUCCESS);
}



