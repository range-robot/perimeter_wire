#include <getopt.h>
#include <string>
#include <thread>
#include <ros/console.h>
#include <perimeter_wire_sensor/driver.h>

using namespace perimeter_wire_sensor;

void usage()
{
  printf("Usage: console [-hD] [-p port] [-d divider] [-s channel]\n");
  printf("Options:\n");
  printf("-h\thelp\n");
  printf("-D\tuse differntial mode\n");
  printf("-d\tset frequency divider\n");
  printf("-c\tset code\n");
  printf("-r\tset repeat\n");
  printf("-p\tuse serial port\n");
  printf("-s\tread sample\n");
}

void readSample(PerimeterWireDriver& driver, int sample, bool differential)
{
  // Start, non continous
  uint8_t flags = (differential ? 0x90 : 0x10) | (0x01 << sample);
  driver.setFlags(flags);

  usleep(10000);

  // read the buffer
  uint16_t bufferLen;
  if (!driver.getBufferLength(bufferLen))
  {
    printf("Failed to read buffer length\n");
    return;
  }
  printf("Buffer length is %d\n", bufferLen);

  uint16_t value, index;
  for (uint16_t i = 0; i < bufferLen; i++) {
    if (!driver.getBufferValue(value))
    {
      printf("Buffer read error\n");
      return;
    }
    else {
      driver.getBufferIndex(index);
      printf("%d: %d\n", index, (int16_t) value);

    }
  }
}

int main(int argc, char **argv)
{
  int opt;
  int divider = 0;
  int code = 0;
  int repeat = 0;
  int sample = -1;
  bool differential = false;
  std::string port("/dev/ttyUSB0");
  while ((opt = getopt(argc, argv, "hDp:d:c:r:s:")) != -1)
  {
    switch (opt)
    {
      case 'h':
        usage();
        exit(EXIT_SUCCESS);
        break;
      case 'D':
        differential = true;
        break;
      case 'p':
        port = optarg;
        break;
      case 'd':
        divider = std::stoi(optarg);
        break;
      case 'c':
        code = std::stoi(optarg);
        break;
      case 'r':
        repeat = std::stoi(optarg);
        break;
      case 's':
        sample = std::stoi(optarg);
        break;
      default: /* '?' */
        usage();
        exit(EXIT_FAILURE);
    }
  }

  PerimeterWireDriver driver(port);
  // Spin thread for serial (required for sync api)
  std::thread thread([&driver]() { driver.run(); });

  if (divider != 0)
  {
    ROS_INFO("Using divider %d", divider);
    for (int i = 0; i < 4; i++)
    {
      if (!driver.setDivider(i, divider))
      {
        ROS_ERROR("Failed to set divider for channel %d", i);
      }
      usleep(10000);
    }
  }
  if (code != 0)
  {
    ROS_INFO("Using code 0x%x", code);
    for (int i = 0; i < 4; i++)
    {
      if (!driver.setCode(i, code))
      {
        ROS_ERROR("Failed to set code for channel %d", i);
      }
      usleep(10000);
    }
  }
  if (repeat != 0)
  {
    ROS_INFO("Using repeat 0x%x", repeat);
    for (int i = 0; i < 4; i++)
    {
      if (!driver.setRepeat(i, repeat))
      {
        ROS_ERROR("Failed to set repeat for channel %d", i);
      }
      usleep(10000);
    }
  }

  if (sample >= 0)
  {
    readSample(driver, sample, differential);

    driver.stop();
    thread.join();

    exit(EXIT_SUCCESS);
  }

  if (differential)
    driver.setFlags(0xBf);
  else
    driver.setFlags(0x3f);
  usleep(10000);
  driver.setControl(true);
  usleep(10000);
  bool control;
  if (!driver.getControl(control))
  {
    ROS_ERROR("Failed to read control.");
  }
  else if (!control)
  {
    ROS_ERROR("Failed to enable device.");
  }

  printf("A\tB\tC\tD\n");
  float a, b, c, d;
  while (true) {
    if (!driver.getChannel(0, a))
    {
      ROS_WARN("Reading channel a failed.");
      continue;
    }
    if (!driver.getChannel(1, b))
    {
      ROS_WARN("Reading channel b failed.");
      continue;
    }
    if (!driver.getChannel(2, c))
    {
      ROS_WARN("Reading channel c failed.");
      continue;
    }
    if (!driver.getChannel(3, d))
    {
      ROS_WARN("Reading channel d failed.");
      continue;
    }
    printf("%.4f\t%.4f\t%.4f\t%.4f\n", a, b, c, d);
    usleep(32000);
  }

  driver.stop();
  thread.join();

  exit(EXIT_SUCCESS);
}



