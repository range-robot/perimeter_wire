#include <getopt.h>
#include <string>
#include <thread>
#include <ros/console.h>
#include <perimeter_wire_sensor/driver.h>
#include <perimeter_wire_sensor_firmware/registers.h>

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

void readSample(PerimeterWireDriver& driver)
{
  // Start, non continous
  uint8_t flags = PWSENS_FLAGS_START | PWSENS_FLAGS_ENABLE;
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

  uint16_t value;
  for (uint16_t i = 0; i < bufferLen; i++) {
    if (!driver.getBufferValue(value))
    {
      printf("Buffer read error\n");
      return;
    }
    else {
      printf("%d %d\n", i, (int16_t) value);
    }
  }

  float a, b, c;
  if (!driver.getChannel(0, a))
  {
    ROS_WARN("Reading channel a failed.");
  }
  if (!driver.getChannel(1, b))
  {
    ROS_WARN("Reading channel b failed.");
  }
  if (!driver.getChannel(2, c))
  {
    ROS_WARN("Reading channel c failed.");
  }
  printf("%.4f\t%.4f\t%.4f\n", a, b, c);

  driver.setFlags(0);
}

int main(int argc, char **argv)
{
  int opt;
  int divider = 0;
  int code = 0;
  int repeat = 0;
  bool sample = false;
  bool sync = false;
  bool differential = false, reset = false, bootload = false;
  std::string port("/dev/ttyUSB0");
  while ((opt = getopt(argc, argv, "hDp:d:c:r:sRBS")) != -1)
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
      case 'B':
        bootload = true;
        break;
      case 'R':
        reset = true;
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
        sample = true;
        break;
      case 'S':
        sync = true;
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
    if (!driver.setDivider(divider))
    {
      ROS_ERROR("Failed to set divider for channel");
    }
    usleep(10000);
  }
  if (code != 0)
  {
    ROS_INFO("Using code 0x%x", code);
    if (!driver.setCode(code))
    {
      ROS_ERROR("Failed to set code for channel");
    }
    usleep(10000);
  }
  if (repeat != 0)
  {
    ROS_INFO("Using repeat 0x%x", repeat);
    if (!driver.setRepeat(repeat))
    {
      ROS_ERROR("Failed to set repeat for channel");
    }
    usleep(10000);
  }

  if (sample)
  {
    readSample(driver);

    driver.stop();
    thread.join();

    exit(EXIT_SUCCESS);
  }

  if (reset || bootload)
  {
    printf("Resetting device");
    driver.reset(bootload);
    usleep(10000);

    driver.stop();
    thread.join();

    exit(EXIT_SUCCESS);
  }

  driver.setFlags(PWSENS_FLAGS_START | PWSENS_FLAGS_CONTINUOUS | PWSENS_FLAGS_ENABLE
                  | (differential ? PWSENS_FLAGS_DIFFERENTIAL : 0)
                  | (sync ? PWSENS_FLAGS_SYNC_MODE : 0));
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

  printf("A\tAQ\tB\tBQ\tC\tCQ\n");
  float a, b, c, aq, bq, cq;
  while (true) {
    if (!driver.getChannel(0, a))
    {
      ROS_WARN("Reading channel a failed.");
      continue;
    }
    if (!driver.getQuality(0, aq))
    {
      ROS_WARN("Reading quality a failed.");
      continue;
    }
    if (!driver.getChannel(1, b))
    {
      ROS_WARN("Reading channel b failed.");
      continue;
    }
    if (!driver.getQuality(1, bq))
    {
      ROS_WARN("Reading quality b failed.");
      continue;
    }
    if (!driver.getChannel(2, c))
    {
      ROS_WARN("Reading channel c failed.");
      continue;
    }
    if (!driver.getQuality(2, cq))
    {
      ROS_WARN("Reading quality c failed.");
      continue;
    }
    printf("%.4f\t%.0f\t%.4f\t%.0f\t%.4f\t%.0f\n", a, aq, b, bq, c, cq);
    usleep(32000);
  }

  driver.stop();
  thread.join();

  exit(EXIT_SUCCESS);
}



