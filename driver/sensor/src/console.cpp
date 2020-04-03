#include <getopt.h>
#include <string>
#include <thread>
#include <chrono>
#include <ros/console.h>
#include <ros/rate.h>
#include <perimeter_wire_sensor/driver.h>
#include <perimeter_wire_sensor_firmware/registers.h>

using namespace perimeter_wire_sensor;

void usage()
{
  printf("Usage: console [-hDSsRB] [-p port] [-d divider] [-f filter] [-F frequency]\n");
  printf("Options:\n");
  printf("-h\thelp\n");
  printf("-R\treset\n");
  printf("-B\treset to bootloader\n");
  printf("-D\tuse differntial mode\n");
  printf("-S\tuse sync mode\n");
  printf("-d\tset frequency divider\n");
  printf("-c\tset code\n");
  printf("-r\tset repeat\n");
  printf("-f\tset filter size\n");
  printf("-p\tuse serial port\n");
  printf("-s\tread sample\n");
  printf("-F\tread frequency\n");
}

bool checkDriver(PerimeterWireDriver& driver)
{
  auto error = driver.getLastError();
  if (error)
  {
    if (error.value() == boost::system::errc::no_such_file_or_directory)
    {
      printf("Interface does not exist any more (device detached)\n");
    }
    else
    {
      printf("IO error (%d): %s\n", error.value(), error.message().c_str());
    }
    return false;
  }
  return true;
}

void readSample(PerimeterWireDriver& driver, bool differential, bool sync)
{
  // Start, non continous
  uint8_t flags = PWSENS_FLAGS_START | PWSENS_FLAGS_ENABLE
                  | (differential ? PWSENS_FLAGS_DIFFERENTIAL : 0)
                  | (sync ? PWSENS_FLAGS_SYNC_MODE : 0);
  if (!driver.setFlags(flags))
  {
    ROS_ERROR("Failed to enable device.\n");
    exit(EXIT_FAILURE);
  }

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

  uint8_t counter;
  if (!driver.getMeasurementCount(counter))
  {
    ROS_WARN("Reading mesaurement count.");
  }
  printf("Counter: %d\n", counter);
  driver.setFlags(0);
}

int main(int argc, char **argv)
{
  int opt;
  int divider = -1;
  int code = 0;
  int filter = -1;
  int repeat = 0;
  float freq = 10;
  bool sample = false;
  bool sync = false;
  bool differential = false, reset = false, bootload = false;
  std::string port("/dev/ttyUSB0");
  while ((opt = getopt(argc, argv, "hDp:d:c:r:f:F:sRBS")) != -1)
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
      case 'f':
        filter = std::stoi(optarg);
        break;
      case 'F':
        freq = std::stof(optarg);
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

  ROS_INFO("Stopping device");
  if (!driver.setFlags(0))
  {
    ROS_ERROR("Failed to stop device.");
  }
  if (divider >= 0)
  {
    ROS_INFO("Using divider %d", divider);
    if (!driver.setDivider(divider))
    {
      ROS_ERROR("Failed to set divider");
    }
    usleep(10000);
  }
  if (code != 0)
  {
    ROS_INFO("Using code 0x%x", code);
    if (!driver.setCode(code))
    {
      ROS_ERROR("Failed to set code");
    }
    usleep(10000);
  }
  if (repeat != 0)
  {
    ROS_INFO("Using repeat 0x%x", repeat);
    if (!driver.setRepeat(repeat))
    {
      ROS_ERROR("Failed to set repeat");
    }
    usleep(10000);
  }
  if (filter >= 0)
  {
    ROS_INFO("Using filter size %d", filter);
    if (!driver.setFilterSize(filter))
    {
      ROS_ERROR("Failed to set filter size");
    }
    usleep(10000);
  }

  if (sample)
  {
    readSample(driver, differential, sync);

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

  if (!driver.setFlags(PWSENS_FLAGS_START | PWSENS_FLAGS_CONTINUOUS | PWSENS_FLAGS_ENABLE
                      | (differential ? PWSENS_FLAGS_DIFFERENTIAL : 0)
                      | (sync ? PWSENS_FLAGS_SYNC_MODE : 0)))
  {
    ROS_ERROR("Failed to enable device.\n");
    exit(EXIT_FAILURE);
  }
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

  printf("sec\tA\tAQ\tB\tBQ\tC\tCQ\tcount\n");
  float a, b, c, aq, bq, cq;
  uint8_t count;
  auto start = std::chrono::steady_clock::now();
  ros::Time::init();
  ros::Rate r(freq);
  while (checkDriver(driver)) {
    if (!driver.getMeasurementCount(count))
    {
      ROS_WARN("Reading mesaurement count.");
      continue;
    }
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
    auto time = std::chrono::steady_clock::now();
    auto seconds = std::chrono::duration_cast<std::chrono::duration<double> >(time - start);
    printf("%.3f\t%.4f\t%.0f\t%.4f\t%.0f\t%.4f\t%.0f\t%d\n", 
      seconds.count(), a, aq, b, bq, c, cq, count);
    r.sleep();
  }

  driver.stop();
  thread.join();

  exit(EXIT_SUCCESS);
}



