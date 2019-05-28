#include <getopt.h>
#include <string>
#include <thread>
#include <ros/console.h>
#include <perimeter_wire_sensor/driver.h>

using namespace perimeter_wire_sensor;

void usage()
{
  printf("Usage: console [-h] [-p port]\n");
  printf("Options:\n");
  printf("-h\thelp\n");
  printf("-p\tuse serial port\n");
}

int main(int argc, char **argv)
{
  int opt;
  std::string port("/dev/ttyUSB0");
  while ((opt = getopt(argc, argv, "hp:")) != -1)
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
      default: /* '?' */
        usage();
        exit(EXIT_FAILURE);
    }
  }

  PerimeterWireDriver driver(port);
  // Spin thread for serial (required for sync api)
  std::thread thread([&driver]() { driver.run(); });

  driver.setEnabled(0x0f);
  driver.setControl(true);

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



