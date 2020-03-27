#ifndef CONSOLE_H_
#define CONSOLE_H_

#include <cstdio>

#define LOG(f, ...) printf(f "\n", ## __VA_ARGS__)
#define ROS_DEBUG
#define ROS_INFO LOG
#define ROS_WARN LOG
#define ROS_ERROR LOG

#endif  // CONSOLE_H_