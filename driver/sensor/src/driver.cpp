
#include <string>
#include <ros/console.h>
#include <perimeter_wire_sensor/driver.h>
#include <perimeter_wire_sensor_firmware/registers.h>
#include "async_serial.h"
#include "data_link_layer.h"
#include "app_layer.h"


#define FIRMWARE_VERSION 1
#define ADC_MAX (4096.0f)
using namespace perimeter_wire_sensor;

const uint8_t channel_register_map[] = {
  REGISTER_CHANNEL_A,
  REGISTER_CHANNEL_B,
  REGISTER_CHANNEL_C,
  REGISTER_CHANNEL_D
};

const uint8_t divider_register_map[] = {
  REGISTER_CHANNEL_A_DIVIDER,
  REGISTER_CHANNEL_B_DIVIDER,
  REGISTER_CHANNEL_C_DIVIDER,
  REGISTER_CHANNEL_D_DIVIDER
};

const uint8_t code_register_map[] = {
  REGISTER_CHANNEL_A_CODE,
  REGISTER_CHANNEL_B_CODE,
  REGISTER_CHANNEL_C_CODE,
  REGISTER_CHANNEL_D_CODE
};

const uint8_t repeat_register_map[] = {
  REGISTER_CHANNEL_A_REPEAT,
  REGISTER_CHANNEL_B_REPEAT,
  REGISTER_CHANNEL_C_REPEAT,
  REGISTER_CHANNEL_D_REPEAT
};

PerimeterWireDriver::PerimeterWireDriver(const std::string& com_port)
{
  using namespace std::placeholders;
  serial_ = std::make_shared<AsyncSerial>(com_port);
  dll_ = std::make_shared<DataLink>(serial_);
  app_ = std::make_shared<AppLayer>(dll_);
  app_->setHelloCallback(std::bind(&PerimeterWireDriver::helloCallback, this, _1));
  app_->setCommandResultCallback(std::bind(&PerimeterWireDriver::cmdResultCallback, this, _1, _2));
}

PerimeterWireDriver::~PerimeterWireDriver()
{
  serial_->runOnce();
  serial_->stop();
}

void PerimeterWireDriver::helloCallback(uint16_t version)
{
  if (version != FIRMWARE_VERSION)
  {
    ROS_ERROR("Firmware version incompatible. Excpected: %i, Got: %i", FIRMWARE_VERSION, (int)version);
  }
}

void PerimeterWireDriver::cmdResultCallback(uint8_t cmd, uint8_t result)
{
  ROS_ERROR("Received orphanded command result. Cmd: %i, Result: %x", (int)cmd, (int)result);
}

void PerimeterWireDriver::run()
{
  serial_->run();
}

void PerimeterWireDriver::runOnce()
{
  serial_->runOnce();
}

void PerimeterWireDriver::stop()
{
  serial_->stop();
}

bool PerimeterWireDriver::getFWVersion(uint16_t& version)
{
  return app_->getVersion(version);
}

void PerimeterWireDriver::reset() {
  app_->reset();
}

bool PerimeterWireDriver::getControl(bool& enabled)
{
  uint8_t control;
  bool res = app_->getReg(REGISTER_CONTROL, control);
  enabled = control & 0x01 != 0;
  return res;
}

bool PerimeterWireDriver::setControl(bool enabled)
{
  uint8_t control = enabled ? 0x01 : 0x00;
  app_->setReg(REGISTER_CONTROL, control);
}

bool PerimeterWireDriver::setFlags(uint8_t flags)
{
  app_->setReg(REGISTER_FLAGS, flags);
}

bool PerimeterWireDriver::getChannel(int channel, float& value)
{
  if (channel < 0 || channel >= sizeof(channel_register_map))
    return false;
  uint8_t reg = channel_register_map[channel];
  uint16_t val16;
  bool res = app_->getReg16(reg, val16);
  value = static_cast<int16_t>(val16) / ADC_MAX;
  return res;
}

bool PerimeterWireDriver::setDivider(int channel, uint8_t divider)
{
  if (channel < 0 || channel >= sizeof(divider_register_map))
    return false;
  app_->setReg(divider_register_map[channel], divider);
  return true;
}

bool PerimeterWireDriver::setCode(int channel, uint16_t code)
{
  if (channel < 0 || channel >= sizeof(code_register_map))
    return false;
  app_->setReg16(code_register_map[channel], code);
  return true;
}

bool PerimeterWireDriver::setRepeat(int channel, uint8_t repeat)
{
  if (channel < 0 || channel >= sizeof(repeat_register_map))
    return false;
  app_->setReg(repeat_register_map[channel], repeat);
  return true;
}

bool PerimeterWireDriver::getBufferIndex(uint16_t& index)
{
  return app_->getReg16(REGISTER_BUFFER_INDEX, index);
}

bool PerimeterWireDriver::setBufferIndex(uint16_t index)
{
  app_->setReg16(REGISTER_BUFFER_INDEX, index);
}

bool PerimeterWireDriver::getBufferLength(uint16_t& length)
{
  return app_->getReg16(REGISTER_BUFFER_LENGTH, length);
}

bool PerimeterWireDriver::getBufferValue(uint16_t& value)
{
  return app_->getReg16(REGISTER_BUFFER_VALUE, value);
}
