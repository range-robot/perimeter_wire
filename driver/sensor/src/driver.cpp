
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
  REGISTER_CHANNEL_C
};

const uint8_t quality_register_map[] = {
  REGISTER_CHANNEL_A_QUAL,
  REGISTER_CHANNEL_B_QUAL,
  REGISTER_CHANNEL_C_QUAL
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

void PerimeterWireDriver::reset(bool bootload) {
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


bool PerimeterWireDriver::getQuality(int channel, float& value)
{
  if (channel < 0 || channel >= sizeof(quality_register_map))
    return false;
  uint8_t reg = quality_register_map[channel];
  uint16_t val16;
  bool res = app_->getReg16(reg, val16);
  value = val16;
  return res;
}

bool PerimeterWireDriver::setDivider(uint8_t divider)
{
  app_->setReg(REGISTER_CHANNEL_DIVIDER, divider);
  return true;
}

bool PerimeterWireDriver::setCode(uint16_t code)
{
  app_->setReg16(REGISTER_CHANNEL_CODE, code);
  return true;
}

bool PerimeterWireDriver::setRepeat(uint8_t repeat)
{
  app_->setReg(REGISTER_CHANNEL_REPEAT, repeat);
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
