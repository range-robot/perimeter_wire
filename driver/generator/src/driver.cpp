
#include <string>
#include <perimeter_wire_generator/driver.h>
#include <perimeter_wire_generator/registers.h>
#include "async_serial.h"
#include "data_link_layer.h"
#include "app_layer.h"
#include "console.h"


#define FIRMWARE_VERSION 1
using namespace perimeter_wire_generator;

GeneratorDriver::GeneratorDriver(const std::string& com_port)
{
  using namespace std::placeholders;
  serial_ = std::make_shared<AsyncSerial>(com_port);
  dll_ = std::make_shared<DataLink>(serial_);
  app_ = std::make_shared<AppLayer>(dll_);
  app_->setHelloCallback(std::bind(&GeneratorDriver::helloCallback, this, _1));
  app_->setCommandResultCallback(std::bind(&GeneratorDriver::cmdResultCallback, this, _1, _2));
}

GeneratorDriver::~GeneratorDriver()
{
  serial_->runOnce();
  serial_->stop();
}

void GeneratorDriver::helloCallback(uint16_t version)
{
  if (version != FIRMWARE_VERSION)
  {
    ROS_ERROR("Firmware version incompatible. Excpected: %i, Got: %i", FIRMWARE_VERSION, (int)version);
  }
}

void GeneratorDriver::cmdResultCallback(uint8_t cmd, uint8_t result)
{
  ROS_ERROR("Received orphanded command result. Cmd: %i, Result: %x", (int)cmd, (int)result);
}

void GeneratorDriver::run()
{
  serial_->run();
}

void GeneratorDriver::runOnce()
{
  serial_->runOnce();
}

void GeneratorDriver::stop()
{
  serial_->stop();
}

bool GeneratorDriver::getFWVersion(uint16_t& version)
{
  return app_->getVersion(version);
}

void GeneratorDriver::reset() {
  app_->reset();
}

bool GeneratorDriver::setControl(bool enabled)
{
  uint8_t control = enabled ? 0x01 : 0x00;
  app_->setReg(REGISTER_CONTROL, control);
  return true;
}

bool GeneratorDriver::getChannelAMode(uint8_t& mode)
{
  return app_->getReg(REGISTER_A_MODE, mode);
}

bool GeneratorDriver::setChannelAMode(uint8_t mode)
{
  app_->setReg(REGISTER_A_MODE, mode);
  return true;
}

bool GeneratorDriver::getChannelBMode(uint8_t& mode)
{
  return app_->getReg(REGISTER_B_MODE, mode);
}

bool GeneratorDriver::setChannelBMode(uint8_t mode)
{
  app_->setReg(REGISTER_B_MODE, mode);
  return true;
}

bool GeneratorDriver::getChannelAFrequency(uint16_t& frequency)
{
  return app_->getReg16(REGISTER_A_FREQ, frequency);
}

bool GeneratorDriver::setChannelAFrequency(uint16_t frequency)
{
  app_->setReg16(REGISTER_A_FREQ, frequency);
  return true;
}

bool GeneratorDriver::getChannelBFrequency(uint16_t& frequency)
{
  return app_->getReg16(REGISTER_B_FREQ, frequency);
}

bool GeneratorDriver::setChannelBFrequency(uint16_t frequency)
{
  app_->setReg16(REGISTER_B_FREQ, frequency);
  return true;
}