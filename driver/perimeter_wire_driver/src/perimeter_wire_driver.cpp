
#include <string>
#include <ros/console.h>
#include <perimeter_wire_driver/perimeter_wire_driver.h>
#include <perimeter_wire_firmware/config.h>
#include "async_serial.h"
#include "data_link_layer.h"
#include "app_layer.h"


#define FIRMWARE_VERSION 1
#define ADC_MAX (1024.0f)
using namespace perimeter_wire_driver;

const uint8_t channel_register_map[] = {
  REGISTER_CHANNEL_A,
  REGISTER_CHANNEL_B,
  REGISTER_CHANNEL_C,
  REGISTER_CHANNEL_D
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

bool PerimeterWireDriver::setControl(bool enabled)
{
  uint8_t control = enabled ? 0x01 : 0x00;
  app_->setReg(REGISTER_CONTROL, control);
}

bool PerimeterWireDriver::setEnabled(uint8_t enabled)
{
  app_->setReg(REGISTER_ENABLED, enabled);
}

bool PerimeterWireDriver::getChannel(int channel, float& value)
{
  if (channel < 0 || channel >= sizeof(channel_register_map))
    return false;
  uint8_t reg = channel_register_map[channel];
  uint16_t val16;
  bool res = app_->getReg16(reg, val16);
  value = val16 / ADC_MAX;
  return res;
}
