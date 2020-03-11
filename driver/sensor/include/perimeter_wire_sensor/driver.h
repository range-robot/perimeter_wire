#ifndef PERIMETER_WIRE_SENSOR_PERIMETER_WIRE_DRIVER_H_
#define PERIMETER_WIRE_SENSOR_PERIMETER_WIRE_DRIVER_H_

#include <memory>

class AsyncSerial;
class DataLink;
class AppLayer;

namespace perimeter_wire_sensor {

class PerimeterWireDriver {
private:
    std::shared_ptr<AsyncSerial> serial_;
    std::shared_ptr<DataLink> dll_;
    std::shared_ptr<AppLayer> app_;

    void helloCallback(uint16_t version);
    void cmdResultCallback(uint8_t cmd, uint8_t result);

public:
    explicit PerimeterWireDriver(const std::string& com_port);
    ~PerimeterWireDriver(void);

    void runOnce();
    void run();
    void stop();

    bool getFWVersion(uint16_t& version);
    void reset(bool bootload=false);

    bool getControl(bool& enabled);
    bool setControl(bool enable);
    bool setFlags(uint8_t flags);

    bool setDivider(int channel, uint8_t divider);
    bool setCode(int channel, uint16_t code);
    bool setRepeat(int channel, uint8_t repeat);
    bool getChannel(int channel, float& value);

    // buffer access
    bool getBufferIndex(uint16_t& index);
    bool setBufferIndex(uint16_t index);
    bool getBufferLength(uint16_t& index);
    bool getBufferValue(uint16_t& index);
};

}  // namespace perimeter_wire_sensor

#endif  // PERIMETER_WIRE_sensor_PERIMETER_WIRE_DRIVER_H_
