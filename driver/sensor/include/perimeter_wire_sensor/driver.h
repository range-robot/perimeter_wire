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
    void reset(void);

    bool setControl(bool enable);
    bool setEnabled(uint8_t channels);

    bool setDivider(int channel, uint8_t divider);
    bool getChannel(int channel, float& value);
};

}  // namespace perimeter_wire_sensor

#endif  // PERIMETER_WIRE_sensor_PERIMETER_WIRE_DRIVER_H_
