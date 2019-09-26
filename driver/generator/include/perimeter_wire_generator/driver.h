#ifndef PERIMETER_WIRE_GENERATOR_DRIVER_H_
#define PERIMETER_WIRE_GENERATOR_DRIVER_H_

#include <string>
#include <memory>
#include <inttypes.h>

class AsyncSerial;
class DataLink;
class AppLayer;

namespace perimeter_wire_generator {

class GeneratorDriver {
private:
    std::shared_ptr<AsyncSerial> serial_;
    std::shared_ptr<DataLink> dll_;
    std::shared_ptr<AppLayer> app_;

    void helloCallback(uint16_t version);
    void cmdResultCallback(uint8_t cmd, uint8_t result);

public:
    explicit GeneratorDriver(const std::string& com_port);
    ~GeneratorDriver(void);

    void runOnce();
    void run();
    void stop();

    bool getFWVersion(uint16_t& version);
    void reset(void);

    bool setControl(bool enable);

    bool getChannelAMode(uint8_t& mode);
    bool setChannelAMode(uint8_t mode);
    bool getChannelBMode(uint8_t& mode);
    bool setChannelBMode(uint8_t mode);
    bool getChannelAFrequency(uint16_t& frequency);
    bool setChannelAFrequency(uint16_t frequency);
    bool getChannelBFrequency(uint16_t& frequency);
    bool setChannelBFrequency(uint16_t frequency);
};

}  // namespace perimeter_wire_generator

#endif  // PERIMETER_WIRE_GENERATOR_DRIVER_H_
