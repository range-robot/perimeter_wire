/*
 * AppLayer
 * Version: 11.03.20
 * 
 * Changelog:
 * 08.05.19: replace boost::function with std::function
 * 24.09.19: extract com_config.h
 * 24.09.19: use bulk write method
 * 11.03.20: add reset to bootloader (from 08.05.19 version)
 */

#ifndef SRC_APP_LAYER_H_
#define SRC_APP_LAYER_H_

#include "com_config.h"
#include <functional>
#include <memory>
#include <condition_variable>
#include <mutex>
#include <vector>
#include <chrono>

class AppLayer {
private:
    typedef std::chrono::steady_clock clock;

    std::function<void(uint16_t version)> hello_callback_;
    std::function<void(uint8_t cmd, uint8_t result)> cmd_result_callback_;
    std::function<void(uint8_t adr, uint8_t bytes)> register_callback_;

    struct Register {
        bool valid;
        uint8_t value;
    };

    std::shared_ptr<DataLink> dll_;
    std::vector<Register> registers_;
    bool version_valid_;
    uint16_t fw_version_;
    bool in_reset_;

    std::mutex sync_lock_;
    std::condition_variable read_condition_, version_condition_;;
    std::chrono::milliseconds timeout_;

    bool checkLen(bool check) {
        if (!check) {
            ROS_ERROR("Received message with invalid length.");
            return false;
        }
        return true;
    }
    inline uint16_t buildUInt16(uint8_t lowerAddress, uint8_t higherAddress) {
        return ((uint16_t)higherAddress) << 8 | lowerAddress;
    }
    inline int16_t readUInt16(const std::vector<uint8_t>& buffer, int pos) {
        uint8_t v1 = buffer[pos];
        uint8_t v2 = buffer[pos + 1];
        return buildUInt16(v1, v2);
    }
    void pushUInt16(std::vector<uint8_t>& buffer, uint16_t value) {
        buffer.push_back((uint8_t)(value & 0xff));
        buffer.push_back((uint8_t)(value >> 8));
    }

    void invalidateAllRegisters() {
        for (std::vector<Register>::iterator it = registers_.begin(); it != registers_.end(); ++it) {
            it->valid = false;
        }
    }
    void readMessage(const std::vector<uint8_t>& msg) {
        size_t len = msg.size();
        if (len == 0)
            ROS_ERROR("Received empty message.");
        else {
            uint8_t header = msg[0];
            switch (header) {
                case MESSAGE_HELLO:
                    if (checkLen(len == 3)) {
                        uint16_t version = readUInt16(msg, 1);
                        {
                            std::unique_lock<std::mutex> lock(sync_lock_);
                            fw_version_ = version;
                            version_valid_ = true;
                            version_condition_.notify_all();
                        }
                        in_reset_ = false;
                        invalidateAllRegisters();
                        if (hello_callback_)
                            hello_callback_(version);
                        ROS_INFO("Received hello. Version: %i", (int)version);
                    }
                    break;
                case MESSAGE_VERSION:
                    if (checkLen(len == 3)) {
                        uint16_t version2 = readUInt16(msg, 1);
                        {
                            std::unique_lock<std::mutex> lock(sync_lock_);
                            fw_version_ = version2;
                            version_valid_ = true;
                            version_condition_.notify_all();
                        }
                        ROS_INFO("Firmware version: %i", (int)version2);
                    }
                    break;
                case MESSAGE_ERROR:
                    if (checkLen(len == 2)) {
                        uint8_t err = msg[1];
                        ROS_ERROR("Firmware error: %i", (int)err);
                    }
                    break;
                case MESSAGE_REG_VALUE:
                    if (checkLen(len == 3)) {
                        uint8_t adr = msg[1];
                        if (adr >= APP_LAYER_MAX_REGISTER)
                            ROS_ERROR("Invalid register address %x", (int)adr);
                        else {
                            uint8_t val = msg[2];
                            ROS_DEBUG("REG: %x: %x", (int)adr, (int)val);
                            {
                                std::unique_lock<std::mutex> lock(sync_lock_);
                                registers_[adr].value = val;
                                registers_[adr].valid = true;
                                read_condition_.notify_all();
                            }
                            if (register_callback_)
                                register_callback_(adr, 1);
                        }
                    }
                    break;
                case MESSAGE_REG_VALUE_16:
                    if (checkLen(len == 4)) {
                        uint8_t adr = msg[1];
                        if (adr+1 >= APP_LAYER_MAX_REGISTER)
                            ROS_ERROR("Invalid register address %x", (int)adr);
                        else {
                            uint8_t val1 = msg[2];
                            uint8_t val2 = msg[3];
                            ROS_DEBUG("REG: %x: %x %x", (int)adr, (int)val1, (int) val2);
                            {
                                std::unique_lock<std::mutex> lock(sync_lock_);
                                registers_[adr].value = val1;
                                registers_[adr].valid = true;
                                registers_[adr+1].value = val2;
                                registers_[adr+1].valid = true;
                                read_condition_.notify_all();
                            }
                            if (register_callback_)
                                register_callback_(adr, 2);
                        }
                    }
                    break;
                case MESSAGE_COMMAND_RESULT:
                    if (checkLen(len ==3)) {
                        uint8_t cmd = msg[1];
                        uint8_t res = msg[2];
                        if (cmd_result_callback_)
                            cmd_result_callback_(cmd, res);
                        ROS_DEBUG("Command %i finished. Result: %x", (int)cmd, (int)res);
                    }
                    break;
                default:
                    ROS_ERROR("Received unknown message. Header: %x", (int)header);
                    break;
            }
        }
    }

    void putMessage(uint8_t header, std::vector<uint8_t> data) {
        dll_->sendMessage(header, data);
    }

public:
    explicit AppLayer(const std::shared_ptr<DataLink> data_link, double timeout = 1.0) :
        dll_(data_link),
        version_valid_(false),
        in_reset_(false),
        timeout_(static_cast<int>(timeout*1000.))
    {
        using namespace std::placeholders;
        dll_->setMessageCallback(std::bind(&AppLayer::readMessage, this, _1));
        registers_.resize(APP_LAYER_MAX_REGISTER);
    }

    void setHelloCallback(const std::function<void(uint16_t version)>& cb) {
        hello_callback_ = cb;
    }
    void setCommandResultCallback(const std::function<void(uint8_t, uint8_t)>& cb) {
        cmd_result_callback_ = cb;
    }
    void setRegisterCallback(const std::function<void(uint8_t, uint8_t)>& cb) {
        register_callback_ = cb;
    }

    void beginGetVersion() {
        std::vector<uint8_t> data = {};
        version_valid_ = false;
        putMessage(MESSAGE_GET_VERSION, data);
    }
    bool endGetVersion(uint16_t& version) {
        if (version_valid_) {
            version = fw_version_;
            return true;
        }
        return false;
    }
    bool getVersion(uint16_t& val) {
        std::unique_lock<std::mutex> lock(sync_lock_);
        clock::time_point end = clock::now() + timeout_;
        beginGetVersion();
        while (!endGetVersion(val)) {
        if (std::cv_status::timeout == version_condition_.wait_until(lock, end))
            return false;
        }
        return true;
    }

    void setReg(uint8_t adr, uint8_t val) {
        if (adr >= APP_LAYER_MAX_REGISTER) {
            BOOST_THROW_EXCEPTION(std::range_error("Index out of range"));
        }
        registers_[adr].valid = false;
        std::vector<uint8_t> data = {adr, val};
        putMessage(MESSAGE_SET_REG, data);
    }

    void setReg16(uint8_t adr, uint16_t val) {
        if (adr+1>= APP_LAYER_MAX_REGISTER) {
            BOOST_THROW_EXCEPTION(std::range_error("Index out of range"));
        }
        registers_[adr].valid = false;
        registers_[adr+1].valid = false;
        std::vector<uint8_t> data = {adr};
        pushUInt16(data, val);
        putMessage(MESSAGE_SET_REG_16, data);
    }

    void beginGetReg(uint8_t adr) {
        if (adr >= APP_LAYER_MAX_REGISTER) {
            BOOST_THROW_EXCEPTION(std::range_error("Index out of range"));
        }
        registers_[adr].valid = false;
        std::vector<uint8_t> data = {adr};
        putMessage(MESSAGE_GET_REG, data);
    }
    bool endGetReg(uint8_t adr, uint8_t& val) {
        if (adr >= APP_LAYER_MAX_REGISTER) {
            BOOST_THROW_EXCEPTION(std::range_error("Index out of range"));
        }
        if (registers_[adr].valid) {
            val = registers_[adr].value;
            return true;
        }
        return false;
    }
    bool getReg(uint8_t adr, uint8_t& val) {
        std::unique_lock<std::mutex> lock(sync_lock_);
        clock::time_point end = clock::now() + timeout_;
        beginGetReg(adr);
        while (!endGetReg(adr, val)) {
          if (std::cv_status::timeout == read_condition_.wait_until(lock, end))
            return false;
        }
        return true;
    }

    void beginGetReg16(uint8_t adr) {
        if (adr+1 >= APP_LAYER_MAX_REGISTER) {
            BOOST_THROW_EXCEPTION(std::range_error("Index out of range"));
        }
        registers_[adr].valid = false;
        registers_[adr+1].valid = false;
        std::vector<uint8_t> data = {adr};
        putMessage(MESSAGE_GET_REG_16, data);
    }
    bool endGetReg16(uint8_t adr, uint16_t& val) {
        if (adr+1 >= APP_LAYER_MAX_REGISTER) {
            BOOST_THROW_EXCEPTION(std::range_error("Index out of range"));
        }
        if (registers_[adr].valid && registers_[adr+1].valid) {
            uint8_t val1 = registers_[adr].value;
            uint8_t val2 = registers_[adr+1].value;
            val = buildUInt16(val1, val2);
            return true;
        }
        return false;
    }
    bool getReg16(uint8_t adr, uint16_t& val) {
       std::unique_lock<std::mutex> lock(sync_lock_);
       clock::time_point end = clock::now() + timeout_;
       beginGetReg16(adr);
       while (!endGetReg16(adr, val)) {
         if (std::cv_status::timeout == read_condition_.wait_until(lock, end))
           return false;
       }
       return true;
    }

    void command(uint8_t cmd) {
        std::vector<uint8_t> data = {cmd};
        putMessage(MESSAGE_COMMAND, data);
    }

    void reset(bool bootloader = false) {
        in_reset_ = true;
        if (bootloader) {
            std::vector<uint8_t> data = {1};
            putMessage(MESSAGE_RESET, data);
        } else {
            std::vector<uint8_t> data = {};
            putMessage(MESSAGE_RESET, data);
        }
    }

    bool isActive() {
        return !in_reset_;
    }
};

#endif  // SRC_APP_LAYER_H_
