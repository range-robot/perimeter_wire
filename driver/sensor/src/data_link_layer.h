/*
 * DataLinkLayer
 * Version: 24.09.19
 *
 * Changelog:
 * 08.05.19: replace boost with std types
 * 24.09.19: extract com_config.h
 * 24.09.19: add sendMessage function (buffered)
 */
#ifndef SRC_DATA_LINK_LAYER_H_
#define SRC_DATA_LINK_LAYER_H_

#include "com_config.h"
#include <vector>
#include <memory>
#include <functional>

class AsyncSerial;

class DataLink {
private:
    std::shared_ptr<AsyncSerial> async_serial_;
    enum {
        STATE_IDLE,
        STATE_MESSAGE,
        STATE_ESC
    } receive_state_;
    std::vector<uint8_t> receive_buffer_;
    std::function<void(const std::vector<uint8_t>&) > message_callback_;
    uint8_t send_checksum_;

    void readCallback(const uint8_t c) {
        static uint8_t checksum = 0;
#ifdef DEBUG_DDL
        ROS_INFO("IN CHAR: %x (%c). STATE: %i", (int)c, (c > ' ') ? c : ' ', receive_state_);
#endif
        switch (receive_state_) {
            case STATE_IDLE:
                if (c == DATALINK_START_FLAG) {
                    // reset receive buffer
                    checksum = 0;
                    receive_buffer_.clear();
                    receive_state_ = STATE_MESSAGE;
                }
                else {
                    ROS_WARN("Discard orphaned char. %x", (int)c);
                }
                break;
            case STATE_MESSAGE:
                if (c == DATALINK_END_FLAG
#if DATALINK_END_FLAG == DATALINK_START_FLAG
                        && receive_buffer_.size() != 0
#endif
                        ) {
                    receive_state_ = STATE_IDLE;
                    if (checksum == 0) {
                        if (message_callback_) {
                            // remove checksum
                            receive_buffer_.pop_back();
                            message_callback_(receive_buffer_);
                        }
                    }
                    else {
                        ROS_ERROR("Received message with invalid checksum.");
                    }
                } else if (c == DATALINK_START_FLAG) {
                    // reset receive buffer
                    checksum = 0;
                    receive_buffer_.clear();
                    receive_state_ = STATE_MESSAGE;
                } else if (c == DATALINK_ESC) {
                    receive_state_ = STATE_ESC;
                } else {
                    checksum ^= c;
                    receive_buffer_.push_back(c);
                }
                break;
            case STATE_ESC:
#if DATALINK_ESC_MASK != 0
                if (c == DATALINK_START_FLAG) {
                    // This makes resynchronization quicker.
                    // reset receive buffer
                    checksum = 0;
                    receive_buffer_.clear();
                    receive_state_ = STATE_MESSAGE;
                } else {
#else
                {
#endif
                    uint8_t c2 = DATALINK_ESC_MASK ^ c;
                    checksum ^= c2;
                    receive_buffer_.push_back(c2);
                    receive_state_ = STATE_MESSAGE;
                }
                break;
        }
    }

public:
    explicit DataLink(const std::shared_ptr<AsyncSerial> async_serial) :
        receive_state_(STATE_IDLE),
        async_serial_(async_serial) {
        using namespace std::placeholders;
        async_serial_->setReadCallback(std::bind(&DataLink::readCallback, this, _1));
    }

    void setMessageCallback(std::function<void(const std::vector<uint8_t>&)> callback) {
        message_callback_ = callback;
    }

private:
    int getLen()
    {
        return 0;
    }
    template<typename... Args>
    int getLen(uint8_t arg, Args... args)
    {
        return 1 + getLen(args...);
    }
    template<typename... Args>
    int getLen(std::vector<uint8_t> arg, Args... args)
    {
        return arg.size() + getLen(args...);
    }

    inline void putToBuffer(std::vector<uint8_t>& buffer, uint8_t& checksum, uint8_t c)
    {
#ifdef DEBUG_DDL
        ROS_INFO("OUT CHAR: %x (%c)", (int)c, (c > ' ') ? c : ' ');
#endif
        checksum ^= c;
        if (c == DATALINK_START_FLAG ||
            c == DATALINK_END_FLAG ||
            c == DATALINK_ESC) {
            buffer.push_back(DATALINK_ESC);
            buffer.push_back(c ^ DATALINK_ESC_MASK);
        } else {
            buffer.push_back(c);
        }
    }
    void copyBuffer(std::vector<uint8_t>& buffer, uint8_t& checksum)
    {
    }
    template<typename... Args>
    void copyBuffer(std::vector<uint8_t>& buffer, uint8_t& checksum, uint8_t arg, Args... args)
    {
        putToBuffer(buffer, checksum, arg);
        copyBuffer(buffer, checksum, args...);
    }
    template<typename... Args>
    void copyBuffer(std::vector<uint8_t>& buffer, uint8_t& checksum, std::vector<uint8_t> arg, Args... args)
    {
        for (auto it = arg.begin(); it != arg.end(); ++it)
            putToBuffer(buffer, checksum, *it);
        copyBuffer(buffer, checksum, args...);
    }

public:
    template<typename... Args>
    void sendMessage(Args... args) // recursive variadic function
    {
        // create message buffer. (add some space for escapes)
        int len = getLen(args...);
        std::vector<uint8_t> buffer;
        buffer.reserve(len + 3 + len / 4);
        
        // begin message
        buffer.push_back(DATALINK_START_FLAG);

        // content
        uint8_t checksum = 0;
        copyBuffer(buffer, checksum, args...);

        // end message
        putToBuffer(buffer, checksum, checksum);
        buffer.push_back(DATALINK_END_FLAG);

#ifdef DEBUG_DDL
        ROS_INFO("OUT MSG len: %d", len + 3);
#endif
        async_serial_->write(buffer);
    }

    void beginMessage() {
#ifdef DEBUG_DDL
        ROS_INFO("OUT BEGIN MSG");
#endif
        send_checksum_ = 0;
        async_serial_->write(DATALINK_START_FLAG);
        ROS_INFO("OUT BEGIN MSG2");
    }

    void putMessageChar(const uint8_t c) {
#ifdef DEBUG_DDL
        ROS_INFO("OUT CHAR: %x (%c)", (int)c, (c > ' ') ? c : ' ');
#endif
        send_checksum_ ^= c;
        if (c == DATALINK_START_FLAG ||
            c == DATALINK_END_FLAG ||
            c == DATALINK_ESC) {
            async_serial_->write(DATALINK_ESC);
            async_serial_->write(c ^ DATALINK_ESC_MASK);
        } else {
            async_serial_->write(c);
        }
    }

    void putMessageData(const std::vector<uint8_t> data) {
        for (std::vector<uint8_t>::const_iterator it = data.begin(); it != data.end(); ++it) {
            putMessageChar(*it);
        }
    }

    void endMessage() {
#ifdef DEBUG_DDL
        ROS_INFO("OUT END MSG");
#endif
        putMessageChar(send_checksum_);
        async_serial_->write(DATALINK_END_FLAG);
    }
};

#endif  // SRC_DATA_LINK_LAYER_H_
