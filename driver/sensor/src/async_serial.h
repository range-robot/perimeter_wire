
/*
 * AsyncSerial
 * Version: 24.09.19
 * 
 * Changelog:
 * 08.05.19: replace boost::function with std::function (except asio)
 * 24.09.19: extract com_config.h
 * 24.09.19: add bulk write method
 * 02.03.20: add error callback
 */

#ifndef SRC_ASYNC_SERIAL_H_
#define SRC_ASYNC_SERIAL_H_

#include "com_config.h"
#define BOOST_BIND_NO_PLACEHOLDERS
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <functional>
#include <vector>
#include <string>

class AsyncSerial {
private:
    boost::asio::io_service io_;
    boost::asio::serial_port serial_port_;
    std::function<void(const uint8_t)> read_callback_;
    std::function<void (const boost::system::error_code&)> error_callback_;
    std::vector<uint8_t> in_;

    void readEnd(const boost::system::error_code& error,
                 const std::size_t bytes_transfered) {
        if (error) {
            if (error_callback_)
                error_callback_(error);
            else
                ROS_ERROR("IO read error (%d): %s", error.value(), error.message().c_str());
        }
        else if (read_callback_) {
            for (std::vector<uint8_t>::iterator it = in_.begin();
                    it != in_.begin() + bytes_transfered; ++it) {
                read_callback_(*it);
            }
        }

        serial_port_.async_read_some(
            boost::asio::buffer(in_),
            boost::bind(&AsyncSerial::readEnd, this,
                      boost::asio::placeholders::error,
                      boost::asio::placeholders::bytes_transferred));
    }

public:
    explicit AsyncSerial(const std::string& com_port) :
        serial_port_(io_) {
        in_.resize(RECEIVE_BUFFER_SIZE);
        serial_port_.open(com_port);
        serial_port_.set_option(boost::asio::serial_port_base::baud_rate(9600));
        serial_port_.set_option(boost::asio::serial_port_base::character_size(8));

        serial_port_.async_read_some(
            boost::asio::buffer(in_),
            boost::bind(&AsyncSerial::readEnd, this,
                      boost::asio::placeholders::error,
                      boost::asio::placeholders::bytes_transferred));
    }

    ~AsyncSerial() {
        if (serial_port_.is_open())
            serial_port_.close();
    }

    void setReadCallback(std::function<void(const uint8_t c)> readCallback) {
        read_callback_ = readCallback;
    }

    void setErrorCallback(std::function<void (const boost::system::error_code&)> errorCallback) {
        error_callback_ = errorCallback;
    }

    void write(uint8_t c) {
      boost::system::error_code ec;
      if (1 != boost::asio::write(serial_port_, boost::asio::buffer(&c, 1), ec))
      {
        if (error_callback_)
          error_callback_(ec);
        else
          ROS_ERROR("IO write error (%d): %s", ec.value(), ec.message().c_str());
      }
    }

    void write(std::vector<uint8_t> buffer) {
      boost::system::error_code ec;
      if (buffer.size() != boost::asio::write(serial_port_, boost::asio::buffer(buffer), ec))
      {
        if (error_callback_)
          error_callback_(ec);
        else
          ROS_ERROR("IO write error (%d): %s", ec.value(), ec.message().c_str());
      }
    }

    void run() {
      io_.run();
    }

    void runOnce() {
        io_.poll();
    }
    void stop() {
        io_.stop();
    }
};

#endif  // SRC_ASYNC_SERIAL_H_
