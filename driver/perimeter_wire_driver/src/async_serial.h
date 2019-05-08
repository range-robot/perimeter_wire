
/*
 * AsyncSerial
 * Version: 08.05.19
 * 
 * Changelog:
 * 08.05.19: replace boost::function with std::function (except asio)
 */

#ifndef SRC_ASYNC_SERIAL_H_
#define SRC_ASYNC_SERIAL_H_

#define BOOST_BIND_NO_PLACEHOLDERS
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <functional>
#include <vector>
#include <string>

// Recevive buffer must keep all bytes received during one cycle
#define RECEIVE_BUFFER_SIZE 2048

class AsyncSerial {
private:
    boost::asio::io_service io_;
    boost::asio::serial_port serial_port_;
    std::function<void(const uint8_t)> read_callback_;
    std::vector<uint8_t> in_;

    void readEnd(const boost::system::error_code& error,
                 const std::size_t bytes_transfered) {
        if (error) {
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

    void write(uint8_t c) {
      boost::system::error_code ec;
      if (1 != boost::asio::write(serial_port_, boost::asio::buffer(&c, 1), ec)) {
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
