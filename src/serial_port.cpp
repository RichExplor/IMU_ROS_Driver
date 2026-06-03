#include "serial_port.h"
#include <boost/asio/deadline_timer.hpp>
#include <boost/bind.hpp>
#include <ros/ros.h>

SerialPort::SerialPort(const std::string& port, int baud, int timeout_ms)
    : port_(port), baud_(baud), timeout_ms_(timeout_ms), serial_(nullptr) {
}

SerialPort::~SerialPort() {
  close();
}

bool SerialPort::open() {
  try {
    serial_ = std::make_unique<boost::asio::serial_port>(io_, port_);
    serial_->set_option(boost::asio::serial_port_base::baud_rate(baud_));
    serial_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    ROS_INFO("Serial port opened: %s @ %d bps", port_.c_str(), baud_);
    return true;
  } catch (const boost::system::system_error& e) {
    ROS_ERROR("Failed to open serial port %s: %s", port_.c_str(), e.what());
    serial_.reset();
    return false;
  }
}

void SerialPort::close() {
  if (serial_ && serial_->is_open()) {
    boost::system::error_code ec;
    serial_->close(ec);
    if (ec) {
      ROS_WARN("Error closing serial port: %s", ec.message().c_str());
    } else {
      ROS_INFO("Serial port closed: %s", port_.c_str());
    }
  }
  serial_.reset();
}

bool SerialPort::isOpen() const {
  return serial_ && serial_->is_open();
}

size_t SerialPort::read(uint8_t* buf, size_t max_len) {
  if (!isOpen())
    return 0;

  if (timeout_ms_ <= 0) {
    // 阻塞读取（无超时）
    boost::system::error_code ec;
    size_t                    n = serial_->read_some(boost::asio::buffer(buf, max_len), ec);
    if (ec) {
      ROS_ERROR("Serial read error: %s", ec.message().c_str());
      return 0;
    }
    return n;
  }

  // 带超时的异步读取
  size_t                    bytes_read = 0;
  boost::system::error_code read_ec;

  boost::asio::deadline_timer timer(io_, boost::posix_time::milliseconds(timeout_ms_));

  // 设置超时回调：取消串口读取
  timer.async_wait([this](const boost::system::error_code& ec_timer) {
    if (!ec_timer && serial_) {
      serial_->cancel();
    }
  });

  // 启动异步读取
  serial_->async_read_some(boost::asio::buffer(buf, max_len),
                           [&bytes_read, &read_ec](const boost::system::error_code& ec, size_t n) {
                             read_ec = ec;
                             if (!ec || ec == boost::asio::error::operation_aborted) {
                               bytes_read = n;
                             }
                           });

  // 运行 I/O 服务直到其中一个操作完成
  io_.restart();
  io_.run_one();

  // 取消可能仍在等待的定时器
  timer.cancel();

  // 处理读取错误（operation_aborted 是超时导致的，不算错误）
  if (read_ec && read_ec != boost::asio::error::operation_aborted) {
    ROS_ERROR("Serial read error: %s", read_ec.message().c_str());
    return 0;
  }

  return bytes_read;
}

void SerialPort::setTimeout(int timeout_ms) {
  timeout_ms_ = timeout_ms;
}
