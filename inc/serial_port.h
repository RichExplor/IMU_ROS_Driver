#pragma once

#include <boost/asio.hpp>
#include <cstdint>
#include <memory>
#include <string>

/// @brief 串口通信封装，支持带超时的同步读取
class SerialPort {
public:
  /// @brief 构造函数
  /// @param port 串口设备路径，如 /dev/ttyUSB0
  /// @param baud 波特率
  /// @param timeout_ms 读取超时时间（毫秒），0 表示阻塞
  SerialPort(const std::string& port, int baud, int timeout_ms = 100);

  ~SerialPort();

  /// @brief 打开串口
  /// @return 成功返回 true
  bool open();

  /// @brief 关闭串口
  void close();

  /// @brief 串口是否已打开
  bool isOpen() const;

  /// @brief 带超时的同步读取
  /// @param buf 接收缓冲区
  /// @param max_len 缓冲区最大长度
  /// @return 实际读取的字节数，超时或错误返回 0
  size_t read(uint8_t* buf, size_t max_len);

  /// @brief 设置读取超时
  /// @param timeout_ms 超时时间（毫秒）
  void setTimeout(int timeout_ms);

  /// @brief 获取串口设备路径
  const std::string& port() const {
    return port_;
  }

  /// @brief 获取波特率
  int baud() const {
    return baud_;
  }

private:
  boost::asio::io_service                   io_;
  std::unique_ptr<boost::asio::serial_port> serial_;
  std::string                               port_;
  int                                       baud_;
  int                                       timeout_ms_;
};
