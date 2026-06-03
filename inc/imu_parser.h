#pragma once

#include <array>
#include <cstdint>
#include <vector>

/// @brief IMU 原始数据（解析后、缩放前）
struct ImuRawData {
  int16_t ax, ay, az; ///< 线性加速度原始值
  int16_t wx, wy, wz; ///< 角速度原始值
  int16_t hx, hy, hz; ///< 磁场原始值
  bool    valid;      ///< 校验是否通过
};

/// @brief IMU 二进制协议解析器
///
/// 协议格式（24 字节/帧）：
///   [0..3]  帧头 0x4E 0x4A 0x13 0x01
///   [4..21] 数据（9 个 int16：ax,ay,az,wx,wy,wz,hx,hy,hz）
///   [22..23] 校验和（uint16 小端，前 22 字节之和）
class ImuParser {
public:
  ImuParser();

  /// @brief 向解析器输入新数据
  /// @param data 数据指针
  /// @param len 数据长度
  void feed(const uint8_t* data, size_t len);

  /// @brief 尝试解析一帧
  /// @param out 输出解析结果
  /// @return 成功解析返回 true，数据不足或校验失败返回 false
  bool parse(ImuRawData& out);

  /// @brief 获取校验失败计数
  size_t checksumFailCount() const {
    return checksum_fails_;
  }

  /// @brief 获取成功解析帧数
  size_t frameCount() const {
    return frame_count_;
  }

  // 协议常量 — 公开以便测试
  static constexpr uint8_t HEADER_BYTE0  = 0x4E;
  static constexpr uint8_t HEADER_BYTE1  = 0x4A;
  static constexpr uint8_t HEADER_BYTE2  = 0x13;
  static constexpr uint8_t HEADER_BYTE3  = 0x01;
  static constexpr size_t  FRAME_SIZE    = 24;  ///< 完整帧长度（字节）
  static constexpr size_t  DATA_SIZE     = 22;  ///< 参与校验的数据长度
  static constexpr size_t  CHECKSUM_SIZE = 2;   ///< 校验和长度
  static constexpr size_t  HEADER_SIZE   = 4;   ///< 帧头长度
  static constexpr size_t  READ_BUF_SIZE = 256; ///< 单次读取缓冲区大小

private:
  std::vector<uint8_t>                buf_;
  static const std::array<uint8_t, 4> HEADER_;
  size_t                              checksum_fails_;
  size_t                              frame_count_;

  /// @brief 验证指定位置的帧校验和
  bool validateChecksum(size_t pos) const;

  /// @brief 从缓冲区指定偏移读取 int16（小端）
  int16_t readI16(size_t offset) const;
};
