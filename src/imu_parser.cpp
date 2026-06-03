#include "imu_parser.h"
#include <algorithm>
#include <cstring>
#include <limits>

const std::array<uint8_t, 4> ImuParser::HEADER_ = {ImuParser::HEADER_BYTE0, ImuParser::HEADER_BYTE1,
                                                   ImuParser::HEADER_BYTE2, ImuParser::HEADER_BYTE3};

ImuParser::ImuParser() : checksum_fails_(0), frame_count_(0) {
  buf_.reserve(512); // 预分配缓冲区，减少频繁扩容
}

void ImuParser::feed(const uint8_t* data, size_t len) {
  buf_.insert(buf_.end(), data, data + len);
}

bool ImuParser::parse(ImuRawData& out) {
  // 需要至少一帧的数据才可能解析
  while (buf_.size() >= FRAME_SIZE) {
    // 查找帧头
    auto it = std::search(buf_.begin(), buf_.end(), HEADER_.begin(), HEADER_.end());
    if (it == buf_.end()) {
      // 未找到帧头，保留最后 HEADER_SIZE-1 字节（可能是部分帧头）
      if (buf_.size() > HEADER_SIZE - 1) {
        buf_.erase(buf_.begin(), buf_.end() - (HEADER_SIZE - 1));
      }
      return false;
    }

    size_t pos = static_cast<size_t>(std::distance(buf_.begin(), it));

    // 丢弃帧头之前的垃圾数据
    if (pos > 0) {
      buf_.erase(buf_.begin(), buf_.begin() + pos);
      pos = 0;
    }

    // 检查是否有完整帧
    if (buf_.size() < FRAME_SIZE) {
      return false; // 等待更多数据
    }

    // 校验和验证
    if (!validateChecksum(pos)) {
      if (checksum_fails_ == std::numeric_limits<size_t>::max())
        checksum_fails_ = 0;
      else
        ++checksum_fails_;
      // 跳过当前帧头的第一个字节，重新搜索
      buf_.erase(buf_.begin());
      continue;
    }

    // 提取数据字段（小端 int16）
    out.ax    = readI16(HEADER_SIZE + 0);
    out.ay    = readI16(HEADER_SIZE + 2);
    out.az    = readI16(HEADER_SIZE + 4);
    out.wx    = readI16(HEADER_SIZE + 6);
    out.wy    = readI16(HEADER_SIZE + 8);
    out.wz    = readI16(HEADER_SIZE + 10);
    out.hx    = readI16(HEADER_SIZE + 12);
    out.hy    = readI16(HEADER_SIZE + 14);
    out.hz    = readI16(HEADER_SIZE + 16);
    out.valid = true;

    // 移除已处理的帧
    buf_.erase(buf_.begin(), buf_.begin() + FRAME_SIZE);
    if (frame_count_ == std::numeric_limits<size_t>::max())
      frame_count_ = 0;
    else
      ++frame_count_;
    return true;
  }

  return false;
}

bool ImuParser::validateChecksum(size_t pos) const {
  // 计算前 DATA_SIZE 字节的累加和
  uint16_t sum = 0;
  for (size_t i = 0; i < DATA_SIZE; ++i) {
    sum += buf_[pos + i];
  }
  // 读取帧中的校验和（小端 uint16）
  uint16_t expected =
      static_cast<uint16_t>(buf_[pos + DATA_SIZE]) | (static_cast<uint16_t>(buf_[pos + DATA_SIZE + 1]) << 8);
  return sum == expected;
}

int16_t ImuParser::readI16(size_t offset) const {
  return static_cast<int16_t>(static_cast<uint16_t>(buf_[offset]) | (static_cast<uint16_t>(buf_[offset + 1]) << 8));
}
