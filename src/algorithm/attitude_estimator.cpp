#include "algorithm/attitude_estimator.h"
#include <algorithm>
#include <cstring>

namespace imu_algorithm {

AttitudeEstimator::AttitudeEstimator(AlgorithmType algo, AxisMode axis_mode, double alpha_acc, double alpha_mag)
    : algo_(algo), axis_mode_(axis_mode), alpha_acc_(alpha_acc), alpha_mag_(alpha_mag) {
  // 根据算法类型创建对应的滤波器实例
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    comp_filter_ = std::make_unique<ComplementaryFilter>(toCompAxisMode(axis_mode_), alpha_acc_, alpha_mag_);
  } else {
    rk4_filter_ = std::make_unique<RK4Integration>(toRK4AxisMode(axis_mode_), alpha_acc_, alpha_mag_);
  }
}

void AttitudeEstimator::update(double gx, double gy, double gz, double ax, double ay, double az, double dt) {
  update(Eigen::Vector3d(gx, gy, gz), Eigen::Vector3d(ax, ay, az), dt);
}

void AttitudeEstimator::update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my,
                               double mz, double dt) {
  update(Eigen::Vector3d(gx, gy, gz), Eigen::Vector3d(ax, ay, az), Eigen::Vector3d(mx, my, mz), dt);
}

void AttitudeEstimator::update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, double dt) {
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    comp_filter_->update(gyro, accel, dt);
  } else {
    rk4_filter_->update(gyro, accel, dt);
  }
}

void AttitudeEstimator::update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, const Eigen::Vector3d& mag,
                               double dt) {
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    comp_filter_->update(gyro, accel, mag, dt);
  } else {
    rk4_filter_->update(gyro, accel, mag, dt);
  }
}

const Eigen::Quaterniond& AttitudeEstimator::quaternion() const {
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    return comp_filter_->quaternion();
  } else {
    return rk4_filter_->quaternion();
  }
}

void AttitudeEstimator::eulerAngle(double& roll, double& pitch, double& yaw) const {
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    comp_filter_->eulerAngle(roll, pitch, yaw);
  } else {
    rk4_filter_->eulerAngle(roll, pitch, yaw);
  }
}

void AttitudeEstimator::reset() {
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    comp_filter_->reset();
  } else {
    rk4_filter_->reset();
  }
}

void AttitudeEstimator::setAlgorithm(AlgorithmType algo) {
  if (algo == algo_)
    return;

  algo_ = algo;

  // 释放旧算法，创建新算法
  comp_filter_.reset();
  rk4_filter_.reset();

  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    comp_filter_ = std::make_unique<ComplementaryFilter>(toCompAxisMode(axis_mode_), alpha_acc_, alpha_mag_);
  } else {
    rk4_filter_ = std::make_unique<RK4Integration>(toRK4AxisMode(axis_mode_), alpha_acc_, alpha_mag_);
  }
}

void AttitudeEstimator::setAxisMode(AxisMode mode) {
  axis_mode_ = mode;

  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    comp_filter_->setAxisMode(toCompAxisMode(mode));
  } else {
    rk4_filter_->setAxisMode(toRK4AxisMode(mode));
  }
}

void AttitudeEstimator::setAlphaAcc(double alpha) {
  alpha_acc_ = alpha;
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    comp_filter_->setAlphaAcc(alpha);
  } else {
    rk4_filter_->setAlphaAcc(alpha);
  }
}

void AttitudeEstimator::setAlphaMag(double alpha) {
  alpha_mag_ = alpha;
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    comp_filter_->setAlphaMag(alpha);
  } else {
    rk4_filter_->setAlphaMag(alpha);
  }
}

double AttitudeEstimator::alphaAcc() const {
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    return comp_filter_->alphaAcc();
  } else {
    return rk4_filter_->alphaAcc();
  }
}

double AttitudeEstimator::alphaMag() const {
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    return comp_filter_->alphaMag();
  } else {
    return rk4_filter_->alphaMag();
  }
}

void AttitudeEstimator::setAccelRejectionThreshold(double threshold) {
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    comp_filter_->setAccelRejectionThreshold(threshold);
  } else {
    rk4_filter_->setAccelRejectionThreshold(threshold);
  }
}

const char* AttitudeEstimator::algorithmName() const {
  switch (algo_) {
  case AlgorithmType::COMPLEMENTARY:
    return "complementary";
  case AlgorithmType::RK4:
    return "rk4";
  default:
    return "unknown";
  }
}

const char* AttitudeEstimator::axisModeName() const {
  switch (axis_mode_) {
  case AxisMode::SIX_AXIS:
    return "6-axis";
  case AxisMode::NINE_AXIS:
    return "9-axis";
  default:
    return "unknown";
  }
}

AttitudeEstimator::AlgorithmType AttitudeEstimator::algorithmFromString(const std::string& name) {
  std::string lower = name;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

  if (lower == "rk4" || lower == "runge_kutta" || lower == "rungekutta") {
    return AlgorithmType::RK4;
  }
  // 默认返回互补滤波
  return AlgorithmType::COMPLEMENTARY;
}

AttitudeEstimator::AxisMode AttitudeEstimator::axisModeFromString(const std::string& name) {
  std::string lower = name;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

  if (lower == "6" || lower == "6axis" || lower == "six_axis" || lower == "six" || lower == "6-axis" ||
      lower == "6_axis") {
    return AxisMode::SIX_AXIS;
  }
  // 默认返回 9 轴
  return AxisMode::NINE_AXIS;
}

ComplementaryFilter::AxisMode AttitudeEstimator::toCompAxisMode(AxisMode mode) {
  return (mode == AxisMode::SIX_AXIS) ? ComplementaryFilter::AxisMode::SIX_AXIS
                                      : ComplementaryFilter::AxisMode::NINE_AXIS;
}

RK4Integration::AxisMode AttitudeEstimator::toRK4AxisMode(AxisMode mode) {
  return (mode == AxisMode::SIX_AXIS) ? RK4Integration::AxisMode::SIX_AXIS : RK4Integration::AxisMode::NINE_AXIS;
}

} // namespace imu_algorithm
