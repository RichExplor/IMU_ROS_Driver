#pragma once

#include "algorithm/complementary_filter.h"
#include "algorithm/rk4_integration.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <string>

namespace imu_algorithm {

/// @brief 姿态解算管理器，提供可配置的算法和轴数选择（基于 Eigen 实现）
///
/// 支持两种算法：
/// - 互补滤波（ComplementaryFilter）：计算量小，适合嵌入式场景
/// - 四阶龙格库塔（RK4Integration）：高精度积分，适合高动态场景
///
/// 支持两种轴数模式：
/// - 6 轴（加速度计 + 陀螺仪）：yaw 会漂移
/// - 9 轴（加速度计 + 陀螺仪 + 磁力计）：全姿态无漂移
///
/// 使用示例：
/// @code
///   AttitudeEstimator estimator(
///       AttitudeEstimator::AlgorithmType::COMPLEMENTARY,
///       AttitudeEstimator::AxisMode::NINE_AXIS);
///   estimator.update(gyro, accel, mag, dt);
///   const auto& q = estimator.quaternion();
/// @endcode
class AttitudeEstimator {
public:
  /// @brief 算法类型
  enum class AlgorithmType {
    COMPLEMENTARY, ///< 互补滤波
    RK4            ///< 四阶龙格库塔
  };

  /// @brief 轴数模式
  enum class AxisMode {
    SIX_AXIS  = 6, ///< 6 轴
    NINE_AXIS = 9  ///< 9 轴
  };

  /// @brief 构造函数
  /// @param algo 算法类型
  /// @param axis_mode 轴数模式
  /// @param alpha_acc 加速度计融合/修正系数
  /// @param alpha_mag 磁力计融合/修正系数
  AttitudeEstimator(AlgorithmType algo = AlgorithmType::COMPLEMENTARY, AxisMode axis_mode = AxisMode::NINE_AXIS,
                    double alpha_acc = 0.02, double alpha_mag = 0.01);

  /// @brief 更新姿态（6轴，标量接口）
  void update(double gx, double gy, double gz, double ax, double ay, double az, double dt);

  /// @brief 更新姿态（9轴，标量接口）
  void update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz,
              double dt);

  /// @brief 更新姿态（6轴，Eigen 向量接口）
  /// @param gyro 陀螺仪角速度（rad/s，机体系）
  /// @param accel 加速度计（m/s²，机体系）
  /// @param dt 时间步长（秒）
  void update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, double dt);

  /// @brief 更新姿态（9轴，Eigen 向量接口）
  /// @param gyro 陀螺仪角速度（rad/s，机体系）
  /// @param accel 加速度计（m/s²，机体系）
  /// @param mag 磁力计（任意单位，机体系）
  /// @param dt 时间步长（秒）
  void update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, const Eigen::Vector3d& mag, double dt);

  /// @brief 获取当前姿态四元数
  const Eigen::Quaterniond& quaternion() const;

  /// @brief 获取欧拉角
  /// @param roll  横滚角（弧度）
  /// @param pitch 俯仰角（弧度）
  /// @param yaw   偏航角（弧度）
  void eulerAngle(double& roll, double& pitch, double& yaw) const;

  /// @brief 重置姿态为单位四元数
  void reset();

  /// @brief 切换算法类型（会重置姿态）
  void setAlgorithm(AlgorithmType algo);

  /// @brief 获取当前算法类型
  AlgorithmType algorithm() const {
    return algo_;
  }

  /// @brief 切换轴数模式（不重置姿态）
  void setAxisMode(AxisMode mode);

  /// @brief 获取当前轴数模式
  AxisMode axisMode() const {
    return axis_mode_;
  }

  /// @brief 设置加速度计融合/修正系数
  void setAlphaAcc(double alpha);

  /// @brief 设置磁力计融合/修正系数
  void setAlphaMag(double alpha);

  /// @brief 获取加速度计融合/修正系数
  double alphaAcc() const;

  /// @brief 获取磁力计融合/修正系数
  double alphaMag() const;

  /// @brief 设置加速度计异常检测阈值
  void setAccelRejectionThreshold(double threshold);

  /// @brief 获取算法名称字符串
  const char* algorithmName() const;

  /// @brief 获取轴数模式名称字符串
  const char* axisModeName() const;

  /// @brief 从字符串解析算法类型
  /// @param name 算法名称（"complementary" 或 "rk4"）
  /// @return 对应的算法类型，默认返回 COMPLEMENTARY
  static AlgorithmType algorithmFromString(const std::string& name);

  /// @brief 从字符串解析轴数模式
  /// @param name 模式名称（"6" 或 "9" 或 "six_axis" 或 "nine_axis"）
  /// @return 对应的轴数模式，默认返回 NINE_AXIS
  static AxisMode axisModeFromString(const std::string& name);

private:
  AlgorithmType algo_;
  AxisMode      axis_mode_;
  double        alpha_acc_;
  double        alpha_mag_;

  std::unique_ptr<ComplementaryFilter> comp_filter_;
  std::unique_ptr<RK4Integration>      rk4_filter_;

  /// @brief 将内部 AxisMode 转换为 ComplementaryFilter::AxisMode
  static ComplementaryFilter::AxisMode toCompAxisMode(AxisMode mode);

  /// @brief 将内部 AxisMode 转换为 RK4Integration::AxisMode
  static RK4Integration::AxisMode toRK4AxisMode(AxisMode mode);
};

} // namespace imu_algorithm
