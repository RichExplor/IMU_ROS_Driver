#pragma once

#include "algorithm/attitude_estimator.h"
#include "imu_parser.h"
#include "imu_ros_driver/ImuData.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

/// @brief IMU 消息发布器，管理自定义消息和标准 sensor_msgs 的发布
///
/// 集成姿态解算模块，支持可配置的算法类型（互补滤波/RK4）和轴数模式（6轴/9轴），
/// 将原始 IMU 数据经过姿态解算后发布带有姿态四元数的消息。
class ImuPublisher {
public:
  /// @brief 构造函数
  /// @param nh 节点句柄（私有命名空间）
  /// @param publish_custom 是否发布自定义 ImuData 消息
  /// @param publish_sensor_msgs 是否发布 sensor_msgs/Imu 和 MagneticField
  /// @param accel_scale 加速度缩放系数
  /// @param gyro_scale 角速度缩放系数
  /// @param mag_scale 磁场缩放系数
  /// @param frame_id 坐标系 ID
  /// @param estimator 姿态解算器（外部传入，共享所有权）
  ImuPublisher(ros::NodeHandle& nh, bool publish_custom, bool publish_sensor_msgs, double accel_scale,
               double gyro_scale, double mag_scale, const std::string& frame_id,
               std::shared_ptr<imu_algorithm::AttitudeEstimator> estimator);

  /// @brief 发布消息
  /// @param raw 解析后的原始数据
  /// @param stamp 时间戳
  void publish(const ImuRawData& raw, const ros::Time& stamp);

  /// @brief 获取姿态解算器
  std::shared_ptr<imu_algorithm::AttitudeEstimator> estimator() {
    return estimator_;
  }

private:
  ros::Publisher pub_custom_;
  ros::Publisher pub_imu_;
  ros::Publisher pub_mag_;

  bool        publish_custom_;
  bool        publish_sensor_msgs_;
  double      accel_scale_;
  double      gyro_scale_;
  double      mag_scale_;
  std::string frame_id_;

  /// @brief 姿态解算器
  std::shared_ptr<imu_algorithm::AttitudeEstimator> estimator_;

  /// @brief 上一帧时间戳，用于计算 dt
  ros::Time last_stamp_;
  bool      first_frame_;

  /// @brief 协方差矩阵常量（未知方向设 -1，其余为 0）
  static constexpr double COV_UNKNOWN = -1.0;
  static constexpr double COV_ZERO    = 0.0;

  /// @brief 转换原始数据为 SI 单位（m/s^2, rad/s, T）
  static constexpr double GRAVITY     = 9.80665;       // 标准重力加速度
  static constexpr double SCALE_ACCEL = 1.0 / 32768.0; // acc缩放因子
  static constexpr double SCALE_GYRO  = 1.0 / 32768.0; // gyro缩放因子
  static constexpr double SCALE_MAG   = 1.0;           // mag缩放因子

  /// @brief 磁场数据缩放因子（Tesla = mGauss * 1e-7）
  static constexpr double SCALE_MAG_TESLA = 1e-7;

  /// @brief 角度转换常量
  static constexpr double Rad2Deg = 180.0 / M_PI;
  static constexpr double Deg2Rad = M_PI / 180.0;

  /// @brief 填充 sensor_msgs/Imu 的协方差矩阵
  /// @param cov 9 元素数组
  /// @param unknown 是否将第一个元素设为 -1（表示方向未知）
  static void fillCovariance(double cov[9], bool unknown = false);

  /// @brief 使用姿态解算器计算当前姿态四元数
  /// @param accel 加速度计数据（m/s²，机体系）
  /// @param gyro 陀螺仪数据（rad/s，机体系）
  /// @param mag 磁力计数据（T，机体系）
  /// @param orientation 输出姿态四元数
  /// @param stamp 当前时间戳
  void attitudeEstimate(const geometry_msgs::Vector3& accel, const geometry_msgs::Vector3& gyro,
                        const geometry_msgs::Vector3& mag, geometry_msgs::Quaternion& orientation,
                        const ros::Time& stamp);
};
