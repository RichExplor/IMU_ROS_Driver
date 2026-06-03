#include "imu_publisher.h"

ImuPublisher::ImuPublisher(ros::NodeHandle& nh, bool publish_custom, bool publish_sensor_msgs, double accel_scale,
                           double gyro_scale, double mag_scale, const std::string& frame_id,
                           std::shared_ptr<imu_algorithm::AttitudeEstimator> estimator)
    : publish_custom_(publish_custom), publish_sensor_msgs_(publish_sensor_msgs), accel_scale_(accel_scale),
      gyro_scale_(gyro_scale), mag_scale_(mag_scale), frame_id_(frame_id), estimator_(estimator), first_frame_(true) {
  // acc 缩放系数：原始 DATA * SCALE_ACCEL = m/s^2
  accel_scale_ = SCALE_ACCEL * accel_scale;

  // gyro 缩放系数：原始 DATA * SCALE_GYRO = deg/s
  gyro_scale_ = SCALE_GYRO * gyro_scale;

  // mag 缩放系数（归一化矢量）
  mag_scale_ = SCALE_MAG * mag_scale;

  if (publish_custom_) {
    pub_custom_ = nh.advertise<imu_ros_driver::ImuData>("imu/data_serial", 10);
    ROS_INFO("Publishing custom ImuData on ~/imu/data_serial");
  }

  if (publish_sensor_msgs) {
    pub_imu_ = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
    pub_mag_ = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 10);
    ROS_INFO("Publishing sensor_msgs/Imu on ~/imu/data_raw, "
             "sensor_msgs/MagneticField on ~/imu/mag");
  }

  if (estimator_) {
    ROS_INFO("Attitude estimation enabled: algorithm=%s, axis_mode=%s", estimator_->algorithmName(),
             estimator_->axisModeName());
  }
}

void ImuPublisher::publish(const ImuRawData& raw, const ros::Time& stamp) {
  // 1. 将原始数据转换为 SI 单位，并应用缩放系数
  geometry_msgs::Vector3    linear_acceleration;
  geometry_msgs::Vector3    angular_velocity;
  geometry_msgs::Vector3    magnetic_field;
  geometry_msgs::Quaternion orientation;

  // 线性加速度
  if (raw.has_accel) {
    linear_acceleration.x = static_cast<double>(raw.ax) * accel_scale_;
    linear_acceleration.y = static_cast<double>(raw.ay) * accel_scale_;
    linear_acceleration.z = static_cast<double>(raw.az) * accel_scale_;
  } else {
    linear_acceleration.x = linear_acceleration.y = linear_acceleration.z = 0.0;
  }

  // 角速度（原数据为 deg/s * 1e-6，转换为 rad/s）
  if (raw.has_gyro) {
    angular_velocity.x = static_cast<double>(raw.wx) * gyro_scale_ * Deg2Rad;
    angular_velocity.y = static_cast<double>(raw.wy) * gyro_scale_ * Deg2Rad;
    angular_velocity.z = static_cast<double>(raw.wz) * gyro_scale_ * Deg2Rad;
  } else {
    angular_velocity.x = angular_velocity.y = angular_velocity.z = 0.0;
  }

  // 磁场：优先使用强度（0x31），否则使用归一化（0x30）
  if (raw.has_mag_strength) {
    // 原始单位：DATA * 0.001 mGauss -> 转 Tesla
    magnetic_field.x = static_cast<double>(raw.hx) * SCALE_MAG_STRENGTH_TO_TESLA;
    magnetic_field.y = static_cast<double>(raw.hy) * SCALE_MAG_STRENGTH_TO_TESLA;
    magnetic_field.z = static_cast<double>(raw.hz) * SCALE_MAG_STRENGTH_TO_TESLA;
  } else if (raw.has_mag_norm) {
    magnetic_field.x = static_cast<double>(raw.hx) * mag_scale_;
    magnetic_field.y = static_cast<double>(raw.hy) * mag_scale_;
    magnetic_field.z = static_cast<double>(raw.hz) * mag_scale_;
  } else {
    magnetic_field.x = magnetic_field.y = magnetic_field.z = 0.0;
  }

  // 2. 如果有四元数直接使用；否则进行姿态解算（需要 accel + gyro）
  if (raw.has_quat) {
    // 四元数分量 DATA * 1e-6 -> 无量纲
    orientation.x = static_cast<double>(raw.q1) * 1e-6;
    orientation.y = static_cast<double>(raw.q2) * 1e-6;
    orientation.z = static_cast<double>(raw.q3) * 1e-6;
    orientation.w = static_cast<double>(raw.q0) * 1e-6;
  } else if (estimator_ && raw.has_accel && raw.has_gyro) {
    attitudeEstimate(linear_acceleration, angular_velocity, magnetic_field, orientation, stamp);
  } else if (raw.has_euler) {
    // 如果只有欧拉角，转换为四元数（单位 deg -> rad）
    double roll = static_cast<double>(raw.roll) * 1e-6 * Deg2Rad;
    double pitch = static_cast<double>(raw.pitch) * 1e-6 * Deg2Rad;
    double yaw = static_cast<double>(raw.yaw) * 1e-6 * Deg2Rad;
    Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = rz * ry * rx;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();
  } else {
    // 无可用姿态信息，使用单位四元数
    orientation.x = 0.0;
    orientation.y = 0.0;
    orientation.z = 0.0;
    orientation.w = 1.0;
  }

  // 3. 发布自定义消息和标准消息
  if (publish_custom_) {
    imu_ros_driver::ImuData custom_msg;
    custom_msg.header.stamp    = stamp;
    custom_msg.header.frame_id = frame_id_;

    custom_msg.orientation         = orientation;
    custom_msg.linear_acceleration = linear_acceleration;
    custom_msg.angular_velocity    = angular_velocity;
    custom_msg.magnetic_field      = magnetic_field;

    custom_msg.valid = raw.valid;
    pub_custom_.publish(custom_msg);
  }

  if (publish_sensor_msgs_) {
    // 发布 sensor_msgs/Imu
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp    = stamp;
    imu_msg.header.frame_id = frame_id_;

    imu_msg.orientation = orientation;
    fillCovariance(imu_msg.orientation_covariance.data(), !estimator_); // 有姿态解算时不标记为未知

    imu_msg.linear_acceleration = linear_acceleration;
    fillCovariance(imu_msg.linear_acceleration_covariance.data(), false);

    imu_msg.angular_velocity = angular_velocity;
    fillCovariance(imu_msg.angular_velocity_covariance.data(), false);

    pub_imu_.publish(imu_msg);

    // 发布 sensor_msgs/MagneticField
    sensor_msgs::MagneticField mag_msg;
    mag_msg.header.stamp    = stamp;
    mag_msg.header.frame_id = frame_id_;
    mag_msg.magnetic_field  = magnetic_field;
    fillCovariance(mag_msg.magnetic_field_covariance.data(), false);

    pub_mag_.publish(mag_msg);
  }
}

void ImuPublisher::fillCovariance(double cov[9], bool unknown) {
  // 全部置零
  std::fill(cov, cov + 9, COV_ZERO);
  // 若方向未知，按 ROS 惯例将 [0] 设为 -1
  if (unknown) {
    cov[0] = COV_UNKNOWN;
  }
}

void ImuPublisher::attitudeEstimate(const geometry_msgs::Vector3& accel, const geometry_msgs::Vector3& gyro,
                                    const geometry_msgs::Vector3& mag, geometry_msgs::Quaternion& orientation,
                                    const ros::Time& stamp) {
  if (!estimator_) {
    // 无姿态解算，使用默认单位四元数
    orientation.x = 0.0;
    orientation.y = 0.0;
    orientation.z = 0.0;
    orientation.w = 1.0;
    return;
  }

  // 0. 计算 dt
  double dt = 0.0;
  if (first_frame_) {
    first_frame_ = false;
    dt           = 0.01; // 首帧默认 10ms
  } else {
    dt = (stamp - last_stamp_).toSec();

    if (dt <= 0.0 || dt > 1.0) {
      dt = 0.01; // 异常 dt，使用默认值
    }
  }
  last_stamp_ = stamp;

  // 1. 将 geometry_msgs 转换为 Eigen 向量
  Eigen::Vector3d gyro_vec(gyro.x, gyro.y, gyro.z);
  Eigen::Vector3d accel_vec(accel.x, accel.y, accel.z);
  Eigen::Vector3d mag_vec(mag.x, mag.y, mag.z);

  // 2. 更新姿态解算器
  if (estimator_->axisMode() == imu_algorithm::AttitudeEstimator::AxisMode::NINE_AXIS) {
    estimator_->update(gyro_vec, accel_vec, mag_vec, dt);
  } else {
    estimator_->update(gyro_vec, accel_vec, dt);
  }

  // 3. 从姿态解算器获取四元数
  const Eigen::Quaterniond& q = estimator_->quaternion();
  orientation.x               = q.x();
  orientation.y               = q.y();
  orientation.z               = q.z();
  orientation.w               = q.w();
}
