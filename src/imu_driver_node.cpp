#include "imu_driver_node.h"
#include <ros/ros.h>

ImuDriverNode::ImuDriverNode(ros::NodeHandle& nh)
    : nh_(nh), total_frames_(0), failed_frames_(0), last_diag_time_(ros::Time::now()) {
  loadParams();
}

void ImuDriverNode::loadParams() {
  // 串口参数
  nh_.param<std::string>("port", port_, "/dev/ttyUSB0");
  nh_.param<int>("baud", baud_, 115200);
  nh_.param<int>("timeout_ms", timeout_ms_, 100);

  // 发布参数
  nh_.param<bool>("publish_custom", publish_custom_, true);
  nh_.param<bool>("publish_sensor_msgs", publish_sensor_msgs_, false);
  nh_.param<double>("accel_scale", accel_scale_, 1.0);
  nh_.param<double>("gyro_scale", gyro_scale_, 1.0);
  nh_.param<double>("mag_scale", mag_scale_, 1.0);
  nh_.param<std::string>("frame_id", frame_id_, "imu_link");

  // 姿态解算参数
  nh_.param<bool>("enable_attitude_estimation", enable_attitude_estimation_, true);
  nh_.param<std::string>("algorithm_type", algorithm_type_, "complementary");
  nh_.param<std::string>("axis_mode", axis_mode_, "9");
  nh_.param<double>("alpha_acc", alpha_acc_, 0.02);
  nh_.param<double>("alpha_mag", alpha_mag_, 0.01);

  ROS_INFO("Parameters loaded:");
  ROS_INFO("  port = %s", port_.c_str());
  ROS_INFO("  baud = %d", baud_);
  ROS_INFO("  timeout_ms = %d", timeout_ms_);
  ROS_INFO("  publish_custom = %s", publish_custom_ ? "true" : "false");
  ROS_INFO("  publish_sensor_msgs = %s", publish_sensor_msgs_ ? "true" : "false");
  ROS_INFO("  accel_scale = %.6f", accel_scale_);
  ROS_INFO("  gyro_scale = %.6f", gyro_scale_);
  ROS_INFO("  mag_scale = %.6f", mag_scale_);
  ROS_INFO("  frame_id = %s", frame_id_.c_str());
  ROS_INFO("  enable_attitude_estimation = %s", enable_attitude_estimation_ ? "true" : "false");
  ROS_INFO("  algorithm_type = %s", algorithm_type_.c_str());
  ROS_INFO("  axis_mode = %s", axis_mode_.c_str());
  ROS_INFO("  alpha_acc = %.6f", alpha_acc_);
  ROS_INFO("  alpha_mag = %.6f", alpha_mag_);
}

bool ImuDriverNode::init() {
  // 创建串口
  serial_ = std::make_unique<SerialPort>(port_, baud_, timeout_ms_);
  if (!serial_->open()) {
    ROS_ERROR("Failed to initialize serial port");
    return false;
  }

  // 创建解析器
  parser_ = std::make_unique<ImuParser>();

  // 创建姿态解算器（如果启用）
  if (enable_attitude_estimation_) {
    auto algo_type = imu_algorithm::AttitudeEstimator::algorithmFromString(algorithm_type_);
    auto axis_mode = imu_algorithm::AttitudeEstimator::axisModeFromString(axis_mode_);
    estimator_     = std::make_shared<imu_algorithm::AttitudeEstimator>(algo_type, axis_mode, alpha_acc_, alpha_mag_);

    ROS_INFO("Attitude estimator created: algorithm=%s, axis_mode=%s, alpha_acc=%.4f, alpha_mag=%.4f",
             estimator_->algorithmName(), estimator_->axisModeName(), alpha_acc_, alpha_mag_);
  } else {
    estimator_ = nullptr;
    ROS_INFO("Attitude estimation disabled, publishing raw data with identity quaternion");
  }

  // 创建发布器
  publisher_ = std::make_unique<ImuPublisher>(nh_, publish_custom_, publish_sensor_msgs_, accel_scale_, gyro_scale_,
                                              mag_scale_, frame_id_, estimator_);

  last_diag_time_ = ros::Time::now();
  return true;
}

void ImuDriverNode::run() {
  uint8_t tmp[ImuParser::READ_BUF_SIZE];

  while (ros::ok()) {
    // 从串口读取数据
    size_t n = serial_->read(tmp, sizeof(tmp));
    if (n == 0) {
      // 超时或错误，继续循环（允许 ros::ok() 检查）
      ros::spinOnce();
      continue;
    }

    // 将数据送入解析器
    parser_->feed(tmp, n);

    // 尝试解析所有可用帧
    ImuRawData raw;
    while (parser_->parse(raw)) {
      if (total_frames_ == std::numeric_limits<size_t>::max())
        total_frames_ = 0;
      else
        ++total_frames_;
      publisher_->publish(raw, ros::Time::now());
    }

    // 定期发布诊断信息
    publishDiagnostics();

    ros::spinOnce();
  }
}

void ImuDriverNode::shutdown() {
  if (serial_) {
    serial_->close();
  }
  ROS_INFO("IMU driver node shutdown. Total frames: %zu, Checksum fails: %zu", total_frames_,
           parser_ ? parser_->checksumFailCount() : 0);
}

void ImuDriverNode::publishDiagnostics() {
  const ros::Time now           = ros::Time::now();
  const double    diag_interval = 60.0; // 每 60 秒输出一次诊断

  if ((now - last_diag_time_).toSec() < diag_interval) {
    return;
  }

  size_t fails = parser_ ? parser_->checksumFailCount() : 0;
  ROS_INFO("Diagnostics: frames_ok=%zu, checksum_fails=%zu", total_frames_, fails);

  last_diag_time_ = now;
}
