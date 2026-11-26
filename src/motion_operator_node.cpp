#include "motion_operator/motion_operator_node.hpp"

#include <algorithm>
#include <chrono>

namespace motion_operator
{

MotionOperatorNode::MotionOperatorNode(const rclcpp::NodeOptions &options)
: rclcpp_lifecycle::LifecycleNode("motion_operator_node", options)
{
  declare_parameters();
}

MotionOperatorNode::CallbackReturn
MotionOperatorNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring motion_operator scaffold");

  load_parameters();

  // 모션 번호 구독
  motion_request_sub_ =
    create_subscription<humanoid_interfaces::msg::MotionOperator>(
      motion_request_topic_,
      1,
      std::bind(&MotionOperatorNode::motion_request_callback, this, std::placeholders::_1));

  motor_status_sub_ =
    create_subscription<dynamixel_hardware_msgs::msg::CurrentMotorStatus>(
      motor_status_topic_,
      rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&MotionOperatorNode::motor_status_callback, this, std::placeholders::_1));

  dynamixel_control_pub_ =
    create_publisher<dynamixel_hardware_msgs::msg::DynamixelControlMsgs>(
      dynamixel_control_topic_, 10);

  // 모션 종료 알림 발행, 모션 종료되면 1 아니면 0
  motion_end_pub_ =
    create_publisher<humanoid_interfaces::msg::MotionOperator>(
      motion_end_topic_, 10);

  tick_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(tick_hz_, 1.0))),
    std::bind(&MotionOperatorNode::tick, this));
  tick_timer_->cancel();

  if (!motion_paths_.InitAddr.empty()) {
    const std::string init_path = resolve_package_path(motion_paths_.InitAddr);
    const bool loaded = motion_sequence_.load_motion_file(init_path);
    RCLCPP_INFO(
      get_logger(),
      "Initial motion path %s: %s",
      init_path.c_str(),
      loaded ? "loaded" : "not loaded");
  }

  return CallbackReturn::SUCCESS;
}

MotionOperatorNode::CallbackReturn
MotionOperatorNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating motion_operator scaffold");
  dynamixel_control_pub_->on_activate();
  motion_end_pub_->on_activate();
  if (tick_timer_) {
    tick_timer_->reset();
  }
  return CallbackReturn::SUCCESS;
}

MotionOperatorNode::CallbackReturn
MotionOperatorNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating motion_operator scaffold");
  if (tick_timer_) {
    tick_timer_->cancel();
  }
  dynamixel_control_pub_->on_deactivate();
  motion_end_pub_->on_deactivate();
  reset_motion_state();
  return CallbackReturn::SUCCESS;
}

MotionOperatorNode::CallbackReturn
MotionOperatorNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up motion_operator scaffold");
  motion_request_sub_.reset();
  motor_status_sub_.reset();
  if (tick_timer_) {
    tick_timer_->cancel();
    tick_timer_.reset();
  }
  dynamixel_control_pub_.reset();
  motion_end_pub_.reset();
  reset_motion_state();
  {
    std::scoped_lock<std::mutex> lock(status_mutex_);
    latest_motor_status_.reset();
  }
  return CallbackReturn::SUCCESS;
}

MotionOperatorNode::CallbackReturn
MotionOperatorNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down motion_operator scaffold");
  if (tick_timer_) {
    tick_timer_->cancel();
  }
  return CallbackReturn::SUCCESS;
}

void MotionOperatorNode::motion_request_callback(
  const humanoid_interfaces::msg::MotionOperator::SharedPtr msg)
{
  if (msg == nullptr) {
    return;
  }

  std::scoped_lock<std::mutex> lock(motion_mutex_);

  if (motion_running_) {
    RCLCPP_WARN(
      get_logger(),
      "Drop motion request %d: motion %d is already running",
      msg->motion_num,
      active_motion_num_);
    return;
  }

  requested_motion_num_ = msg->motion_num;
  motion_request_pending_ = true;

  RCLCPP_INFO(
    get_logger(),
    "Accepted motion request: motion_num=%d",
    requested_motion_num_);
}

void MotionOperatorNode::motor_status_callback(
  const dynamixel_hardware_msgs::msg::CurrentMotorStatus::SharedPtr msg)
{
  if (msg == nullptr) {
    return;
  }

  std::scoped_lock<std::mutex> lock(status_mutex_);
  latest_motor_status_ = *msg;
  if (latest_motor_status_->position.size() > 6) {
    latest_motor_status_->position.erase(
      latest_motor_status_->position.begin() + 6);
  }
}

}  // namespace motion_operator
