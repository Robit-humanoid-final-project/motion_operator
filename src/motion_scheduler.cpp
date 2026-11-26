#include "motion_operator/motion_operator_node.hpp"

#include <algorithm>

namespace motion_operator
{

void MotionOperatorNode::tick()
{
  std::scoped_lock<std::mutex> lock(motion_mutex_);

  if (!motion_running_ && motion_request_pending_) {
    if (!start_pending_motion()) {
      reset_motion_state();
    }
    return;
  }

  if (!motion_running_) {
    return;
  }

  if (current_step_index_ >= motion_sequence_.steps().size()) {
    publish_motion_end(active_motion_num_);
    reset_motion_state();
    return;
  }

  const auto &step = motion_sequence_.steps()[current_step_index_];

  if (!current_step_command_sent_) {
    dynamixel_hardware_msgs::msg::DynamixelControlMsgs control_msg;
    dynamixel_hardware_msgs::msg::DynamixelMsgs motor_msg;

    const auto fallback_positions = get_status_positions_snapshot();
    const auto goal_positions =
      motion_sequence_.get_goal_positions(current_step_index_);
    const auto previous_positions =
      motion_sequence_.get_previous_positions(
        current_step_index_, fallback_positions);
    const auto velocities =
      motion_sequence_.compute_velocities(
        goal_positions, previous_positions, step.time_ms);

    for (std::size_t i = 0; i < goal_positions.size(); ++i) {
      motor_msg.goal_position = goal_positions[i];
      motor_msg.profile_velocity =
        i < velocities.size() ? velocities[i] : 0.0;
      motor_msg.profile_acceleration = 0.0;
      control_msg.motor_control.push_back(motor_msg);
    }

    if (
      dynamixel_control_pub_ != nullptr &&
      dynamixel_control_pub_->is_activated())
    {
      dynamixel_control_pub_->publish(control_msg);
    }

    current_step_start_time_ = now();
    current_step_command_sent_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Sent motion %d step %zu/%zu (%s)",
      active_motion_num_,
      current_step_index_ + 1,
      motion_sequence_.steps().size(),
      step.name.c_str());
    return;
  }

  const auto elapsed_ms =
    (now() - current_step_start_time_).nanoseconds() / 1000000;
  // `time` is the commanded movement duration and `delay` is the hold time
  // after reaching the target.  Waiting for delay alone advances to the next
  // step before the current movement can finish.
  const int movement_ms = std::max(step.time_ms, 0);
  const int hold_ms = std::max(step.delay_ms, 0);
  const int timeout_ms =
    movement_ms + hold_ms > 0
      ? movement_ms + hold_ms
      : default_step_timeout_ms_;

  if (elapsed_ms < timeout_ms) {
    return;
  }

  RCLCPP_WARN(
    get_logger(),
    "Motion %d step %zu timeout after %lld ms, advancing",
    active_motion_num_,
    current_step_index_ + 1,
    static_cast<long long>(elapsed_ms));

  ++current_step_index_;
  current_step_command_sent_ = false;
}

bool MotionOperatorNode::start_pending_motion()
{
  const std::string motion_path =
    resolve_motion_path(requested_motion_num_);

  if (motion_path.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "No motion path configured for motion_num=%d",
      requested_motion_num_);
    return false;
  }

  if (!motion_sequence_.load_motion_file(motion_path)) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to load motion file for motion_num=%d: %s",
      requested_motion_num_,
      motion_path.c_str());
    return false;
  }

  active_motion_num_ = requested_motion_num_;
  active_motion_path_ = motion_path;
  motion_running_ = true;
  motion_request_pending_ = false;
  current_step_index_ = 0;
  current_step_command_sent_ = false;

  RCLCPP_INFO(
    get_logger(),
    "Start motion %d from %s",
    active_motion_num_,
    active_motion_path_.c_str());

  return true;
}

void MotionOperatorNode::reset_motion_state()
{
  motion_running_ = false;
  motion_request_pending_ = false;
  requested_motion_num_ = -1;
  active_motion_num_ = -1;
  current_step_index_ = 0;
  current_step_command_sent_ = false;
  current_step_start_time_ =
    rclcpp::Time(0, 0, get_clock()->get_clock_type());
  active_motion_path_.clear();
  motion_sequence_.clear();
}

// 모션 종료 발행 함수, 모션 종료 시 motion_end를 1로 변경
void MotionOperatorNode::publish_motion_end(int motion_num)
{
  if (motion_end_pub_ == nullptr || !motion_end_pub_->is_activated()) {
    return;
  }

  humanoid_interfaces::msg::MotionOperator motion_end_msg;
  motion_end_msg.motion_num = motion_num;
  motion_end_msg.motion_end = 1;
  motion_end_pub_->publish(motion_end_msg);

  RCLCPP_INFO(get_logger(), "Motion %d finished", motion_num);
}

std::vector<double> MotionOperatorNode::get_status_positions_snapshot() const
{
  std::scoped_lock<std::mutex> lock(status_mutex_);
  if (!latest_motor_status_.has_value()) {
    return {};
  }

  return latest_motor_status_->position;
}

}  // namespace motion_operator
