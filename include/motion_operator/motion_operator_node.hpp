#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "dynamixel_hardware_msgs/msg/current_motor_status.hpp"
#include "dynamixel_hardware_msgs/msg/dynamixel_control_msgs.hpp"
#include "humanoid_interfaces/msg/motion_operator.hpp"
#include "motion_operator/motion_operator_utils.hpp"

namespace motion_operator
{

  class MotionOperatorNode : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    explicit MotionOperatorNode(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~MotionOperatorNode() override = default;

  private:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

    void declare_parameters();
    void load_parameters();
    void tick();
    bool start_pending_motion();
    void reset_motion_state();
    void publish_motion_end(int motion_num);
    std::string resolve_package_path(const std::string &path) const;
    std::string resolve_motion_path(int motion_num) const;
    std::vector<double> get_status_positions_snapshot() const;

    void motion_request_callback(
        const humanoid_interfaces::msg::MotionOperator::SharedPtr msg);
    void motor_status_callback(
        const dynamixel_hardware_msgs::msg::CurrentMotorStatus::SharedPtr msg);

    std::string motion_request_topic_;
    std::string motion_end_topic_;
    std::string motor_status_topic_;
    std::string dynamixel_control_topic_;
    std::string init_motion_path_;
    double tick_hz_{50.0};
    int default_step_timeout_ms_{500};

    std::mutex motion_mutex_;
    mutable std::mutex status_mutex_;
    std::optional<dynamixel_hardware_msgs::msg::CurrentMotorStatus> latest_motor_status_;
    bool motion_running_{false};
    bool motion_request_pending_{false};
    int requested_motion_num_{-1};
    int active_motion_num_{-1};
    std::size_t current_step_index_{0};
    bool current_step_command_sent_{false};
    rclcpp::Time current_step_start_time_{0, 0, RCL_ROS_TIME};
    std::string active_motion_path_;

    rclcpp::Subscription<humanoid_interfaces::msg::MotionOperator>::SharedPtr
        motion_request_sub_;
    rclcpp::Subscription<dynamixel_hardware_msgs::msg::CurrentMotorStatus>::SharedPtr
        motor_status_sub_;

    rclcpp_lifecycle::LifecyclePublisher<dynamixel_hardware_msgs::msg::DynamixelControlMsgs>::SharedPtr
        dynamixel_control_pub_;
    rclcpp_lifecycle::LifecyclePublisher<humanoid_interfaces::msg::MotionOperator>::SharedPtr
        motion_end_pub_;
    rclcpp::TimerBase::SharedPtr tick_timer_;

    MotionPaths motion_paths_;
    MotionSequence motion_sequence_;
  };

} // namespace motion_operator
