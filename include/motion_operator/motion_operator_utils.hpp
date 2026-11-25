#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "dynamixel_hardware_msgs/msg/dynamixel_control_msgs.hpp"
#include "dynamixel_hardware_msgs/msg/dynamixel_msgs.hpp"

struct DynamixelPosition
{
  int id{0};
  double position{0.0};
  uint16_t p_gain{0};
  uint16_t pwm{0};
};

struct MotionStep
{
  int time_ms{0};
  int delay_ms{0};
  int repeat{0};
  std::string name;
  bool selected{false};
  std::vector<DynamixelPosition> joints;
};

struct MotionPaths
{
  // IRC
  std::string IrcInitAddr;
  std::string IrcBall_grabAddr;
  std::string IrcHurdleAddr;
  std::string IrcShootingAddr;
  std::string IrcManiCatchAddr;
  std::string IrcManiCatchRiseAddr;
  std::string IrcManiPlaceAddr;
  std::string IrcManiPlaceBackAddr;

  // RCKO
  std::string InitAddr;
  std::string KickRAddr;
  std::string KickLAddr;
  std::string FrontRiseAddr;
  std::string BackRiseAddr;
  std::string RightRiseAddr;
  std::string LeftRiseAddr;

  // Technical Challenge
  std::string CornerKickReadyAddr;
  std::string CornerKickAddr;
  std::string pourkour_motion1Addr;
  std::string pourkour_motion2Addr;
  std::string pourkour_motion3Addr;
  std::string highkick_motionAddr;

  // EVENT
  std::string EVENT_motion1Addr;
  std::string EVENT_motion2Addr;
  std::string EVENT_motion3Addr;
  std::string EVENT_motion4Addr;
};

namespace motion_operator
{

class MotionSequence
{
public:
  bool load_motion_file(const std::string &motion_path);
  void clear();
  std::vector<double> get_goal_positions(std::size_t step_index) const;
  std::vector<double> get_previous_positions(
    std::size_t step_index,
    const std::vector<double> &fallback_positions) const;
  std::vector<double> compute_velocities(
    const std::vector<double> &goal_positions,
    const std::vector<double> &current_positions,
    int time_ms) const;

  const std::vector<MotionStep> &steps() const;

private:
  std::vector<MotionStep> steps_;
};

}  // namespace motion_operator
