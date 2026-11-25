#include "motion_operator/motion_operator_utils.hpp"

#include <cmath>

namespace motion_operator
{

namespace
{

// ID 6 is controlled independently through /gripper_control.  The hardware
// interface excludes it from the positional /dynamixel_control array, so it
// must also be excluded here to preserve the expected motor-array mapping.
constexpr int kGripperMotorId = 6;

}  // namespace

bool MotionSequence::load_motion_file(const std::string &motion_path)
{
  clear();

  if (motion_path.empty()) {
    return false;
  }

  try {
    YAML::Node file = YAML::LoadFile(motion_path);

    if (!file[2] || !file[2]["type"]) {
      return false;
    }

    if (file[2]["type"].as<std::string>() != "motion") {
      return false;
    }

    for (std::size_t i = 3; i < file.size(); ++i) {
      MotionStep step;
      step.time_ms = file[i]["time"].as<int>();
      step.delay_ms = file[i]["delay"].as<int>();
      step.repeat = file[i]["repeat"].as<int>();
      step.name = file[i]["name"].as<std::string>();
      step.selected = file[i]["selected"].as<bool>();

      for (const auto &dxl : file[i]["dxl"]) {
        DynamixelPosition joint;
        joint.id = dxl["id"].as<int>();
        if (joint.id == kGripperMotorId) {
          continue;
        }
        joint.position = dxl["position"].as<double>();
        step.joints.push_back(joint);
      }

      steps_.push_back(step);
    }
  } catch (const YAML::Exception &) {
    clear();
    return false;
  }

  return !steps_.empty();
}

void MotionSequence::clear()
{
  steps_.clear();
}

const std::vector<MotionStep> &MotionSequence::steps() const
{
  return steps_;
}

std::vector<double> MotionSequence::get_goal_positions(std::size_t step_index) const
{
  std::vector<double> goal_positions;

  if (step_index >= steps_.size()) {
    return goal_positions;
  }

  goal_positions.reserve(steps_[step_index].joints.size());
  for (const auto &joint : steps_[step_index].joints) {
    goal_positions.push_back(joint.position);
  }

  return goal_positions;
}

std::vector<double> MotionSequence::get_previous_positions(
  std::size_t step_index,
  const std::vector<double> &fallback_positions) const
{
  if (step_index == 0 || step_index > steps_.size()) {
    return fallback_positions;
  }

  return get_goal_positions(step_index - 1);
}

std::vector<double> MotionSequence::compute_velocities(
  const std::vector<double> &goal_positions,
  const std::vector<double> &current_positions,
  int time_ms) const
{
  std::vector<double> velocities;
  velocities.reserve(goal_positions.size());

  const double time_sec =
    time_ms > 0 ? static_cast<double>(time_ms) * 0.001 : 0.0;

  for (std::size_t i = 0; i < goal_positions.size(); ++i) {
    const double current =
      i < current_positions.size() ? current_positions[i] : 0.0;

    if (time_sec <= 0.0) {
      velocities.push_back(0.0);
      continue;
    }

    velocities.push_back(
      std::fabs(goal_positions[i] - current) / time_sec);
  }

  return velocities;
}

}  // namespace motion_operator
