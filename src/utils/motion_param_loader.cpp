#include "motion_operator/motion_operator_node.hpp"

#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace motion_operator
{

void MotionOperatorNode::declare_parameters()
{
  declare_parameter<std::string>("topics.motion_request", "/motion_operator");
  declare_parameter<std::string>("topics.motion_end", "/motion_end");
  declare_parameter<std::string>("topics.motor_status", "/motor_status");
  declare_parameter<std::string>("topics.dynamixel_control", "/dynamixel_control");
  declare_parameter<double>("tick_hz", 50.0);
  declare_parameter<int>("default_step_timeout_ms", 500);
  declare_parameter<std::string>("motions.Irc.InitAddr", "");
  declare_parameter<std::string>("motions.Irc.Ball_grabAddr", "");
  declare_parameter<std::string>("motions.Irc.HurdleAddr", "");
  declare_parameter<std::string>("motions.Irc.ShootingAddr", "");
  declare_parameter<std::string>("motions.Irc.ManiCatchAddr", "");
  declare_parameter<std::string>("motions.Irc.ManiCatchRiseAddr", "");
  declare_parameter<std::string>("motions.Irc.ManiPlaceAddr", "");
  declare_parameter<std::string>("motions.Irc.ManiPlaceBackAddr", "");

  declare_parameter<std::string>("motions.RCKO.InitAddr", "");
  declare_parameter<std::string>("motions.RCKO.KickRAddr", "");
  declare_parameter<std::string>("motions.RCKO.KickLAddr", "");
  declare_parameter<std::string>("motions.RCKO.FrontRiseAddr", "");
  declare_parameter<std::string>("motions.RCKO.BackRiseAddr", "");
  declare_parameter<std::string>("motions.RCKO.RightRiseAddr", "");
  declare_parameter<std::string>("motions.RCKO.LeftRiseAddr", "");

  declare_parameter<std::string>(
    "motions.Technical_Challenge.CornerKickReceiver.CornerKickReadyAddr", "");
  declare_parameter<std::string>(
    "motions.Technical_Challenge.CornerKickReceiver.CornerKickAddr", "");
  declare_parameter<std::string>(
    "motions.Technical_Challenge.Pourkour.pourkour_motion1", "");
  declare_parameter<std::string>(
    "motions.Technical_Challenge.Pourkour.pourkour_motion2", "");
  declare_parameter<std::string>(
    "motions.Technical_Challenge.Pourkour.pourkour_motion3", "");
  declare_parameter<std::string>(
    "motions.Technical_Challenge.High_kick.highkick_motion", "");

  declare_parameter<std::string>("motions.ROBIT_final_project.PowerFullyKick", "");

  declare_parameter<std::string>("motions.EVENT.EVENT_motion1Addr", "");
  declare_parameter<std::string>("motions.EVENT.EVENT_motion2Addr", "");
  declare_parameter<std::string>("motions.EVENT.EVENT_motion3Addr", "");
  declare_parameter<std::string>("motions.EVENT.EVENT_motion4Addr", "");
}

void MotionOperatorNode::load_parameters()
{
  motion_request_topic_ =
    get_parameter("topics.motion_request").as_string();
  motion_end_topic_ =
    get_parameter("topics.motion_end").as_string();
  motor_status_topic_ =
    get_parameter("topics.motor_status").as_string();
  dynamixel_control_topic_ =
    get_parameter("topics.dynamixel_control").as_string();
  tick_hz_ =
    get_parameter("tick_hz").as_double();
  default_step_timeout_ms_ =
    get_parameter("default_step_timeout_ms").as_int();
  
  motion_paths_.IrcInitAddr =
    get_parameter("motions.Irc.InitAddr").as_string();
  motion_paths_.IrcBall_grabAddr =
    get_parameter("motions.Irc.Ball_grabAddr").as_string();
  motion_paths_.IrcHurdleAddr =
    get_parameter("motions.Irc.HurdleAddr").as_string();
  motion_paths_.IrcShootingAddr =
    get_parameter("motions.Irc.ShootingAddr").as_string();
  motion_paths_.IrcManiCatchAddr =
    get_parameter("motions.Irc.ManiCatchAddr").as_string();
  motion_paths_.IrcManiCatchRiseAddr =
    get_parameter("motions.Irc.ManiCatchRiseAddr").as_string();
  motion_paths_.IrcManiPlaceAddr =
    get_parameter("motions.Irc.ManiPlaceAddr").as_string();
  motion_paths_.IrcManiPlaceBackAddr =
    get_parameter("motions.Irc.ManiPlaceBackAddr").as_string();

  motion_paths_.InitAddr =
    get_parameter("motions.RCKO.InitAddr").as_string();
  motion_paths_.KickRAddr =
    get_parameter("motions.RCKO.KickRAddr").as_string();
  motion_paths_.KickLAddr =
    get_parameter("motions.RCKO.KickLAddr").as_string();
  motion_paths_.FrontRiseAddr =
    get_parameter("motions.RCKO.FrontRiseAddr").as_string();
  motion_paths_.BackRiseAddr =
    get_parameter("motions.RCKO.BackRiseAddr").as_string();
  motion_paths_.RightRiseAddr =
    get_parameter("motions.RCKO.RightRiseAddr").as_string();
  motion_paths_.LeftRiseAddr =
    get_parameter("motions.RCKO.LeftRiseAddr").as_string();

  motion_paths_.CornerKickReadyAddr =
    get_parameter(
    "motions.Technical_Challenge.CornerKickReceiver.CornerKickReadyAddr").as_string();
  motion_paths_.CornerKickAddr =
    get_parameter(
    "motions.Technical_Challenge.CornerKickReceiver.CornerKickAddr").as_string();
  motion_paths_.pourkour_motion1Addr =
    get_parameter(
    "motions.Technical_Challenge.Pourkour.pourkour_motion1").as_string();
  motion_paths_.pourkour_motion2Addr =
    get_parameter(
    "motions.Technical_Challenge.Pourkour.pourkour_motion2").as_string();
  motion_paths_.pourkour_motion3Addr =
    get_parameter(
    "motions.Technical_Challenge.Pourkour.pourkour_motion3").as_string();
  motion_paths_.highkick_motionAddr =
    get_parameter(
    "motions.Technical_Challenge.High_kick.highkick_motion").as_string();

  motion_paths_.PowerFullyKick =
    get_parameter(
    "motions.ROBIT_final_project.PowerFullyKick").as_string();

  motion_paths_.EVENT_motion1Addr =
    get_parameter("motions.EVENT.EVENT_motion1Addr").as_string();
  motion_paths_.EVENT_motion2Addr =
    get_parameter("motions.EVENT.EVENT_motion2Addr").as_string();
  motion_paths_.EVENT_motion3Addr =
    get_parameter("motions.EVENT.EVENT_motion3Addr").as_string();
  motion_paths_.EVENT_motion4Addr =
    get_parameter("motions.EVENT.EVENT_motion4Addr").as_string();

  init_motion_path_ = motion_paths_.InitAddr;
}

std::string MotionOperatorNode::resolve_package_path(const std::string &path) const
{
  if (path.empty()) {
    return "";
  }

  const std::filesystem::path fs_path(path);
  if (fs_path.is_absolute()) {
    return path;
  }

  return (
    std::filesystem::path(
      ament_index_cpp::get_package_share_directory("motion_operator")) /
    fs_path).string();
}



}  // namespace motion_operator
