#include "motion_operator/motion_operator_node.hpp"

namespace motion_operator
{

    std::string MotionOperatorNode::resolve_motion_path(int motion_num) const
    {
        switch (motion_num)
        {
        // ---- RCKO ----
        /*
        case 0:
            return resolve_package_path(motion_paths_.KickRAddr);
        case 1:
            return resolve_package_path(motion_paths_.KickLAddr);
        case 2:
            return resolve_package_path(motion_paths_.FrontRiseAddr);
        case 3:
            return resolve_package_path(motion_paths_.BackRiseAddr);
        case 4:
            return resolve_package_path(motion_paths_.RightRiseAddr);
        case 5:
            return resolve_package_path(motion_paths_.LeftRiseAddr);
        */

        // ---- Technical Challenge ----
        case 50:
            return resolve_package_path(motion_paths_.CornerKickReadyAddr);
        case 52:
            return resolve_package_path(motion_paths_.CornerKickAddr);
        case 61:
            return resolve_package_path(motion_paths_.pourkour_motion1Addr);
        case 63:
            return resolve_package_path(motion_paths_.pourkour_motion2Addr);
        case 65:
            return resolve_package_path(motion_paths_.pourkour_motion3Addr);
        case 72:
            return resolve_package_path(motion_paths_.highkick_motionAddr);
        case 73:
            return resolve_package_path(motion_paths_.IrcManiCatchAddr);
        case 74:
            return resolve_package_path(motion_paths_.IrcManiCatchRiseAddr);
        case 75:
            return resolve_package_path(motion_paths_.IrcManiPlaceAddr);
        case 76:
            return resolve_package_path(motion_paths_.IrcManiPlaceBackAddr);

        // ---- ROBIT_final_project
        case 1:
            return resolve_package_path(motion_paths_.PowerFullyKick);

        // ---- EVENT ----
        case 101:
            return resolve_package_path(motion_paths_.EVENT_motion1Addr);
        case 102:
            return resolve_package_path(motion_paths_.EVENT_motion2Addr);
        case 103:
            return resolve_package_path(motion_paths_.EVENT_motion3Addr);
        case 104:
            return resolve_package_path(motion_paths_.EVENT_motion4Addr);

        case 151:
            return resolve_package_path(motion_paths_.IrcInitAddr);
        case 152:
            return resolve_package_path(motion_paths_.IrcBall_grabAddr);
        case 153:
            return resolve_package_path(motion_paths_.IrcHurdleAddr);
        case 154:
            return resolve_package_path(motion_paths_.IrcShootingAddr);

        default:
            RCLCPP_ERROR(get_logger(), "Invalid motion number: %d", motion_num);
            return "";
        }
    }

} // namespace motion_operator
