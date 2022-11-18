//
// Created by zhutuoji on 22-11-12.
//

#ifndef SRC_RM_CHASSIS_CONTROLLER_H
#define SRC_RM_CHASSIS_CONTROLLER_H

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include "rm_common/hardware_interface/robot_state_interface.h"
#include "rm_common/ros_utilities.h"

#include <utility>

namespace rm_control_study {

    class ChassisController
            : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                    rm_control::RobotStateInterface> {
    public:
        ChassisController() = default;
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
        void update(const ros::Time& time, const ros::Duration& period) override;

    private:
        void commandCB(const std_msgs::Float64::ConstPtr& msg);

        hardware_interface::EffortJointInterface* effort_joint_interface_{};
        effort_controllers::JointVelocityController ctrl_lf,ctrl_lb,ctrl_rf,ctrl_rb;
        ros::Subscriber cmd_subscriber_;
        double lf_joint_cmd_{},rf_joint_cmd_{},lb_joint_cmd_{},rb_joint_cmd_{};
    };

}  // namespace rm_control_study


#endif //SRC_RM_CHASSIS_CONTROLLER_H
