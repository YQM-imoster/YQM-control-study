//
// Created by zhutuoji on 22-11-12.
//

#include "../include/rm_chassis_controller.h"

#include <std_msgs/Float64.h>

#include <pluginlib/class_list_macros.hpp>

namespace rm_control_study {
    bool ChassisController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh,
                                        ros::NodeHandle &controller_nh) {
        ros::NodeHandle nh_lf = ros::NodeHandle(controller_nh, "left_front");
        ros::NodeHandle nh_rf = ros::NodeHandle(controller_nh, "right_front");
        ros::NodeHandle nh_lb = ros::NodeHandle(controller_nh, "left_back");
        ros::NodeHandle nh_rb = ros::NodeHandle(controller_nh, "right_back");
        cmd_subscriber_ = root_nh.subscribe<std_msgs::Float64>("chassis_command", 1, &ChassisController::commandCB, this);
        effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (!ctrl_lf.init(effort_joint_interface_, nh_lf)||
            !ctrl_rf.init(effort_joint_interface_, nh_rf) ||
            !ctrl_lb.init(effort_joint_interface_, nh_lb) ||
            !ctrl_rb.init(effort_joint_interface_, nh_rb)) {
            return false;
        }
        return true;
    }

    void ChassisController::update(const ros::Time &time, const ros::Duration &period) {
        double vel_lf = ctrl_lf.joint_.getVelocity();
        double vel_rf = ctrl_rf.joint_.getVelocity();
        double vel_lb = ctrl_lb.joint_.getVelocity();
        double vel_rb = ctrl_rb.joint_.getVelocity();
        ROS_INFO("lf vel_lf is : %f", vel_lf);
        ROS_INFO("lf vel_rf is : %f", vel_rf);
        ROS_INFO("lf vel_lb is : %f", vel_lb);
        ROS_INFO("lf vel_rb is : %f", vel_rb);
        ctrl_lf.setCommand(lf_joint_cmd_);
        ctrl_rf.setCommand(rf_joint_cmd_);
        ctrl_lb.setCommand(lb_joint_cmd_);
        ctrl_rb.setCommand(rb_joint_cmd_);
        ctrl_lf.update(time, period);
        ctrl_rf.update(time, period);
        ctrl_lb.update(time, period);
        ctrl_rb.update(time, period);
    }

    void ChassisController::commandCB(const std_msgs::Float64::ConstPtr &msg) {
        lf_joint_cmd_ = msg->data;
        rf_joint_cmd_ = msg->data;
        lb_joint_cmd_ = msg->data;
        rb_joint_cmd_ = msg->data;
    }
}  // namespace rm_control_study


PLUGINLIB_EXPORT_CLASS(rm_control_study::ChassisController, controller_interface::ControllerBase)
