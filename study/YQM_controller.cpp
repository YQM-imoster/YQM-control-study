#include "YQM_controller.h"

namespace rm_control_study {

    //初始化函数**********************************************************************************************************
    bool ChassisBase::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh,
                           ros::NodeHandle &controller_nh) {

        //chassis_base
        //判断是否有这些参数？？是否赋值？？，否则判错？？-----------------------------------------------------------------------
        if (!controller_nh.getParam("publish_rate", publish_rate_) || !controller_nh.getParam("timeout", timeout_)) {
            ROS_ERROR("Some chassis params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
            return false;
        }

        //从参数服务器里面赋值（没写完）--------------------------------------------------------------------------------------
        controller_nh.getParam("wheel_radius", wheel_radius_);
        controller_nh.getParam("wheel_track", wheel_track_);
        controller_nh.param("enable_odom_tf", enable_odom_tf_, true);
        controller_nh.getParam("wheel_base", wheel_base_);

        XmlRpc::XmlRpcValue twist_cov_list;
        controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);

        //从hw提取电机数据-------------------------------------------------------------------------------------------------
        effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();

        //odom初始化-----------------------------------------------------------------------------------------------------
        odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh, "odom", 100));
        odom_pub_->msg_.header.frame_id = "odom";
        odom_pub_->msg_.child_frame_id = "base_link";
        odom_pub_->msg_.twist.covariance = {static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0., 0.,
                                            static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0., 0., 0.,
                                            static_cast<double>(twist_cov_list[2]), 0., 0., 0., 0., 0., 0.,
                                            static_cast<double>(twist_cov_list[3]), 0., 0., 0., 0., 0., 0.,
                                            static_cast<double>(twist_cov_list[4]), 0., 0., 0., 0., 0., 0.,
                                            static_cast<double>(twist_cov_list[5])};

        //滤波器初始化----------------------------------------------------------------------------------------------------
        ramp_x_ = new rcf_filters::RampFilter<double>(0, 0.001);
        ramp_y_ = new rcf_filters::RampFilter<double>(0, 0.001);
        ramp_w_ = new rcf_filters::RampFilter<double>(0, 0.001);

        //tf初始化-------------------------------------------------------------------------------------------------------
        tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
        if (enable_odom_tf_) {
            odom2base_.header.frame_id = "odom";
            odom2base_.header.stamp = ros::Time::now();
            odom2base_.child_frame_id = "base_link";
            odom2base_.transform.rotation.w = 1;
            tf_broadcaster_.sendTransform(odom2base_);
        }

        //订阅加速度和速度，把数据放入函数------------------------------------------------------------------------------------
        cmd_chassis_sub_ =
                controller_nh.subscribe<rcf_msgs::ChassisCmd>("command", 1, &ChassisBase::cmdChassisCallback, this);
        cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &ChassisBase::cmdVelCallback, this);

        //mecarnum
        //创造四个轮子句柄------------------------------------------------------------------------------------------------
        ros::NodeHandle nh_lf = ros::NodeHandle(controller_nh, "left_front");
        ros::NodeHandle nh_rf = ros::NodeHandle(controller_nh, "right_front");
        ros::NodeHandle nh_lb = ros::NodeHandle(controller_nh, "left_back");
        ros::NodeHandle nh_rb = ros::NodeHandle(controller_nh, "right_back");

        //ctrl是否初始化-------------------------------------------------------------------------------------------------
        if (!ctrl_lf_.init(effort_joint_interface_, nh_lf) || !ctrl_rf_.init(effort_joint_interface_, nh_rf) ||
            !ctrl_lb_.init(effort_joint_interface_, nh_lb) || !ctrl_rb_.init(effort_joint_interface_, nh_rb)) {
            return false;
        }

        //放在维克多容器后面？？？-----------------------------------------------------------------------------------------
        joint_handles_.push_back(ctrl_lf_.joint_);
        joint_handles_.push_back(ctrl_rf_.joint_);
        joint_handles_.push_back(ctrl_lb_.joint_);
        joint_handles_.push_back(ctrl_rb_.joint_);
        //返回-----------------------------------------------------------------------------------------------------------
        return true;
    }

    //更新 cmd_rt_buffer_里面存的是callback数据，数据是加速度和速度************************************************************
    void ChassisBase::update(const ros::Time &time, const ros::Duration &period) {
        rcf_msgs::ChassisCmd cmd_chassis = cmd_rt_buffer_.readFromRT()->cmd_chassis_;
        geometry_msgs::Twist cmd_vel = cmd_rt_buffer_.readFromRT()->cmd_vel_;

        //判断是否超时？--------------------------------------------------------------------------------------------------
        //超时立刻停止移动
        //未超时滤波器读取加速度和速度数据
        if ((time - cmd_rt_buffer_.readFromRT()->stamp_).toSec() > timeout_) {
            vel_cmd_.x = 0.;
            vel_cmd_.y = 0.;
            vel_cmd_.z = 0.;
        } else {
            ramp_x_->setAcc(cmd_chassis.accel.linear.x);
            ramp_y_->setAcc(cmd_chassis.accel.linear.y);
            ramp_x_->input(cmd_vel.linear.x);
            ramp_y_->input(cmd_vel.linear.y);
            vel_cmd_.x = ramp_x_->output();
            vel_cmd_.y = ramp_y_->output();
            vel_cmd_.z = cmd_vel.angular.z;
        }

        //更新odom-------------------------------------------------------------------------------------------------------
        updateOdom(time, period);

        //（没写完）
        ramp_w_->setAcc(cmd_chassis.accel.angular.z);
        ramp_w_->input(vel_cmd_.z);
        vel_cmd_.z = ramp_w_->output();

        //更新电机--------------------------------------------------------------------------------------------------------
        moveJoint(time, period);
    }  //  ChassisBase::update

    //更新odom***********************************************************************************************************
    void ChassisBase::updateOdom(const ros::Time &time, const ros::Duration &period) {

        //获取小车底盘速度-------------------------------------------------------------------------------------------------
        geometry_msgs::Twist vel_base = forwardKinematics();

        if (enable_odom_tf_) {
            //获取base_link相对与odom的位置关系
            geometry_msgs::Vector3 linear_vel_odom, angular_vel_odom;
            try {
                odom2base_ = tf_buffer_.lookupTransform("odom", "base_link", ros::Time(0));
            } catch (tf2::TransformException &ex) {
                tf_broadcaster_.sendTransform(odom2base_);
                ROS_WARN("%s", ex.what());
                return;
            }

            //doTransform将base的速度通过关系变为Odom的速度存到vector3里面
            // 设原odom为一个固定点，移动odom获取关于固定点的相对位置
            odom2base_.header.stamp = time;
            tf2::doTransform(vel_base.linear, linear_vel_odom, odom2base_);
            tf2::doTransform(vel_base.angular, angular_vel_odom, odom2base_);
            odom2base_.transform.translation.x += linear_vel_odom.x * period.toSec();
            odom2base_.transform.translation.y += linear_vel_odom.y * period.toSec();
            odom2base_.transform.translation.z += linear_vel_odom.z * period.toSec();
            double length =//（没写完）
                    std::sqrt(std::pow(angular_vel_odom.x, 2) + std::pow(angular_vel_odom.y, 2) + std::pow(angular_vel_odom.z, 2));
            if (length > 0.001) { //（没写完）
                //角度变成四元数之看不懂系列
                tf2::Quaternion odom2base_quat, trans_quat;
                tf2::fromMsg(odom2base_.transform.rotation, odom2base_quat);
                trans_quat.setRotation(
                        tf2::Vector3(angular_vel_odom.x / length, angular_vel_odom.y / length, angular_vel_odom.z / length),
                        length * period.toSec());
                odom2base_quat = trans_quat * odom2base_quat;
                odom2base_quat.normalize();
                odom2base_.transform.rotation = tf2::toMsg(odom2base_quat);
            }

            //广播tf
            tf_broadcaster_.sendTransform(odom2base_);
        }

        //真实发布odom数据------------------------------------------------------------------------------------------------
        if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time) {//条件
            if (odom_pub_->trylock()) {
                odom_pub_->msg_.header.stamp = time;
                odom_pub_->msg_.twist.twist.linear.x = vel_base.linear.x;
                odom_pub_->msg_.twist.twist.linear.y = vel_base.linear.y;
                odom_pub_->msg_.twist.twist.angular.z = vel_base.angular.z;
                odom_pub_->unlockAndPublish();
            }
            last_publish_time_ = time;
        }
    }  // ChassisBase::updateOdom

    //力学，底盘数据******************************************************************************************************
    geometry_msgs::Twist MecanumController::forwardKinematics() {
        geometry_msgs::Twist vel_data;
        double k = wheel_radius_ / 4.0;
        double lf_velocity = ctrl_lf_.joint_.getVelocity();
        double rf_velocity = ctrl_rf_.joint_.getVelocity();
        double lb_velocity = ctrl_lb_.joint_.getVelocity();
        double rb_velocity = ctrl_rb_.joint_.getVelocity();
        vel_data.linear.x = (rf_velocity + lf_velocity + lb_velocity + rb_velocity) * k;
        vel_data.linear.y = (rf_velocity - lf_velocity + lb_velocity - rb_velocity) * k;
        vel_data.angular.z = 2 * (rf_velocity - lf_velocity - lb_velocity + rb_velocity) * k / (wheel_base_ + wheel_track_);
        return vel_data;
    }

    //给轮子分配速度******************************************************************************************************
    void MecanumController::moveJoint(const ros::Time &time, const ros::Duration &period) {
        double a = (wheel_base_ + wheel_track_) / 2.0;
        ctrl_lf_.setCommand((vel_cmd_.x - vel_cmd_.y - vel_cmd_.z * a) / wheel_radius_);
        ctrl_rf_.setCommand((vel_cmd_.x + vel_cmd_.y + vel_cmd_.z * a) / wheel_radius_);
        ctrl_lb_.setCommand((vel_cmd_.x + vel_cmd_.y - vel_cmd_.z * a) / wheel_radius_);
        ctrl_rb_.setCommand((vel_cmd_.x - vel_cmd_.y + vel_cmd_.z * a) / wheel_radius_);
        ctrl_lf_.update(time, period);
        ctrl_rf_.update(time, period);
        ctrl_lb_.update(time, period);
        ctrl_rb_.update(time, period);
    }

    //加速度回调*********************************************************************************************************
    void ChassisBase::cmdChassisCallback(const rcf_msgs::ChassisCmd::ConstPtr &msg) {
        cmd_struct_.cmd_chassis_ = *msg;
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    }

    //速度回调***********************************************************************************************************
    void ChassisBase::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
        cmd_struct_.cmd_vel_ = *msg;
        cmd_struct_.stamp_ = ros::Time::now();
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    }

}  //  namespace rm_control_study

PLUGINLIB_EXPORT_CLASS(rm_control_study::ChassisController, controller_interface::ControllerBase)