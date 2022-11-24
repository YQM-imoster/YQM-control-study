#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <rcf_common/filters/filters.h>
#include <rcf_msgs/ChassisCmd.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <pluginlib/class_list_macros.hpp>
#include <string>

namespace rm_control_study {

class ChassisBase : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface> {
 public:
    ChassisBase() = default;
    bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
    void update(const ros::Time &time, const ros::Duration &period) override;
    void moveJoint(const ros::Time &time, const ros::Duration &period) override;
    geometry_msgs::Twist forwardKinematics() override;
    void updateOdom(const ros::Time &time, const ros::Duration &period);
    void cmdChassisCallback(const rcf_msgs::ChassisCmd::ConstPtr &msg);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
 protected:
    XmlRpc::XmlRpcValue twist_cov_list;
    hardware_interface::EffortJointInterface *effort_joint_interface_{};
    std::vector<hardware_interface::JointHandle> joint_handles_{};
    double wheel_base_{}, wheel_track_{}, wheel_radius_{}, publish_rate_{}, timeout_{};
    bool enable_odom_tf_ = false;
    bool publish_odom_tf_ = false;
    rcf_filters::RampFilter<double> *ramp_x_{}, *ramp_y_{}, *ramp_w_{};
    ros::Time last_publish_time_;
    geometry_msgs::TransformStamped odom2base_{};
    geometry_msgs::Vector3 vel_cmd_{};  // x, y
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber cmd_chassis_sub_;
    Command cmd_struct_;
    realtime_tools::RealtimeBuffer<Command> cmd_rt_buffer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener *tf_listener_{};
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    effort_controllers::JointVelocityController ctrl_lf_, ctrl_rf_, ctrl_lb_, ctrl_rb_;
};
}//namespace rm_control_study

#endif //GIT2_YQMCAR_BASE_H
