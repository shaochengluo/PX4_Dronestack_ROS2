#ifndef OFFB_NODE_SITL_H
#define OFFB_NODE_SITL_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp> 
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/qos.hpp>
#include <Eigen/Dense>

bool position_control(rclcpp::Time& last_request);

Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw);

inline rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
inline rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub;
inline rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr current_vel_sub;
inline rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_pose_sub;

inline rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_pub;
inline rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub;
inline rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr px4_vision_pose_pub;
inline rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr px4_vision_vel_pub;

inline rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;
inline rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;

inline mavros_msgs::msg::State current_state;
inline geometry_msgs::msg::PoseStamped current_pose, target_pose, vision_pose_msg;
inline geometry_msgs::msg::TwistStamped vision_twist_msg;
inline geometry_msgs::msg::TwistStamped vel_msg;
inline geometry_msgs::msg::PoseStamped vicon_pose;

inline Eigen::Vector3d target;
inline Eigen::Vector3d target_point;
inline std::vector<int> tasks;
inline int state = 1; // Should not modify


void state_cb(const mavros_msgs::msg::State::SharedPtr msg);
void currentPose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
void currentVelocity_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
void viconPose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
void handle_mode_switch(mavros_msgs::srv::SetMode::Request& mode_cmd, rclcpp::Time& last_request);
void handle_arm_command(mavros_msgs::srv::CommandBool::Request& arm_cmd, rclcpp::Time& last_request);
bool isAtPosition(double x, double y, double z, double xy_offset, double z_offset);
Eigen::Vector3d transformPoint(const Eigen::Vector3d &translation, const Eigen::Quaterniond &rotation, const Eigen::Vector3d &point);


#endif 
