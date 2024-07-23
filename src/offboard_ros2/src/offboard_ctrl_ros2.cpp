#include "offboard_ros2/offboard_ctrl_ros2.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/qos.hpp>
#include <Eigen/Dense>

// hardcode take-off position in case callback has not executed immediately (vicon)
// will store real take-off position from vicon after viconPose_cb is called
double vicon_posi_x = 0.0;
double vicon_posi_y = 0.0;
double vicon_posi_z = 0.4180;


// define translation and rotation matrix
Eigen::Quaterniond q; // (gazebo)

// // (gazebo)
// Eigen::Vector3d translation(-vicon_posi_x, -vicon_posi_y, -vicon_posi_z);


bool vicon_data_stored = false;

int num_of_task_to_run = 5;

std::shared_ptr<rclcpp::Node> node;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("offb_node");

    // // (gazebo)
    // q = rpyToQuaternion(vicon_orient_r, vicon_orient_p, vicon_orient_y).conjugate();


    auto sensor_qos = rclcpp::SensorDataQoS();

    // Subscriber initialization
    state_sub = node->create_subscription<mavros_msgs::msg::State>("mavros/state", 10, state_cb);
    current_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>("mavros/local_position/pose", sensor_qos, currentPose_cb);
    vicon_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>("/vrpn_mocap/x8_1/pose", sensor_qos, viconPose_cb);
    // odom_subscription_ = node->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, odom_cb);

    // Publisher initialization
    pos_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
    vel_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    px4_vision_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/vision_pose/pose", 1);
    px4_vision_vel_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("mavros/vision_pose/twist", 1);

    // Client service initialization (arming and mode)
    arming_client = node->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    set_mode_client = node->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

    // Debug statement to confirm node creation
    RCLCPP_INFO(node->get_logger(), "Node initialized and subscription created");

    rclcpp::Rate rate(20.0);

    // wait for FCU connection
    while (rclcpp::ok() && !current_state.connected) {
        rclcpp::spin_some(node->get_node_base_interface());
        rate.sleep();
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = vicon_posi_x;
    pose.pose.position.y = vicon_posi_y;
    pose.pose.position.z = vicon_posi_z + 0.3;

    //send a few setpoints before starting
    for(int i = 100; rclcpp::ok() && i > 0; --i){
        pos_pub->publish(pose);
        rclcpp::spin_some(node->get_node_base_interface());
        rate.sleep();
    }

    // initialize variables
    auto offb_set_mode = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    auto land_mode = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    offb_set_mode->custom_mode = "OFFBOARD";
    land_mode->custom_mode = "AUTO.LAND";
    auto arm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_cmd->value = true;
    auto last_request = node->now();


    while (rclcpp::ok()) {

        // publish position to vision pose for replacing GPS (vicon) Must write this before switching mode and arming !
        px4_vision_pose_pub->publish(vision_pose_msg);
        
        // switch mode to offb_node (need to be inside the while loop to maintain offboard mode)
        handle_mode_switch(*offb_set_mode, last_request);

        // arm the drone (need to be inside the while loop to maintain arming)
        handle_arm_command(*arm_cmd, last_request);

        // major position controller 
        bool ifLand = position_control(last_request);

        if(ifLand == true) {
            break;
        }

        

        rclcpp::spin_some(node->get_node_base_interface());
        rate.sleep();
    }

    return 0;

}

// Callback functions
void state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
    current_state = *msg;

}

void currentPose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose = *msg;

}

void viconPose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!vicon_data_stored) {
        vicon_posi_x = msg->pose.position.x;
        vicon_posi_y = msg->pose.position.y;
        vicon_posi_z = msg->pose.position.z;

        vicon_data_stored = true;
    }
    
    vicon_pose = *msg;

    // Create and publish pose message to vision
    vision_pose_msg.header.frame_id = "map";
    vision_pose_msg.header.stamp = node->now();
    vision_pose_msg.pose.position.x = msg->pose.position.x;
    vision_pose_msg.pose.position.y = msg->pose.position.y;
    vision_pose_msg.pose.position.z = msg->pose.position.z;
    vision_pose_msg.pose.orientation.x = msg->pose.orientation.x;
    vision_pose_msg.pose.orientation.y = msg->pose.orientation.y;
    vision_pose_msg.pose.orientation.z = msg->pose.orientation.z;
    vision_pose_msg.pose.orientation.w = msg->pose.orientation.w;

    RCLCPP_INFO(node->get_logger(), "Check vision_pose: x: %.2f, y: %.2f, z: %.2f", vision_pose_msg.pose.position.x, vision_pose_msg.pose.position.y, vision_pose_msg.pose.position.z);

    px4_vision_pose_pub->publish(vision_pose_msg);
}




bool position_control(rclcpp::Time& last_request) {

    // vel_msg.twist.linear.x = 0;
    // vel_msg.twist.linear.y = 0;
    // vel_msg.twist.linear.z = 0;
    // double v = 0.5;

    if (state == 0) {
        target = Eigen::Vector3d(current_pose.pose.position.x, current_pose.pose.position.y, vicon_posi_z + 0.13);
    } 
    else if (state == 1) {
        target = Eigen::Vector3d(vicon_posi_x, vicon_posi_y, vicon_posi_z + 0.3);
    } 
    else if (state == 2) {
        target = Eigen::Vector3d(vicon_posi_x, vicon_posi_y + 0.5, vicon_posi_z + 0.3);
    } 
    else if (state == 3) {
        target = Eigen::Vector3d(vicon_posi_x + 0.5, vicon_posi_y + 0.5, vicon_posi_z + 0.3);
    } 
    else if (state == 4) {
        target = Eigen::Vector3d(vicon_posi_x + 0.5, vicon_posi_y, vicon_posi_z + 0.3);
    } 
    else if (state == 5) {
        target = Eigen::Vector3d(vicon_posi_x, vicon_posi_y, vicon_posi_z + 0.3);
    }

    target_pose.header.frame_id = "map";
    target_pose.header.stamp = node->now();
    target_pose.pose.position.x = target[0];
    target_pose.pose.position.y = target[1];
    target_pose.pose.position.z = target[2];
    target_pose.pose.orientation.x = 0;
    target_pose.pose.orientation.y = 0;
    target_pose.pose.orientation.z = 0;
    target_pose.pose.orientation.w = 0.99;


    // // if using gazebo and velocity controller (gazebo)
    // vel_msg.twist.linear.x = (desired[0] - current_pose.pose.position.x) * v;
    // vel_msg.twist.linear.y = (desired[1] - current_pose.pose.position.y) * v;
    // vel_msg.twist.linear.z = (desired[2] - current_pose.pose.position.z) * v;

    // // if using vicon and velocity controller (vicon)
    // vel_msg.twist.linear.x = (desired[0] - vicon_pose.transform.translation.x) * v;
    // vel_msg.twist.linear.y = (desired[1] - vicon_pose.transform.translation.y) * v;
    // vel_msg.twist.linear.z = (desired[2] - vicon_pose.transform.translation.z) * v;

    // publish target setpoint to mavros
    pos_pub->publish(target_pose); // pose controller

    // vel_pub->publish(vel_msg);

    // 0.1 and 0.15 for real drone
    if(isAtPosition(target[0], target[1], target[2], 0.1, 0.15)) {
        if(state == 0) {
            RCLCPP_INFO(node->get_logger(), "Reached Landing Position. Prepare to stop");

            return true;
        }
    }
    
    if (isAtPosition(target[0], target[1], target[2], 0.1, 0.15) && (node->now() - last_request > rclcpp::Duration::from_seconds(5.0))) {
        // if the state is already the last task to run, set the state to zero so that the drone can land
        // it is safer to write ">=" than "=="
        if (state >= num_of_task_to_run) {
            state = 0;
            
        } 
        else {
            state++;
        }

        last_request = node->now();
        RCLCPP_INFO(node->get_logger(), "Position x:%.2f, y:%.2f, z:%.2f reached. Moving to next desired position. State is:%.2d", target[0], target[1], target[2], state);
    }

    return false;


}

void handle_mode_switch(mavros_msgs::srv::SetMode::Request& mode_cmd, rclcpp::Time& last_request) {
    if (current_state.mode != "OFFBOARD" && (node->now() - last_request > rclcpp::Duration::from_seconds(5.0))) {
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>(mode_cmd);
        auto result = set_mode_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
            if (result.get()->mode_sent) {
                RCLCPP_INFO(node->get_logger(), "Offboard enabled");
                last_request = node->now();
            } else {
                RCLCPP_WARN(node->get_logger(), "Failed to set Offboard mode");
            }
        } else {
            RCLCPP_WARN(node->get_logger(), "Failed to call service set_mode");
        }
    }
}

void handle_arm_command(mavros_msgs::srv::CommandBool::Request& arm_cmd, rclcpp::Time& last_request) {
    if (!current_state.armed && (node->now() - last_request > rclcpp::Duration::from_seconds(5.0))) {
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>(arm_cmd);
        auto result = arming_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
            if (result.get()->success) {
                RCLCPP_INFO(node->get_logger(), "Vehicle armed");
                last_request = node->now();
            } else {
                RCLCPP_WARN(node->get_logger(), "Failed to arm vehicle");
            }
        } else {
            RCLCPP_WARN(node->get_logger(), "Failed to call service arming");
        }
    }
}

// Eigen::Vector3d OffboardControlSITL::transformPoint(const Eigen::Vector3d &translation,
//                                const Eigen::Quaterniond &rotation,
//                                const Eigen::Vector3d &point) {
//     Eigen::Affine3d transform = Eigen::Affine3d::Identity();
//     transform.translate(translation);
//     transform.rotate(rotation);
//     Eigen::Vector3d transformed_point = transform * point;
//     return transformed_point;
// }

Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw) {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

bool isAtPosition(double x, double y, double z, double xy_offset, double z_offset) {
    Eigen::Vector2d desired_posi_xy(x, y);

    // (gazebo)
    Eigen::Vector2d current_posi_xy(current_pose.pose.position.x, current_pose.pose.position.y);
    return ((desired_posi_xy - current_posi_xy).norm() < xy_offset && std::abs(z - current_pose.pose.position.z) < z_offset);

    // // (vicon)  
    // Eigen::Vector2d current_posi_xy(vicon_pose.transform.translation.x, vicon_pose.transform.translation.y);
    // return ((desired_posi_xy - current_posi_xy).norm() < xy_offset && std::abs(z - vicon_pose.transform.translation.z) < z_offset);
}



