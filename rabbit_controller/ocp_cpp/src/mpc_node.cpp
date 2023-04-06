#include "rclcpp/rclcpp.hpp"
#include "ocp_cpp/auto_mecanum.hpp"
#include <Eigen/Dense>
#include <map>
#include <chrono>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <functional>


class MPC_ROS: public rclcpp::Node
{
    public:
        MPC_ROS()
        : Node("mpc_node")
        {
            publisher_controls_ = this->create_publisher<std_msgs::msg::Floa32tMultiArray>("input_control", 10);
            controls_timer = this->create_wall_timer(2ms, std::bind(&MPC_ROS::controls_timer,this));
            
            subscriber_odom = this->create_publisher<geometry_msg::msg::Vector3>(
                "odometry", 10
            );
        }

    private:
        void controls_timer()
        {
            auto 
        }
}
