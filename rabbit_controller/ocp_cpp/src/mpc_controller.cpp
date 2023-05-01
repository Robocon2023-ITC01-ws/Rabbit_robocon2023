#include "ocp_cpp/mpc_controller.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <memory>
#include <vector>
#include <string>

namespace rabbit_controller_core
{
    namespace mpc_controller
    {
        MPC_NODE::MPC_NODE(){}

        MPC_NODE::~MPC_NODE(){}

        void MPC_NODE::intialize(rclcpp::Node * parent, const std::string & plugin_name)
        {
            parent = parent;

            con_pub_ = parent->create_publisher<std_msgs::msg::Float32MultiArray>(
                ""
            )
        }
    }
}