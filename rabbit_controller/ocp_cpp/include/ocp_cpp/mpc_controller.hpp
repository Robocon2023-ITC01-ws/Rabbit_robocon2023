#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <ocp_cpp/controller_core.hpp>
#include <ocp_cpp/mpc_controller_core.hpp>
#include <string>
#include <memory>
#include <vector>


#ifndef MPC_CONTROLLER_HPP_
#define MPC_CONTROLLER_HPP_

namespace mpc_controller
{
    class MPC_NODE: public rabbit_controller_core::ControllerCore
    {
        public:
            MPC_NODE();

            ~MPC_NODE();

            void intialize(rclcpp::Node *parent, const std::string & plugin_name) override;

            void mpc_callback();

            void control_callback();

        private:

            rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr con_pub_;
            rclcpp::Subscriber<std_msgs::msg::Float32MultiArray>::SharedPtr con_sub_;
            rclcpp::Subscriber<geometry_msgs::msg::Vector3>::SharedPtr pose_sub_;

            rclcpp::TimerBase::SharedPtr mpc_timer_;
            rclcpp::TimerBase::SharedPtr control_timer_;

    }
};


#endif