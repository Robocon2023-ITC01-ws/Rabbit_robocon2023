#ifndef MPC_CONTROLLER_CORE_HPP_
#define MPC_CONTROLLER_CORE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


namespace rabbit_controller_core
{
    class ControllerCore
    {
        public:
            using Ptr = std::shared_ptr<rabbit_controller_core::ControllerCore>;

            virtual ~ControllerCore() {}


            virtual void intialize(
                rclcpp::Node* parent, const std::string* plugin_name
            ) = 0;
    };
}

#endif