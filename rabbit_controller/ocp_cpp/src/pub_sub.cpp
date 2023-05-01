#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>


#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "pub_sub.hpp"

namespace mixed_node
{
    PublisherSubscriber::PublisherSubscriber(const rclcpp::NodeOptions& options)
    : rclcpp::Node("mixed_node", options)
    {
        publisher_ = create_publisher<std_msgs::msg::Float32>(
            "mixed_topic", 10
        );
        

        timer_ = create_wall_timer(
            100ms, std::bind(&mixed_node::PublisherSubscriber::timer_callback , this);
        );

        subscriber_ = create_subscription<std_msgs::msg::Float32>(
            "mixed_topic", 10,
            std::bind(&mixed_node::PublisherSubscriber::callback_msg, this, _1)
        );
    }

    PublisherSubscriber::~PublisherSubscriber(){}


    PublisherSubscriber::timer_callback()
    {
        
    }
}