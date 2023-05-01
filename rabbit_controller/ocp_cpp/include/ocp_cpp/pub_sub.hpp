#ifndef PUB_SUB_HPP__
#define PUB_SUB_HPP__

#include <future>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace std::chrono_literals;

using std::placeholders::_1;

namespace mixed_node
{
    class PublisherSubscriber: public rclcpp::Node
    {
        public:
            explicit PublisherSubscriber(const rclcpp::NodeOptions & options);

            virtual ~PublisherSubscriber();


            void timer_callback();

            void callback_msg(const std_msgs::msg::Float32::SharedPtr &msg);


        private:

            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;

            rclcpp::TimerBase::SharedPtr timer_;

            int count_ = 0;
    };
}

#endif