#include <memory>
#include <iostream>
#include <cstdlib>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


using namespace std::literals;
using std::placeholders::_1;


class PublisherSubscriber: public rclcpp::Node
{
    public:
        PublisherSubscriber()
        : Node("pub_sub_node")
        {
            publisher_ = this->create_publisher<std_msgs::msg::String>(
                "chatter", 10
            );

            timer_ = this->create_wall_timer(
                100ms, std::bind(&PublisherSubscriber::timer_callback, this)
            );

            subscriber_ = this->create_subscription<std_msgs::msg::String>(
                "chatter", 10, std::bind(&PublisherSubscriber::callback_fn, this, _1)
            );
        }

        ~PublisherSubscriber(){}

        void timer_callback()
        {
            auto text = std_msgs::msg::String();

            text.data = "Hello world" ;
            publisher_->publish(text);

        }

        void callback_fn(const std_msgs::msg::String::SharedPtr msg)
        {
            auto text = msg->data;
            
            std::cout << text << std::endl;
        }


    private:

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;

        size_t count_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PublisherSubscriber>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}