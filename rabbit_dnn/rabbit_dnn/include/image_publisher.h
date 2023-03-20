// Reference librealsense

#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"


class image_publisher
{
	public:
		virtual void publish(sensor_msgs::msg::Image::SharedPtr image_ptr) = 0;
		virtual size_t get_subscription_count() const = 0;
		virtual ~image_publisher() = default;

};


class image_rcl_publisher: public image_publisher
{
	public: 
		image_rcl_publisher(rclcpp::Node& node,
				    const std::string& topic_name,
				    const rmw_qos_profile_t& qos);
		void publish(sensor_msgs::msg::Image::UniquePtr image_ptr) override;
		size_t get_subscription_count() const override;
	private:
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_impl;
};


class image_transport_publisher: public image_publisher
{
	public:
		image_transport_publisher(rclcpp::Node& node,
				          const std::string& topic_name,
					  const rwm_qos_profile_t& qos);
		void publish(sensor_msgs::msg::Image::UniquePtr image_ptr) override;
		size_t get_subscription_count() const override;

	private:
		std::shared_ptr<image_transport::Publisher> image_publisher_impl;
};
