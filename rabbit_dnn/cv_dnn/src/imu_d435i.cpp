#include <librealsense2/rs.hpp>
#include <mutex>
#include "example.hpp"
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "geometry/msg/vector.h"
#include "tf2_ros/transform_broadcaster.h"
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/logger.hpp>
#include <map>
#include <cmath>
#include <chrono>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 480
#define DEPTH_FPS 6 



class RealsenseNode: public rclcpp::Node
{
	public:
		RealsenseNode()
		: Node("realsense_node")
		{
			image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/input/rgb", 10);
			image_timer_ = this->create_wall_timer(
					1/30ms, std::bind(&RealsenseNode::image_callback, this));

			gyro_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3("camera/gyro/info", 10);
			gyro_timer_ = this->create_wall_timer(
					1/100ms, std::bind(&RealsenseNode::gyro_callback, this));

			accel_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3("camera/accel/info", 10);
			accel_timer_ = this->create_wall_timer(
					1/50ms, std::bind(&RealsenseNode::accel_callback, this));

		}
	private:

		void image_callback()
		{

		}
		
		void process_gyro(rs2_vector gyro_data, double ts)
                {
                        if (firstGyro_)
                        {
                                firstGyro_ = false;
                                last_ts_gyro_ = tf;
                                return;
                        }
                        float3 gyro_angle;

                        // Initialize gyro_angle with data from gyro
                        gyro_angle.x = gyro_data.x;
                        gyro_angle.y = gyro_data.y;
                        gyro_angle.z = gyro_data.z;

                        // Compute the difference between arrival times of previous and current gyro frames
                        double dt_gyro = (ts - last_ts_gyro_) / 1000.0;
                        last_ts_gyro_ = ts;

                        // Change in angle equals gyro measures * time passed since last measurement
                        gyro_angle = gyro_angle * static_cast<float>(dt_gyro);

                        // Apply the calculated change of angle to the current angle (theta)
                        std::lock_guard<std::mutex> lock(theta_mtx_);
                        theta_.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
                }


		void process_accel(rs2_vector accel_data)
                {
                        // Holds the angle as calculated froma accelerometer data
                        float3 accel_angle;

                        // Calculate roation angle from accelerometer data
                        accel_angle.z = atan2(accel_data.x, accel_data.z);
                        accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

                        // If it is the first iteration, set initial pose of camera according to accelerometer
                        std::lock_guard<std::mutex> lock(theta_mtx_);
                        if (firstAccel_)
                        {
                                firstAccel_ = false;
                                theta_ = accel_angle;

                                theta_.y = PI_FL;
                        }
                        else
                        {
                                // Apply complementary filter
                                theta_.x = theta_.x * alpha + accel_angle.x * (1-alpha);
                                theta_.z = theta_.z * alpha + accel_angle.z * (1-alpha);
                        }
                }

	
		


		// Angle of camera
        	float3 theta_;
        	std::mutex theta_mtx_;

        	float alpha_ = 0.98f;
        	bool firstGyro_ = true;
        	bool firstAccel_ = true;
        	// Keeps the arrivals time of previous frame
        	double last_ts_gyro_ = 0;


		bool found_gyro_ = false;
		bool found_accel_ = false;
			
		// Realsense Context
		rs2::context ctx_;
		// Pipeline
		rs2::pipeline pipe_;
		// Config
		rs2::config config_;
		camera_renderer camera;

			
		// Create publisher
		rclcpp::Publisher<sensor_msgs::msg::Image::SharedPtr image_publisher_;
		rclcpp::Publisher<sensor_msgs::msg::CompressedImage::SharedPtr image_compressed_publisher_;
		rclcpp::Publisher<geometry_msgs::msg::Vector3::SharedPtr gyro_publisher_;
		rclcpp::Publisher<geometry_msgs::msg::Vector3::SharedPtr accel_publisher_;
		rclcpp::TimerBase::SharedPtr timer_;
	
}



