#include <librealsense2/rs.hpp>
#include <mutex>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/vector3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "cv_bridge/cv_bridge.h"
#include <rclcpp/logger.hpp>
#include <cmath>
#include <chrono>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 480
#define DEPTH_FPS 6 
#define PI_FL  3.141592f

using namespace std::chrono_literals;

struct float3 {
    float x, y, z;
    float3 operator*(float t)
    {
        return { x * t, y * t, z * t };
    }

    float3 operator-(float t)
    {
        return { x - t, y - t, z - t };
    }

    void operator*=(float t)
    {
        x = x * t;
        y = y * t;
        z = z * t;
    }

    void operator=(float3 other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    void add(float t1, float t2, float t3)
    {
        x += t1;
        y += t2;
        z += t3;
    }
};

class RealsenseNode: public rclcpp::Node
{
	public:
		RealsenseNode()
		: Node("realsense_node")
		{


			// image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/input/rgb", 10);
			image_publisher_ = image_transport::create_publisher(this, "camera/input/rgb");
			image_timer_ = this->create_wall_timer(
					33ms, std::bind(&RealsenseNode::image_callback, this));

			gyro_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("camera/gyro/info", 10);
			gyro_timer_ = this->create_wall_timer(
					10ms, std::bind(&RealsenseNode::gyro_callback, this));

			accel_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("camera/accel/info", 10);
			accel_timer_ = this->create_wall_timer(
					20ms, std::bind(&RealsenseNode::accel_callback, this));

		SetUpDevice();

		}
	private:
		void SetUpDevice()
		{
			config_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, fps_);
			config_.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
			config_.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
			auto profile = pipe_.start(config_, [&](rs2::frame frame)
			{	
				// Cast the frame
				auto motion = frame.as<rs2::motion_frame>();
				// If casting is succeed 
				if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
				{
					// get timestamp of current frame
					double ts = motion.get_timestamp();
					rs2_vector gyro_data = motion.get_motion_data();
					process_gyro(gyro_data, ts);
				}

				if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
				{
					rs2_vector accel_data = motion.get_motion_data();

					process_accel(accel_data);
				}
			});

			// Start Data pipeline
			// pipe_.start(config_);
			RCLCPP_INFO(this->get_logger(), "Pipeline has started!");
		}


		void gyro_callback()
		{

			auto gyro_data = geometry_msgs::msg::Vector3();
			gyro_data.x = theta_gyro_.x;
			gyro_data.y = theta_gyro_.y;
			gyro_data.z = theta_gyro_.z;

			gyro_publisher_->publish(gyro_data);

		}


		void accel_callback()
		{
			auto accel_data = geometry_msgs::msg::Vector3();
			accel_data.x = theta_accel_.x;
			accel_data.y = theta_accel_.y;
			accel_data.z = theta_accel_.z;

			accel_publisher_->publish(accel_data);

		}


			
		void image_callback()
		{
			if (rs2::video_frame image_frame = aligned_frameset_.first_or_default(RS2_STREAM_COLOR))
			{
				cv::Mat image_raw;
				image_raw = cv::Mat(cv::Size(image_frame.get_width(), image_frame.get_height()), CV_8UC3, const_cast<void *>(image_frame.get_data()), cv::Mat::AUTO_STEP);

				sensor_msgs::msg::Image::SharedPtr img_msg;
				img_msg = cv_bridge::CvImage(
						std_msgs::msg::Header(),
						sensor_msgs::image_encodings::RGB8, image_raw).toImageMsg();
				// Byte per pixel
				auto bpp = image_frame.get_bytes_per_pixel();
				auto width = image_frame.get_width();
				img_msg->width = width;
				img_msg->height = image_frame.get_height();
				img_msg->is_bigendian = false;
				img_msg->step = width * bpp;
				img_msg->header.frame_id = "camera_d345i";
				// img_msg->header.stamp = std::chrono::high_resolution_clock::now;
				image_publisher_.publish(img_msg);
			}
		}
		
		void process_gyro(rs2_vector gyro_data, double ts)
        {
                        if (firstGyro_)
                        {
                                firstGyro_ = false;
                                last_ts_gyro_ = ts;
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
                        theta_gyro_.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
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
                                theta_accel_ = accel_angle;

                                theta_accel_.y = PI_FL;
                        }
                        else
                        {
                                // Apply complementary filter
                                theta_accel_.x = theta_accel_.x * alpha_ + accel_angle.x * (1-alpha_);
                                theta_accel_.z = theta_accel_.z * alpha_ + accel_angle.z * (1-alpha_);
                        }
        }


		// Angle of camera
        float3 theta_gyro_;
		float3 theta_accel_;
        std::mutex theta_mtx_;

        float alpha_ = 0.98f;
        bool firstGyro_ = true;
        bool firstAccel_ = true;
        // Keeps the arrivals time of previous frame
        double last_ts_gyro_ = 0;


		bool found_gyro_ = false;
		bool found_accel_ = false;


		// RGB frame
		int fps_ = 30;
			
		// Realsense Context
		std::unique_ptr<rs2::context> ctx_;
		// Pipeline
		rs2::pipeline pipe_;
		// Config
		rs2::config config_;

		// Frameset
		rs2::frameset aligned_frameset_;

			
		// Create publisher
		image_transport::Publisher image_publisher_;
		rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gyro_publisher_;
		rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr accel_publisher_;
		rclcpp::TimerBase::SharedPtr image_timer_;
		rclcpp::TimerBase::SharedPtr gyro_timer_;
		rclcpp::TimerBase::SharedPtr accel_timer_;

};



int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);

	auto imu_d435i = std::make_shared<RealsenseNode>();

	rclcpp::spin(imu_d435i);

	rclcpp::shutdown();

	return 0;
}