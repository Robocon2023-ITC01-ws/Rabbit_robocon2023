#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud2_iterator.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsuitl.hpp>
#include <librealsense2/hpp/rs_processing.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/logger.hpp>
#include "realsense_ros2/constants.hpp"
#include <chrono>
#include <map>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


#define IMAGE_FORMAT_DEPTH CV_16UC1
#define DEPTH_WITDH 640
#define DEPTH_HEIGHT 480
#define DEPTH_FPS 6

using namespace std::chorno_literals;
using stream_index_pair = std::pair<rs2_stream, int>;

const stream_index_pair COLOR{RS2_STREAM_COLOR, 0};
const stream_index_pair DEPTH{RS2_STREAM_DEPTH, 0};
const stream_index_pair INFRA1{RS2_STREAM_INFRARED, 1};
const stream_index_pair INFRA2{RS2_STREAM_INFRARED, 2};


class CameraNode: public rclcpp::Node
{
	public:
		CameraNode()
		: Node("realsense_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
		{
			// Declare parameter
			this->declare_parameter<bool>("is_color", false);
			this->declare_parameter<bool>("publish_depth", true);
			this->declare_parameter<bool>("publish_pointcloud", false);
			this->declare_parameter<bool>("publishg_image_raw", false);
			this->declare_parameter<int>("frame_rate", 30);

			this->get_parameter("is_color", is_color_);
			this->get_parameter("publish_depth",  publish_depth_);
			this->get_parameter("publish_pointcloud", publish_point_cloud_);
			this->get_parameter("publish_image_raw", publish_image_raw_);
			this->get_parameter("frame_rate", fps_);


			begin_ = std::chrono::steady_clock::now();

			// Setup device and stream 
			SetUpDevice();
			SetUpStream();


			// Start pipeline
			rs2::config config;
			config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, fps_);
			if (if_color_ || publish_image_raw_)
				config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_RGB8, fps_);
			pipeline_.start(config);
			RCLCPP_INFO(logger_, "Capture Pipeline started!");

			// Publishers
			if (publish_depth_)
			{
				aligh_depth_publisher_ = image_transport::create_publisher(this, "rs_d435i/aligned_depth/image_raw", 10);
				depth_camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("rs_d435i/camera_info", 10);
			}

			if (publish_pointcloud_)
			{
				pcl_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("rs_d435i/point_cloud", 10);
			}
			if (publish_image_raw_)
			{
				image_raw_publisher_ = image_transport::create_publisher(this, "rs_d435i/camera/image_raw", 10);
			}

			// Timer
			timer_ = this->create_wall_timer(1ms, std::bind(&CameraNode::timer_callback, this));
		}
	private:

		void SetUpDevice()
		{
			try
			{
				// Reset context
				ctx_.reset(new rs2::context());
				// query realsense devices
				auto list = ctx_->query_devices();
				if (0 == list.size())
				{
					ctx_.reset();
					RCLCPP_INFO(logger_, "No RealSense Device was found!....");
					rclcpp::shutdown();
					exit(1);
				}

				// Front device correspond to depth camera
				dev_ = list.front();
				RCLCPP_INFO(logger_, "Device set up");
			}
			catch (const std::exception &ex)
			{
				RCLCPP_ERROR(logger_, "An exception has been thrown: %s", ex.what());
				throw;
			}

			catch (...)
			{
				RCLCPP_ERROR(logger_, "Unknown exception has occured!");
				throw;
			}
		}

		void SetUpStream()
		{
			// parameters
			rs2_format format = RS2_FORMAT_RGB8;
			rs2_format format_depth = RS2_FORMAT_Z16;
			std::string encoding = sensor_msgs::image_encoding::RGB8;
			std::string stream_name = "color";
			std::string module_name = "0";

			auto dev_sensors = dev_.query_sensors();
			try
			{
				for (auto &&elem: dev_sensors)
				{
					module_name = elem.get_info(RS2_CAMERA_INFO_NAME);
					RCLCPP_INFO(logger_, "\n\nModule name: %s", module_name.c_str());
					auto sensor = new rs2::sensor(elem);
					auto sensors = std::unique_ptr<rs2::sensor>(sensor);
					// Get Stereo Module
					if ("Stereo Module" == module_name || "Coded-Light Depth Sensor" == module_name)
					{
						auto depth_sensor = sensors->as<rs2::depth_sensor>();
						depth_scale_meters_ = depth_sensor.get_depth_scale();

					}

					// Get the video profile
					auto profiles = sensors->get_stream_profiles();
					for (auto &profile: profiles)
					{
						auto video_profile = profile.as<rs2::video_stream_profile>();
						RCLCPP_DEBUG(logger_, "Video profile found with format: %d, W: %d, H: %d, FPS: %d", video_profile.format(), video_profile.width(), video_profile.height(), video_profile.fps());

						// Choose right profile depending on parameters
						if (video_profile.width() == DEPTH_WIDTH && video_profile.height() == DEPTH_HEIGHT && video_profile.fps() == fps_)
						{
							if (video_profile.format() == format)
							{
								// Update calibration data with information
								UpdateCalibrateData(video_profile);
								video_profile_ = profile;
								image_ = cv::Mat(video_profile.with(), video_profile.height(), format, cv::Scalar(0, 0, 0));
								RCLCPP_INFO(logger_, "%s stream is enabled - width: %d, height: %d, fps: %d", module_name.c_str(), video_profile.width(), video_profile.height(), video_profile.fps());
								break;
							}
							else if (video_profile.format() == format_depth)
							{
								depth_video_profile_ = profile;
							}
						}
					}
				}
				UpdateCalibrateData(depth_video_profile_.as<rs2::video_stream_profile>());
			}
			catch (const std::exception &ex)
			{
				RCLCPP_ERROR(logger_, "An exception has been thrown: %s", ex.what());
				throw;
			}
			catch (...)
			{
				RCLCPP_ERROR(logger_, "Unknown exceptioni has occured!");
				throw;
			}
		}

		void UpdateCalibrateData(const rs2::video_stream_profile &video_profile)
		{
			stream_index_pair stream_index{video_profile.stream_type(), video_profile.stream_index()};
			// Get profile intrinsics and save 
			auto intrinsic = video_profile.get_intrinsics();
			strean_intrinsics_ = intrinsic;
			if (stream_index == COLOR)
			{
				camera_info_.width = intrinsic.width;
				camera_info_.height = intrisic.height;
