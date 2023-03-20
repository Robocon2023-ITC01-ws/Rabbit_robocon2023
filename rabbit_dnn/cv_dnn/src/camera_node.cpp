#include <rclcpp/rclcpp.hpp>
#include <librealsense2/rs.hpp>

#include <sensor_msgs/msg/image.h>
#include <sensor_smgs/msg/compressed_image.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <image_transport/image_transport.h>

class ImagePublisher: rclcpp::Node
{
	public:
		ImagePublisher()
		: Node("camera_node")
		{
			image_publisherRGB_ = this->create_publisher<sensor_msgs::msg::Image>("camera/input/rgb", 100);
			image_publisherDEPTH_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/input/depth", 100);

			publish_timer_ = this->create_wall_timer(300ms, std::bind(&ImagePublisher::image_callback, this));
		}


	private:
		rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

		// Declare realsense pipeline
		rs2::pipeline pipe;

		pipe.start();

		void image_callback()
		{
			cv_bridge::CvImagePtr cv_ptr;
			cv::Mat img(cv::Size(1280, 720), CV_8UC3);
			cv::randu(img, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

			sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();

			image_publisher_-> publish(*img_msg.get());
			this->get_logger.info("Publishing image data .... ");
		}

		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisherRGB_;
		rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_publisherDEPTH_;

		rclcpp::TimerBase publish_timer_;
		


};


int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto realsense_node = std::make_shared<ImagePublisher>();
	rclcpp::spin(realsense_node);
	rclcpp::shutdown();

	return 0;
}

