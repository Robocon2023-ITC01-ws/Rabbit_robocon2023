#include <memory>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <mpc_action/auto_omni.hpp>
#include <Eigen/Core>
#include <casadi/casadi.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32_multi_array.h>

using namespace std::literals;
using std::placeholders::_1;

class MPCNODE: public rclcpp::Node
{
    public:
        MPCNODE()
        :Node ("mpc_node")
        {
            auto mpc_controller = std::make_shared<AUTO_OMNI>(AUTO_OMNI(
                r_, L_, dt_,
                prediction_horizons_, n_states_, n_controls_
            ));

            mpc_controller -> setup_mpc();

            mpc_controller -> set_boundary(
                x_min, x_max, u_min, u_max
            );

            path_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "beizer_path", 10, std::bind(&MPCNODE::path_sub, this, _1)
            );

            input_con_sub = this->create_subscription<std_msgs::msg:Float32MultiArray>(
                "feedback_encoder", 10, std::bind(&MPCNODE::control_sub, this, _1)
            );

            imu_sub = this->create_subscriptioni<sensor_msgs::msg::Imu>(
                "imu/data2", 10, std::bind(&MPCNODE::imu_callback, this, _1)
            );

            opt_con_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(
                "input_control", 10
            );

        }

        void path_sub(const std_msgs::msg::Float32MultiArray path_msg)
        {
            
        }

        void mpc_solver()
        {

        }

        void control_sub(const std_msgs::msg::Float32MultiArray::SharedPtr con_msg)
        {

        }

        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
        {

        }

    
    private:
        // ROS Declaration
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr opt_con_pub;
        rclcpp::Subscriber<std_msgs::msg::Float32MultiArray>::SharedPtr input_con_sub;
        rclcpp::Subscriber<sensor_msgs::msg::Imu>::SharedPtr> imu_sub;
        rclcpp::Subscriber<std_msgs::msg::Float32MultiArray>::SharedPtr path_sub;
        rclcpp::TimerBase::SharedPtr con_timer_;
        rclcpp::TimerBase::SharedPtr mpc_timer_;

        // Params

        Eigen::Vector3d current_states_;
        Eigen::Vector4d current_controls_;
        Eigen::VectorXd next_trajectories_;
        Eigen::VectorXd next_controls_;

        // Robot params
        double r_ = 0.06;
        double L_ = 0.22;
        double dt = 0.1;
        int prediction_horizons_ = 30;
        int n_states_ = 3;
        int n_controls_ = 4;

        // Callback params
        double x_ = 0.0;
        double y_ = 0.0;
        double yaw_ = 0.0;

        double u1_ = 0.0;
        double u2_ = 0.0;
        double u3_ = 0.0;
        double u4_ = 0.0;

        // Set boundary
        std::vector<double> x_min{-10, -10, -1.57};
        std::vector<double> x_max{ 10,  10, 1.57};
        std::vector<double> u_min{-10, -10, -10, -10};
        std::vector<double> u_max{10, 10, 10, 10};
}