#include <memory>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <mpc_action/auto_omni.hpp>
#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>


using namespace std::literals;
using std::placeholders::_1;

class MPCNODE: public rclcpp::Node
{
    public:
        MPCNODE()
        :Node ("mpc_node")
        {
            mpc_controller -> setup_mpc(); 
            mpc_controller -> set_boundary(
                x_min, x_max, u_min, u_max
            );

            path_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "beizer_path", 10, std::bind(&MPCNODE::path_callback, this, _1)
            );

            odom_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "state_est", 10, std::bind(&MPCNODE::odom_callback, this, _1)
            );

            imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
                "imu/data2", 10, std::bind(&MPCNODE::imu_callback, this, _1)
            );

            input_con_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "feedback_encoder", 10, std::bind(&MPCNODE::control_callback, this, _1)
            );

            opt_con_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(
                "input_control", 10
            );

            mpc_timer_ = this->create_wall_timer(
                100ms, std::bind(&MPCNODE::mpc_solver, this)
            );

        }

        void path_callback(const std_msgs::msg::Float32MultiArray::SharedPtr path_msg)
        {
            path_x_ = path_msg->data[0];
            path_y_ = path_msg->data[1];
            path_yaw_ = path_msg->data[2];

        }

        void odom_callback(const std_msgs::msg::Float32MultiArray::SharedPtr odom_msg)
        {
            x_ = odom_msg->data[0];
            y_ = odom_msg->data[1];
        }

        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
        {
            auto q1 = imu_msg->orientation.x;
            auto q2 = imu_msg->orientation.y;
            auto q3 = imu_msg->orientation.z;
            auto q4 = imu_msg->orientation.w;

        }

        void control_callback(const std_msgs::msg::Float32MultiArray::SharedPtr con_msg)
        {

            fd_u1_ = con_msg->data[0];
            fd_u2_ = con_msg->data[1];
            fd_u3_ = con_msg->data[2];
            fd_u4_ = con_msg->data[3];

        }

        void mpc_solver()
        {
            Eigen::Vector3d next_trajectories_ = Eigen::Vector3d(3);
            next_trajectories_ << path_x_, path_y_, path_yaw_;

            Eigen::Vector4d next_controls_ = Eigen::Vector4d(4);
            next_controls_ << 15, 15, 15, 15;

            Eigen::Vector3d current_states_ = Eigen::Vector3d(3);
            Eigen::Vector3d feedback_states_ = Eigen::Vector3d(3);
            feedback_states_ << x_, y_, yaw_;

            Eigen::Vector4d feedback_controls_ = Eigen::Vector4d(4);
            feedback_controls_ << fd_u1_, fd_u2_, fd_u3_, fd_u4_;

            mpc_controller->input_trajectory(
            current_states_, feedback_controls_,
            next_trajectories_, next_controls_);

            results_all = mpc_controller->optimal_solution();

            result_x.assign(results_all.begin(), results_all.begin()+(prediction_horizons_+1)*3);
            result_u.assign(results_all.begin()+(prediction_horizons_+1)*3,
                            results_all.begin()+(prediction_horizons_+1)*3 + (prediction_horizons_)*4);

            u1_ = result_u[0];
            u2_ = result_u[1];
            u3_ = result_u[2];
            u4_ = result_u[3];

            opt_yaw = result_x[2];

            current_states_ = feedback_states_ + dt_ * mpc_controller->forward_kinematic(
                u1_, u2_, u3_, u4_, opt_yaw
            );

            std::cout << "Current states: " << current_states_ << std::endl;

            con_pub_.data = {float(u1_), float(u2_), float(u3_), float(u4_)};

            opt_con_pub->publish(con_pub_);

        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    private:
        // ROS Declaration
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr opt_con_pub;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr input_con_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr odom_sub;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr path_sub;
        // rclcpp::TimerBase::SharedPtr con_timer_;
        rclcpp::TimerBase::SharedPtr mpc_timer_;

        std_msgs::msg::Float32MultiArray con_pub_;

        // Params
        // Eigen::Vector3d feedback_states_ = Eigen::Vector3d(3);
        // Eigen::Vector4d feedback_controls_ = Eigen::Vector4d(4);
        // Eigen::Vector3d current_states_ = Eigen::Vector3d(3);
        // Eigen::Vector4d current_controls_  = Eigen::Vector4d(4);
        // Eigen::Vector3d next_trajectories_ = Eigen::Vector3d(3);
        // Eigen::Vector4d next_controls_ = Eigen::Vector4d(4);

        // Eigen::Vector3d feedback_states_;
        // Eigen::Vector4d feedback_controls_;
        // Eigen::Vector3d current_states_;
        // Eigen::Vector4d current_controls_;
        // Eigen::Vector3d next_trajectories_;
        // Eigen::Vector4d next_controls_;


        // Robot params
        double r_ = 0.06;
        double L_ = 0.22;
        double dt_ = 0.1;
        int prediction_horizons_ = 10;
        int n_states_ = 3;
        int n_controls_ = 4;

        // Callback params
        double x_ = 0.0;
        double y_ = 0.0;
        double yaw_ = 0.0;

        // Path message
        double path_x_ = 0.0;
        double path_y_ = 0.0;
        double path_yaw_ = 0.0;

        double u1_ = 0.0;
        double u2_ = 0.0;
        double u3_ = 0.0;
        double u4_ = 0.0;
        double fd_u1_ = 0.0;
        double fd_u2_ = 0.0;
        double fd_u3_ = 0.0;
        double fd_u4_ = 0.0;
        double opt_yaw = 0.0;

        // Set boundary
        std::vector<double> x_min{-10, -10, -1.57};
        std::vector<double> x_max{ 10,  10, 1.57};
        std::vector<double> u_min{-10, -10, -10, -10};
        std::vector<double> u_max{10, 10, 10, 10};

        std::shared_ptr<AUTO_OMNI> mpc_controller = std::make_shared<AUTO_OMNI>(AUTO_OMNI(
                r_, L_, dt_,
                prediction_horizons_, n_states_, n_controls_
            ));

        // Solution
        std::vector<double> results_all;
        std::vector<double> result_x;
        std::vector<double> result_u;

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto mpc_node = std::make_shared<MPCNODE>();

    rclcpp::spin(mpc_node);

    rclcpp::shutdown();

    return 0;
}