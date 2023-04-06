#include "auto_mecanum.hpp"



AUTO_MECANUM::AUTO_MECANUM(double Lx, double Ly, double wheel_radius)
: Lx_(Lx), Ly_(Ly), wheel_radius_(wheel_radius)
{}

AUTO_MECANUM::~AUTO_MECANUM() {}


Eigen::VectorXd AUTO_MECANUM::forward_kinematic(double angle, double w1, double w2, double w3, double w4)
{
	Eigen::MatrixXd rot_mat(3,3);
	Eigen::VectorXd inv_vec(4,1);
	inv_vec << w1, w2, w3, w4;
	rot_mat(0,0) = std::cos(angle);
	rot_mat(0,1) = std::sin(angle);
	rot_mat(0,2) = 0;
	rot_mat(1,0) = -std::sin(angle);
	rot_mat(1,1) = std::cos(angle);
	rot_mat(1,2) = 0;
	rot_mat(2,0) = 0;
	rot_mat(2,1) = 0;
	rot_mat(2,2) = 1;

	Eigen::MatrixXd J_for(3,4);
	J_for(0,0) = 1;
	J_for(0,1) = -1;
	J_for(0,2) = -1;
	J_for(0,3) = 1;
	J_for(1,0) = 1;
	J_for(1,1) = 1;
	J_for(1,2) = -1;
	J_for(1,3) = -1;
	J_for(2,0) = 1/(Lx_+Ly_);
	J_for(2,1) = 1/(Lx_+Ly_);
	J_for(2,2) = 1/(Lx_+Ly_);
	J_for(2,3) = 1/(Lx_+Ly_);
	J_for = (1/wheel_radius_)*J_for;

	auto forward_velocity = rot_mat*J_for*inv_vec;

	return forward_velocity;
}


Eigen::VectorXd AUTO_MECANUM::inverse_kinematic(double vx, double vy, double vth)
{
	Eigen::MatrixXd J_inv(4,3);
	Eigen::VectorXd for_vec(3,1);
	for_vec << vx, vy, vth;
	J_inv(0,0) = 1;
	J_inv(0,1) = 1;
	J_inv(0,2) = (Lx_+Ly_);
	J_inv(1,0) = -1;
	J_inv(1,1) = 1;
	J_inv(1,2) = (Lx_+Ly_);
	J_inv(2,0) = -1;
	J_inv(2,1) = -1;
	J_inv(2,2) = (Lx_+Ly_);
	J_inv(3,0) = 1;
	J_inv(3,1) = -1;
	J_inv(3,2) = (Lx_+Ly_);

	auto inverse_velocity = J_inv*for_vec;
}

