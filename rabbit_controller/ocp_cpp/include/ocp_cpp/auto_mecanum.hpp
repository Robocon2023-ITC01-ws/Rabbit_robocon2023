#ifndef __AUTO_MECANUM_HPP
#define __AUTO_MECANUM_HPP

#include <iostream>
#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <chrono>
#include <map>

using namespace casadi;

class AUTO_MECANUM
{
	private:
		// MECANUM Params
		double Lx_;
		double Ly_;
		double wheel_radius_;
		// MPC Params
		double dt_;
		double sim_time_;
		int prediction_horizons_;

		/// Casadi Params
		Slice all;
		DM Q = DM::zeros(3,3);
		DM R = DM::zeros(4,4);
		MX rot_mat = MX::ones(3,3);
		MX J_for = MX::ones(3,4);
		MX cost_fn = 0.0;
		MX g;

		MX RHS;

		std::vector<double> lbx;
		std::vector<double> ubx;

		DM lbg = 0.0;
		DM ubg = 0.0;

		DMDict args_;
		Function solver_;
		std::map<std::string, DM> results_;
		Function system_kinematic_;

	public:
		AUTO_MECANUM(
			double Lx, double Ly, double wheel_radius,
			double dt, double sim_time, int prediction_horizons
		);

		virtual ~AUTO_MECANUM();

		Eigen::Vector3d forward_kinematic(double theta, double w1, double w2, double w3, double w4);
		Eigen::Vector3d inverse_kinematic(double theta, double vx, double vy, double vth);


		void setup_auto_mecanum();

		void set_boundary(std::vector<double> x_min, std::vector<double> x_max,
						  std::vector<double> u_min, std::vector<double> u_max);

		void input_trajectory(
        					Eigen::Vector3d current_states, Eigen::Vector4d current_controls,
        					Eigen::Vector3d goal_states, Eigen::Vector4d goal_controls);


		std::vector<double> get_optimal_solution();

};






#endif