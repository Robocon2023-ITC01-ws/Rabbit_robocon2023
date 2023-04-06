#ifndef __AUTO_MECANUM_HPP__
#define __AUTO_MECANUM_HPP__


#include <Eigen/Core>
#include <iostream>
#include <casadi/casadi.hpp>


class AUTO_MECANUM
{
	private:
		// MECANUM Geometry
		const double Lx_;
		const double Ly_;
		const double wheel_radius_;

	public:
		AUTO_MECANUM(
				double Lx_, double Ly, double wheel_radius_
			    );

		virtual ~AUTO_MECANUM();
			
		Eigen::VectorXd forward_kinematic(double angle, double w1, double w2, double w3, double w4);
		Eigen::VectorXd inverse_kinematic(double vx, double vy, double vth);

};

#endif

