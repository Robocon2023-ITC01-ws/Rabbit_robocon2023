#include "auto_mecanum.hpp"


#define PI 3.142324

int main()
{
	double Lx = 0.165;
	double Ly = 0.18;
	double wheel_radius = 0.05;

	auto auto_mecanum = std::make_shared<AUTO_MECANUM>(AUTO_MECANUM(Lx, Ly, wheel_radius));

	double angle = M_PI/4; 
	double w1 = 30;
	double w2 = 30;
	double w3 = 30;
	double w4 = 30;

	auto forward_velocity = auto_mecanum->forward_kinematic(angle, w1, w2, w3, w4);

	std::cout << forward_velocity << std::endl;

	return 0;
}
