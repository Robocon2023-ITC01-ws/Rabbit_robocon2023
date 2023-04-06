#include <iostream>
#include <Eigen/Core>


int main(int argc, char * argv[])
{
	Eigen::MatrixXd rot_mat(3,3);
	rot_mat << 1, 2, 3, 3, 4, 5, 23, 45, 3;
	std::cout << "Rotation Matrix" << std::endl;
	std::cout << rot_mat(2, 1) << std::endl;

	return 0;
}
