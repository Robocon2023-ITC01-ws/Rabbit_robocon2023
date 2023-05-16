#ifndef IPOPT_MPC_HPP__
#define IPOPT_MPC_HPP__

#include <vector>
#include <map>
#include <Eigen/Core>

using namespace std;

class MPC
{
    public:
        MPC();

        vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
        vector<double> mpc_x;
        vector<double> mpc_y;
        vector<double> mpc_yaw;

        double _mpc_totalcost;
        double _mpc_ctecost;
        double _mpc_ethetacost;
        double _mpc_velcost;

        void loadParams(const std::map<std::string, double> &params);

    private:
        // Paramaters for mpc solvers

        double _max_angvel, _max_x, _max_y, max_yaw;
        int _mpc_steps, _x_start, _y_start, _yaw_start, _angvel_start;
        std::map<string, double> _params;

        unsigned int dis_cnt;
};

#endif
