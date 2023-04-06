#include "ocp_cpp/auto_mecanum.hpp"


AUTO_MECANUM::AUTO_MECANUM(
    double Lx, double Ly, double wheel_radius,
    double dt, double sim_time, int prediction_horizons
)
{
    Lx_ = Lx;
    Ly_ = Ly;
    wheel_radius_ = wheel_radius;
    dt_ = dt;
    sim_time_ = sim_time;
    prediction_horizons_ = prediction_horizons;
}

AUTO_MECANUM::~AUTO_MECANUM(){}

void AUTO_MECANUM::setup_auto_mecanum()
{
    // States
    MX x = MX::sym("x");
    MX y = MX::sym("y");
    MX theta = MX::sym("theta");
    MX states = MX::vertcat({x, y, theta});
    int num_states = states.size().first;
    // Controls
    MX w1 = MX::sym("w1");
    MX w2 = MX::sym("w2");
    MX w3 = MX::sym("w3");
    MX w4 = MX::sym("w4");
    MX controls = MX::vertcat({w1, w2, w3, w4});
    int num_controls = controls.size().first;

    MX X = MX::sym("X", num_states, prediction_horizons_+1);
    MX X_ref = MX::sym("X_ref", num_states, prediction_horizons_+1);
    MX U = MX::sym("U", num_controls, prediction_horizons_);
    MX U_ref = MX::sym("U_ref", num_controls, prediction_horizons_);

    // Weight Matrix Diagonalize
    Q(0,0) = 75;
    Q(1,1) = 75;
    Q(2,2) = 90;
    R(0,0) = 0.01;
    R(1,1) = 0.01;
    R(2,2) = 0.01;
    R(3,3) = 0.01;

    // Right-hand side equation
    /// Rotation Matrix
    rot_mat(0,0) = MX::cos(theta);
    rot_mat(0,1) = MX::sin(theta);
    rot_mat(0,2) = 0;
    rot_mat(1,0) =-MX::sin(theta);
    rot_mat(1,1) = MX::cos(theta);
    rot_mat(1,2) = 0;
    rot_mat(2,0) = 0;
    rot_mat(2,1) = 0;
    rot_mat(2,2) = 1;

    J_for(0,0) = 1;
    J_for(0,1) = 1;
    J_for(0,2) = 1;
    J_for(0,3) = 1;
    J_for(1,0) = -1;
    J_for(1,1) = 1;
    J_for(1,2) = 1;
    J_for(1,3) = -1;
    J_for(2,0) = -1/(Lx_+Ly_);
    J_for(2,1) = 1/(Lx_+Ly_);
    J_for(2,2) = -1/(Lx_+Ly_);
    J_for(2,3) = 1/(Lx_+Ly_);

    RHS = MX::mtimes({rot_mat.T(), J_for, controls});

    system_kinematic_ = Function("f", {states, controls}, {RHS});

    g = X(all, 0) - X_ref(all, 0);
    g = MX::reshape(g, -1, 1);


    for (int k = 0; k < prediction_horizons_; k++)
    {
        MX st_err = X(all, k) - X_ref(all, k);
        MX con_err = U(all, k) - U_ref(all, k);
        std::vector<MX> input(2);
        input[0] = X(all, k);
        input[1] = U(all, k);
        cost_fn = cost_fn + MX::mtimes({st_err.T(),Q,st_err}) + MX::mtimes({con_err.T(),R,con_err});
        MX x_next = X(all, k+1);
        MX x_next_euler = system_kinematic_(input).at(0) * dt_ + X(all, k);
        g = MX::vertcat({g, x_next-x_next_euler});
    }

    cost_fn = MX::mtimes({(X(all, prediction_horizons_)-X_ref(all, prediction_horizons_)).T(),
                           Q,
                           (X(all, prediction_horizons_)-X_ref(all, prediction_horizons_))});
    
    MX opt_dec = MX::vertcat({
        MX::reshape(X, -1, 1),
        MX::reshape(U, -1, 1)
    });
    MX opt_par = MX::vertcat({
        MX::reshape(X_ref, -1, 1),
        MX::reshape(U_ref, -1, 1)
    });

    MXDict nlp_prob = {
        {"f", cost_fn},
        {"x", opt_dec},
        {"p", opt_par},
        {"g", g}
    };

    std::string solver_name = "ipopt";
    Dict nlp_opts;
    nlp_opts["expand"] = true;
    nlp_opts["ipopt.max_iter"] = 1000;
    nlp_opts["ipopt.print_level"] = 0;
    nlp_opts["print_time"] = 0;
    nlp_opts["ipopt.acceptable_tol"] = 1e-8;
    nlp_opts["ipopt.acceptable_obj_change_tol"] = 1e-6;

    solver_ = nlpsol("nlpsol", solver_name, nlp_prob, nlp_opts);
}

void AUTO_MECANUM::set_boundary(std::vector<double> x_min, std::vector<double> x_max,
                                std::vector<double> u_min, std::vector<double> u_max)
{
    for (int k = 0; k < prediction_horizons_; k++)
    {
        lbx.push_back(x_min[0]);
        lbx.push_back(x_min[1]);
        lbx.push_back(x_min[2]);

        ubx.push_back(x_max[0]);
        ubx.push_back(x_max[1]);
        ubx.push_back(x_max[2]);
    }

    for (int k = 0; k < prediction_horizons_ -  1; k++)
    {
        lbx.push_back(u_min[0]);
        lbx.push_back(u_min[1]);
        lbx.push_back(u_min[2]);
        lbx.push_back(u_min[3]);

        ubx.push_back(u_max[0]);
        ubx.push_back(u_max[1]);
        ubx.push_back(u_max[2]);
        ubx.push_back(u_max[3]);

    }
}

void AUTO_MECANUM::input_trajectory(
        Eigen::Vector3d current_states, Eigen::Vector4d current_controls,
        Eigen::Vector3d goal_states, Eigen::Vector4d goal_controls)
{
    // Params
    Eigen::MatrixXd init_states;
    Eigen::MatrixXd init_controls;
    Eigen::MatrixXd next_trajectories;
    Eigen::MatrixXd next_controls;

    init_states = current_states.replicate(1, prediction_horizons_+1).reshaped();
    init_controls = current_controls.replicate(1, prediction_horizons_).reshaped();
    next_trajectories = goal_states.replicate(1, prediction_horizons_+1).reshaped();
    next_controls = goal_controls.replicate(1, prediction_horizons_).reshaped();


    Eigen::VectorXd variables(init_states.rows()+init_controls.rows());
    Eigen::VectorXd params(next_trajectories.rows()+next_controls.rows());

    variables << init_states,
                init_controls;

    params << next_trajectories,
              next_controls;


    std::vector<double> x0(variables.data(), variables.data()+variables.size());
    std::vector<double> p(params.data(), params.data()+params.size());

    args_ = {
        {"lbg", lbg},
        {"ubg", ubg},
        {"x0", x0},
        {"p", p}
    };
}


std::vector<double> AUTO_MECANUM::get_optimal_solution()
{
    results_ = solver_(args_);

    // std::cout << "Stage 8" << std::endl;

    std::vector<double> results_all(results_.at("x"));

    // std::cout << result_u[0] << std::endl;

    return results_all;
}