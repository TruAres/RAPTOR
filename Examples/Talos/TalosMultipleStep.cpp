#include "TalosMultipleStepOptimizer.h"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <yaml-cpp/yaml.h>
#include <iomanip>

using namespace RAPTOR;
using namespace Talos;
using namespace Ipopt;

const std::string filepath = "../Examples/Talos/data/";

int main(int argc, char* argv[]) {
    // define robot model
    const std::string urdf_filename = "../Robots/talos/talos_reduced_armfixed_floatingbase.urdf";
    
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    // manually define the joint axis of rotation
    // 1 for Rx, 2 for Ry, 3 for Rz
    // 4 for Px, 5 for Py, 6 for Pz
    // not sure how to extract this from a pinocchio model so define outside here.
    if (model.nq != 18) {
        throw std::invalid_argument("Error: Incorrect number of joints in the robot model!");
    }
    Eigen::VectorXi jtype(model.nq);
    jtype << 4, 5, 6, 1, 2, 3, 
             3, 1, 2, 2, 2, 1,
             3, 1, 2, 2, 2, 1;
    
    // ignore all motor dynamics
    model.rotorInertia.setZero();
    model.damping.setZero();
    model.friction.setZero();

    // load settings
    YAML::Node config;

    const double T = 0.4;
    int NSteps = 2;
    TimeDiscretization time_discretization = Uniform;
    int N = 14;
    int degree = 5;
    
    std::vector<GaitParameters> gps(NSteps);

    try {
        config = YAML::LoadFile("../Examples/Talos/multiplestep_optimization_settings.yaml");

        NSteps = config["NSteps"].as<int>();
        N = config["N"].as<int>();
        degree = config["degree"].as<int>();
        std::string time_discretization_str = config["time_discretization"].as<std::string>();
        time_discretization = (time_discretization_str == "Uniform") ? Uniform : Chebyshev;

        auto swingfoot_midstep_z_des = config["swingfoot_midstep_z_des"].as<std::vector<double>>();
        auto swingfoot_begin_x_des = config["swingfoot_begin_x_des"].as<std::vector<double>>();
        auto swingfoot_end_x_des = config["swingfoot_end_x_des"].as<std::vector<double>>();

        if (swingfoot_midstep_z_des.size() != NSteps || 
            swingfoot_begin_x_des.size() != NSteps || 
            swingfoot_end_x_des.size() != NSteps) {
            throw std::runtime_error("Error parsing YAML file: Incorrect number of gait parameters!");
        }

        gps.resize(NSteps);
        for (int i = 0; i < NSteps; i++) {
            gps[i].swingfoot_midstep_z_des = swingfoot_midstep_z_des[i];
            gps[i].swingfoot_begin_x_des = swingfoot_begin_x_des[i];
            gps[i].swingfoot_end_x_des = swingfoot_end_x_des[i];
        }
    } 
    catch (std::exception& e) {
        std::cerr << "Error parsing YAML file: " << e.what() << std::endl;
        throw std::runtime_error("Error parsing YAML file! Check previous error message!");
    }
    
    Eigen::VectorXd z = Utils::initializeEigenMatrixFromFile(filepath + "initial-talos-multiple-step.txt");

    SmartPtr<TalosMultipleStepOptimizer> mynlp = new TalosMultipleStepOptimizer();
    try {
	    mynlp->set_parameters(NSteps,
                              z,
                              T,
                              N,
                              time_discretization,
                              degree,
                              model,
                              jtype,
                              gps);
        mynlp->constr_viol_tol = config["constr_viol_tol"].as<double>();
    }
    catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        throw std::runtime_error("Error initializing Ipopt class! Check previous error message!");
    }

    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

    try {
        app->Options()->SetNumericValue("tol", config["tol"].as<double>());
        app->Options()->SetNumericValue("constr_viol_tol", mynlp->constr_viol_tol);
        app->Options()->SetNumericValue("max_wall_time", config["max_wall_time"].as<double>());
        app->Options()->SetIntegerValue("max_iter", config["max_iter"].as<int>());
        app->Options()->SetNumericValue("obj_scaling_factor", config["obj_scaling_factor"].as<double>());
        app->Options()->SetIntegerValue("print_level", config["print_level"].as<double>());
        app->Options()->SetStringValue("mu_strategy", config["mu_strategy"].as<std::string>().c_str());
        app->Options()->SetStringValue("linear_solver", config["linear_solver"].as<std::string>().c_str());
        app->Options()->SetStringValue("ma57_automatic_scaling", "yes");

        if (mynlp->enable_hessian) {
            app->Options()->SetStringValue("hessian_approximation", "exact");
        }
        else {
            app->Options()->SetStringValue("hessian_approximation", "limited-memory");
        }
        // app->Options()->SetStringValue("nlp_scaling_method", "none");
    }
    catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        throw std::runtime_error("Error setting optimization options! Check previous error message!");
    }

    if (config["gredient_check"].as<bool>()) {
        app->Options()->SetStringValue("output_file", "ipopt.out");
        if (mynlp->enable_hessian) {
            app->Options()->SetStringValue("derivative_test", "second-order");
        }
        else {
            app->Options()->SetStringValue("derivative_test", "first-order");
        }
        app->Options()->SetNumericValue("point_perturbation_radius", 1e-3);
        // app->Options()->SetIntegerValue("derivative_test_first_index", 168);
        app->Options()->SetNumericValue("derivative_test_perturbation", 1e-7);
        app->Options()->SetNumericValue("derivative_test_tol", 1e-4);
    }

    // Initialize the IpoptApplication and process the options
    ApplicationReturnStatus status;
    status = app->Initialize();
    if( status != Solve_Succeeded ) {
		throw std::runtime_error("Error during initialization of optimization!");
    }

    try {
        auto start = std::chrono::high_resolution_clock::now();

        // Ask Ipopt to solve the problem
        status = app->OptimizeTNLP(mynlp);

        auto end = std::chrono::high_resolution_clock::now();
        double solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0;
        
        std::cout << "Data needed for comparison: " << mynlp->obj_value_copy << ' ' << mynlp->final_constr_violation << ' ' << solve_time << std::endl;
    }
    catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        throw std::runtime_error("Error solving optimization problem! Check previous error message!");
    }

    // Print the solution
    if (mynlp->solution.size() == mynlp->numVars) {
        std::ofstream solution(filepath + "solution-talos-multiple-step.txt");
        solution << std::setprecision(20);
        for (int i = 0; i < mynlp->numVars; i++) {
            solution << mynlp->solution[i] << std::endl;
        }
        solution.close();

        for (int p = 0; p < NSteps; p++) {
            std::ofstream trajectory(filepath + "trajectory-talos-multiple-step-" + std::to_string(p) + ".txt");
            trajectory << std::setprecision(20);
            const auto& cidPtr_ = mynlp->stepOptVec_[p]->cidPtr_;
            for (int i = 0; i < NUM_JOINTS; i++) {
                for (int j = 0; j < cidPtr_->N; j++) {
                    trajectory << cidPtr_->q(j)(i) << ' ';
                }
                trajectory << std::endl;
            }
            for (int i = 0; i < NUM_JOINTS; i++) {
                for (int j = 0; j < cidPtr_->N; j++) {
                    trajectory << cidPtr_->v(j)(i) << ' ';
                }
                trajectory << std::endl;
            }
            for (int i = 0; i < NUM_JOINTS; i++) {
                for (int j = 0; j < cidPtr_->N; j++) {
                    trajectory << cidPtr_->a(j)(i) << ' ';
                }
                trajectory << std::endl;
            }
            for (int i = 0; i < NUM_INDEPENDENT_JOINTS; i++) {
                for (int j = 0; j < cidPtr_->N; j++) {
                    trajectory << cidPtr_->tau(j)(i) << ' ';
                }
                trajectory << std::endl;
            }
            for (int i = 0; i < NUM_DEPENDENT_JOINTS; i++) {
                for (int j = 0; j < cidPtr_->N; j++) {
                    trajectory << cidPtr_->lambda(j)(i) << ' ';
                }
                trajectory << std::endl;
            }
            trajectory.close();
        }
    }

    return 0;
}