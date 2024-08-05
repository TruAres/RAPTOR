#ifndef TALOS_CUSTOMIZED_CONSTRAINTS_H
#define TALOS_CUSTOMIZED_CONSTRAINTS_H

#include "Constraints.h"
#include "FourierCurves.h"
#include "TalosConstrainedInverseDynamics.h"
#include "TalosDynamicsConstraints.h"
#include "Utils.h"

namespace RAPTOR {
namespace Talos {

typedef struct GaitParameters_ {
    double eps_torso_angle = Utils::deg2rad(3); // 3 degrees
    double swingfoot_midstep_z_des = 0.15; // meters
    double swingfoot_begin_x_des = 0.00; // meters 
    double swingfoot_begin_y_des = -0.17; // meters
    double swingfoot_end_x_des = 0.00; // meters
    double swingfoot_end_y_des = -0.17; // meters
} GaitParameters;

class TalosCustomizedConstraints : public Constraints {
public:
    using Model = pinocchio::Model;
    using VecX = Eigen::VectorXd;
    using MatX = Eigen::MatrixXd;

    // Constructor
    TalosCustomizedConstraints() = default;

    // Constructor
    TalosCustomizedConstraints(const Model& model_input,
                               const Eigen::VectorXi& jtype_input,
                               std::shared_ptr<Trajectories>& trajPtr_input,
                               std::shared_ptr<TalosDynamicsConstraints>& dcPtr_input,
                               const GaitParameters& gp_input);

    // Destructor
    ~TalosCustomizedConstraints() = default;

    // class methods:
        // compute constraints
    void compute(const VecX& z, 
                 bool compute_derivatives = true,
                 bool compute_hessian = false) final override;

        // compute constraints lower bounds and upper bounds
    void compute_bounds() final override;

        // print violation information
    void print_violation_info() final override;

    // class variables:
    GaitParameters gp;

    std::shared_ptr<Trajectories> trajPtr_;
    std::shared_ptr<TalosDynamicsConstraints> ddcPtr_;

    std::unique_ptr<Model> modelPtr_;

    std::unique_ptr<ForwardKinematicsSolver> fkPtr_;

        // jtype copy
    Eigen::VectorXi jtype;

        // swing foot information
    Model::JointIndex swingfoot_id = 0;

    Transform swingfoot_endT;

    Transform leftfoot_endT;
    Transform rightfoot_endT;

        // updated in compute()
    MatX q;
    MatX swingfoot_xyzrpy;

    Eigen::Array<MatX, 1, Eigen::Dynamic> pq_pz;
    Eigen::Array<MatX, 1, Eigen::Dynamic> pswingfoot_xyzrpy_pz;

        // forward kinematics derivatives
    std::vector<Transform> dTdq;

        // constraints
    VecX g1, g1_lb, g1_ub;
    VecX g2, g2_lb, g2_ub;
    VecX g3, g3_lb, g3_ub;
    VecX g4, g4_lb, g4_ub;
    VecX g5, g5_lb, g5_ub;
    VecX g6, g6_lb, g6_ub;
    VecX g7, g7_lb, g7_ub;
    VecX g8, g8_lb, g8_ub;
    VecX g9, g9_lb, g9_ub;
};

} // namespace Talos
} // namespace RAPTOR

#endif // TALOS_CUSTOMIZED_CONSTRAINTS_H
