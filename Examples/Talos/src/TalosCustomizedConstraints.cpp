#include "TalosCustomizedConstraints.h"

namespace RAPTOR {
namespace Talos {

TalosCustomizedConstraints::TalosCustomizedConstraints(const Model& model_input,
                                                       const Eigen::VectorXi& jtype_input,
                                                       std::shared_ptr<Trajectories>& trajPtr_input,
                                                       std::shared_ptr<TalosDynamicsConstraints>& ddcPtr_input,
                                                       const GaitParameters& gp_input) : 
    jtype(jtype_input),
    trajPtr_(trajPtr_input),
    ddcPtr_(ddcPtr_input),
    gp(gp_input) {
    modelPtr_ = std::make_unique<Model>(model_input);
    fkPtr_ = std::make_unique<ForwardKinematicsSolver>(modelPtr_.get(), jtype);

    leftfoot_endT.p << 0, 0, -0.107;
    rightfoot_endT.p << 0, 0, -0.107;

    q = MatX::Zero(modelPtr_->nv, trajPtr_->N);
    pq_pz.resize(1, trajPtr_->N);
    swingfoot_xyzrpy = MatX::Zero(6, trajPtr_->N);
    pswingfoot_xyzrpy_pz.resize(1, trajPtr_->N);

    m = trajPtr_->N * 8 + 4;
    initialize_memory(m, trajPtr_->varLength, false);
}

void TalosCustomizedConstraints::compute(const VecX& z, 
                                         bool compute_derivatives,
                                         bool compute_hessian) {
    if (compute_hessian) {
        throw std::invalid_argument("TalosCustomizedConstraints does not support hessian computation");
    }

    trajPtr_->compute(z, compute_derivatives, compute_hessian);

    const int fk_order = compute_derivatives ? 1 : 0;

    // compute full joint trajectories and swing foot forward kinematics
    for (int i = 0; i < trajPtr_->N; i++) {
        if (ddcPtr_->stanceLeg == 'L') {
            swingfoot_endT = rightfoot_endT;
            swingfoot_id = modelPtr_->getJointId("leg_right_6_joint");
        }
        else {
            swingfoot_endT = leftfoot_endT;
            swingfoot_id = modelPtr_->getJointId("leg_left_6_joint");
        }
        
        VecX qi(modelPtr_->nq);
        ddcPtr_->fill_independent_vector(qi, trajPtr_->q(i));
        ddcPtr_->setupJointPosition(qi, compute_derivatives);
        q.col(i) = qi;

        fkPtr_->compute(0, swingfoot_id, qi, nullptr, &swingfoot_endT, fk_order);

        swingfoot_xyzrpy.col(i).head(3) = fkPtr_->getTranslation();
        swingfoot_xyzrpy.col(i).tail(3) = fkPtr_->getRPY();

        if (compute_derivatives) {
            pq_pz(i).resize(modelPtr_->nv, trajPtr_->varLength);
            
            // fill in independent joints derivatives directly
            for (int j = 0; j < ddcPtr_->numIndependentJoints; j++) {
                int indenpendentJointIndex = ddcPtr_->return_independent_joint_index(j);
                pq_pz(i).row(indenpendentJointIndex) = trajPtr_->pq_pz(i).row(j);
            }
            // compute and fill in dependent joints derivatives
            MatX pq_dep_pz = ddcPtr_->pq_dep_pq_indep * trajPtr_->pq_pz(i);
            for (int j = 0; j < ddcPtr_->numDependentJoints; j++) {
                int denpendentJointIndex = ddcPtr_->return_dependent_joint_index(j);
                pq_pz(i).row(denpendentJointIndex) = pq_dep_pz.row(j);
            }

            pswingfoot_xyzrpy_pz(i).resize(6, trajPtr_->varLength);
            pswingfoot_xyzrpy_pz(i).topRows(3) = fkPtr_->getTranslationJacobian() * pq_pz(i);
            pswingfoot_xyzrpy_pz(i).bottomRows(3) = fkPtr_->getRPYJacobian() * pq_pz(i);
        }
    }

    // (1) swingfoot height always larger than 0
    //               height equal to 0 at the beginning and at the end
    //               height higher than the desired value in the middle
    g1 = swingfoot_xyzrpy.row(2);

    // (2) swingfoot always flat and points forward
    g2 = Utils::wrapToPi(swingfoot_xyzrpy.row(3)); // swingfoot roll
    g3 = Utils::wrapToPi(swingfoot_xyzrpy.row(4)); // swingfoot pitch
    g4 = Utils::wrapToPi(swingfoot_xyzrpy.row(5)); // swingfoot yaw

    // (3) swingfoot xy equal to desired value 
    //     at the beginning and at the end of each walking step
    g5 = VecX::Zero(4);
    if (ddcPtr_->stanceLeg == 'L') {
        g5 << swingfoot_xyzrpy(0, 0)               - gp.swingfoot_begin_x_des,
              swingfoot_xyzrpy(1, 0)               - gp.swingfoot_begin_y_des,
              swingfoot_xyzrpy(0, trajPtr_->N - 1) - gp.swingfoot_end_x_des,
              swingfoot_xyzrpy(1, trajPtr_->N - 1) - gp.swingfoot_end_y_des;
    }
    else {
        g5 << swingfoot_xyzrpy(0, 0)               - gp.swingfoot_begin_x_des,
              swingfoot_xyzrpy(1, 0)               + gp.swingfoot_begin_y_des,
              swingfoot_xyzrpy(0, trajPtr_->N - 1) - gp.swingfoot_end_x_des,
              swingfoot_xyzrpy(1, trajPtr_->N - 1) + gp.swingfoot_end_y_des;
    }

    // (4) torso height always larger than 1 meter
    //           roll and pitch always close to 0
    //           yaw always close to 0 when walking forward
    g6 = q.row(2); // torso height
    g7 = Utils::wrapToPi(q.row(3)); // torso roll
    g8 = Utils::wrapToPi(q.row(4)); // torso pitch
    g9 = Utils::wrapToPi(q.row(5)); // torso yaw

    // std::cout << g1.transpose() << std::endl;
    // std::cout << g2.transpose() << std::endl;
    // std::cout << g3.transpose() << std::endl;
    // std::cout << g4.transpose() << std::endl;
    // std::cout << g5.transpose() << std::endl;
    // std::cout << g6.transpose() << std::endl;
    // std::cout << g7.transpose() << std::endl;
    // std::cout << g8.transpose() << std::endl;
    // std::cout << g9.transpose() << std::endl;
    // std::cout << std::endl;

    g << g1, g2, g3, g4, g5, g6, g7, g8, g9;

    if (compute_derivatives) {
        int iter = 0;
        for (int i = 0; i < trajPtr_->N; i++) {
            pg_pz.row(iter++) = pswingfoot_xyzrpy_pz(i).row(2);
        }
        for (int i = 0; i < trajPtr_->N; i++) {
            pg_pz.row(iter++) = pswingfoot_xyzrpy_pz(i).row(3);
        }
        for (int i = 0; i < trajPtr_->N; i++) {
            pg_pz.row(iter++) = pswingfoot_xyzrpy_pz(i).row(4);
        }
        for (int i = 0; i < trajPtr_->N; i++) {
            pg_pz.row(iter++) = pswingfoot_xyzrpy_pz(i).row(5);
        }
        pg_pz.row(iter++) = pswingfoot_xyzrpy_pz(0).row(0);
        pg_pz.row(iter++) = pswingfoot_xyzrpy_pz(0).row(1);
        pg_pz.row(iter++) = pswingfoot_xyzrpy_pz(trajPtr_->N - 1).row(0);
        pg_pz.row(iter++) = pswingfoot_xyzrpy_pz(trajPtr_->N - 1).row(1);
        for (int i = 0; i < trajPtr_->N; i++) {
            pg_pz.row(iter++) = pq_pz(i).row(2);
        }
        for (int i = 0; i < trajPtr_->N; i++) {
            pg_pz.row(iter++) = pq_pz(i).row(3);
        }
        for (int i = 0; i < trajPtr_->N; i++) {
            pg_pz.row(iter++) = pq_pz(i).row(4);
        }
        for (int i = 0; i < trajPtr_->N; i++) {
            pg_pz.row(iter++) = pq_pz(i).row(5);
        }
    }
}

void TalosCustomizedConstraints::compute_bounds() {
    // (1) swingfoot height always larger than 0
    //               height equal to 0 at the beginning and at the end
    //               height higher than the desired value in the middle
    g1_lb = VecX::Zero(trajPtr_->N);
    g1_ub = VecX::Constant(trajPtr_->N, 1e19);
    g1_lb(trajPtr_->N / 2) = gp.swingfoot_midstep_z_des;
    g1_ub(0) = 0;
    g1_ub(trajPtr_->N - 1) = 0;

    // (2) swingfoot always flat and points forward
    g2_lb = VecX::Zero(trajPtr_->N);
    g2_ub = VecX::Zero(trajPtr_->N);
    g3_lb = VecX::Zero(trajPtr_->N);
    g3_ub = VecX::Zero(trajPtr_->N);
    g4_lb = VecX::Zero(trajPtr_->N);
    g4_ub = VecX::Zero(trajPtr_->N);

    // (3) swingfoot xy equal to desired value at the beginning and at the end
    g5_lb = VecX::Zero(4);
    g5_ub = VecX::Zero(4);

    // (4) torso height always larger than 0.9 meter
    //     roll and pitch always close to 0
    //     yaw always close to 0 when walking forward
    g6_lb = VecX::Constant(trajPtr_->N, 0.9);
    g6_ub = VecX::Constant(trajPtr_->N, 1e19);
    g7_lb = VecX::Constant(trajPtr_->N, -gp.eps_torso_angle);
    g7_ub = VecX::Constant(trajPtr_->N, gp.eps_torso_angle);
    g8_lb = VecX::Constant(trajPtr_->N, -gp.eps_torso_angle);
    g8_ub = VecX::Constant(trajPtr_->N, gp.eps_torso_angle);
    g9_lb = VecX::Constant(trajPtr_->N, -gp.eps_torso_angle);
    g9_ub = VecX::Constant(trajPtr_->N, gp.eps_torso_angle);

    g_lb << g1_lb, g2_lb, g3_lb, g4_lb, g5_lb, g6_lb, g7_lb, g8_lb, g9_lb;
    g_ub << g1_ub, g2_ub, g3_ub, g4_ub, g5_ub, g6_ub, g7_ub, g8_ub, g9_ub;
}

}; // namespace Talos
}; // namespace RAPTOR