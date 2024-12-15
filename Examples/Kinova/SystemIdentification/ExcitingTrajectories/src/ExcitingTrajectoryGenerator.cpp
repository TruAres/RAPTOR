#include "ExcitingTrajectoryGenerator.h"

namespace RAPTOR {
namespace Kinova {

// // constructor
// ExcitingTrajectoryGenerator::ExcitingTrajectoryGenerator()
// {
// }


// // destructor
// ExcitingTrajectoryGenerator::~ExcitingTrajectoryGenerator()
// {
// }

bool ExcitingTrajectoryGenerator::set_parameters(
    const VecX& x0_input,
    const Number T_input,
    const int N_input,
    const int degree_input,
    const double base_frequency_input,
    const VecX& q0_input,
    const VecX& q_d0_input,
    const Model& model_input, 
    const std::string& regroupMatrixFileName,
    const std::vector<Vec3>& boxCenters,
    const std::vector<Vec3>& boxOrientations,
    const std::vector<Vec3>& boxSizes,
    const VecX& joint_limits_buffer_input,
    const VecX& velocity_limits_buffer_input,
    const VecX& torque_limits_buffer_input,
    const bool include_gripper_or_not,
    const double collison_buffer_input,
    Eigen::VectorXi jtype_input
) {
    enable_hessian = false;
    // x0 = x0_input;
    x0 = x0_input.segment(0,(2*degree_input+1)*model_input.nv);
    regroupMatrix = Utils::initializeEigenMatrixFromFile(regroupMatrixFileName);

    // fixed frequency fourier curves
    trajPtr_ = std::make_shared<FixedFrequencyFourierCurves>(T_input, 
                                                             N_input, 
                                                             model_input.nv, 
                                                             TimeDiscretization::Uniform, 
                                                             degree_input,
                                                             base_frequency_input,
                                                             q0_input,
                                                             q_d0_input);

    ridPtr_ = std::make_shared<RegressorInverseDynamics>(model_input, 
                                                         trajPtr_,
                                                         true,
                                                         jtype_input);

    // read joint limits from KinovaConstants.h
    VecX JOINT_LIMITS_LOWER_VEC = 
        Utils::deg2rad(
            Utils::initializeEigenVectorFromArray(JOINT_LIMITS_LOWER, NUM_JOINTS)) + 
        joint_limits_buffer_input;

    VecX JOINT_LIMITS_UPPER_VEC =
        Utils::deg2rad(
            Utils::initializeEigenVectorFromArray(JOINT_LIMITS_UPPER, NUM_JOINTS)) -
        joint_limits_buffer_input;

    // read velocity limits from KinovaConstants.h
    VecX VELOCITY_LIMITS_LOWER_VEC = 
        Utils::deg2rad(
            Utils::initializeEigenVectorFromArray(VELOCITY_LIMITS_LOWER, NUM_JOINTS)) +
        velocity_limits_buffer_input;

    VecX VELOCITY_LIMITS_UPPER_VEC = 
        Utils::deg2rad(
            Utils::initializeEigenVectorFromArray(VELOCITY_LIMITS_UPPER, NUM_JOINTS)) -
        velocity_limits_buffer_input;

    // read torque limits from KinovaConstants.h
    VecX TORQUE_LIMITS_LOWER_VEC = 
        Utils::initializeEigenVectorFromArray(TORQUE_LIMITS_LOWER, NUM_JOINTS) + 
        torque_limits_buffer_input;

    VecX TORQUE_LIMITS_UPPER_VEC = 
        Utils::initializeEigenVectorFromArray(TORQUE_LIMITS_UPPER, NUM_JOINTS) -
        torque_limits_buffer_input;

    // Joint limits
    constraintsPtrVec_.push_back(std::make_unique<JointLimits>(trajPtr_, 
                                                               JOINT_LIMITS_LOWER_VEC, 
                                                               JOINT_LIMITS_UPPER_VEC));
    constraintsNameVec_.push_back("joint limits");

    // Velocity limits
    constraintsPtrVec_.push_back(std::make_unique<VelocityLimits>(trajPtr_, 
                                                                  VELOCITY_LIMITS_LOWER_VEC, 
                                                                  VELOCITY_LIMITS_UPPER_VEC));
    constraintsNameVec_.push_back("velocity limits");        

    // Torque limits
    constraintsPtrVec_.push_back(std::make_unique<TorqueLimits>(trajPtr_, 
                                                                ridPtr_,
                                                                TORQUE_LIMITS_LOWER_VEC, 
                                                                TORQUE_LIMITS_UPPER_VEC));
    constraintsNameVec_.push_back("torque limits"); 

    // Customized constraints (collision avoidance)  
    if (boxCenters.size() > 0) {
    constraintsPtrVec_.push_back(std::make_unique<KinovaCustomizedConstraints>(trajPtr_,
                                                                               model_input,
                                                                               boxCenters,
                                                                               boxOrientations,
                                                                               boxSizes,
                                                                               include_gripper_or_not,
                                                                               collison_buffer_input,
                                                                               jtype_input));  
    constraintsNameVec_.push_back("obstacle avoidance constraints"); 
    }
    
    // check dimensions of regroupMatrix
    if (ridPtr_->Y.cols() != regroupMatrix.rows()) {
        throw std::invalid_argument("ExcitingTrajectoryGenerator: regroupMatrix has wrong dimensions!");
    }

    return true;
}

bool ExcitingTrajectoryGenerator::get_nlp_info(
        Index&          n,
        Index&          m,
        Index&          nnz_jac_g,
        Index&          nnz_h_lag,
        IndexStyleEnum& index_style
) {
    // number of decision variables
    numVars = trajPtr_->varLength;
    n = numVars;

    // number of inequality constraint
    numCons = 0;
    for ( Index i = 0; i < constraintsPtrVec_.size(); i++ ) {
        numCons += constraintsPtrVec_[i]->m;
    }
    m = numCons;

    nnz_jac_g = n * m;
    nnz_h_lag = n * (n + 1) / 2;

    // use the C style indexing (0-based)
    index_style = TNLP::C_STYLE;

    return true;
}

bool ExcitingTrajectoryGenerator::eval_f(
    Index         n,
    const Number* x,
    bool          new_x,
    Number&       obj_value
) {
    if(n != numVars){
       THROW_EXCEPTION(IpoptException, "*** Error wrong value of n in eval_f!");
    }

    VecX z = Utils::initializeEigenVectorFromArray(x, n);

    ridPtr_->compute(z, false);

    MatX regroupedObservationMatrix = ridPtr_->Y * regroupMatrix;
    Eigen::JacobiSVD<MatX> svd(regroupedObservationMatrix, 
                               Eigen::ComputeThinU | Eigen::ComputeThinV);
    const VecX& singularValues = svd.singularValues();
    const MatX& U = svd.matrixU();
    const MatX& V = svd.matrixV();

    const Number sigmaMax = singularValues(0);
    const Number sigmaMin = singularValues(singularValues.size() - 1);

    // log of 2-norm condition number (sigmaMax / sigmaMin)
    obj_value = std::log(sigmaMax) - std::log(sigmaMin);

    update_minimal_cost_solution(n, z, new_x, obj_value);

    return true;
}

bool ExcitingTrajectoryGenerator::eval_grad_f(
    Index         n,
    const Number* x,
    bool          new_x,
    Number*       grad_f
) {
    if(n != numVars){
       THROW_EXCEPTION(IpoptException, "*** Error wrong value of n in eval_grad_f!");
    }

    VecX z = Utils::initializeEigenVectorFromArray(x, n);

    ridPtr_->compute(z, true);

    MatX regroupedObservationMatrix = ridPtr_->Y * regroupMatrix;
    Eigen::JacobiSVD<MatX> svd(regroupedObservationMatrix, 
                               Eigen::ComputeThinU | Eigen::ComputeThinV);
    const VecX& singularValues = svd.singularValues();
    const MatX& U = svd.matrixU();
    const MatX& V = svd.matrixV();

    const Index lastRow = singularValues.size() - 1;
    const Number sigmaMax = singularValues(0);
    const Number sigmaMin = singularValues(lastRow);

    // refer to https://j-towns.github.io/papers/svd-derivative.pdf
    // for analytical gradient of singular values
    for (Index i = 0; i < n; i++) {
        MatX gradRegroupedObservationMatrix = ridPtr_->pY_pz(i) * regroupMatrix;

        const Number gradSigmaMax = U.col(0).transpose()       * gradRegroupedObservationMatrix * V.col(0);
        const Number gradSigmaMin = U.col(lastRow).transpose() * gradRegroupedObservationMatrix * V.col(lastRow);

        grad_f[i] = gradSigmaMax / sigmaMax - gradSigmaMin / sigmaMin;
    }

    return true;
}

}; // namespace Kinova
}; // namespace RAPTOR