#include "H1ConstrainedInverseDynamics.h"

namespace RAPTOR {
namespace H1 {

H1ConstrainedInverseDynamics::H1ConstrainedInverseDynamics(const Model& model_input, 
                                                           std::shared_ptr<Trajectories>& trajPtr_input,
                                                           int NUM_DEPENDENT_JOINTS_input,
                                                           char stanceLeg_input, 
                                                           const Transform& stance_foot_T_des_input) :
    ConstrainedInverseDynamics(model_input, trajPtr_input, NUM_DEPENDENT_JOINTS_input) {
    ddcPtr_ = std::make_shared<H1DynamicsConstraints>(modelPtr_, 
                                                      stanceLeg_input,
                                                      stance_foot_T_des_input);
    dcPtr_ = ddcPtr_; // convert to base class
}

}; // namespace H1
}; // namespace RAPTOR