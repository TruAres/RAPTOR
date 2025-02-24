
#ifndef H1_CONSTRAINED_INVERSE_DYNAMICS_H
#define H1_CONSTRAINED_INVERSE_DYNAMICS_H

#include "ConstrainedInverseDynamics.h"
#include "H1DynamicsConstraints.h"

namespace RAPTOR {
namespace H1 {

class H1ConstrainedInverseDynamics : public ConstrainedInverseDynamics {
public:
    using Model = pinocchio::Model;

    // Constructor
    H1ConstrainedInverseDynamics() = default;

    // Constructor
    H1ConstrainedInverseDynamics(const Model& model_input, 
                                 std::shared_ptr<Trajectories>& trajPtr_input,
                                 int numDependentJoints_input,
                                 char stanceLeg, 
                                 const Transform& stance_foot_T_des_input);

    // Destructor
    ~H1ConstrainedInverseDynamics() = default;

    // class members:
    // a pointer type of H1DynamicsConstraints, 
    // that shares the same memory with dcPtr_ defined in base class ConstrainedInverseDynamics
    // so that other Digit-related class can access specific field in H1DynamicsConstraints
    // such as stanceLeg, stance_foot_T_des, etc.
    std::shared_ptr<H1DynamicsConstraints> ddcPtr_;
};

}; // namespace H1
}; // namespace RAPTOR

#endif // H1_CONSTRAINED_INVERSE_DYNAMICS_H
