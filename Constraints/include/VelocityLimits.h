#ifndef VELOCITYLIMITS_H
#define VELOCITYLIMITS_H

#include <memory>

#include "Constraints.h"
#include "Trajectories.h"
#include "Utils.h"

namespace IDTO {

class VelocityLimits : public Constraints {
public:
    using VecX = Eigen::VectorXd;
    using MatX = Eigen::MatrixXd;

    // Constructor
    VelocityLimits() = default;

    // Constructor
    VelocityLimits(std::shared_ptr<Trajectories>& trajPtr_input, 
                const VecX& lowerLimits_input, 
                const VecX& upperLimits_input);

    // Destructor
    ~VelocityLimits() = default;

    // class methods:
        // compute constraints
    virtual void compute(const VecX& z, bool compute_derivatives = true) override;

        // compute constraints lower bounds and upper bounds
    virtual void compute_bounds() override;

    // class variables:
    std::shared_ptr<Trajectories> trajPtr_;

    VecX lowerLimits;
    VecX upperLimits;
};

}; // namespace IDTO

#endif // VELOCITYLIMITS_H