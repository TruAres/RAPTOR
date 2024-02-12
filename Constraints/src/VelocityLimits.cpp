#include "VelocityLimits.h"

namespace IDTO {

VelocityLimits::VelocityLimits(std::shared_ptr<Trajectories>& trajPtr_input,
                         const VecX& lowerLimits_input, 
                         const VecX& upperLimits_input) : 
    lowerLimits(lowerLimits_input), 
    upperLimits(upperLimits_input) {
    trajPtr_ = trajPtr_input;

    if (lowerLimits.size() != upperLimits.size()) {
        throw std::invalid_argument("lowerLimits and upperLimits must be the same size");
    }

    if (lowerLimits.size() != trajPtr_->Nact) {
        throw std::invalid_argument("lowerLimits and upperLimits must be the same size as the number of actuated joints");
    }

    m = trajPtr_->N * trajPtr_->Nact;
    
    g = VecX::Zero(m);
    g_lb = VecX::Zero(m);
    g_ub = VecX::Zero(m);
    pg_pz.resize(m, trajPtr_->varLength);
}

void VelocityLimits::compute(const VecX& z, bool compute_derivatives) {
    if (is_computed(z, compute_derivatives)) {
        return;
    }

    if (compute_derivatives) {
        pg_pz.setZero();
    }

    trajPtr_->compute(z, compute_derivatives);

    for (int i = 0; i < trajPtr_->N; i++) {
        g.block(i * trajPtr_->Nact, 0, trajPtr_->Nact, 1) = trajPtr_->q_d(i).head(trajPtr_->Nact);

        if (compute_derivatives) {
            pg_pz.block(i * trajPtr_->Nact, 0, trajPtr_->Nact, trajPtr_->varLength) = trajPtr_->pq_d_pz(i);
        }
    }
}

void VelocityLimits::compute_bounds() {
    for (int i = 0; i < trajPtr_->N; i++) {
        g_lb.block(i * trajPtr_->Nact, 0, trajPtr_->Nact, 1) = lowerLimits;
        g_ub.block(i * trajPtr_->Nact, 0, trajPtr_->Nact, 1) = upperLimits;
    }
}

}; // namespace IDTO