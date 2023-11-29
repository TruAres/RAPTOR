#include "Trajectories.h"

namespace IDTO {

Trajectories::Trajectories(const VecX& tspan_input, int Nact_input) :
    tspan(tspan_input),
    Nact(Nact_input){
    T = tspan.bottomLeftCorner<1,1>().value();
    N = tspan.size();
    
    q.resize(1, N);
    q_d.resize(1, N);
    q_dd.resize(1, N);

    pq_pz.resize(1, N);
    pq_d_pz.resize(1, N);
    pq_dd_pz.resize(1, N);
}

Trajectories::Trajectories(double T_input, int N_input, int Nact_input, TimeDiscretization time_discretization) :
    T(T_input),
    N(N_input),
    Nact(Nact_input) {;
    if (time_discretization == Uniform) {
        tspan = VecX::LinSpaced(N, 0, T);
    } 
    else if (time_discretization == Chebyshev) {
        tspan = VecX::Zero(N);
        for (int i = 1; i < N - 1; i++) {
            tspan(i) = 0.5 * T * (1 - cos(M_PI * (2 * i - 1) / (2 * (N - 2))));
        }
        tspan(0) = 0;
        tspan(N - 1) = T;
    }
    
    q.resize(1, N);
    q_d.resize(1, N);
    q_dd.resize(1, N);

    pq_pz.resize(1, N);
    pq_d_pz.resize(1, N);
    pq_dd_pz.resize(1, N);
}

void Trajectories::compute(const VecX& z, bool compute_derivatives) {
    assert(z.size() == varLength);

    for (int i = 0; i < N; i++) {
        q(i) = VecX::Zero(Nact);
        q_d(i) = VecX::Zero(Nact);
        q_dd(i) = VecX::Zero(Nact);
    }

    if (compute_derivatives) {
        for (int i = 0; i < N; i++) {
            pq_pz(i).resize(Nact, varLength);
            pq_d_pz(i).resize(Nact, varLength);
            pq_dd_pz(i).resize(Nact, varLength);
        }
    }
}

}; // namespace IDTO
