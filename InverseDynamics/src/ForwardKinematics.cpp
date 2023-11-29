#include "ForwardKinematics.h"

namespace IDTO {

void ForwardKinematicsHighOrderDerivative::fk(Transform& T, 
        const Model& model, 
        const Eigen::VectorXi& jtype,
        const int end, 
        const int start, 
        const Eigen::VectorXd& q, 
        const Transform& endT, 
        const Transform& startT) {
    int numJoints = model.nq;

    // find the kinematics chain
    chain.clear();
    int search_id = end;
    while (search_id != start) {
        chain.push_back(search_id - 1);

        if (search_id < 0 || search_id > numJoints) {
            throw std::runtime_error("forwardkinematics.cpp: fk(): Can not find end properly!");
        }

        search_id = model.parents[search_id];
    }
    std::reverse(chain.begin(), chain.end());

    T = startT;

    for (auto i : chain) {
        T *= model.jointPlacements[i + 1];

        Transform Tj(jtype[i], q[i]);
        T *= Tj;
    }

    T *= endT;
}

void ForwardKinematicsHighOrderDerivative::fk_jacobian(std::vector<Transform>& dTdq, 
                 const Model& model,
                 const Eigen::VectorXi& jtype,
                 const int end, 
                 const int start, 
                 const Eigen::VectorXd& q, 
                 const Transform& endT, 
                 const Transform& startT) {
    int numJoints = model.nq;

    // find the kinematics chain
    // chain declared as class public variable. 
    // always assume that fk_jacobian called after fk
    
    // std::vector<int> chain;
    // int search_id = end;
    // while (search_id != start) {
    //     chain.push_back(search_id - 1);

    //     if (search_id < 0 || search_id >= numJoints) {
    //         cout << "forwardkinematics.cpp: fk(): Can not find end properly!\n";
    //         throw -1;
    //     }

    //     search_id = model.parents[search_id];
    // }
    // std::reverse(chain.begin(), chain.end());

    dTdq.resize(numJoints);
    for (auto i : chain) {
        dTdq[i] = startT;
    }

    Transform temp;
    for (auto i : chain) {
        Transform Tj(jtype[i], q[i]);
        Transform dTjdq(jtype[i], q[i], 1);

        for (auto j : chain) {
            dTdq[j] *= model.jointPlacements[i + 1];
            if (j == i) {
                dTdq[j] *= dTjdq;
            }
            else {
                dTdq[j] *= Tj;
            }
        }
    }

    for (auto i : chain) {
        dTdq[i] *= endT;
    }
}

void ForwardKinematicsHighOrderDerivative::fk_hessian(std::vector<std::vector<Transform>>& ddTddq, 
                const Model& model, 
                const Eigen::VectorXi& jtype,
                const int end,
                const int start, 
                const Eigen::VectorXd& q, 
                const Transform& endT, 
                const Transform& startT) {
    int numJoints = model.nq;

    // find the kinematics chain
    // chain declared as class public variable. 
    // always assume that fk_hessian called after fk

    // std::vector<int> chain;
    // int search_id = end;
    // while (search_id != start) {
    //     chain.push_back(search_id - 1);

    //     if (search_id < 0 || search_id >= numJoints) {
    //         cout << "forwardkinematics.cpp: fk(): Can not find end properly!\n";
    //         throw -1;
    //     }

    //     search_id = model.parents[search_id];
    // }
    // std::reverse(chain.begin(), chain.end());

    if (ddTddq.size() != numJoints) {
        ddTddq.resize(numJoints);
    }
    for (int i = 0; i < numJoints; i++) {
        ddTddq[i].resize(numJoints);
    }

    for (auto i : chain) {
        for (auto j : chain) {
            ddTddq[i][j] = startT;
        }
    }

    Transform temp;
    for (auto i : chain) {
        Transform Tj(jtype[i], q[i]);
        Transform dTjdq(jtype[i], q[i], 1);
        Transform ddTjddq(jtype[i], q[i], 2);
        
        for (auto j : chain) {
            for (auto k : chain) {
                ddTddq[j][k] *= model.jointPlacements[i + 1];
                if (k >= j) {
                    if (j == i && k == i) {
                        ddTddq[j][k] *= ddTjddq;
                    } 
                    else if (j == i || k == i) {
                        ddTddq[j][k] *= dTjdq;
                    } 
                    else {
                        ddTddq[j][k] *= Tj;
                    }
                } 
                else {
                    ddTddq[j][k] = ddTddq[k][j];
                }
            }
        }
    }

    for (auto i : chain) {
        for (auto j : chain) {
            ddTddq[i][j] *= endT;
        }
    }
}

void ForwardKinematicsHighOrderDerivative::fk_thirdorder(std::vector<std::vector<std::vector<Transform>>>& dddTdddq, 
                   const Model& model, 
                   const Eigen::VectorXi& jtype,
                   const int end,
                   const int start, 
                   const Eigen::VectorXd& q, 
                   const Transform& endT, 
                   const Transform& startT) {
    int numJoints = model.nq;

    // find the kinematics chain
    // chain declared as class public variable. 
    // always assume that fk_hessian called after fk

    // std::vector<int> chain;
    // int search_id = end;
    // while (search_id != start) {
    //     chain.push_back(search_id - 1);

    //     if (search_id < 0 || search_id >= numJoints) {
    //         cout << "forwardkinematics.cpp: fk(): Can not find end properly!\n";
    //         throw -1;
    //     }

    //     search_id = model.parents[search_id];
    // }
    // std::reverse(chain.begin(), chain.end());

    if (dddTdddq.size() != numJoints) {
        dddTdddq.resize(numJoints);
    }
    for (int i = 0; i < numJoints; i++) {
        if (dddTdddq[i].size() != numJoints) {
            dddTdddq[i].resize(numJoints);
        }
        for (int j = 0; j < numJoints; j++) {
            dddTdddq[i][j].resize(numJoints);
        }
    }

    for (auto i : chain) {
        for (auto j : chain) {
            for (auto k : chain) {
                dddTdddq[i][j][k] = startT;
            }
        }
    }

    Transform temp;
    for (auto i : chain) {
        Transform Tj(jtype[i], q[i]);
        Transform dTjdq(jtype[i], q[i], 1);
        Transform ddTjddq(jtype[i], q[i], 2);
        Transform dddTjdddq(jtype[i], q[i], 3);
        
        for (auto j : chain) {
            for (auto k : chain) {
                for (auto h : chain) {
                    dddTdddq[j][k][h] *= model.jointPlacements[i + 1];
                    if (j == i && k == i && h == i) {
                        dddTdddq[j][k][h] *= dddTjdddq;
                    } else if ((j == i && k == i) || (j == i && h == i) || (k == i && h == i)) {
                        dddTdddq[j][k][h] *= ddTjddq;
                    } 
                    else if (j == i || k == i || h == i) {
                        dddTdddq[j][k][h] *= dTjdq;
                    }
                    else {
                        dddTdddq[j][k][h] *= Tj;
                    }
                }
            }
        }
    }

    for (auto i : chain) {
        for (auto j : chain) {
            for (auto k : chain) {
                dddTdddq[i][j][k] *= endT;
            }
        }
    }
}

Eigen::VectorXd ForwardKinematicsHighOrderDerivative::Transform2xyzrpy(const Transform& T) {
    const Eigen::MatrixXd& R = T.R;
    Eigen::VectorXd x(6);

    x(0) = T.p(0); // x
    x(1) = T.p(1); // y
    x(2) = T.p(2); // z
    
    x(3) = atan2(-R(1,2),R(2,2)); // roll
    x(4) = asin(R(0,2));          // pitch
    x(5) = atan2(-R(0,1),R(0,0)); // yaw

    return x;
}

void ForwardKinematicsHighOrderDerivative::Transform2xyzrpyJacobian(Eigen::MatrixXd& J, 
                              const Transform& T, 
                              const std::vector<Transform>& dTdq) {
    const Eigen::MatrixXd& R = T.R;
    
    // chain declared as class public variable. 
    // always assume that fk_hessian called after fk

    // result container has been allocated outside the function
    J.setZero();

    double t2 = R(0,0) * R(0,0);
    double t3 = R(0,1) * R(0,1);
    double t4 = R(1,2) * R(1,2);
    double t5 = R(2,2) * R(2,2);
    double t6 = t2 + t3;
    double t7 = t4 + t5;
    double t8 = 1.0 / t6;
    double t9 = 1.0 / t7;

    for (auto i : chain) {
        J(0, i) = dTdq[i].p(0);
        J(1, i) = dTdq[i].p(1);
        J(2, i) = dTdq[i].p(2);

        J(3, i) = R(1,2)*dTdq[i].R(2,2)*t9-R(2,2)*dTdq[i].R(1,2)*t9;
        J(4, i) = dTdq[i].R(0,2)/sqrt(-R(0,2)*R(0,2)+1.0); 
        J(5, i) = -R(0,0)*dTdq[i].R(0,1)*t8+R(0,1)*dTdq[i].R(0,0)*t8;
    }
}

void ForwardKinematicsHighOrderDerivative::Transform2xyzrpyHessian(Eigen::Array<Eigen::MatrixXd, 6, 1>& H,
                             const Transform& T, 
                             const std::vector<Transform>& dTdq,
                             const std::vector<std::vector<Transform>>& ddTddq) {
    const Eigen::MatrixXd& R = T.R;

    // chain declared as class public variable. 
    // always assume that fk_hessian called after fk

    // result container has been allocated outside the function
    double t2 = R(0,0) * R(0,0);
    double t3 = R(0,1) * R(0,1);
    double t4 = R(0,2) * R(0,2);
    double t5 = R(1,2) * R(1,2);
    double t6 = R(2,2) * R(2,2);
    double t7 = -t4;
    double t8 = t2 + t3;
    double t9 = t5 + t6;
    double t10 = t7 + 1.0;
    double t11 = 1.0 / t8;
    double t13 = 1.0 / t9;
    double t12 = t11 * t11;
    double t14 = t13 * t13;

    for (int i = 0; i < 6; i++) {
        H(i).setZero();
    }

    for (auto i : chain) {
        for (auto j : chain) {
            H(0)(i, j) = ddTddq[i][j].p(0);
            H(1)(i, j) = ddTddq[i][j].p(1);
            H(2)(i, j) = ddTddq[i][j].p(2);
            H(3)(i, j) = dTdq[j].R(1,2)*(dTdq[i].R(2,2)*t13-dTdq[i].R(2,2)*t5*t14*2.0+R(1,2)*R(2,2)*dTdq[i].R(1,2)*t14*2.0)-dTdq[j].R(2,2)*(dTdq[i].R(1,2)*t13-dTdq[i].R(1,2)*t6*t14*2.0+R(1,2)*R(2,2)*dTdq[i].R(2,2)*t14*2.0)+R(1,2)*ddTddq[i][j].R(2,2)*t13-R(2,2)*ddTddq[i][j].R(1,2)*t13;
            H(4)(i, j) = ddTddq[i][j].R(0,2)/sqrt(t10)+R(0,2)*dTdq[i].R(0,2)*dTdq[j].R(0,2)/pow(t10,1.5);
            H(5)(i, j) = -dTdq[j].R(0,0)*(dTdq[i].R(0,1)*t11-dTdq[i].R(0,1)*t2*t12*2.0+R(0,0)*R(0,1)*dTdq[i].R(0,0)*t12*2.0)+dTdq[j].R(0,1)*(dTdq[i].R(0,0)*t11-dTdq[i].R(0,0)*t3*t12*2.0+R(0,0)*R(0,1)*dTdq[i].R(0,1)*t12*2.0)-R(0,0)*ddTddq[i][j].R(0,1)*t11+R(0,1)*ddTddq[i][j].R(0,0)*t11;
        }
    }
}

void ForwardKinematicsHighOrderDerivative::Transform2xyzrpyThirdOrder(Eigen::Array<Eigen::MatrixXd, 6, 1>& TOx,
                                const Eigen::VectorXd& x,
                                const Transform& T, 
                                const std::vector<Transform>& dTdq,
                                const std::vector<std::vector<Transform>>& ddTddq,
                                const std::vector<std::vector<std::vector<Transform>>>& dddTdddq) {
    assert(x.size() == TOx(0).cols());

    const Eigen::MatrixXd& R = T.R;
    
    double t2 = R(0,0)*R(0,0);
    double t3 = R(0,1)*R(0,1);
    double t4 = R(0,2)*R(0,2);
    double t5 = R(1,2)*R(1,2);
    double t6 = R(2,2)*R(2,2);
    double t7 = -t4;
    double t8 = t2+t3;
    double t9 = t5+t6;
    double t10 = t7+1.0;
    double t11 = 1.0/t8;
    double t14 = 1.0/t9;
    double t12 = t11*t11;
    double t13 = t11*t11*t11;
    double t15 = t14*t14;
    double t16 = t14*t14*t14;
    double t17 = 1.0/pow(t10,1.5);

    for (int i = 0; i < 6; i++) {
        TOx(i).setZero();
    }

    for (auto i : chain) {
        for (auto j : chain) {
            for (auto k : chain) {
                double t18 = R(0,0)*dTdq[i].R(0,0)*t12*2.0;
                double t19 = R(0,1)*dTdq[i].R(0,1)*t12*2.0;
                double t20 = R(1,2)*dTdq[i].R(1,2)*t15*2.0;
                double t21 = R(2,2)*dTdq[i].R(2,2)*t15*2.0;
                double t24 = R(0,0)*dTdq[i].R(0,0)*t3*t13*8.0;
                double t25 = R(0,1)*dTdq[i].R(0,1)*t2*t13*8.0;
                double t26 = R(1,2)*dTdq[i].R(1,2)*t6*t16*8.0;
                double t27 = R(2,2)*dTdq[i].R(2,2)*t5*t16*8.0;
                double t22 = -t19;
                double t23 = -t21;
                double t28 = -t24;
                double t29 = -t26;
                double t30 = t18+t22+t25+t28;
                double t31 = t20+t23+t27+t29;

                double T1_i_j_k = dddTdddq[i][j][k].p(0);
                double T2_i_j_k = dddTdddq[i][j][k].p(1);
                double T3_i_j_k = dddTdddq[i][j][k].p(2);
                double T4_i_j_k = -ddTddq[j][k].R(2,2)*(R(1,2)*t21+dTdq[i].R(1,2)*t14-dTdq[i].R(1,2)*t6*t15*2.0)+ddTddq[j][k].R(1,2)*(R(2,2)*t20+dTdq[i].R(2,2)*t14-dTdq[i].R(2,2)*t5*t15*2.0)+dTdq[k].R(1,2)*(-dTdq[j].R(1,2)*(R(1,2)*dTdq[i].R(2,2)*t15*6.0-R(2,2)*dTdq[i].R(1,2)*t15*2.0-(R(1,2)*R(1,2)*R(1,2))*dTdq[i].R(2,2)*t16*8.0+R(2,2)*dTdq[i].R(1,2)*t5*t16*8.0)+dTdq[j].R(2,2)*t31+ddTddq[i][j].R(2,2)*t14-ddTddq[i][j].R(2,2)*t5*t15*2.0+R(1,2)*R(2,2)*ddTddq[i][j].R(1,2)*t15*2.0)-dTdq[k].R(2,2)*(dTdq[j].R(2,2)*(R(1,2)*dTdq[i].R(2,2)*t15*2.0-R(2,2)*dTdq[i].R(1,2)*t15*6.0+(R(2,2)*R(2,2)*R(2,2))*dTdq[i].R(1,2)*t16*8.0-R(1,2)*dTdq[i].R(2,2)*t6*t16*8.0)-dTdq[j].R(1,2)*t31+ddTddq[i][j].R(1,2)*t14-ddTddq[i][j].R(1,2)*t6*t15*2.0+R(1,2)*R(2,2)*ddTddq[i][j].R(2,2)*t15*2.0)-ddTddq[i][k].R(1,2)*(dTdq[j].R(2,2)*(t14-t6*t15*2.0)-R(1,2)*R(2,2)*dTdq[j].R(1,2)*t15*2.0)+ddTddq[i][k].R(2,2)*(dTdq[j].R(1,2)*(t14-t5*t15*2.0)-R(1,2)*R(2,2)*dTdq[j].R(2,2)*t15*2.0)+R(1,2)*dddTdddq[i][j][k].R(2,2)*t14-R(2,2)*dddTdddq[i][j][k].R(1,2)*t14;
                double T5_i_j_k = dddTdddq[i][j][k].R(0,2)/sqrt(t10)+dTdq[k].R(0,2)*(R(0,2)*ddTddq[i][j].R(0,2)*t17+dTdq[i].R(0,2)*dTdq[j].R(0,2)*t17+dTdq[i].R(0,2)*dTdq[j].R(0,2)*t4/pow(t10,2.5)*3.0)+R(0,2)*dTdq[i].R(0,2)*ddTddq[j][k].R(0,2)*t17+R(0,2)*dTdq[j].R(0,2)*ddTddq[i][k].R(0,2)*t17;
                double T6_i_j_k = -ddTddq[j][k].R(0,0)*(R(0,1)*t18+dTdq[i].R(0,1)*t11-dTdq[i].R(0,1)*t2*t12*2.0)+ddTddq[j][k].R(0,1)*(R(0,0)*t19+dTdq[i].R(0,0)*t11-dTdq[i].R(0,0)*t3*t12*2.0)-dTdq[k].R(0,0)*(-dTdq[j].R(0,0)*(R(0,0)*dTdq[i].R(0,1)*t12*6.0-R(0,1)*dTdq[i].R(0,0)*t12*2.0-(R(0,0)*R(0,0)*R(0,0))*dTdq[i].R(0,1)*t13*8.0+R(0,1)*dTdq[i].R(0,0)*t2*t13*8.0)+dTdq[j].R(0,1)*t30+ddTddq[i][j].R(0,1)*t11-ddTddq[i][j].R(0,1)*t2*t12*2.0+R(0,0)*R(0,1)*ddTddq[i][j].R(0,0)*t12*2.0)+dTdq[k].R(0,1)*(dTdq[j].R(0,1)*(R(0,0)*dTdq[i].R(0,1)*t12*2.0-R(0,1)*dTdq[i].R(0,0)*t12*6.0+(R(0,1)*R(0,1)*R(0,1))*dTdq[i].R(0,0)*t13*8.0-R(0,0)*dTdq[i].R(0,1)*t3*t13*8.0)-dTdq[j].R(0,0)*t30+ddTddq[i][j].R(0,0)*t11-ddTddq[i][j].R(0,0)*t3*t12*2.0+R(0,0)*R(0,1)*ddTddq[i][j].R(0,1)*t12*2.0)+ddTddq[i][k].R(0,0)*(dTdq[j].R(0,1)*(t11-t3*t12*2.0)-R(0,0)*R(0,1)*dTdq[j].R(0,0)*t12*2.0)-ddTddq[i][k].R(0,1)*(dTdq[j].R(0,0)*(t11-t2*t12*2.0)-R(0,0)*R(0,1)*dTdq[j].R(0,1)*t12*2.0)-R(0,0)*dddTdddq[i][j][k].R(0,1)*t11+R(0,1)*dddTdddq[i][j][k].R(0,0)*t11;

                TOx(0)(i, j) += T1_i_j_k * x(k);
                TOx(1)(i, j) += T2_i_j_k * x(k);
                TOx(2)(i, j) += T3_i_j_k * x(k);
                TOx(3)(i, j) += T4_i_j_k * x(k);
                TOx(4)(i, j) += T5_i_j_k * x(k);
                TOx(5)(i, j) += T6_i_j_k * x(k);
            }
        }
    }
}

}  // namespace IDTO