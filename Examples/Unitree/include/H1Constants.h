#pragma once

namespace RAPTOR {
namespace H1 {

// constants related to H1
constexpr int NUM_JOINTS = 18;
constexpr int NUM_DEPENDENT_JOINTS = 6;
constexpr int NUM_INDEPENDENT_JOINTS = 12;

// pulled from talos_reduced_armfixed.urdf
constexpr double JOINT_LIMITS_LOWER[NUM_JOINTS] = {
    -1000,           // Px
    -1000,           // Py
    -1000,           // Pz
    -1000,           // Rx
    -1000,           // Ry
    -1000,           // Rz
    -0.43,         // left_hip_yaw_joint
    -3.14,         // left_hip_pitch_joint
    -0.43,         // left_hip_roll_joint
    -0.26,       // left_knee_joint
    -0.897334,         // left_ankle_pitch_joint
    -0.261799,         // left_ankle_roll_joint
    -0.43,         // right_hip_yaw_joint 
    -3.14,         // right_hip_pitch_joint
    -3.14,         // right_hip_roll_joint
    -0.26,       // right_knee_joint   
    -0.897334,        // right_ankle_pitch_joint
    -0.261799          // right_ankle_roll_joint
};

// pulled from talos_reduced_armfixed.urdf
constexpr double JOINT_LIMITS_UPPER[NUM_JOINTS] = {
    1000,           // Px
    1000,           // Py
    1000,           // Pz
    1000,           // Rx
    1000,           // Ry
    1000,           // Rz
    0.43,         // left_hip_yaw_joint
    2.5,         // left_hip_pitch_joint
    3.14,         // left_hip_roll_joint
    2.05,         // left_knee_joint
    0.523598,         // left_ankle_pitch_joint
    0.261799,         // left_ankle_roll_joint   
    0.43,         // right_hip_yaw_joint
    2.5,         // right_hip_pitch_joint
    0.43,         // right_hip_roll_joint
    2.05,         // right_knee_joint
    0.523598,          // right_ankle_pitch_joint
    0.261799          // right_ankle_roll_joint
};

constexpr double TORQUE_LIMITS_LOWER[NUM_INDEPENDENT_JOINTS] = {
    -220,  // left_hip_yaw_joint
    -220,  // left_hip_pitch_joint
    -220,  // left_hip_roll_joint
    -360, // left_knee_joint
    -45,  // left_ankle_pitch_joint
    -45,  // left_ankle_roll_joint
    -220, // right_hip_yaw_joint
    -220, // right_hip_pitch_joint
    -220, // right_hip_roll_joint
    -360, // right_knee_joint
    -45, // right_ankle_pitch_joint
    -45  // right_ankle_roll_joint
};

constexpr double TORQUE_LIMITS_UPPER[NUM_INDEPENDENT_JOINTS] = {
    220,  // left_hip_yaw_joint
    220,  // left_hip_pitch_joint
    220,  // left_hip_roll_joint
    360, // left_knee_joint
    45,  // left_ankle_pitch_joint
    45,  // left_ankle_roll_joint
    220,  // right_hip_yaw_joint
    220,  // right_hip_pitch_joint
    220,  // right_hip_roll_joint
    360, // right_knee_joint
    45,  // right_ankle_pitch_joint
    45   // right_ankle_roll_joint
};

constexpr double MU = 0.7;
constexpr double GAMMA = 0.7;
constexpr double FOOT_WIDTH = 0.08; // (m)
constexpr double FOOT_LENGTH = 0.26; // (m)

constexpr char LEFT_FOOT_NAME[] = "left_ankle_roll_joint";
constexpr char RIGHT_FOOT_NAME[] = "right_ankle_roll_joint";

}; // namespace H1
}; // namespace RAPTOR