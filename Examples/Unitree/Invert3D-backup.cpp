
// #include <pinocchio/algorithm/frames.hpp>
// #include <pinocchio/algorithm/kinematics.hpp>
// #include <pinocchio/parsers/urdf.hpp>
// #include <fstream>
// #include <iostream>
// #include <vector>
// #include <Eigen/Dense>
// #include <sstream>

// int main() {
//     const std::string filepath = "../Examples/Unitree/data/";
//     const std::string urdf_filename = "../Robots/Unitree/h1_2_12dof_floatingbase.urdf";
    
//     // 1. 构建机器人模型
//     pinocchio::Model model;
//     try {
//         pinocchio::urdf::buildModel(urdf_filename, model);
//         std::cout << "Model built successfully." << std::endl;
//         std::cout << "Number of joints (nq): " << model.nq << std::endl;
//         std::cout << "Total joints (njoints): " << model.njoints << std::endl;
//         std::cout << "Joint names:" << std::endl;
//         for (pinocchio::JointIndex i = 1; i < model.njoints; ++i) {
//             std::cout << "  " << i << ": " << model.names[i] << std::endl;
//         }
//     } catch (const std::exception& e) {
//         std::cerr << "Error building model: " << e.what() << std::endl;
//         return 1;
//     }
    
//     // 2. 读取关节角度轨迹文件
//     std::ifstream input(filepath + "full-trajectory-h1-forward.txt");
//     if (!input.is_open()) {
//         std::cerr << "Error opening trajectory file." << std::endl;
//         return 1;
//     }
    
//     std::vector<Eigen::VectorXd> jointTrajectories;
//     std::string line;
//     while (std::getline(input, line)) {
//         std::istringstream iss(line);
//         Eigen::VectorXd q(model.nq);
//         for (int i = 0; i < model.nq; ++i) {
//             iss >> q[i];
//         }
//         jointTrajectories.push_back(q);
//     }
//     std::cout << "\nRead " << jointTrajectories.size() << " configurations." << std::endl;
    
//     // 3. 准备输出文件
//     std::ofstream output(filepath + "all_joint_positions.txt");
//     if (!output.is_open()) {
//         std::cerr << "Error creating output file." << std::endl;
//         return 1;
//     }
    
//     // 4. 创建数据对象
//     pinocchio::Data data(model);
    
//     // 5. 遍历所有时间步
//     for (size_t i = 0; i < jointTrajectories.size(); ++i) {
//         const auto& q = jointTrajectories[i];
//         Eigen::VectorXd q_full = q;
        
//         // 计算正向运动学
//         pinocchio::forwardKinematics(model, data, q_full);
        
//         // 输出所有关节位置（从索引1开始，跳过世界坐标系）
//         for (pinocchio::JointIndex j = 1; j < model.njoints; ++j) {
//             const auto& joint_pos = data.oMi[j].translation();
//             output << joint_pos[0] << " " << joint_pos[1] << " " << joint_pos[2] << " ";
//         }
//         output << "\n";
        
//         // 调试输出
//         if (i < 2) {
//             std::cout << "\nConfiguration " << i << " joint positions:" << std::endl;
//             for (pinocchio::JointIndex j = 1; j < model.njoints; ++j) {
//                 const auto& pos = data.oMi[j].translation();
//                 std::cout << "  " << model.names[j] << ": " << pos.transpose() << std::endl;
//             }
//         }
//     }
    
//     std::cout << "\nAll joint positions saved to " << filepath + "all_joint_positions.txt" << std::endl;
//     return 0;
// }


// #include <pinocchio/algorithm/frames.hpp>
// #include <pinocchio/algorithm/kinematics.hpp>
// #include <pinocchio/parsers/urdf.hpp>
// #include <fstream>
// #include <iostream>
// #include <vector>
// #include <Eigen/Dense>
// #include <sstream>

// int main() {
//     const std::string filepath = "../Examples/Unitree/data/";
//     const std::string urdf_filename_lower = "../Robots/Unitree/h1_2_12dof_floatingbase.urdf";
//     const std::string urdf_filename_upper = "../Robots/Unitree/h1_2_27dof.urdf";

//     // 1. 构建下半身机器人模型
//     pinocchio::Model model_lower;
//     try {
//         pinocchio::urdf::buildModel(urdf_filename_lower, model_lower);
//         std::cout << "Lower body model built successfully." << std::endl;
//         std::cout << "Number of joints (nq): " << model_lower.nq << std::endl;
//         std::cout << "Total joints (njoints): " << model_lower.njoints << std::endl;
//     } catch (const std::exception& e) {
//         std::cerr << "Error building lower body model: " << e.what() << std::endl;
//         return 1;
//     }

//     // 2. 构建上半身机器人模型
//     pinocchio::Model model_upper;
//     try {
//         pinocchio::urdf::buildModel(urdf_filename_upper, model_upper);
//         std::cout << "Upper body model built successfully." << std::endl;
//         std::cout << "Number of joints (nq): " << model_upper.nq << std::endl;
//         std::cout << "Total joints (njoints): " << model_upper.njoints << std::endl;
//     } catch (const std::exception& e) {
//         std::cerr << "Error building upper body model: " << e.what() << std::endl;
//         return 1;
//     }

//     // 3. 读取下半身关节角度轨迹文件
//     std::ifstream input(filepath + "full-trajectory-h1-forward.txt");
//     if (!input.is_open()) {
//         std::cerr << "Error opening trajectory file." << std::endl;
//         return 1;
//     }

//     std::vector<Eigen::VectorXd> jointTrajectoriesLower;
//     std::string line;
//     while (std::getline(input, line)) {
//         std::istringstream iss(line);
//         Eigen::VectorXd q_lower(model_lower.nq);
//         for (int i = 0; i < model_lower.nq; ++i) {
//             iss >> q_lower[i];
//         }
//         jointTrajectoriesLower.push_back(q_lower);
//     }
//     std::cout << "\nRead " << jointTrajectoriesLower.size() << " lower body configurations." << std::endl;

//     // 4. 准备输出文件
//     std::ofstream output(filepath + "all_joint_positions_with_upper_body.txt");
//     if (!output.is_open()) {
//         std::cerr << "Error creating output file." << std::endl;
//         return 1;
//     }

//     // 5. 创建数据对象
//     pinocchio::Data data_lower(model_lower);
//     pinocchio::Data data_upper(model_upper);

//     // 6. 遍历所有时间步
//     for (size_t i = 0; i < jointTrajectoriesLower.size(); ++i) {
//         const auto& q_lower = jointTrajectoriesLower[i];

//         // 创建完整的关节角度向量（下半身 + 上半身）
//         Eigen::VectorXd q_full(model_upper.nq);
//         q_full.head(model_lower.nq) = q_lower; // 下半身角度
//         q_full.tail(model_upper.nq - model_lower.nq).setZero(); // 上半身角度初始化为 0

//         // 计算正向运动学
//         pinocchio::forwardKinematics(model_upper, data_upper, q_full);

//         // 输出所有关节位置（从索引1开始，跳过世界坐标系）
//         for (pinocchio::JointIndex j = 1; j < model_upper.njoints; ++j) {
//             const auto& joint_pos = data_upper.oMi[j].translation();
//             output << joint_pos[0] << " " << joint_pos[1] << " " << joint_pos[2] << " ";
//         }
//         output << "\n";

//         // 调试输出
//         if (i < 2) {
//             std::cout << "\nConfiguration " << i << " joint positions:" << std::endl;
//             for (pinocchio::JointIndex j = 1; j < model_upper.njoints; ++j) {
//                 const auto& pos = data_upper.oMi[j].translation();
//                 std::cout << "  " << model_upper.names[j] << ": " << pos.transpose() << std::endl;
//             }
//         }
//     }

//     std::cout << "\nAll joint positions saved to " << filepath + "all_joint_positions_with_upper_body.txt" << std::endl;
//     return 0;
// }

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <sstream>
#include <map>

int main() {
    const std::string filepath = "../Examples/Unitree/data/";
    const std::string urdf_filename_lower = "../Robots/Unitree/h1_2_12dof_floatingbase.urdf";
    const std::string urdf_filename_upper = "../Robots/Unitree/h1_2_27dof.urdf";

    // 1. 构建下半身机器人模型
    pinocchio::Model model_lower;
    try {
        pinocchio::urdf::buildModel(urdf_filename_lower, model_lower);
        std::cout << "Lower body model built successfully." << std::endl;
        std::cout << "Number of joints (nq): " << model_lower.nq << std::endl;
        std::cout << "Total joints (njoints): " << model_lower.njoints << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error building lower body model: " << e.what() << std::endl;
        return 1;
    }

    // 2. 构建上半身机器人模型
    pinocchio::Model model_upper;
    try {
        pinocchio::urdf::buildModel(urdf_filename_upper, model_upper);
        std::cout << "Upper body model built successfully." << std::endl;
        std::cout << "Number of joints (nq): " << model_upper.nq << std::endl;
        std::cout << "Total joints (njoints): " << model_upper.njoints << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error building upper body model: " << e.what() << std::endl;
        return 1;
    }

    // 3. 验证关节兼容性
    std::cout << "\nVerifying joint compatibility..." << std::endl;
    
    // 创建关节名称到索引的映射
    std::map<std::string, int> lower_joint_map;
    for (int i = 1; i < model_lower.njoints; ++i) { // 跳过世界坐标系
        lower_joint_map[model_lower.names[i]] = i;
    }

    std::map<std::string, int> upper_joint_map;
    for (int i = 1; i < model_upper.njoints; ++i) { // 跳过世界坐标系
        upper_joint_map[model_upper.names[i]] = i;
    }

    // 检查关键关节是否在两个模型中存在
    const std::vector<std::string> key_joints = {
        "left_hip_yaw_joint", "left_hip_pitch_joint", "left_hip_roll_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_yaw_joint", "right_hip_pitch_joint", "right_hip_roll_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"
    };

    for (const auto& joint : key_joints) {
        if (lower_joint_map.find(joint) == lower_joint_map.end()) {
            std::cerr << "Error: Joint '" << joint << "' missing in lower body model!" << std::endl;
            return 1;
        }
        if (upper_joint_map.find(joint) == upper_joint_map.end()) {
            std::cerr << "Error: Joint '" << joint << "' missing in upper body model!" << std::endl;
            return 1;
        }
    }
    std::cout << "All key joints present in both models." << std::endl;

    // 4. 读取下半身关节角度轨迹文件
    std::ifstream input(filepath + "full-trajectory-h1-forward.txt");
    if (!input.is_open()) {
        std::cerr << "Error opening trajectory file." << std::endl;
        return 1;
    }

    std::vector<Eigen::VectorXd> jointTrajectoriesLower;
    std::string line;
    while (std::getline(input, line)) {
        std::istringstream iss(line);
        Eigen::VectorXd q_lower(model_lower.nq);
        for (int i = 0; i < model_lower.nq; ++i) {
            iss >> q_lower[i];
        }
        jointTrajectoriesLower.push_back(q_lower);
    }
    std::cout << "\nRead " << jointTrajectoriesLower.size() << " lower body configurations." << std::endl;

    // 5. 准备输出文件
    std::ofstream output(filepath + "all_joint_positions_with_upper_body.txt");
    if (!output.is_open()) {
        std::cerr << "Error creating output file." << std::endl;
        return 1;
    }

    // 6. 创建数据对象
    pinocchio::Data data_lower(model_lower);
    pinocchio::Data data_upper(model_upper);

    // 7. 创建关节映射
    std::vector<int> joint_index_map(model_lower.njoints - 1, -1); // 跳过世界坐标系
    for (int i = 1; i < model_lower.njoints; ++i) {
        const std::string& name = model_lower.names[i];
        if (upper_joint_map.find(name) != upper_joint_map.end()) {
            joint_index_map[i - 1] = upper_joint_map[name];
        }
    }

    // 8. 确定浮动基座自由度
    const int floating_dofs = 6; // 根据URDF，下半身模型有6个浮动基座自由度

    // 9. 遍历所有时间步
    for (size_t i = 0; i < jointTrajectoriesLower.size(); ++i) {
        const auto& q_lower = jointTrajectoriesLower[i];
        
        // 创建完整的关节角度向量
        Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model_upper.nq);
        
        // 复制真实关节值（跳过浮动基座）
        for (int j = floating_dofs; j < model_lower.nq; j++) {
            const int joint_idx = j - floating_dofs;
            if (joint_index_map[joint_idx] != -1) {
                const int upper_idx = model_upper.joints[joint_index_map[joint_idx]].idx_q();
                q_full[upper_idx] = q_lower[j];
            }
        }

        // 计算正向运动学
        pinocchio::forwardKinematics(model_upper, data_upper, q_full);

        // 输出所有关节位置（从索引1开始，跳过世界坐标系）
        for (pinocchio::JointIndex j = 1; j < model_upper.njoints; ++j) {
            const auto& joint_pos = data_upper.oMi[j].translation();
            output << joint_pos[0] << " " << joint_pos[1] << " " << joint_pos[2] << " ";
        }
        output << "\n";

        // 调试输出
        if (i < 2) {
            std::cout << "\nConfiguration " << i << " joint positions:" << std::endl;
            for (pinocchio::JointIndex j = 1; j < model_upper.njoints; ++j) {
                const auto& pos = data_upper.oMi[j].translation();
                std::cout << "  " << model_upper.names[j] << ": " << pos.transpose() << std::endl;
            }
        }
    }

    std::cout << "\nAll joint positions saved to " << filepath + "all_joint_positions_with_upper_body.txt" << std::endl;
    return 0;
}
