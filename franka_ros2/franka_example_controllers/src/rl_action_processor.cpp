#include "franka_example_controllers/rl_action_processor.hpp"


#include <algorithm>
#include <cmath>
#include <iostream>
using namespace std;

constexpr double PI = 3.14159265358979323846;


double RlActionProcessor::clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
}

Eigen::Quaterniond RlActionProcessor::normalizedQuat(const Eigen::Quaterniond& q) {
    Eigen::Quaterniond qn = q;
    qn.normalize();
    return qn;
}

Eigen::Quaterniond RlActionProcessor::quatFromAngleAxis(double angle, const Eigen::Vector3d& axis_unit) {
    Eigen::AngleAxisd aa(angle, axis_unit.normalized());
    Eigen::Quaterniond q(aa);
    q.normalize();
    return q;
}

Eigen::Quaterniond RlActionProcessor::quatFromEulerXYZ(double roll, double pitch, double yaw) {
    const Eigen::AngleAxisd rx(roll,  Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd rz(yaw,   Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = Eigen::Quaterniond(rx * ry * rz);
    q.normalize();
    return q;
}

Eigen::Vector3d RlActionProcessor::eulerXYZFromQuat(const Eigen::Quaterniond& q_in) {
    Eigen::Quaterniond q = normalizedQuat(q_in);
    Eigen::Matrix3d R = q.toRotationMatrix();

    double r00 = R(0, 0);
    double r01 = R(0, 1);
    double r02 = R(0, 2);
    double r12 = R(1, 2);
    double r22 = R(2, 2);

    double pitch = std::asin(clamp(r02, -1.0, 1.0));
    double roll = 0.0;
    double yaw = 0.0;

    if (std::abs(std::cos(pitch)) > 1e-9) {
        roll = std::atan2(-r12, r22);
        yaw  = std::atan2(-r01, r00);
    } else {
        roll = 0.0;
        yaw  = std::atan2(R(1, 0), R(1, 1));
    }

    return Eigen::Vector3d(roll, pitch, yaw);
}

double RlActionProcessor::wrapToPi(double a) {
    while (a > PI) a -= 2.0 * PI;
    while (a < -PI) a += 2.0 * PI;
    return a;
}



void RlActionProcessor::process() {
    
    // initialization 
    if (!reference_yaw_initialized_) {
        Eigen::Vector3d init_euler = eulerXYZFromQuat(fingertip_quat);
        reference_yaw_ = init_euler[2];
        reference_yaw_initialized_ = true;
        std::cout<<"init_euler:"<<init_euler.transpose()<<std::endl;
    }    
        
    
    
    // ------------------------------------------------------------
    // 1) 对齐 Python: _pre_physics_step()
    // self.actions = ema * action + (1-ema) * self.actions
    // ------------------------------------------------------------
    used_action =
        ema_factor * policy_action
        + (1.0 - ema_factor) * prev_used_action;

    // ------------------------------------------------------------
    // 2) 对齐 Python: _apply_action()
    // pos_actions = self.actions[:, 0:3] * self.pos_threshold
    // ------------------------------------------------------------
    Eigen::Vector3d pos_actions =
        used_action.head<3>().cwiseProduct(pos_threshold);

    // ctrl_target_pos = current_pos + pos_actions
    Eigen::Vector3d ctrl_target_pos =
        fingertip_pos + pos_actions;

    // ------------------------------------------------------------
    // 3) 位置 clip，对齐 Python:
    // delta_pos = ctrl_target_pos - fixed_pos_action_frame
    // pos_error_clipped = clip(delta_pos, -lower, upper)
    // ctrl_target_pos = fixed_pos_action_frame + pos_error_clipped
    // ------------------------------------------------------------
    Eigen::Vector3d delta_pos =
        ctrl_target_pos - fixed_pos_action_frame;

    Eigen::Vector3d pos_error_clipped;
    for (int i = 0; i < 3; ++i) {
        pos_error_clipped[i] = clamp(
            delta_pos[i],
             pos_action_bounds_lower[i],
             pos_action_bounds_upper[i]
        );
    }

    ctrl_target_fingertip_midpoint_pos =
        fixed_pos_action_frame + pos_error_clipped;

    // ------------------------------------------------------------
    // 4) 旋转动作，对齐 Python:
    // rot_actions = self.actions[:, 3:6]
    // if unidirectional_rot:
    //     rot_actions[2] = -(rot_actions[2] + 1.0) * 0.5
    // rot_actions *= rot_threshold
    // ------------------------------------------------------------
    Eigen::Vector3d rot_actions = used_action.tail<3>();
    
    // std::cout<<"-------------- RL post procession-----"<<std::endl;
    // std::cout<<"rot_actions:"<<rot_actions.transpose()<<endl;
    // std::cout<<"fingertip_quat ="<<fingertip_quat.x()<<", "
    // <<fingertip_quat.y()<<", "
    // <<fingertip_quat.z()<<", "
    // <<fingertip_quat.w()
    // <<std::endl;    
    
    
    if (unidirectional_rot) {
        rot_actions[2] = -(rot_actions[2] + 1.0) * 0.5;
    }

    rot_actions = rot_actions.cwiseProduct(rot_threshold);

    // ------------------------------------------------------------
    // 5) axis-angle -> quaternion，对齐 Python
    // ------------------------------------------------------------
    double angle = rot_actions.norm();

    Eigen::Quaterniond rot_actions_quat = Eigen::Quaterniond::Identity();
    if (angle > 1e-6) {
        Eigen::Vector3d axis_unit = rot_actions / angle;
        rot_actions_quat = quatFromAngleAxis(angle, axis_unit);
    }

    // ------------------------------------------------------------
    // 6) 相对当前姿态叠加，对齐 Python:
    // ctrl_target_quat = quat_mul(rot_actions_quat, fingertip_quat)
    // ------------------------------------------------------------
    Eigen::Quaterniond ctrl_target_quat =
        normalizedQuat(rot_actions_quat * fingertip_quat);

    // ------------------------------------------------------------
    // 7) 强制 upright，对齐 Python:
    // roll = pi, pitch = 0, yaw 保留
    // ------------------------------------------------------------
    Eigen::Vector3d target_euler_xyz = eulerXYZFromQuat(ctrl_target_quat);

    double target_roll  = 3.14159;
    double target_pitch = 0.0;
    double raw_target_yaw   = target_euler_xyz[2];


    // 相对初始 yaw 的变化
    double delta_yaw = wrapToPi(raw_target_yaw - reference_yaw_);

    // 限制到 [-75°, 75°] for ppo_eal//60 for ppo 
    const double yaw_limit = 75.0 * PI / 180.0;
    delta_yaw = clamp(delta_yaw, -yaw_limit, yaw_limit);

    // 最终目标 yaw
    double target_yaw = wrapToPi(reference_yaw_ + delta_yaw);

    ctrl_target_fingertip_midpoint_quat =
        quatFromEulerXYZ(target_roll, target_pitch, target_yaw);



    // ctrl_target_fingertip_midpoint_quat =
    //     quatFromEulerXYZ(target_roll, target_pitch, target_yaw);

    
    // std::cout<<"ctrl_target_fingertip_midpoint_quat from rpy ="<<ctrl_target_fingertip_midpoint_quat.x()<<", "
    // <<ctrl_target_fingertip_midpoint_quat.y()<<", "
    // <<ctrl_target_fingertip_midpoint_quat.z()<<", "
    // <<ctrl_target_fingertip_midpoint_quat.w()
    // <<std::endl;     
    
    // std::cout<<"-------------- RL post procession finished -----"<<std::endl;


    // ------------------------------------------------------------
    // 8) 对齐 Python: gripper target 固定为 0
    // ------------------------------------------------------------
    ctrl_target_gripper_dof_pos = 0.0;
}