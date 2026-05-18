#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

class RlActionProcessor {
public:
    using Vector6d = Eigen::Matrix<double, 6, 1>;

public:
    RlActionProcessor() = default;
    ~RlActionProcessor() = default;

    // =========================
    // 输入：主循环直接写这些变量
    // =========================

    // RL policy 原始输出
    Vector6d policy_action = Vector6d::Zero();

    // 上一时刻 EMA 后动作
    Vector6d prev_used_action = Vector6d::Zero();

    // EMA 系数
    double ema_factor = 0.2;

    // 当前末端状态
    Eigen::Vector3d fingertip_pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond fingertip_quat = Eigen::Quaterniond::Identity();

    // Python中的 self.fixed_pos_action_frame
    Eigen::Vector3d fixed_pos_action_frame = Eigen::Vector3d::Zero();

    // 阈值 threhold
    // Eigen::Vector3d pos_threshold = Eigen::Vector3d(0.02, 0.02, 0.02);
    // Eigen::Vector3d rot_threshold = Eigen::Vector3d(0.097, 0.097, 0.097);    
    Eigen::Vector3d pos_threshold = Eigen::Vector3d(0.02, 0.02, 0.02);
    Eigen::Vector3d rot_threshold = Eigen::Vector3d(0.097, 0.097, 0.097);

    // 位置动作边界
    // Eigen::Vector3d pos_action_bounds_lower = Eigen::Vector3d(-0.05, -0.05, -0.00);
    // Eigen::Vector3d pos_action_bounds_upper = Eigen::Vector3d(0.05, 0.05, 0.05);
    Eigen::Vector3d pos_action_bounds_lower = Eigen::Vector3d(-0.0075, -0.0075, -0.005);//###-0.005 for press
    Eigen::Vector3d pos_action_bounds_upper = Eigen::Vector3d(0.0075, 0.0075, 0.05);
    

    // /// for generalization position 2
    // Eigen::Vector3d pos_action_bounds_lower = Eigen::Vector3d(-0.0075, -0.0075, -0.005);//###-0.005 for press
    // Eigen::Vector3d pos_action_bounds_upper = Eigen::Vector3d(0.0075, 0.0075, 0.05);

    /// for PPO only
    // 阈值
    // Eigen::Vector3d pos_threshold = Eigen::Vector3d(0.02, 0.02, 0.02);
    // Eigen::Vector3d rot_threshold = Eigen::Vector3d(0.097, 0.097, 0.097);
    // //// to generate gentle motion: threshold1:
    // Eigen::Vector3d pos_threshold = Eigen::Vector3d(0.01, 0.01, 0.02);
    // Eigen::Vector3d rot_threshold = Eigen::Vector3d(0.097, 0.097, 0.097);   
      
    
    // // //  //// to generate gentle motion: threshold3:
    // Eigen::Vector3d pos_threshold = Eigen::Vector3d(0.01, 0.01, 0.02);
    // Eigen::Vector3d rot_threshold = Eigen::Vector3d(0.097, 0.097, 0.06);       


    // // 位置动作边界
    // Eigen::Vector3d pos_action_bounds_lower = Eigen::Vector3d(-0.0075, -0.0075, -0.005);//###-0.005 for press
    // Eigen::Vector3d pos_action_bounds_upper = Eigen::Vector3d(0.0075, 0.0075, 0.05);



    // 是否单向旋转
    bool unidirectional_rot = false;

    // =========================
    // 输出：process() 后直接读取
    // =========================

    // EMA 后实际使用的动作
    Vector6d used_action = Vector6d::Zero();

    // post-process 之后的目标
    Eigen::Vector3d ctrl_target_fingertip_midpoint_pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond ctrl_target_fingertip_midpoint_quat = Eigen::Quaterniond::Identity();
    double ctrl_target_gripper_dof_pos = 0.0;

    double wrapToPi(double);
    bool reference_yaw_initialized_{false};
    double reference_yaw_=0;

public:
    // 主处理函数：直接读取 public 输入成员，并写入 public 输出成员
    void process();

private:
    static double clamp(double v, double lo, double hi);

    static Eigen::Quaterniond normalizedQuat(const Eigen::Quaterniond& q);

    static Eigen::Quaterniond quatFromAngleAxis(double angle, const Eigen::Vector3d& axis_unit);

    static Eigen::Quaterniond quatFromEulerXYZ(double roll, double pitch, double yaw);

    static Eigen::Vector3d eulerXYZFromQuat(const Eigen::Quaterniond& q);
};