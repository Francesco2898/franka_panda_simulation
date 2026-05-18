#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>
#include <unordered_map>

#include <Eigen/Dense>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace franka_example_controllers
{

/**
 * @brief 运动学：FK / Jacobian / IK（数值迭代）
 *
 * 关键设计（本次修复）：
 * - 保留完整 URDF 模型（包含 gripper），因此 model_.nq 可能是 9
 * - 但对外接口仍然使用 7DOF 手臂 q（Vector7d）
 * - 内部用 q_full (size = model_.nq) 补齐 finger 等自由度，避免 Pinocchio 报 “expected 9, got 7”
 *
 * 这样你仍可对 fr3_hand_tcp / gripper center / finger link 等任何 frame 做 FK/J/IK。
 */
class PinocchioKinematics
{
public:
  static constexpr int kNumJoints = 7;

  using Vector7d  = Eigen::Matrix<double, kNumJoints, 1>;
  using Matrix6x7 = Eigen::Matrix<double, 6, kNumJoints>;

  struct Options
  {
    std::string arm_id{"fr3"};            // e.g. "fr3"
    bool use_robot_hand{true};            // true-> "<arm_id>_hand_tcp", false-> "<arm_id>_link8"
    std::string ee_frame_override{""};    // 若非空，强制用这个 frame 名
    std::string base_frame_override{""};  // 预留（一般不需要）
  };

  struct IkOptions
  {
    int max_iters{100};
    double eps{1e-4};          // twist error norm threshold
    double step_size{1.0};     // 0~1
    double damping{1e-4};      // DLS damping
    double w_rot{0.2};         /// rotation err weight
    bool clamp_to_limits{false};
  };

  PinocchioKinematics() = default;

  // 保留原接口：直接从 URDF XML + 末端帧名初始化
  bool initFromURDFString(const std::string& urdf_xml, const std::string& ee_frame_name);

  // 推荐接口：从 URDF XML + Options 初始化
  bool initFromUrdfXml(const std::string& urdf_xml, const Options& opt);

  bool isInitialized() const { return initialized_; }

  const std::string& armId() const { return opt_.arm_id; }
  const std::string& eeFrame() const { return ee_frame_; }
  pinocchio::FrameIndex eeFrameId() const { return ee_frame_id_; }

  // 打印模型摘要：nq/nv/joints/frames（用于 debug）
  std::string modelSummary(std::size_t max_items = 200) const;

  /**
   * @brief 设置“额外关节”（如 finger）的位置（可选）。
   *  - key: joint name (URDF joint 名)
   *  - value: position
   */
  void setExtraJointPositions(const std::unordered_map<std::string, double>& q_extra);

  /**
   * @brief FK：输入 7DOF 手臂 q，输出末端位姿（SE3）
   */
  pinocchio::SE3 fk(const Vector7d& q_arm);

  /**
   * @brief Jacobian：输入 7DOF 手臂 q，输出 6x7 Jacobian（LOCAL_WORLD_ALIGNED）
   */
  Matrix6x7 jacobian(const Vector7d& q_arm);

  /**
   * @brief IK：给定目标末端位姿，输出 7DOF 手臂 q_solution（数值迭代）
   */
  bool ik(
    const pinocchio::SE3& target,
    const Vector7d& q_init,
    Vector7d& q_sol,
    const IkOptions& ik_opt,
    double* final_err = nullptr);

  bool ik_position_only(
    const pinocchio::SE3& target,
    const Vector7d& q_init,
    Vector7d& q_sol,
    const IkOptions& ik_opt,
    double* final_err = nullptr);



  bool ik(
    const pinocchio::SE3& target,
    const Vector7d& q_init,
    Vector7d& q_sol,
    double* final_err = nullptr);




  void fk_rt(const Vector7d& q_arm, pinocchio::SE3& out);

private:
  std::string resolveEeFrameName(const Options& opt) const;
  bool resolveFrameIds();
  bool buildArmIndexMaps();                 // 建立 arm joints 在 q_full / v_full 的索引映射
  Eigen::VectorXd buildQFull(const Vector7d& q_arm) const;  // 由 7DOF arm q 构造 nq 维 q_full

  void ensureInitOrThrow(const char* func) const;

private:
  Options opt_;
  bool initialized_{false};

  pinocchio::Model model_;
  pinocchio::Data data_;

  std::string ee_frame_;
  pinocchio::FrameIndex ee_frame_id_{0};

  // 完整配置向量（size = model_.nq），以及“额外关节”（finger等）的默认值
  Eigen::VectorXd q_full_neutral_;
  std::unordered_map<std::string, double> q_extra_map_;

  // arm 7 joints 的名称与索引映射（映射到完整模型的 q/v 索引）
  std::vector<std::string> arm_joint_names_;
  std::array<int, kNumJoints> arm_q_idx_{};
  std::array<int, kNumJoints> arm_v_idx_{};

  // rt loop modification
  Eigen::VectorXd q_full_rt_;
  std::vector<std::pair<int, double>> extra_q_idx_and_pos_rt_; 



};

}  // namespace franka_example_controllers
