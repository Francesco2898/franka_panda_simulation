#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace franka_example_controllers
{

class PinocchioKinematics
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr int kNumJoints = 7;

  using Vector7d  = Eigen::Matrix<double, kNumJoints, 1>;
  using Matrix6x7 = Eigen::Matrix<double, 6, kNumJoints>;

  struct Options
  {
    std::string arm_id{"fr3"};
    bool use_robot_hand{true};
    std::string ee_frame_override{""};
    std::string base_frame_override{""};
  };

  struct IkOptions
  {
    int max_iters{100};
    double eps{1e-4};
    double step_size{1.0};
    double damping{1e-4};
    double w_rot{0.2};
    bool clamp_to_limits{false};
  };

  PinocchioKinematics() = default;

  bool initFromURDFString(const std::string& urdf_xml, const std::string& ee_frame_name);
  bool initFromUrdfXml(const std::string& urdf_xml, const Options& opt);

  bool isInitialized() const { return initialized_; }

  const std::string& armId() const { return opt_.arm_id; }
  const std::string& eeFrame() const { return ee_frame_; }
  pinocchio::FrameIndex eeFrameId() const { return ee_frame_id_; }

  std::string modelSummary(std::size_t max_items = 200) const;
  void setExtraJointPositions(const std::unordered_map<std::string, double>& q_extra);

  pinocchio::SE3 fk(const Vector7d& q_arm);
  Matrix6x7 jacobian(const Vector7d& q_arm);

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

  pinocchio::FrameIndex frameIdByName(const std::string& name) const;
  bool FramePoseRt(const Vector7d& q_arm, pinocchio::FrameIndex frame_id, pinocchio::SE3& out);
  bool FramePoses4Rt(const Vector7d& q_arm, const std::array<pinocchio::FrameIndex, 4>& frame_ids, std::array<pinocchio::SE3, 4>& out);

private:
  std::string resolveEeFrameName(const Options& opt) const;
  bool resolveFrameIds();
  bool buildArmIndexMaps();
  void refreshExtraJointIndexCache();
  void ensureInitOrThrow(const char* func) const;

  void fillQFullFromArmInPlace(const Vector7d& q_arm, Eigen::VectorXd& q_full) const;
  void extractArmFromQFullInPlace(const Eigen::VectorXd& q_full, Vector7d& q_arm) const;
  void clampArmToLimitsInPlace(Vector7d& q_arm) const;
  void integrateArmInPlace(const Vector7d& q, const Vector7d& dq, double alpha, Vector7d& q_next) const;

private:
  Options opt_;
  bool initialized_{false};

  pinocchio::Model model_;
  pinocchio::Data data_;

  std::string ee_frame_;
  pinocchio::FrameIndex ee_frame_id_{0};

  Eigen::VectorXd q_full_neutral_;
  std::unordered_map<std::string, double> q_extra_map_;
  std::vector<std::string> arm_joint_names_;
  std::array<int, kNumJoints> arm_q_idx_{{0,0,0,0,0,0,0}};
  std::array<int, kNumJoints> arm_v_idx_{{0,0,0,0,0,0,0}};
  std::vector<std::pair<int, double>> extra_q_idx_and_pos_rt_;

  // Reusable buffers for 1 kHz calls (pre-allocated in init).
  Eigen::VectorXd q_full_rt_;
  Eigen::VectorXd q_full_work_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> J_full_;
};

}  // namespace franka_example_controllers