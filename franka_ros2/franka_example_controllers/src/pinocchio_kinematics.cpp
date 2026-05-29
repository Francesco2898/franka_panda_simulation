#include "franka_example_controllers/pinocchio_kinematics.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace franka_example_controllers
{

namespace
{
inline double clampScalar(double v, double lo, double hi)
{
  return std::min(std::max(v, lo), hi);
}

inline void computeError6DWorld(
  const pinocchio::SE3& current,
  const pinocchio::SE3& target,
  Eigen::Matrix<double, 6, 1>& e_out,
  double& pos_err_out,
  double& rot_err_out)
{
  const Eigen::Vector3d ep = target.translation() - current.translation();
  const Eigen::Matrix3d R_err = current.rotation().transpose() * target.rotation();
  const Eigen::Vector3d eo_local = pinocchio::log3(R_err);
  const Eigen::Vector3d eo_world = current.rotation() * eo_local;

  e_out.head<3>() = ep;
  e_out.tail<3>() = eo_world;
  pos_err_out = ep.norm();
  rot_err_out = eo_world.norm();
}
}

void PinocchioKinematics::ensureInitOrThrow(const char* func) const
{
  if (!initialized_)
  {
    std::ostringstream oss;
    oss << "PinocchioKinematics::" << func << " called before initialization";
    throw std::runtime_error(oss.str());
  }
}

std::string PinocchioKinematics::resolveEeFrameName(const Options& opt) const
{
  if (!opt.ee_frame_override.empty())
  {
    return opt.ee_frame_override;
  }
  return opt.use_robot_hand ? (opt.arm_id + std::string{"_hand_tcp"})
                            : (opt.arm_id + std::string{"_link8"});
}

bool PinocchioKinematics::resolveFrameIds()
{
  ee_frame_id_ = model_.getFrameId(ee_frame_);
  return ee_frame_id_ != pinocchio::FrameIndex(-1);
}

bool PinocchioKinematics::buildArmIndexMaps()
{
  arm_joint_names_.clear();
  arm_joint_names_.reserve(kNumJoints);

  for (int i = 1; i <= kNumJoints; ++i)
  {
    arm_joint_names_.push_back(opt_.arm_id + std::string{"_joint"} + std::to_string(i));
  }

  for (int i = 0; i < kNumJoints; ++i)
  {
    const pinocchio::JointIndex jid = model_.getJointId(arm_joint_names_[i]);
    if (jid == pinocchio::JointIndex(-1))
    {
      return false;
    }
    arm_q_idx_[i] = model_.joints[jid].idx_q();
    arm_v_idx_[i] = model_.joints[jid].idx_v();
  }

  return true;
}

void PinocchioKinematics::refreshExtraJointIndexCache()
{
  extra_q_idx_and_pos_rt_.clear();
  extra_q_idx_and_pos_rt_.reserve(q_extra_map_.size());

  for (const auto& kv : q_extra_map_)
  {
    const pinocchio::JointIndex jid = model_.getJointId(kv.first);
    if (jid == pinocchio::JointIndex(-1))
    {
      continue;
    }

    const int idx_q = model_.joints[jid].idx_q();
    if (idx_q >= 0 && idx_q < model_.nq)
    {
      extra_q_idx_and_pos_rt_.push_back({idx_q, kv.second});
    }
  }
}

void PinocchioKinematics::fillQFullFromArmInPlace(const Vector7d& q_arm, Eigen::VectorXd& q_full) const
{
  q_full = q_full_neutral_;
  for (int i = 0; i < kNumJoints; ++i)
  {
    q_full[arm_q_idx_[i]] = q_arm[i];
  }
  for (const auto& kv : extra_q_idx_and_pos_rt_)
  {
    q_full[kv.first] = kv.second;
  }
}

void PinocchioKinematics::extractArmFromQFullInPlace(const Eigen::VectorXd& q_full, Vector7d& q_arm) const
{
  for (int i = 0; i < kNumJoints; ++i)
  {
    q_arm[i] = q_full[arm_q_idx_[i]];
  }
}

void PinocchioKinematics::clampArmToLimitsInPlace(Vector7d& q_arm) const
{
  for (int i = 0; i < kNumJoints; ++i)
  {
    const int idx_q = arm_q_idx_[i];
    q_arm[i] = clampScalar(q_arm[i], model_.lowerPositionLimit[idx_q], model_.upperPositionLimit[idx_q]);
  }
}

void PinocchioKinematics::integrateArmInPlace(
  const Vector7d& q,
  const Vector7d& dq,
  double alpha,
  Vector7d& q_next) const
{
  q_next.noalias() = q + alpha * dq;
}

bool PinocchioKinematics::initFromURDFString(const std::string& urdf_xml, const std::string& ee_frame_name)
{
  Options opt;
  opt.ee_frame_override = ee_frame_name;
  return initFromUrdfXml(urdf_xml, opt);
}

bool PinocchioKinematics::initFromUrdfXml(const std::string& urdf_xml, const Options& opt)
{
  opt_ = opt;
  ee_frame_ = resolveEeFrameName(opt_);

  try
  {
    model_ = pinocchio::Model();
    pinocchio::urdf::buildModelFromXML(urdf_xml, model_);
    data_ = pinocchio::Data(model_);
  }
  catch (const std::exception&)
  {
    initialized_ = false;
    return false;
  }

  q_full_neutral_ = pinocchio::neutral(model_);

  if (!resolveFrameIds() || !buildArmIndexMaps())
  {
    initialized_ = false;
    return false;
  }

  refreshExtraJointIndexCache();

  q_full_rt_.resize(model_.nq);
  q_full_work_.resize(model_.nq);
  J_full_.resize(6, model_.nv);

  q_full_rt_ = q_full_neutral_;
  q_full_work_ = q_full_neutral_;
  J_full_.setZero();

  initialized_ = true;
  return true;
}

std::string PinocchioKinematics::modelSummary(std::size_t max_items) const
{
  if (!initialized_)
  {
    return "<PinocchioKinematics not initialized>";
  }

  std::ostringstream oss;
  oss << "Pinocchio Model Summary\n";
  oss << "  nq=" << model_.nq << " nv=" << model_.nv << "\n";
  oss << "  joints=" << model_.njoints << " frames=" << model_.nframes << "\n";
  oss << "  ee_frame='" << ee_frame_ << "' id=" << ee_frame_id_ << "\n";

  const std::size_t n_joint_print = std::min<std::size_t>(max_items, model_.names.size());
  oss << "  joint names (first " << n_joint_print << "):";
  for (std::size_t i = 0; i < n_joint_print; ++i)
  {
    oss << "\n    - " << model_.names[i];
  }

  const std::size_t n_frame_print = std::min<std::size_t>(max_items, model_.frames.size());
  oss << "\n  frame names (first " << n_frame_print << "):";
  for (std::size_t i = 0; i < n_frame_print; ++i)
  {
    oss << "\n    - " << model_.frames[i].name;
  }

  return oss.str();
}

void PinocchioKinematics::setExtraJointPositions(const std::unordered_map<std::string, double>& q_extra)
{
  q_extra_map_ = q_extra;
  if (initialized_)
  {
    refreshExtraJointIndexCache();
  }
}

void PinocchioKinematics::fk_rt(const Vector7d& q_arm, pinocchio::SE3& out)
{
  ensureInitOrThrow("fk_rt");

  fillQFullFromArmInPlace(q_arm, q_full_rt_);
  pinocchio::forwardKinematics(model_, data_, q_full_rt_);
  pinocchio::updateFramePlacements(model_, data_);
  out = data_.oMf[ee_frame_id_];
}

pinocchio::SE3 PinocchioKinematics::fk(const Vector7d& q_arm)
{
  pinocchio::SE3 out;
  fk_rt(q_arm, out);
  return out;
}

PinocchioKinematics::Matrix6x7 PinocchioKinematics::jacobian(const Vector7d& q_arm)
{
  ensureInitOrThrow("jacobian");

  fillQFullFromArmInPlace(q_arm, q_full_work_);
  pinocchio::forwardKinematics(model_, data_, q_full_work_);
  pinocchio::updateFramePlacements(model_, data_);

  J_full_.setZero();
  pinocchio::computeFrameJacobian(
    model_, data_, q_full_work_, ee_frame_id_,
    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_full_);

  Matrix6x7 J_arm;
  for (int i = 0; i < kNumJoints; ++i)
  {
    J_arm.col(i) = J_full_.col(arm_v_idx_[i]);
  }
  return J_arm;
}

bool PinocchioKinematics::ik(
  const pinocchio::SE3& target,
  const Vector7d& q_init,
  Vector7d& q_sol,
  const IkOptions& ik_opt,
  double* final_err)
{
  ensureInitOrThrow("ik");

  Vector7d q = q_init;
  Vector7d q_candidate = q_init;
  double last_err = -1.0;

  Eigen::Matrix<double, 6, 6> W = Eigen::Matrix<double, 6, 6>::Identity();
  W.bottomRightCorner<3, 3>() *= ik_opt.w_rot;

  for (int it = 0; it < ik_opt.max_iters; ++it)
  {
    fillQFullFromArmInPlace(q, q_full_work_);
    pinocchio::forwardKinematics(model_, data_, q_full_work_);
    pinocchio::updateFramePlacements(model_, data_);

    const pinocchio::SE3& current = data_.oMf[ee_frame_id_];

    Eigen::Matrix<double, 6, 1> e;
    double pos_err = 0.0;
    double rot_err = 0.0;
    computeError6DWorld(current, target, e, pos_err, rot_err);

    if (!e.allFinite())
    {
      if (final_err) *final_err = last_err;
      q_sol = q;
      return false;
    }

    const Eigen::Matrix<double, 6, 1> ew = W * e;
    const double err = ew.norm();
    last_err = err;
    if (final_err) *final_err = err;

    if (err < ik_opt.eps)
    {
      q_sol = q;
      return true;
    }

    J_full_.setZero();
    pinocchio::computeFrameJacobian(
      model_, data_, q_full_work_, ee_frame_id_,
      pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_full_);

    Matrix6x7 J_arm;
    for (int i = 0; i < kNumJoints; ++i)
    {
      J_arm.col(i) = J_full_.col(arm_v_idx_[i]);
    }

    const Matrix6x7 Jw = W * J_arm;
    Eigen::Matrix<double, 6, 6> JJt = Jw * Jw.transpose();
    const double lambda = ik_opt.damping;
    JJt.diagonal().array() += lambda * lambda;

    const Eigen::Matrix<double, 6, 1> v = JJt.ldlt().solve(ew);
    Vector7d dq = Jw.transpose() * v;
    if (!dq.allFinite())
    {
      if (final_err) *final_err = last_err;
      q_sol = q;
      return false;
    }

    const double err0 = err;
    double alpha = ik_opt.step_size;
    double err_candidate = err0;
    bool accepted = false;

    for (int ls = 0; ls < 10; ++ls)
    {
      integrateArmInPlace(q, dq, alpha, q_candidate);
      if (ik_opt.clamp_to_limits)
      {
        clampArmToLimitsInPlace(q_candidate);
      }

      fillQFullFromArmInPlace(q_candidate, q_full_work_);
      pinocchio::forwardKinematics(model_, data_, q_full_work_);
      pinocchio::updateFramePlacements(model_, data_);

      Eigen::Matrix<double, 6, 1> e2;
      double pos_err2 = 0.0;
      double rot_err2 = 0.0;
      computeError6DWorld(data_.oMf[ee_frame_id_], target, e2, pos_err2, rot_err2);
      err_candidate = (W * e2).norm();

      if (std::isfinite(err_candidate) && err_candidate < err0)
      {
        accepted = true;
        break;
      }
      alpha *= 0.5;
    }

    if (!accepted)
    {
      break;
    }

    q = q_candidate;
  }

  if (ik_opt.clamp_to_limits)
  {
    clampArmToLimitsInPlace(q);
  }

  if (final_err) *final_err = last_err;
  q_sol = q;
  return false;
}

bool PinocchioKinematics::ik_position_only(
  const pinocchio::SE3& target,
  const Vector7d& q_init,
  Vector7d& q_sol,
  const IkOptions& ik_opt,
  double* final_err)
{
  ensureInitOrThrow("ik_position_only");

  Vector7d q = q_init;
  Vector7d q_candidate = q_init;
  double last_err = -1.0;

  fillQFullFromArmInPlace(q_init, q_full_work_);
  pinocchio::forwardKinematics(model_, data_, q_full_work_);
  pinocchio::updateFramePlacements(model_, data_);

  pinocchio::SE3 target_locked = target;
  target_locked.rotation() = data_.oMf[ee_frame_id_].rotation();

  for (int it = 0; it < ik_opt.max_iters; ++it)
  {
    fillQFullFromArmInPlace(q, q_full_work_);
    pinocchio::forwardKinematics(model_, data_, q_full_work_);
    pinocchio::updateFramePlacements(model_, data_);

    const Eigen::Vector3d ep = target_locked.translation() - data_.oMf[ee_frame_id_].translation();
    if (!ep.allFinite())
    {
      if (final_err) *final_err = last_err;
      q_sol = q;
      return false;
    }

    const double err = ep.norm();
    last_err = err;
    if (final_err) *final_err = err;

    if (err < ik_opt.eps)
    {
      q_sol = q;
      return true;
    }

    J_full_.setZero();
    pinocchio::computeFrameJacobian(
      model_, data_, q_full_work_, ee_frame_id_,
      pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_full_);

    Eigen::Matrix<double, 3, 7> Jp;
    for (int i = 0; i < kNumJoints; ++i)
    {
      Jp.col(i) = J_full_.block<3, 1>(0, arm_v_idx_[i]);
    }

    Eigen::Matrix3d JJt = Jp * Jp.transpose();
    const double lambda = ik_opt.damping;
    JJt.diagonal().array() += lambda * lambda;

    const Eigen::Vector3d v = JJt.ldlt().solve(ep);
    Vector7d dq = Jp.transpose() * v;
    if (!dq.allFinite())
    {
      if (final_err) *final_err = last_err;
      q_sol = q;
      return false;
    }

    const double err0 = err;
    double alpha = ik_opt.step_size;
    double err_candidate = err0;
    bool accepted = false;

    for (int ls = 0; ls < 10; ++ls)
    {
      integrateArmInPlace(q, dq, alpha, q_candidate);
      if (ik_opt.clamp_to_limits)
      {
        clampArmToLimitsInPlace(q_candidate);
      }

      fillQFullFromArmInPlace(q_candidate, q_full_work_);
      pinocchio::forwardKinematics(model_, data_, q_full_work_);
      pinocchio::updateFramePlacements(model_, data_);

      const Eigen::Vector3d ep2 = target_locked.translation() - data_.oMf[ee_frame_id_].translation();
      err_candidate = ep2.norm();
      if (std::isfinite(err_candidate) && err_candidate < err0)
      {
        accepted = true;
        break;
      }
      alpha *= 0.5;
    }

    if (!accepted)
    {
      break;
    }

    q = q_candidate;
  }

  if (ik_opt.clamp_to_limits)
  {
    clampArmToLimitsInPlace(q);
  }

  if (final_err) *final_err = last_err;
  q_sol = q;
  return false;
}

bool PinocchioKinematics::ik(
  const pinocchio::SE3& target,
  const Vector7d& q_init,
  Vector7d& q_sol,
  double* final_err)
{
  IkOptions opt;
  return ik(target, q_init, q_sol, opt, final_err);
}

// ======================================================================================================================================================
// Additional functions to compute the probe points
pinocchio::FrameIndex PinocchioKinematics::frameIdByName(
  const std::string& frame_name) const
{
  ensureInitOrThrow("frameIdByName");

  for (pinocchio::FrameIndex i = 0;
       i < static_cast<pinocchio::FrameIndex>(model_.frames.size());
       ++i)
  {
    if (model_.frames[i].name == frame_name)
    {
      return i;
    }
  }

  return pinocchio::FrameIndex(-1);
}

bool PinocchioKinematics::FramePoseRt(
  const Vector7d& q_arm,
  pinocchio::FrameIndex frame_id,
  pinocchio::SE3& out)
{
  ensureInitOrThrow("framePoseRt");

  if (frame_id == pinocchio::FrameIndex(-1) ||
      frame_id >= static_cast<pinocchio::FrameIndex>(model_.frames.size()))
  {
    return false;
  }

  fillQFullFromArmInPlace(q_arm, q_full_rt_);

  pinocchio::forwardKinematics(model_, data_, q_full_rt_);
  pinocchio::updateFramePlacements(model_, data_);

  out = data_.oMf[frame_id];

  return true;
}

bool PinocchioKinematics::FramePoses4Rt(
  const Vector7d& q_arm,
  const std::array<pinocchio::FrameIndex, 4>& frame_ids,
  std::array<pinocchio::SE3, 4>& out)
{
  ensureInitOrThrow("framePoses4Rt");

  for (const auto& id : frame_ids)
  {
    if (id == pinocchio::FrameIndex(-1) ||
        id >= static_cast<pinocchio::FrameIndex>(model_.frames.size()))
    {
      return false;
    }
  }

  fillQFullFromArmInPlace(q_arm, q_full_rt_);

  pinocchio::forwardKinematics(model_, data_, q_full_rt_);
  pinocchio::updateFramePlacements(model_, data_);

  for (size_t i = 0; i < 4; ++i)
  {
    out[i] = data_.oMf[frame_ids[i]];
  }

  return true;
}

}  // namespace franka_example_controllers