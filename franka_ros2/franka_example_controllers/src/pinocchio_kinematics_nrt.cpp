#include "franka_example_controllers/pinocchio_kinematics.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/spatial/explog.hpp>   // log6 / Jlog6
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Cholesky>

#include <cmath>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace franka_example_controllers
{

namespace
{
inline double clamp(double v, double lo, double hi)
{
  return std::min(std::max(v, lo), hi);
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
  // Default behavior: use hand tcp or link8
  if (opt.use_robot_hand)
  {
    return opt.arm_id + std::string{"_hand_tcp"};
  }
  return opt.arm_id + std::string{"_link8"};
}

bool PinocchioKinematics::resolveFrameIds()
{
  ee_frame_id_ = model_.getFrameId(ee_frame_);
  if (ee_frame_id_ == pinocchio::FrameIndex(-1))
  {
    std::cerr << "[PinocchioKinematics] Cannot find frame: " << ee_frame_ << std::endl;
    return false;
  }
  return true;
}

bool PinocchioKinematics::buildArmIndexMaps()
{
  // Expected 7 joint names for the arm.
  arm_joint_names_.clear();
  for (int i = 1; i <= kNumJoints; ++i)
  {
    arm_joint_names_.push_back(opt_.arm_id + std::string{"_joint"} + std::to_string(i));
  }

  for (int i = 0; i < kNumJoints; ++i)
  {
    const std::string& jn = arm_joint_names_[i];
    pinocchio::JointIndex jid = model_.getJointId(jn);
    if (jid == pinocchio::JointIndex(-1))
    {
      std::cerr << "[PinocchioKinematics] Cannot find joint: " << jn << std::endl;
      return false;
    }

    // For 1DoF revolute joints in Franka, nq/nv = 1 per joint.
    arm_q_idx_[i] = model_.joints[jid].idx_q();
    arm_v_idx_[i] = model_.joints[jid].idx_v();
  }

  return true;
}

Eigen::VectorXd PinocchioKinematics::buildQFull(const Vector7d& q_arm) const
{
  Eigen::VectorXd q_full = q_full_neutral_;

  // Fill arm joints
  for (int i = 0; i < kNumJoints; ++i)
  {
    const int idx = arm_q_idx_[i];
    if (idx >= 0 && idx < q_full.size())
    {
      q_full[idx] = q_arm[i];
    }
  }

  // Fill extra joints (e.g., fingers)
  for (const auto& kv : q_extra_map_)
  {
    const std::string& joint_name = kv.first;
    const double pos = kv.second;

    pinocchio::JointIndex jid = model_.getJointId(joint_name);
    if (jid == pinocchio::JointIndex(-1))
    {
      continue;
    }
    const int idx_q = model_.joints[jid].idx_q();
    if (idx_q >= 0 && idx_q < q_full.size())
    {
      q_full[idx_q] = pos;
    }
  }

  return q_full;
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
  catch (const std::exception& e)
  {
    std::cerr << "[PinocchioKinematics] buildModelFromXML failed: " << e.what() << std::endl;
    initialized_ = false;
    return false;
  }

  q_full_neutral_ = pinocchio::neutral(model_);

  q_full_rt_ = q_full_neutral_;

  if (!resolveFrameIds())
  {
    initialized_ = false;
    return false;
  }

  if (!buildArmIndexMaps())
  {
    initialized_ = false;
    return false;
  }


  extra_q_idx_and_pos_rt_.clear();
  for (const auto& kv : q_extra_map_)
  {
    const std::string& joint_name = kv.first;
    const double pos = kv.second;

    pinocchio::JointIndex jid = model_.getJointId(joint_name);
    if (jid == pinocchio::JointIndex(-1))
    {
      continue;
    }

    const int idx_q = model_.joints[jid].idx_q();
    if (idx_q >= 0 && idx_q < model_.nq)
    {
      extra_q_idx_and_pos_rt_.push_back({idx_q, pos});
    }
  }







  initialized_ = true;

  std::cout << "===== Pinocchio Joints =====" << std::endl;
  for (pinocchio::JointIndex jid = 1; jid < model_.joints.size(); ++jid)
  {
    const auto& joint = model_.joints[jid];
    std::cout
      << "jid = " << jid
      << ", name = " << model_.names[jid]
      << ", nq = " << joint.nq()
      << ", nv = " << joint.nv()
      << ", idx_q = " << joint.idx_q()
      << ", idx_v = " << joint.idx_v()
      << std::endl;
  }

  std::cout << "===== Pinocchio Frames =====" << std::endl;
  for (pinocchio::FrameIndex fid = 0; fid < model_.frames.size(); ++fid)
  {
    const auto& frame = model_.frames[fid];
    std::cout
      << "fid = " << fid
      << ", name = " << frame.name
      << ", parent joint = " << frame.parent
      << " (" << model_.names[frame.parent] << ")"
      << ", type = ";

    switch (frame.type)
    {
      case pinocchio::FrameType::OP_FRAME:   std::cout << "OP_FRAME"; break;
      case pinocchio::FrameType::JOINT:      std::cout << "JOINT"; break;
      case pinocchio::FrameType::FIXED_JOINT:std::cout << "FIXED_JOINT"; break;
      case pinocchio::FrameType::BODY:       std::cout << "BODY"; break;
      case pinocchio::FrameType::SENSOR:     std::cout << "SENSOR"; break;
      default:                               std::cout << "UNKNOWN"; break;
    }



    std::cout << std::endl;
  }


  auto fid = model_.getFrameId(ee_frame_);
  if (fid == pinocchio::FrameIndex(-1))
  {
    std::cerr << "[ERROR] EE frame not found: " << ee_frame_ << std::endl;
  }
  else
  {
    const auto& frame = model_.frames[fid];
    std::cout
      << "[OK] EE frame found: " << ee_frame_
      << ", fid = " << fid
      << ", parent joint = " << frame.parent
      << " (" << model_.names[frame.parent] << ")"
      << std::endl;
  }


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
  oss << "  joint names (first " << n_joint_print << "):\n";
  for (std::size_t i = 0; i < n_joint_print; ++i)
  {
    oss << "    - " << model_.names[i] << "\n";
  }

  const std::size_t n_frame_print = std::min<std::size_t>(max_items, model_.frames.size());
  oss << "  frame names (first " << n_frame_print << "):\n";
  for (std::size_t i = 0; i < n_frame_print; ++i)
  {
    oss << "    - " << model_.frames[i].name << "\n";
  }

  return oss.str();
}

void PinocchioKinematics::setExtraJointPositions(const std::unordered_map<std::string, double>& q_extra)
{
  q_extra_map_ = q_extra;
}



void PinocchioKinematics::fk_rt(const Vector7d& q_arm, pinocchio::SE3& out)
{
  ensureInitOrThrow("fk_rt");

  // reset from neutral
  q_full_rt_ = q_full_neutral_;

  // fill arm joints
  for (int i = 0; i < kNumJoints; ++i)
  {
    const int idx = arm_q_idx_[i];
    if (idx >= 0 && idx < q_full_rt_.size())
    {
      q_full_rt_[idx] = q_arm[i];
    }
  }

  // fill precomputed extra joints
  for (const auto& kv : extra_q_idx_and_pos_rt_)
  {
    q_full_rt_[kv.first] = kv.second;
  }

  pinocchio::forwardKinematics(model_, data_, q_full_rt_);
  pinocchio::updateFramePlacements(model_, data_);

  out = data_.oMf[ee_frame_id_];
}


pinocchio::SE3 PinocchioKinematics::fk(const Vector7d& q_arm)
{
  // ensureInitOrThrow("fk");

  // const Eigen::VectorXd q_full = buildQFull(q_arm);
  // pinocchio::forwardKinematics(model_, data_, q_full);
  // pinocchio::updateFramePlacements(model_, data_);
  // return data_.oMf[ee_frame_id_];
  
  /// rt -friendly loop ////
  pinocchio::SE3 out;
  fk_rt(q_arm, out);
  return out;



}




PinocchioKinematics::Matrix6x7 PinocchioKinematics::jacobian(const Vector7d& q_arm)
{
  ensureInitOrThrow("jacobian");

  const Eigen::VectorXd q_full = buildQFull(q_arm);

  // Ensure placements are updated
  pinocchio::forwardKinematics(model_, data_, q_full);
  // pinocchio::updateFramePlacements(model_, data_);

  Eigen::Matrix<double, 6, Eigen::Dynamic> J_full(6, model_.nv);
  J_full.setZero();

  pinocchio::computeFrameJacobian(
    model_, data_, q_full, ee_frame_id_,
    pinocchio::ReferenceFrame::LOCAL, J_full);

  Matrix6x7 J_arm;
  for (int i = 0; i < kNumJoints; ++i)
  {
    J_arm.col(i) = J_full.col(arm_v_idx_[i]);
  }
  return J_arm;
}



bool PinocchioKinematics::ik(
  const pinocchio::SE3& target,     // target pose in WORLD: position + orientation
  const Vector7d& q_init,
  Vector7d& q_sol,
  const IkOptions& ik_opt,
  double* final_err)
{
  ensureInitOrThrow("ik");

  // ---------------- Debug header ----------------
  // std::cerr << "[PINOCCHIO_IK6D_ENTER] ee_frame='" << ee_frame_ << "' id=" << ee_frame_id_
  //           << " nq=" << model_.nq << " nv=" << model_.nv
  //           << " q_init=[" << q_init.transpose() << "]"
  //           << " opt{max_iters=" << ik_opt.max_iters
  //           << ", eps=" << ik_opt.eps
  //           << ", step=" << ik_opt.step_size
  //           << ", damp=" << ik_opt.damping
  //           << ", clamp=" << (ik_opt.clamp_to_limits ? "true" : "false")
  //           << ", w_rot=" << ik_opt.w_rot
  //           << "}"
  //           << std::endl;

  // --------------- Init ---------------
  Vector7d q = q_init;
  double last_err = -1.0;

  // Weighting: position [m], rotation [rad]
  Eigen::Matrix<double,6,6> W = Eigen::Matrix<double,6,6>::Identity();
  W.bottomRightCorner<3,3>() *= ik_opt.w_rot;  // e.g. w_rot=0.5 (you decide)

  auto computeError6D_World = [&](const pinocchio::SE3& cur,
                                 const pinocchio::SE3& des,
                                 Eigen::Matrix<double,6,1>& e_out,
                                 double& pos_err_out,
                                 double& rot_err_out)
  {
    const Eigen::Vector3d p_cur = cur.translation();
    const Eigen::Matrix3d R_cur = cur.rotation();

    const Eigen::Vector3d p_des = des.translation();
    const Eigen::Matrix3d R_des = des.rotation();

    // Position error in WORLD
    const Eigen::Vector3d ep = p_des - p_cur;

    // Orientation error:
    // R_err maps vectors from desired to current in current coordinates? We want "current -> desired":
    // Use R_err = R_cur^T * R_des  (rotation that brings current to desired, expressed in current frame)
    const Eigen::Matrix3d R_err = R_cur.transpose() * R_des;

    // so3 log gives a 3D rotation vector (in current/local coordinates)
    const Eigen::Vector3d eo_local = pinocchio::log3(R_err);

    // Convert to WORLD coordinates to match LOCAL_WORLD_ALIGNED Jacobian angular part (world-aligned)
    const Eigen::Vector3d eo_world = R_cur * eo_local;

    e_out.head<3>() = ep;
    e_out.tail<3>() = eo_world;

    pos_err_out = ep.norm();
    rot_err_out = eo_world.norm();
  };

  for (int it = 0; it < ik_opt.max_iters; ++it)
  {
    // Build full q (nq=9) from arm q (7)
    const Eigen::VectorXd q_full = buildQFull(q);
    if (q_full.size() != model_.nq)
    {
      // std::cerr << "[PINOCCHIO_IK_EARLY_FAIL] reason=BAD_QFULL_SIZE expected=" << model_.nq
      //           << " got=" << q_full.size() << std::endl;
      if (final_err) *final_err = last_err;
      q_sol = q;
      return false;
    }

    // FK
    pinocchio::forwardKinematics(model_, data_, q_full);
    pinocchio::updateFramePlacements(model_, data_);
    const pinocchio::SE3& current = data_.oMf[ee_frame_id_];

    // 6D error (WORLD expression)
    Eigen::Matrix<double,6,1> e;
    double pos_err = 0.0, rot_err = 0.0;
    computeError6D_World(current, target, e, pos_err, rot_err);

    if (!e.allFinite())
    {
      // std::cerr << "[PINOCCHIO_IK_EARLY_FAIL] reason=E_NOT_FINITE it=" << it
      //           << " cur.t=[" << current.translation().transpose() << "]"
      //           << " tgt.t=[" << target.translation().transpose() << "]"
      //           << std::endl;
      if (final_err) *final_err = last_err;
      q_sol = q;
      return false;
    }

    // Weighted norm as objective
    const Eigen::Matrix<double,6,1> ew = W * e;
    const double err = ew.norm();
    last_err = err;
    if (final_err) *final_err = err;

    // if (it == 0 || (it % 10) == 0)
    // {
    //   std::cerr << std::fixed << std::setprecision(6)
    //             << "[PINOCCHIO_IK6D_ITER] it=" << it
    //             << " err=" << err
    //             << " pos_err=" << pos_err
    //             << " rot_err=" << rot_err
    //             << " e=[" << e.transpose() << "]"
    //             << std::endl;
    // }

    if (err < ik_opt.eps)
    {
      q_sol = q;
      return true;
    }

    // Jacobian in LOCAL_WORLD_ALIGNED (world-aligned linear/angular parts)
    Eigen::Matrix<double, 6, Eigen::Dynamic> J_full(6, model_.nv);
    J_full.setZero();
    pinocchio::computeFrameJacobian(
      model_, data_, q_full, ee_frame_id_,
      pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_full);

    // Extract 6x7 arm Jacobian
    Matrix6x7 J_arm;
    for (int i = 0; i < kNumJoints; ++i)
      J_arm.col(i) = J_full.col(arm_v_idx_[i]);

    // Weighted DLS: dq = Jw^T (Jw Jw^T + lambda^2 I)^-1 ew
    const Eigen::Matrix<double,6,7> Jw = W * J_arm;

    Eigen::Matrix<double,6,6> JJt = Jw * Jw.transpose();
    const double lambda = ik_opt.damping;
    JJt.diagonal().array() += lambda * lambda;

    const Eigen::Matrix<double,6,1> v = JJt.ldlt().solve(ew);
    Vector7d dq = Jw.transpose() * v;

    if (!dq.allFinite())
    {
      // std::cerr << "[PINOCCHIO_IK_EARLY_FAIL] reason=NON_FINITE_DQ it=" << it
      //           << " dq=[" << dq.transpose() << "]" << std::endl;
      if (final_err) *final_err = last_err;
      q_sol = q;
      return false;
    }

    // ---------------- Line-search (monotonic decrease of weighted error) ----------------
    double alpha = ik_opt.step_size;
    const double err0 = err;
    Vector7d q_candidate = q;
    double err_candidate = err0;

    // Precompute full state and full velocity
    const Eigen::VectorXd q_full0 = buildQFull(q);
    Eigen::VectorXd dq_full = Eigen::VectorXd::Zero(model_.nv);
    for (int i = 0; i < kNumJoints; ++i)
      dq_full[arm_v_idx_[i]] = dq[i];

    for (int ls = 0; ls < 10; ++ls)
    {
      const Eigen::VectorXd q_full_cand_full = pinocchio::integrate(model_, q_full0, alpha * dq_full);

      Vector7d q_arm_cand;
      for (int i = 0; i < kNumJoints; ++i)
        q_arm_cand[i] = q_full_cand_full[arm_q_idx_[i]];

      const Eigen::VectorXd q_full_cand = buildQFull(q_arm_cand);
      pinocchio::forwardKinematics(model_, data_, q_full_cand);
      pinocchio::updateFramePlacements(model_, data_);
      const pinocchio::SE3& cur2 = data_.oMf[ee_frame_id_];

      Eigen::Matrix<double,6,1> e2;
      double pos_err2 = 0.0, rot_err2 = 0.0;
      computeError6D_World(cur2, target, e2, pos_err2, rot_err2);
      const Eigen::Matrix<double,6,1> ew2 = W * e2;
      err_candidate = ew2.norm();

      // if (it == 0)
      // {
      //   std::cerr << "[LS] ls=" << ls
      //             << " alpha=" << alpha
      //             << " err_candidate=" << err_candidate
      //             << " pos_err2=" << pos_err2
      //             << " rot_err2=" << rot_err2
      //             << std::endl;
      // }

      if (std::isfinite(err_candidate) && err_candidate < err0)
      {
        q_candidate = q_arm_cand;
        break;
      }

      alpha *= 0.5;
    }

    if (!(std::isfinite(err_candidate) && err_candidate < err0))
    {
      // std::cerr << "[PINOCCHIO_IK6D_FAIL] line-search cannot reduce error, it=" << it
      //           << " err0=" << err0 << " err_candidate=" << err_candidate
      //           << std::endl;
      break;
    }

    q = q_candidate;

    // Optional clamp to limits
    if (ik_opt.clamp_to_limits)
    {
      Eigen::VectorXd q_fullx = buildQFull(q);
      for (int i = 0; i < kNumJoints; ++i)
      {
        const int idx_q = arm_q_idx_[i];
        q_fullx[idx_q] = clamp(q_fullx[idx_q],
                               model_.lowerPositionLimit[idx_q],
                               model_.upperPositionLimit[idx_q]);
      }
      for (int i = 0; i < kNumJoints; ++i)
        q[i] = q_fullx[arm_q_idx_[i]];
    }
  }

  if (final_err) *final_err = last_err;
  q_sol = q;
  return false;
}


// ### position only-function####
bool PinocchioKinematics::ik_position_only(
  const pinocchio::SE3& target,
  const Vector7d& q_init,
  Vector7d& q_sol,
  const IkOptions& ik_opt,
  double* final_err)
{
  ensureInitOrThrow("ik");

  // --- Debug header (always printed) ---
  // std::cerr << "[PINOCCHIO_IK_ENTER] ee_frame='" << ee_frame_ << "' id=" << ee_frame_id_
  //           << " nq=" << model_.nq << " nv=" << model_.nv
  //           << " q_init=[" << q_init.transpose() << "]"
  //           << " opt{max_iters=" << ik_opt.max_iters
  //           << ", eps=" << ik_opt.eps
  //           << ", step=" << ik_opt.step_size
  //           << ", damp=" << ik_opt.damping
  //           << ", clamp=" << (ik_opt.clamp_to_limits ? "true" : "false")
  //           << "}"
  //           << std::endl;

  // =======================
  // 1) Lock EE orientation to initial pose (only for target display / consistency)
  //    IMPORTANT: orientation is NOT optimized; position-only IK.
  // =======================
  Vector7d q = q_init;

  const Eigen::VectorXd q_full_init = buildQFull(q_init);
  pinocchio::forwardKinematics(model_, data_, q_full_init);
  pinocchio::updateFramePlacements(model_, data_);
  const pinocchio::SE3 current0 = data_.oMf[ee_frame_id_];

  pinocchio::SE3 target_locked = target;
  target_locked.rotation() = current0.rotation();

  // // Print rotations once
  // std::cerr << "[IK] initial ee rotation:\n" << current0.rotation() << std::endl;
  // std::cerr << "[IK] target  ee rotation (input):\n" << target.rotation() << std::endl;
  // std::cerr << "[IK] target  ee rotation (locked):\n" << target_locked.rotation() << std::endl;

  // Optional mild posture regularization (DISABLED by default here)
  // const double k_posture = 0.0;

  double last_err = -1.0;

  for (int it = 0; it < ik_opt.max_iters; ++it)
  {
    // std::cerr << "[IK-LOOP] it=" << it << std::endl;

    const Eigen::VectorXd q_full = buildQFull(q);

    if (q_full.size() != model_.nq)
    {
      // std::cerr << "[PINOCCHIO_IK_EARLY_FAIL] reason=BAD_QFULL_SIZE expected=" << model_.nq
      //           << " got=" << q_full.size() << std::endl;
      if (final_err) *final_err = last_err;
      q_sol = q;
      return false;
    }

    // --- FK ---
    pinocchio::forwardKinematics(model_, data_, q_full);
    pinocchio::updateFramePlacements(model_, data_);
    const pinocchio::SE3& current = data_.oMf[ee_frame_id_];

    // =======================
    // 2) Position-only error in WORLD coordinates
    //    ep = p_target - p_current
    // =======================
    const Eigen::Vector3d ep = target_locked.translation() - current.translation();

    if (!ep.allFinite())
    {
      // std::cerr << "[PINOCCHIO_IK_EARLY_FAIL] reason=EP_NOT_FINITE"
      //           << " cur.t=[" << current.translation().transpose() << "]"
      //           << " tgt.t=[" << target_locked.translation().transpose() << "]"
      //           << std::endl;
      if (final_err) *final_err = last_err;
      q_sol = q;
      return false;
    }

    const double err = ep.norm();
    last_err = err;
    if (final_err) *final_err = err;

    // if (it == 0 || (it % 10) == 0)
    // {
    //   std::cerr << std::fixed << std::setprecision(6)
    //             << "[PINOCCHIO_IK_ITER] it=" << it
    //             << " pos_err=" << err
    //             << " ep=[" << ep.transpose() << "]"
    //             << " cur.t=[" << current.translation().transpose() << "]"
    //             << " tgt.t=[" << target_locked.translation().transpose() << "]"
    //             << std::endl;
    // }

    if (err < ik_opt.eps)
    {
      q_sol = q;
      return true;
    }

    // =======================
    // 3) Position Jacobian in WORLD coordinates
    //    Use LOCAL_WORLD_ALIGNED so linear part is expressed in WORLD.
    // =======================
    Eigen::Matrix<double, 6, Eigen::Dynamic> J_full(6, model_.nv);
    J_full.setZero();
    pinocchio::computeFrameJacobian(
      model_, data_, q_full, ee_frame_id_,
      pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_full);

    // Extract 3x7 arm position Jacobian
    Eigen::Matrix<double, 3, 7> Jp;
    for (int i = 0; i < kNumJoints; ++i)
    {
      Jp.col(i) = J_full.block<3,1>(0, arm_v_idx_[i]);
    }

    // Damped least squares in R^3:
    // dq = Jp^T (Jp Jp^T + lambda^2 I)^{-1} ep
    Eigen::Matrix3d JJt = Jp * Jp.transpose();
    const double lambda = ik_opt.damping;
    JJt.diagonal().array() += lambda * lambda;

    const Eigen::Vector3d v = JJt.ldlt().solve(ep);
    Vector7d dq = Jp.transpose() * v;

    if (!dq.allFinite())
    {
      // std::cerr << "[PINOCCHIO_IK_EARLY_FAIL] reason=NON_FINITE_DQ it=" << it
      //           << " dq=[" << dq.transpose() << "]" << std::endl;
      if (final_err) *final_err = last_err;
      q_sol = q;
      return false;
    }

    // if (it == 0 || (it % 10) == 0)
    // {
    //   std::cerr << "[IK] ||Jp||=" << Jp.norm()
    //             << " ||dq||=" << dq.norm()
    //             << " dq=[" << dq.transpose() << "]"
    //             << std::endl;
    // }

    // =======================
    // 4) Line-search (monotonic position error decrease)
    //    IMPORTANT: integrate in FULL nq space (9), then project back to 7 arm joints.
    // =======================
    double alpha = ik_opt.step_size;
    const double err0 = err;

    Vector7d q_candidate = q;
    double err_candidate = err0;

    // Build full state once
    const Eigen::VectorXd q_full0 = buildQFull(q);

    // Build full velocity (nv)
    Eigen::VectorXd dq_full = Eigen::VectorXd::Zero(model_.nv);
    for (int i = 0; i < kNumJoints; ++i)
      dq_full[arm_v_idx_[i]] = dq[i];

    for (int ls = 0; ls < 10; ++ls)
    {
      const Eigen::VectorXd q_full_candidate =
        pinocchio::integrate(model_, q_full0, alpha * dq_full);

      Vector7d q_arm_cand;
      for (int i = 0; i < kNumJoints; ++i)
        q_arm_cand[i] = q_full_candidate[arm_q_idx_[i]];

      // Evaluate candidate
      const Eigen::VectorXd q_full_cand = buildQFull(q_arm_cand);
      pinocchio::forwardKinematics(model_, data_, q_full_cand);
      pinocchio::updateFramePlacements(model_, data_);
      const pinocchio::SE3& cur2 = data_.oMf[ee_frame_id_];

      const Eigen::Vector3d ep2 = target_locked.translation() - cur2.translation();
      err_candidate = ep2.norm();

      // if (it == 0)
      // {
      //   std::cerr << "[LS] ls=" << ls
      //             << " alpha=" << alpha
      //             << " err_candidate=" << err_candidate
      //             << " ep2=[" << ep2.transpose() << "]"
      //             << std::endl;
      // }

      if (std::isfinite(err_candidate) && err_candidate < err0)
      {
        q_candidate = q_arm_cand;
        break;
      }

      alpha *= 0.5;
    }

    if (!(std::isfinite(err_candidate) && err_candidate < err0))
    {
      // std::cerr << "[PINOCCHIO_IK_FAIL] line-search cannot reduce POS error, it=" << it
      //           << " err0=" << err0 << " err_candidate=" << err_candidate
      //           << std::endl;
      break;
    }

    // if (it == 0)
    // {
    //   std::cerr << "[IK-CHECK] it=0 accepted" << std::endl;
    //   std::cerr << "[IK-CHECK] q_old = " << q.transpose() << std::endl;
    //   std::cerr << "[IK-CHECK] q_new = " << q_candidate.transpose() << std::endl;
    // }

    q = q_candidate;

    // =======================
    // 5) Optional clamp to limits (on full q, then project back)
    // =======================
    if (ik_opt.clamp_to_limits)
    {
      Eigen::VectorXd q_fullx = buildQFull(q);
      for (int i = 0; i < kNumJoints; ++i)
      {
        const int idx_q = arm_q_idx_[i];
        q_fullx[idx_q] = clamp(q_fullx[idx_q],
                               model_.lowerPositionLimit[idx_q],
                               model_.upperPositionLimit[idx_q]);
      }
      for (int i = 0; i < kNumJoints; ++i)
        q[i] = q_fullx[arm_q_idx_[i]];
    }
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

}  // namespace franka_example_controllers
