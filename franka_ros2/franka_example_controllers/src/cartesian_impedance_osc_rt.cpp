#include "franka_example_controllers/cartesian_impedance_osc_rt.hpp"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"


#include <cmath>
#include <iostream>

using pinocchio::SE3;

#ifdef USE_YAML_CPP
static bool isSeqSize(const YAML::Node& n, std::size_t sz)
{
  return n && n.IsSequence() && n.size() == sz;
}

static void overrideVec6(const YAML::Node& root, const std::string& key,
                         Eigen::Matrix<double,6,1>& v)
{
  const YAML::Node a = root[key];
  if (!isSeqSize(a, 6)) return;
  for (std::size_t i=0;i<6;i++) v[(int)i] = a[i].as<double>();
}

static void overrideVec7(const YAML::Node& root, const std::string& key,
                         Eigen::Matrix<double,7,1>& v)
{
  const YAML::Node a = root[key];
  if (!isSeqSize(a, 7)) return;
  for (std::size_t i=0;i<7;i++) v[(int)i] = a[i].as<double>();
}

bool CartesianImpedanceOscConfig::LoadFromYamlFile(const std::string& path,
                                                   CartesianImpedanceOscConfig& out_cfg)
{
  try
  {
    YAML::Node root = YAML::LoadFile(path);
    if (!root) return false;

    if (root["ee_frame"]) out_cfg.ee_frame_name = root["ee_frame"].as<std::string>();
    if (root["kp_null"]) out_cfg.kp_null = root["kp_null"].as<double>();
    if (root["kd_null"]) out_cfg.kd_null = root["kd_null"].as<double>();
    if (root["tau_limit"]) out_cfg.tau_limit = root["tau_limit"].as<double>();
    if (root["use_quat_shortest_path"]) out_cfg.use_quat_shortest_path = root["use_quat_shortest_path"].as<bool>();
    if (root["dt"]) out_cfg.dt = root["dt"].as<double>();

    overrideVec6(root, "Kp6", out_cfg.Kp6);
    overrideVec6(root, "Kd6", out_cfg.Kd6);
    overrideVec7(root, "q_default", out_cfg.q_default);

    return true;
  }
  catch (...)
  {
    return false;
  }
}
#endif

CartesianImpedanceOscRT::CartesianImpedanceOscRT(const pinocchio::Model& model, int arm_dofs)
// [CHANGED] model_ is now owned-by-value; build data_ from the owned model_.
: model_(model), data_(model_), arm_dofs_(arm_dofs)
{
  resizeWorkspace();
}

void CartesianImpedanceOscRT::resizeWorkspace()
{
  q_full_.resize(model_.nq);
  dq_full_.resize(model_.nv);
  q_full_.setZero();
  dq_full_.setZero();

  J6_full_.resize(6, model_.nv);
  J6_full_.setZero();
}

bool CartesianImpedanceOscRT::configure(const CartesianImpedanceOscConfig& cfg)
{
  if (arm_dofs_ != 7)
  {
    std::cerr << "[CartesianImpedanceOscRT] arm_dofs must be 7 for strict Python reproduction.\n";
    configured_ = false;
    return false;
  }

  cfg_ = cfg;

  try
  {
    ee_frame_id_ = model_.getFrameId(cfg_.ee_frame_name);
  }
  catch (...)
  {
    std::cerr << "[CartesianImpedanceOscRT] cannot find ee frame: " << cfg_.ee_frame_name << "\n";
    configured_ = false;
    return false;
  }

  configured_ = true;
  return true;
}

bool CartesianImpedanceOscRT::configureFromYamlOrDefault(const std::string& yaml_path)
{
  // Always start from default
  CartesianImpedanceOscConfig cfg = CartesianImpedanceOscConfig::Default();

#ifdef USE_YAML_CPP
  CartesianImpedanceOscConfig cfg_yaml = cfg;
  bool ok = CartesianImpedanceOscConfig::LoadFromYamlFile(yaml_path, cfg_yaml);
  if (ok)
  {
    std::cerr << "[CartesianImpedanceOscRT] Loaded controller YAML: " << yaml_path << "\n";
    cfg = cfg_yaml;
  }
  else
  {
    std::cerr << "[CartesianImpedanceOscRT] WARN: Cannot load YAML (" << yaml_path
              << "), fallback to default parameters.\n";
  }
#else
  std::cerr << "[CartesianImpedanceOscRT] YAML disabled (no USE_YAML_CPP). Using default parameters.\n";
#endif

  return configure(cfg);
}

Eigen::Quaterniond CartesianImpedanceOscRT::flipQuatIfNeededShortestPath(
    const Eigen::Quaterniond& q_des, const Eigen::Quaterniond& q_curr)
{
  if (q_des.dot(q_curr) >= 0.0) return q_des;
  Eigen::Quaterniond q = q_des;
  q.coeffs() *= -1.0;
  return q;
}

// Eigen::Vector3d CartesianImpedanceOscRT::axisAngleFromQuatLikePython(const Eigen::Quaterniond& q_err_in)
// {
//   Eigen::Quaterniond q_err = q_err_in.normalized();
//   double w = std::max(-1.0, std::min(1.0, q_err.w()));
//   Eigen::Vector3d v(q_err.x(), q_err.y(), q_err.z());
//   double v_norm = v.norm();

//   const double eps = 1e-8;
//   if (v_norm < eps) return 2.0 * v;
  
//   Eigen::Quaterniond q_err = q_err_in.normalized();
//   if (q_err.w() < 0.0) q_err.coeffs() *= -1.0;  // 强制最短路（对齐 Python）

//   double angle = 2.0 * std::atan2(v_norm, w);
//   Eigen::Vector3d axis = v / v_norm;
//   return axis * angle;
// }

Eigen::Vector3d CartesianImpedanceOscRT::axisAngleFromQuatLikePython(const Eigen::Quaterniond& q_err_in)
{
  Eigen::Quaterniond q = q_err_in.normalized();
  if (q.w() < 0.0) q.coeffs() *= -1.0;   // 对齐 Python

  double w = std::clamp(q.w(), -1.0, 1.0);
  Eigen::Vector3d v(q.x(), q.y(), q.z());
  double v_norm = v.norm();

  if (v_norm < 1e-8) return 2.0 * v;

  double angle = 2.0 * std::atan2(v_norm, w);
  return (v / v_norm) * angle;
}



void CartesianImpedanceOscRT::computeTorque(
    const Eigen::Matrix<double, 7, 1>& q7,
    const Eigen::Matrix<double, 7, 1>& dq7,
    const SE3& ee_des,
    Eigen::Matrix<double, 7, 1>& tau7_out,
    double elapse_time,
    Eigen::Matrix<double, 6, 1>* wrench6_out
    )
{
  if(model_.nq !=9)
  {
    std::cout<<"CartesianImpedanceOscRT::computeTorque() model.nq:"<<model_.nq<<std::endl;
    std::cout<<"q7 size:"<<q7.size()<<std::endl;
  }

  if (!configured_)
  {
    tau7_out.setZero();
    if (wrench6_out) wrench6_out->setZero();
    return;
  }

  // 0) Fill full q/dq (gripper fixed)
  q_full_.setZero();
  dq_full_.setZero();
  q_full_.head<7>() = q7;
  dq_full_.head<7>() = dq7;

  // 1) FK
  pinocchio::forwardKinematics(model_, data_, q_full_, dq_full_);
  pinocchio::updateFramePlacements(model_, data_);

  const SE3& oMe = data_.oMf[ee_frame_id_];
  const Eigen::Vector3d x = oMe.translation();
  const Eigen::Matrix3d R = oMe.rotation();

  // 2) Desired
  const Eigen::Vector3d xd = ee_des.translation();
  const Eigen::Matrix3d Rd = ee_des.rotation();

  // 3) Pose error
  Eigen::Vector3d pos_error = xd - x;

  Eigen::Quaterniond q_curr(R);
  Eigen::Quaterniond q_des(Rd);
  if (cfg_.use_quat_shortest_path)
    q_des = flipQuatIfNeededShortestPath(q_des, q_curr);

  Eigen::Quaterniond q_err = q_des * q_curr.conjugate();
  Eigen::Vector3d axis_angle_error = axisAngleFromQuatLikePython(q_err);

  delta_pose_.head<3>() = pos_error;
  delta_pose_.tail<3>() = axis_angle_error;

  // 4) Jacobian + v = J dq (stable & consistent)
  // pinocchio::computeJointJacobians(model_, data_, q_full_);  
  // pinocchio::getFrameJacobian(model_, data_, ee_frame_id_,
  //                             pinocchio::LOCAL_WORLD_ALIGNED, J6_full_);

  pinocchio::computeFrameJacobian(model_, data_, q_full_, ee_frame_id_,
                                  pinocchio::LOCAL_WORLD_ALIGNED, J6_full_);

  J6_ = J6_full_.leftCols<7>();
  v6_ = J6_ * dq7; // should use the real ee velocity!!!!!

  // 5) Task wrench (strict)
  wrench6_.head<3>() =
      cfg_.Kp6.head<3>().cwiseProduct(delta_pose_.head<3>()) +
      cfg_.Kd6.head<3>().cwiseProduct(-v6_.head<3>());
  wrench6_.tail<3>() =
      cfg_.Kp6.tail<3>().cwiseProduct(delta_pose_.tail<3>()) +
      cfg_.Kd6.tail<3>().cwiseProduct(-v6_.tail<3>());

  // 6) tau_task = J^T wrench
  tau_task_ = J6_.transpose() * wrench6_;

  // 7) Mass matrix M7 via CRBA (official)
  pinocchio::crba(model_, data_, q_full_);
  data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.transpose().triangularView<Eigen::StrictlyLower>();
  M7_ = data_.M.topLeftCorner<7,7>();

  ldlt_M_.compute(M7_);

  // 8) X = M^{-1} J^T (solve), A = J X
  Eigen::Matrix<double, 7, 6> X = ldlt_M_.solve(J6_.transpose());
  A6_ = J6_ * X;
  ldlt_A_.compute(A6_);

  // 9) j_eef_inv = A^{-1} * (J M^{-1}) = A^{-1} * X^T
  JMInv_ = X.transpose();
  j_eef_inv_ = ldlt_A_.solve(JMInv_);

  // 10) Nullspace
  dist_default_ = cfg_.q_default - q7;
  for (int i=0;i<7;i++) dist_default_[i] = wrapToPi(dist_default_[i]);

  u_null_ = cfg_.kd_null * (-dq7) + cfg_.kp_null * dist_default_;
  u_null_mass_ = M7_ * u_null_;
  // 现在：直接把 u_null_ 当作关节力矩命令，再由 N7_ 投影
  //u_null_mass_ = u_null_;

  N7_.setIdentity();
  N7_.noalias() -= J6_.transpose() * j_eef_inv_;

  tau_null_ = N7_ * u_null_mass_;


  // 11) total + clamp
  // >>> CHANGE: add g7 + c7 into command torque
  tau_total_ = tau_task_ + tau_null_ ;//+ nle7;
  // <<< CHANGE END
  // tau_total_ =  g7_;//+ nle7;

  // >>> CHANGE: add gravity + coriolis compensation (Pinocchio)
  // gravity: g(q)
  // coriolis/centrifugal: C(q, dq) * dq
  // nle = c(q,dq) + g(q)
  pinocchio::nonLinearEffects(model_, data_, q_full_, dq_full_);
  const Eigen::Matrix<double, 7, 1> nle7 = data_.nle.head<7>();
  // <<< CHANGE END

  // --- compute pure gravity term for testing: g(q) = nle(q,0) ---
  Eigen::Matrix<double, 9, 1> dq_zero;
  dq_zero.setZero();
  pinocchio::nonLinearEffects(model_, data_, q_full_, dq_zero);
  g7_ = data_.nle.head<7>();

  tau_total_ += (nle7 - g7_);


  const double lim = std::abs(cfg_.tau_limit);
  for (int i=0;i<7;i++)
  {
    if (tau_total_[i] > lim) tau_total_[i] = lim;
    else if (tau_total_[i] < -lim) tau_total_[i] = -lim;
  }

  tau7_out = tau_total_;



  // static int dbg = 0;
  // if (((dbg++ % 2) == 0) &&((elapse_time>0.0)&&(elapse_time<0.005))) {
  //   std::cout
  //     << "[OSC DBG] configured=" << configured_
  //     << " tau_limit=" << cfg_.tau_limit
  //     << " |pos_err|=" << delta_pose_.head<3>().norm()
  //     << " |rot_err|=" << delta_pose_.tail<3>().norm()
  //     << " |J|=" << J6_.norm()
  //     << " |wrench|=" << wrench6_.norm()
  //     << " |tau_task|=" << tau_task_.norm()
  //     << " |tau_total|=" << tau_total_.norm()
  //     << std::endl;
  // }






  if (wrench6_out) *wrench6_out = wrench6_;
}


void CartesianImpedanceOscRT::printCurrentConfig() const
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4);

  oss << "\n========== OSC Current Config ==========\n";
  oss << "configured: " << configured_ << "\n";
  oss << "dt: " << cfg_.dt << "\n";
  oss << "tau_limit: " << cfg_.tau_limit << "\n";
  oss << "use_quat_shortest_path: " << cfg_.use_quat_shortest_path << "\n";

  oss << "Kp6: [";
  for (int i = 0; i < 6; ++i) {
    oss << cfg_.Kp6[i];
    if (i < 5) oss << ", ";
  }
  oss << "]\n";

  oss << "Kd6: [";
  for (int i = 0; i < 6; ++i) {
    oss << cfg_.Kd6[i];
    if (i < 5) oss << ", ";
  }
  oss << "]\n";

  oss << "q_default: [";
  for (int i = 0; i < 7; ++i) {
    oss << cfg_.q_default[i];
    if (i < 6) oss << ", ";
  }
  oss << "]\n";

  oss << "kp_null: " << cfg_.kp_null << "\n";
  oss << "kd_null: " << cfg_.kd_null << "\n";
  oss << "========================================";

  std::cout << oss.str() << std::endl;
}