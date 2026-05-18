#pragma once

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <Eigen/Dense>
#include <string>
#include <cmath>

#ifdef USE_YAML_CPP
#include <yaml-cpp/yaml.h>
#endif

struct CartesianImpedanceOscConfig
{
  std::string ee_frame_name = "fr3_hand_tcp";
  // to stiff
  // Eigen::Matrix<double, 6, 1> Kp6 = (Eigen::Matrix<double, 6, 1>() <<
  //     500.0, 500.0, 500.0,   100.0,  100.0,  100.0).finished();

  // Eigen::Matrix<double, 6, 1> Kd6 = (Eigen::Matrix<double, 6, 1>() <<
  //     5.0,   5.0,   5.0,    1,   1,   1).finished();
  
  // /// debug 1: worse
  // Eigen::Matrix<double, 6, 1> Kp6 = (Eigen::Matrix<double, 6, 1>() <<
  //     300.0, 300.0, 300.0,   80.0,  80.0,  80.0).finished();

  // Eigen::Matrix<double, 6, 1> Kd6 = (Eigen::Matrix<double, 6, 1>() <<
  //     5.0,   5.0,   5.0,    1,   1,   1).finished();
  
  // // /// debug 2: better
  // Eigen::Matrix<double, 6, 1> Kp6 = (Eigen::Matrix<double, 6, 1>() <<
  //     500.0, 500.0, 500.0,   100.0,  100.0,  100.0).finished();

  // Eigen::Matrix<double, 6, 1> Kd6 = (Eigen::Matrix<double, 6, 1>() <<
  //     8.0,   8.0,   8.0,    2,   2,   2).finished();  


  // // // /// debug 3: much better in RL stable
  // Eigen::Matrix<double, 6, 1> Kp6 = (Eigen::Matrix<double, 6, 1>() <<
  //     500.0, 500.0, 500.0,   100.0,  100.0,  100.0).finished();

  // Eigen::Matrix<double, 6, 1> Kd6 = (Eigen::Matrix<double, 6, 1>() <<
  //     10.0,   10.0,   10.0,    2.5,   2.5,   2.5).finished();  

  // /// debug 2: better, more balanced
  Eigen::Matrix<double, 6, 1> Kp6 = (Eigen::Matrix<double, 6, 1>() <<
      500.0, 500.0, 500.0,   100.0,  100.0,  100.0).finished();

  Eigen::Matrix<double, 6, 1> Kd6 = (Eigen::Matrix<double, 6, 1>() <<
      9.0,   9.0,   9.0,    2,   2,   2).finished();  



  double kp_null = 0.0;
  double kd_null = 2.0 * std::sqrt(kp_null);

  Eigen::Matrix<double, 7, 1> q_default = (Eigen::Matrix<double, 7, 1>() <<
      0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785).finished();

  double tau_limit = 100.0;
  bool use_quat_shortest_path = true;
  double dt = 0.001;

  // Always available default
  static CartesianImpedanceOscConfig Default()
  {
    return CartesianImpedanceOscConfig{};
  }

#ifdef USE_YAML_CPP
  // Load & override fields from YAML (throws only if you want; here we keep it safe)
  static bool LoadFromYamlFile(const std::string& path, CartesianImpedanceOscConfig& out_cfg);
#endif
};

class CartesianImpedanceOscRT
{
public:
  CartesianImpedanceOscRT(const pinocchio::Model& model, int arm_dofs = 7);

  // Configure with provided cfg (caller can pass Default()).
  // Call once before realtime loop.
  bool configure(const CartesianImpedanceOscConfig& cfg);

  // Optional helper: try YAML; if fail, fallback to Default().
  // This function is meant to be called in non-realtime init stage.
  bool configureFromYamlOrDefault(const std::string& yaml_path);

  void computeTorque(const Eigen::Matrix<double, 7, 1>& q7,
                     const Eigen::Matrix<double, 7, 1>& dq7,
                     const pinocchio::SE3& ee_des,
                     Eigen::Matrix<double, 7, 1>& tau7_out,
                     double elapse_time,
                     Eigen::Matrix<double, 6, 1>* wrench6_out = nullptr
                     );

  pinocchio::FrameIndex eeFrameId() const { return ee_frame_id_; }

  const Eigen::Matrix<double, 7, 1>& getG7() const { return g7_; }

  void printCurrentConfig() const;

private:
  static inline double wrapToPi(double x)
  {
    const double two_pi = 2.0 * M_PI;
    x = std::fmod(x + M_PI, two_pi);
    if (x < 0) x += two_pi;
    return x - M_PI;
  }

  static Eigen::Vector3d axisAngleFromQuatLikePython(const Eigen::Quaterniond& q_err);
  static Eigen::Quaterniond flipQuatIfNeededShortestPath(const Eigen::Quaterniond& q_des,
                                                         const Eigen::Quaterniond& q_curr);

  void resizeWorkspace();

  // [CHANGED] Own the Pinocchio model by value.
  // Fixes dangling-reference issue when caller passes a temporary/local Model.
  pinocchio::Model model_;
  pinocchio::Data data_;

  int arm_dofs_;
  pinocchio::FrameIndex ee_frame_id_;
  bool configured_ = false;

  CartesianImpedanceOscConfig cfg_;

  Eigen::VectorXd q_full_;
  Eigen::VectorXd dq_full_;

  Eigen::Matrix<double, 6, Eigen::Dynamic> J6_full_;
  Eigen::Matrix<double, 6, 7> J6_;

  Eigen::Matrix<double, 7, 7> M7_;
  Eigen::LDLT<Eigen::Matrix<double, 7, 7>> ldlt_M_;

  Eigen::Matrix<double, 6, 1> delta_pose_;
  Eigen::Matrix<double, 6, 1> v6_;
  Eigen::Matrix<double, 6, 1> wrench6_;

  Eigen::Matrix<double, 6, 6> A6_;
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt_A_;
  Eigen::Matrix<double, 6, 7> JMInv_;
  Eigen::Matrix<double, 6, 7> j_eef_inv_;

  Eigen::Matrix<double, 7, 1> tau_task_;
  Eigen::Matrix<double, 7, 1> dist_default_;
  Eigen::Matrix<double, 7, 1> u_null_;
  Eigen::Matrix<double, 7, 1> u_null_mass_;
  Eigen::Matrix<double, 7, 7> N7_;
  Eigen::Matrix<double, 7, 1> tau_null_;
  Eigen::Matrix<double, 7, 1> tau_total_;

  Eigen::Matrix<double, 7, 1> g7_;
};
