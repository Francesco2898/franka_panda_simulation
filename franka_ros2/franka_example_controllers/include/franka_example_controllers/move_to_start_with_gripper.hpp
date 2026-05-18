#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "franka_example_controllers/pinocchio_kinematics.hpp"

namespace franka_example_controllers {

class MoveToStartWithGripper : public controller_interface::ControllerInterface {
public:
//   using Vector8d = Eigen::Matrix<double, 8, 1>;
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

private:
  void read_state();

  // ---- params / state ----
  std::vector<std::string> joint_names_;

  Vector7d q_{Vector7d::Zero()};
  Vector7d dq_{Vector7d::Zero()};
  Vector7d dq_filtered_{Vector7d::Zero()};

  Vector7d q_start_{Vector7d::Zero()};
  Vector7d q_goal_{Vector7d::Zero()};
  Vector7d q_des_{Vector7d::Zero()};

  double elapsed_time_{0.0};
  double move_duration_{10.0};
  double finish_tolerance_{0.01};
  bool hold_position_{true};
  bool finished_{false};

  // ---- pinocchio IK (仍然只对前7个arm关节) ----
  bool use_pinocchio_ik_{false};
  std::string robot_description_topic_{"robot_description"};
  bool use_robot_hand_{true};
  std::string ee_frame_override_{""};

  Eigen::Vector3d target_p_{0.5, 0.0, 0.4};
  Eigen::Vector4d target_q_xyzw_{0.0, 0.0, 0.0, 1.0};

  PinocchioKinematics::IkOptions ik_opt_{};
  PinocchioKinematics pino_;
  bool kin_initialized_{false};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  std::mutex urdf_mutex_;
  std::string urdf_xml_;
  std::atomic_bool urdf_received_{false};

  // helpers
  std::string resolve_ee_frame_name() const;
  pinocchio::SE3 make_target_se3() const;
  bool init_pinocchio_if_needed_with_wait();

  Vector7d k_gains_{Vector7d::Zero()};
  Vector7d d_gains_{Vector7d::Zero()};


};

}  // namespace franka_example_controllers
