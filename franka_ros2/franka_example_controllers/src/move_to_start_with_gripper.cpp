#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <array>

#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

#include "franka_example_controllers/pinocchio_kinematics.hpp"
// #include "franka_example_controllers/move_to_start_with_gripper.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"
#include "franka_msgs/msg/franka_robot_state.hpp"


namespace franka_example_controllers
{

class MoveToStartWithGripper : public controller_interface::ControllerInterface
{
public:
  // using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  controller_interface::CallbackReturn on_init() override
  {
    auto node = get_node();

    auto declare_if_needed = [node](const std::string& name, auto default_value) {
      using T = decltype(default_value);
      if (!node->has_parameter(name)) {
        node->declare_parameter<T>(name, default_value);
      }
    };

    // --- Existing parameters (move-to-start)
    declare_if_needed("joint_names", std::vector<std::string>{});
    declare_if_needed("q_goal", std::vector<double>{});
    declare_if_needed("move_duration", 10.0);
    declare_if_needed("finish_tolerance", 0.01);
    declare_if_needed("hold_position", true);

    // --- Pinocchio IK enable switch + target pose (EE)
    declare_if_needed("use_pinocchio_ik", false);
    declare_if_needed("robot_description_topic", std::string("robot_description"));

    declare_if_needed("use_robot_hand", true);
    declare_if_needed("ee_frame_override", std::string(""));  // if not empty, overrides above

    // target pose for IK (position + quaternion xyzw)
    declare_if_needed("target_position", std::vector<double>{0.5, 0.0, 0.4});
    declare_if_needed("target_orientation_xyzw", std::vector<double>{0.0, 0.0, 0.0, 1.0});

    // IK options
    declare_if_needed("ik.max_iters", 100);
    declare_if_needed("ik.eps", 1e-4);
    declare_if_needed("ik.step_size", 1.0);
    declare_if_needed("ik.damping", 1e-4);
    declare_if_needed("ik.clamp_to_limits", false);
    declare_if_needed("ik.w_rot", 1.0);

    // state marker
    declare_if_needed("process_finished", false);

    declare_if_needed("arm_id", std::string("fr3"));

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& j : joint_names_) {
      cfg.names.push_back(j + "/effort");
    }
    return cfg;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& j : joint_names_) {
      cfg.names.push_back(j + "/position");
      cfg.names.push_back(j + "/velocity");
    }

    cfg.names.push_back(arm_id_ + "/robot_state");
    return cfg;
  }

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override
  {
    auto node = get_node();

    // read params
    node->get_parameter("joint_names", joint_names_);
    std::vector<double> q_goal_vec;
    node->get_parameter("q_goal", q_goal_vec);
    node->get_parameter("move_duration", move_duration_);
    node->get_parameter("finish_tolerance", finish_tolerance_);
    node->get_parameter("hold_position", hold_position_);

    node->get_parameter("use_pinocchio_ik", use_pinocchio_ik_);
    node->get_parameter("robot_description_topic", robot_description_topic_);
    node->get_parameter("use_robot_hand", use_robot_hand_);
    node->get_parameter("ee_frame_override", ee_frame_override_);

    std::vector<double> target_pos, target_quat;
    node->get_parameter("target_position", target_pos);
    node->get_parameter("target_orientation_xyzw", target_quat);

    node->get_parameter("ik.max_iters", ik_opt_.max_iters);
    node->get_parameter("ik.eps", ik_opt_.eps);
    node->get_parameter("ik.step_size", ik_opt_.step_size);
    node->get_parameter("ik.damping", ik_opt_.damping);
    node->get_parameter("ik.clamp_to_limits", ik_opt_.clamp_to_limits);
    node->get_parameter("ik.w_rot", ik_opt_.w_rot);

    // ===== CHANGED: 7 -> 7 =====
    if (joint_names_.size() != 7) {
      RCLCPP_ERROR(node->get_logger(), "hardware joint_names must have 7 joints, got %zu", joint_names_.size());
      return controller_interface::CallbackReturn::ERROR;
    }
    // >>> CHANGE: always require 7 q_goal values so gripper (7th) always comes from YAML
    if (q_goal_vec.size() != 7) {
      RCLCPP_ERROR(node->get_logger(), "hardware q_goal must have exactly 7 values (7 arm + 0 gripper), got %zu", q_goal_vec.size());
      return controller_interface::CallbackReturn::ERROR;
    }
    for (size_t i = 0; i < 7; ++i) {
      q_goal_(static_cast<int>(i)) = q_goal_vec[i];
    }
    // ===========================

    if (target_pos.size() == 3) {
      target_p_ = Eigen::Vector3d(target_pos[0], target_pos[1], target_pos[2]);
    }
    if (target_quat.size() == 4) {
      target_q_xyzw_ = Eigen::Vector4d(target_quat[0], target_quat[1], target_quat[2], target_quat[3]);
    }

    // Subscribe robot_description (URDF string) via topic
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();
    qos.reliable();

    robot_description_sub_ =
      node->create_subscription<std_msgs::msg::String>(
      robot_description_topic_, qos,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (urdf_received_.load()) {
          return;
        }
        {
          std::lock_guard<std::mutex> lk(urdf_mutex_);
          urdf_xml_ = msg->data;
        }
        urdf_received_.store(true);
        RCLCPP_INFO(get_node()->get_logger(), "Received robot_description from topic '%s' (len=%zu)",
                    robot_description_topic_.c_str(), msg->data.size());
      });

    // reset
    elapsed_time_ = 0.0;
    finished_ = false;
    kin_initialized_ = false;
    get_node()->set_parameter(rclcpp::Parameter("process_finished", false));

    auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
    auto d_gains = get_node()->get_parameter("d_gains").as_double_array();

    if (k_gains.empty()) {
      RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
      return CallbackReturn::FAILURE;
    }
    if (k_gains.size() != 7) {
      RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                  7, k_gains.size());
      return CallbackReturn::FAILURE;
    }
    if (d_gains.empty()) {
      RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
      return CallbackReturn::FAILURE;
    }
    if (d_gains.size() != 7) {
      RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                  7, d_gains.size());
      return CallbackReturn::FAILURE;
    }
    for (int i = 0; i < 7; ++i) {
      d_gains_(i) = d_gains.at(i);
      k_gains_(i) = k_gains.at(i);
    }

    std::cout<<"============d_gain:"<< d_gains_.transpose()<<std::endl;
    std::cout<<"============d_gain:"<< k_gains_.transpose()<<std::endl;


    node->get_parameter("arm_id", arm_id_);

    std::string robot_description;
    if (!node->get_parameter("robot_description", robot_description)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to get robot_description parameter");
      return controller_interface::CallbackReturn::ERROR;
    }

    franka_robot_state_ =
        std::make_unique<franka_semantic_components::FrankaRobotState>(
            arm_id_ + "/robot_state", robot_description);

    franka_robot_state_->initialize_robot_state_msg(franka_state_msg_);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override
  {
    // Read initial state
    read_state();
    q_start_ = q_;
    dq_filtered_.setZero();

    // >>> CHANGE: logging state
    log_elapsed_time_ = 0.0;
    printed_last_loop_ = false;

    // If IK enabled: compute q_goal_ via Pinocchio IK (arm only: first 7 joints)
    if (use_pinocchio_ik_) {
      if (init_pinocchio_if_needed_with_wait()) {
        pinocchio::SE3 target = make_target_se3();

        // Heuristic: if target quat ~ identity, keep current EE orientation
        {
          Eigen::Quaterniond q_in(target_q_xyzw_[3], target_q_xyzw_[0], target_q_xyzw_[1], target_q_xyzw_[2]); // w,x,y,z
          q_in.normalize();

          const Eigen::Quaterniond q_id = Eigen::Quaterniond::Identity();
          const double dot = std::abs(q_in.dot(q_id));
          const bool is_identity = (1.0 - dot) < 1e-6;

          RCLCPP_INFO(get_node()->get_logger(),
                      "IK target quat input xyzw=[%.6f %.6f %.6f %.6f], |dot(identity)|=%.9f, is_identity=%s",
                      target_q_xyzw_[0], target_q_xyzw_[1], target_q_xyzw_[2], target_q_xyzw_[3],
                      dot, (is_identity ? "true" : "false"));

          if (is_identity) {
            // ===== CHANGED: fk 用 arm 7 维 =====
            Vector7d q_arm = q_.head<7>();
            const pinocchio::SE3 T_curr = pino_.fk(q_arm);
            target.rotation() = T_curr.rotation();
            RCLCPP_INFO(get_node()->get_logger(),
                        "IK target orientation ~ identity; keep current EE orientation to improve feasibility.");
          }
        }

        // ===== CHANGED: ik 输入/输出都用 arm 7 维 =====
        Vector7d q_init_arm = q_.head<7>();
        Vector7d q_sol = q_init_arm;
        double final_err = -1.0;
        bool ok = pino_.ik(target, q_init_arm, q_sol, ik_opt_, &final_err);

        if (!ok) {
          RCLCPP_WARN(get_node()->get_logger(),
                      "Pinocchio IK failed (final_err=%.6f); fallback to configured q_goal.", final_err);
        } else {
          // only overwrite first 7 arm joints;
          q_goal_.head<7>() = q_sol;
          RCLCPP_INFO(get_node()->get_logger(), "Pinocchio IK success. Using IK solution as q_goal (arm only).");
        }
      } else {
        RCLCPP_WARN(get_node()->get_logger(),
                    "Pinocchio init not ready (URDF not received). Using configured q_goal directly.");
      }
    }

    if (!franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                  "Failed to assign franka robot state interface.");
      return controller_interface::CallbackReturn::ERROR;
    }


    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override
  {
    if (franka_robot_state_) {
      franka_robot_state_->release_interfaces();
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) override
  {
    const double dt = period.seconds();
    read_state();

    if (!franka_robot_state_->get_values_as_message(franka_state_msg_)) {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                            "Failed to read franka robot state.");
      return controller_interface::return_type::ERROR;
    }

    // 1) 最后一个关节的估计外力矩
    tau_ext_last_joint_ = franka_state_msg_.tau_ext_hat_filtered.effort[6];

    // 2) 刚度坐标系下的末端估计 wrench
    wrench_ext_k_[0] = franka_state_msg_.k_f_ext_hat_k.wrench.force.x;
    wrench_ext_k_[1] = franka_state_msg_.k_f_ext_hat_k.wrench.force.y;
    wrench_ext_k_[2] = franka_state_msg_.k_f_ext_hat_k.wrench.force.z;
    wrench_ext_k_[3] = franka_state_msg_.k_f_ext_hat_k.wrench.torque.x;
    wrench_ext_k_[4] = franka_state_msg_.k_f_ext_hat_k.wrench.torque.y;
    wrench_ext_k_[5] = franka_state_msg_.k_f_ext_hat_k.wrench.torque.z;

    // 3) 基坐标系下的末端估计 wrench
    wrench_ext_o_[0] = franka_state_msg_.o_f_ext_hat_k.wrench.force.x;
    wrench_ext_o_[1] = franka_state_msg_.o_f_ext_hat_k.wrench.force.y;
    wrench_ext_o_[2] = franka_state_msg_.o_f_ext_hat_k.wrench.force.z;
    wrench_ext_o_[3] = franka_state_msg_.o_f_ext_hat_k.wrench.torque.x;
    wrench_ext_o_[4] = franka_state_msg_.o_f_ext_hat_k.wrench.torque.y;
    wrench_ext_o_[5] = franka_state_msg_.o_f_ext_hat_k.wrench.torque.z;


    // >>> CHANGE: detect last loop
    bool just_finished = false;

    if (!finished_) {
      elapsed_time_ += dt;
      const double s = std::clamp(elapsed_time_ / std::max(1e-6, move_duration_), 0.0, 1.0);
      q_des_ = (1.0 - s) * q_start_ + s * q_goal_;

      if ((q_ - q_goal_).norm() < finish_tolerance_) {
        finished_ = true;
        just_finished = true;
        get_node()->set_parameter(rclcpp::Parameter("process_finished", true));
        RCLCPP_INFO(get_node()->get_logger(), "Move-to-start finished.");
      }
    } else if (hold_position_) {
      q_des_ = q_goal_;
    } else {
      q_des_ = q_;
    }

    // >>> CHANGE: print every 1s ONLY before move_duration (i.e., during motion)
    (void)just_finished;  // keep variable unchanged, but silence unused warning
    // if ((elapsed_time_ <= (move_duration_ * 20.0))) {
    log_elapsed_time_ += dt;
    if (log_elapsed_time_ >= 1.0) {
      log_elapsed_time_ = 0.0;

      std::ostringstream oss;
      oss.setf(std::ios::fixed);
      oss << std::setprecision(6);
      oss << "ref(q_des)=";
      for (size_t i = 0; i < 7; ++i) {
        if (i) oss << ",";
        oss << q_des_(static_cast<int>(i));
      }
      oss << " | err(q-q_ref)=";
      for (size_t i = 0; i < 7; ++i) {
        if (i) oss << ",";
        oss << (q_(static_cast<int>(i)) - q_des_(static_cast<int>(i)));
      }

      oss << " | tau_ext_last=" << tau_ext_last_joint_
          << " | wrenchK=["
          << wrench_ext_k_[0] << "," << wrench_ext_k_[1] << "," << wrench_ext_k_[2] << ","
          << wrench_ext_k_[3] << "," << wrench_ext_k_[4] << "," << wrench_ext_k_[5] << "]";

      // NOTE: gripper is not used in the hardware
      RCLCPP_INFO(get_node()->get_logger(), "%s", oss.str().c_str());


    }
    // }

    // Simple joint PD torque
    // ===== CHANGED: 7 -> 7 =====
    for (size_t i = 0; i < 7; ++i) {

      const double tau = k_gains_[i] * (q_des_(static_cast<int>(i)) - q_(static_cast<int>(i)))
                       - d_gains_[i] * dq_(static_cast<int>(i));

      command_interfaces_[i].set_value(tau);
      
      // command_interfaces_[i].set_value(0);

    }
    // ===========================

    return controller_interface::return_type::OK;
  }

private:
  // ===== CHANGED: read_state map sizes 7 -> 7, and fill 7 =====
  void read_state()
  {
    static bool map_built = false;
    static std::array<int, 7> pos_idx;
    static std::array<int, 7> vel_idx;

    if (!map_built) {
      pos_idx.fill(-1);
      vel_idx.fill(-1);

      for (size_t k = 0; k < state_interfaces_.size(); ++k) {
        const auto & si = state_interfaces_[k];
        const std::string full = si.get_name();

        for (size_t j = 0; j < 7; ++j) {
          const std::string & jn = joint_names_[j];
          if (full == (jn + "/position")) pos_idx[j] = static_cast<int>(k);
          if (full == (jn + "/velocity")) vel_idx[j] = static_cast<int>(k);
        }
      }

      for (size_t j = 0; j < 7; ++j) {
        if (pos_idx[j] < 0 || vel_idx[j] < 0) {
          RCLCPP_ERROR(get_node()->get_logger(),
            "read_state(): cannot find state interfaces for joint '%s' (pos_idx=%d vel_idx=%d). "
            "Check controller joint names and ros2_control interfaces.",
            joint_names_[j].c_str(), pos_idx[j], vel_idx[j]);
          return;
        }
      }

      map_built = true;

      for (size_t j = 0; j < 7; ++j) {
        RCLCPP_INFO(get_node()->get_logger(),
          "state idx map: %s pos=%d vel=%d",
          joint_names_[j].c_str(), pos_idx[j], vel_idx[j]);
      }
    }

    for (size_t j = 0; j < 7; ++j) {
      q_(static_cast<int>(j))  = state_interfaces_[pos_idx[j]].get_value();
      dq_(static_cast<int>(j)) = state_interfaces_[vel_idx[j]].get_value();
    }

    dq_filtered_ = 0.9 * dq_filtered_ + 0.1 * dq_;
  }
  // ===========================================================

  std::string resolve_ee_frame_name() const
  {
    if (!ee_frame_override_.empty()) {
      return ee_frame_override_;
    }
    if (use_robot_hand_) {
      return std::string("fr3_hand");
    }
    return std::string("fr3_flange");
  }

  pinocchio::SE3 make_target_se3() const
  {
    Eigen::Quaterniond q(target_q_xyzw_[3], target_q_xyzw_[0], target_q_xyzw_[1], target_q_xyzw_[2]);  // w,x,y,z
    q.normalize();
    Eigen::Matrix3d R = q.toRotationMatrix();
    return pinocchio::SE3(R, target_p_);
  }

  bool init_pinocchio_if_needed_with_wait()
  {
    if (kin_initialized_) {
      return true;
    }

    const auto t0 = std::chrono::steady_clock::now();
    while (!urdf_received_.load()) {
      const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
      if (dt.count() > 2000) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::string urdf;
    {
      std::lock_guard<std::mutex> lk(urdf_mutex_);
      urdf = urdf_xml_;
    }
    if (urdf.empty()) {
      return false;
    }

    const std::string ee_frame = resolve_ee_frame_name();
    bool ok = pino_.initFromURDFString(urdf, ee_frame);
    if (!ok) {
      RCLCPP_ERROR(get_node()->get_logger(), "Pinocchio init failed. ee_frame='%s'", ee_frame.c_str());
      return false;
    }

    kin_initialized_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "Pinocchio initialized. ee_frame='%s'", ee_frame.c_str());
    return true;
  }

private:
  std::vector<std::string> joint_names_;

  Vector7d q_{Vector7d::Zero()};
  Vector7d dq_{Vector7d::Zero()};
  Vector7d dq_filtered_{Vector7d::Zero()};

  Vector7d q_start_{Vector7d::Zero()};
  Vector7d q_goal_{Vector7d::Zero()};
  Vector7d q_des_{Vector7d::Zero()};

  double elapsed_time_{0.0};
  // >>> CHANGE: logging timer/state
  double log_elapsed_time_{0.0};
  bool printed_last_loop_{false};
  double move_duration_{10.0};
  double finish_tolerance_{0.01};
  bool hold_position_{true};
  bool finished_{false};

  bool use_pinocchio_ik_{false};
  std::string robot_description_topic_{"robot_description"};
  bool use_robot_hand_{true};
  std::string ee_frame_override_{""};

  Eigen::Vector3d target_p_{0.5, 0.0, 0.4};
  Eigen::Vector4d target_q_xyzw_{0.0, 0.0, 0.0, 1.0};

  Eigen::Vector3d target_p_2_{0.5, 0.0, 0.4};

  PinocchioKinematics::IkOptions ik_opt_{};
  PinocchioKinematics pino_;
  bool kin_initialized_{false};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  std::mutex urdf_mutex_;
  std::string urdf_xml_;
  std::atomic_bool urdf_received_{false};
  Vector7d k_gains_{Vector7d::Zero()};
  Vector7d d_gains_{Vector7d::Zero()};


  std::string arm_id_{"fr3"};

  std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state_;
  franka_msgs::msg::FrankaRobotState franka_state_msg_;

  double tau_ext_last_joint_{0.0};
  std::array<double, 6> wrench_ext_k_{{0, 0, 0, 0, 0, 0}};
  std::array<double, 6> wrench_ext_o_{{0, 0, 0, 0, 0, 0}};


  
};

}  // namespace franka_example_controllers

// ===== CHANGED: plugin export class name =====
PLUGINLIB_EXPORT_CLASS(
  franka_example_controllers::MoveToStartWithGripper,
  controller_interface::ControllerInterface)
