#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <array>   // NEW

#include <Eigen/Eigen>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <pinocchio/spatial/se3.hpp>

#include "franka_example_controllers/cartesian_impedance_osc_rt.hpp"
#include "franka_example_controllers/pinocchio_kinematics.hpp"

#include "franka_example_controllers/onnx_policy.hpp"

// [FT PATCH] begin
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <realtime_tools/realtime_buffer.hpp>
// [FT PATCH] end

#include "franka_example_controllers/rl_action_processor.hpp"    

#include "franka_semantic_components/franka_robot_state.hpp"
#include "franka_msgs/msg/franka_robot_state.hpp"

#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>
#include <Eigen/Geometry>

#include <rclcpp_action/rclcpp_action.hpp>
#include <franka_msgs/action/grasp.hpp>

using Grasp = franka_msgs::action::Grasp;
using GraspGoalHandle = rclcpp_action::ClientGoalHandle<Grasp>;

#include <franka_msgs/action/move.hpp>
using Move = franka_msgs::action::Move;
using MoveGoalHandle = rclcpp_action::ClientGoalHandle<Move>;

namespace franka_example_controllers {

class MoveCatersianImpWithGripper : public controller_interface::ControllerInterface {
public:
  // using Vector8d = Eigen::Matrix<double, 8, 1>;
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // ---- state read ----
  bool read_state();
  void buildPickPlaceObservation(double phase_mask);

  void applyPickPlacePolicyAction(
      const std::array<float, 9>& action,
      double dt_policy);

  Eigen::Vector3d computeOrientationErrorIsaacLabStyle(
      const Eigen::Quaterniond& q_target,
      const Eigen::Quaterniond& q_hand) const;

  // ---- helpers (IK/FK/OSC) ----
  std::string resolve_ee_frame_name() const;
  pinocchio::SE3 make_target_se3(int i) const;
  Eigen::Matrix<double, 6, 1> se3ToXyzRpy(const pinocchio::SE3& T) const;
  Eigen::Matrix<double, 7, 1> se3ToXyzQuat(const pinocchio::SE3& T) const;
  bool init_pinocchio_if_needed_with_wait();
  bool init_cartesian_osc_if_needed_with_wait();

  // ---- parameters ----
  std::vector<std::string> joint_names_;

  // Move-to-start params/state
  double elapsed_time_{0.0};
  double move_duration_{10.0};
  double finish_tolerance_{0.01};
  bool hold_position_{true};
  bool move_1_finished_{true};
  bool grasp_finished_{true};
  bool move_2_finished_{true};
  bool rl_finished_{true};

  double move_duration_grasp_{10.0};
  double move_duration_2_{10.0};
  double rl_duration_{10.0};

  std::vector<double> q_goal_vec;
  std::vector<double> q_goal_vec_grasp;

  // Logging
  double log_elapsed_time_{0.0};
  bool printed_last_loop_{false};

  // Switch: send OSC torque (arm) or joint PD torque
  bool catersian_imp_{false};

  // test graivty
  bool test_compensation_{false};

  // Joint PD gains (arm 7 + 0 gripper 1)
  Vector7d k_gains_{Vector7d::Constant(24.0)};
  Vector7d d_gains_{Vector7d::Constant(2.0)};

  // ---- runtime state (7 joints: 7 arm + 0 gripper) ----
  Vector7d q_{Vector7d::Zero()};
  Vector7d dq_{Vector7d::Zero()};
  Vector7d dq_filtered_{Vector7d::Zero()};
  Vector7d tau_meas_{Vector7d::Zero()};

  Vector7d q_start_{Vector7d::Zero()};
  Vector7d q_goal_{Vector7d::Zero()};
  Vector7d q_des_{Vector7d::Zero()};
  Vector7d q_init_arm{Vector7d::Zero()};
  Vector7d q_sol{Vector7d::Zero()};

  Vector7d q_goal_2_{Vector7d::Zero()};
  Eigen::Vector3d target_p_2_{0.5, 0.0, 0.4};
  Eigen::Vector4d target_q_xyzw_2_{0.0, 0.0, 0.0, 1.0};  

  // torque buffers
  Vector7d tau_joint_pd_{Vector7d::Zero()};
  Vector7d tau_osc_cmd_{Vector7d::Zero()};

  // EE (xyz + rpy)
  Eigen::Matrix<double, 6, 1> ee6_ref_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> ee6_mea_{Eigen::Matrix<double, 6, 1>::Zero()};

  // Time counters (debug)
  long int N_rt_{0};
  long int N_torque_{0};
  double time_total_{0.0};

  // ---- Pinocchio IK/FK (arm 7 joints) ----
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

  // ---- Cartesian impedance OSC (arm 7 joints) ----
  std::unique_ptr<CartesianImpedanceOscRT> osc_;
  bool osc_initialized_{false};
  Vector7d g7_{Vector7d::Zero()};

  // ---- debug publisher ----
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gait_data_pub_;
  sensor_msgs::msg::JointState joint2simulation_;

  // ===========================
  // NEW: RT-safety helpers (only for moving param/log out of update)
  // ===========================

  // warmup parameter cached (read in on_configure, used in RT without parameter access)
  double warmup_seconds_{0.0};

  // warmup runtime state (moved from static locals -> members, so we can reset in lifecycle)
  bool warmup_inited_{false};
  double warmup_elapsed_{0.0};
  Vector7d warmup_q_hold_{Vector7d::Zero()};

  bool warmup_cmd_map_built_{false};
  std::array<int, 7> warmup_eff_cmd_idx_{};

  // Non-warmup command map log moved out of RT (only for reporting)
  std::atomic_bool rt_cmd_map_ready_{false};

  // Request to set process_finished parameter (set in RT, executed in non-RT timer)
  std::atomic_bool request_set_process_finished_param_{false};

  // RT error reporting (no logging in RT)
  std::atomic_int rt_missing_warmup_eff_joint_{-1};
  std::atomic_int rt_missing_eff_joint_{-1};
  std::atomic_int rt_missing_state_joint_{-1};
  std::atomic_bool rt_move_finished_event_{false};

  // optional: provide a non-RT place to print logs and do set_parameter
  rclcpp::TimerBase::SharedPtr nonrt_housekeeping_timer_;


  // [FT PATCH] begin
  // FT topic param (read in on_configure)
  std::string ft_topic_{"/fr3/wrist_ft"};

  // Non-RT subscription
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_sub_;

  // RT-safe buffer: callback writes, RT reads
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::WrenchStamped::SharedPtr> ft_rt_buffer_;
  // [FT PATCH] end  


  // 命令枚举（不要在RT里比对字符串）
  enum class TaskCmd : int {
    NONE = 0,
    MOVE_POSE_1,
    MOVE_POSE_2,
    GRASP,
    OPEN_GRIPPER,
    CLOSE_GRIPPER,
    RUN_RL,
    SAVE_LOG,
    STOP
  };

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_cmd_sub_;

  // RT loop 只读这个
  std::atomic<int> task_cmd_{static_cast<int>(TaskCmd::NONE)};
  std::atomic<bool> task_cmd_new_{false};   // 可选：边沿触发  
  std::atomic<int> last_cmd_{static_cast<int>(TaskCmd::NONE)};

  bool allow_move_pose_1;

  //

  Vector7d tau_to_send{Vector7d::Zero()};

  bool prev_use_osc{false};
  double osc_blend_alpha{0.0};
  double osc_blend_duration{0.1};


  std::unique_ptr<franka_example_controllers::OnnxPolicy> policy_;
  std::string policy_name_;

  std::array<float, 46> rl_obs_{};
  std::array<float, 9>  rl_prev_action_{};
  bool rl_test_{true};  
  Vector7d q_policy_target_{Vector7d::Zero()};

  std::array<double, 9> robot_dof_lower_limit_{{
    -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
    0.0, 0.0
  }};

  std::array<double, 9> robot_dof_upper_limit_{{
    2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973,
    0.04, 0.04
  }};

  std::array<double, 9> robot_dof_speed_scales_{{
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0
  }};
  double finger_open_pos_{0.04};
  double finger_closed_pos_{0.0};

  double action_scale_{1.0};
  double dof_velocity_scale_{0.1};
  double r_in_{0.025};

  double object_init_z_{0.0};
  double success_height_{0.005};
  double object_z_estimate_{0.0};

  // Probe frames for clear_obs[20]
  std::array<std::string, 4> probe_frame_names_{{
    "fr3_link2",
    "fr3_link4",
    "fr3_link6",
    "fr3_hand"
  }};
  
  std::array<pinocchio::FrameIndex, 4> probe_frame_ids_{{
    pinocchio::FrameIndex(-1),
    pinocchio::FrameIndex(-1),
    pinocchio::FrameIndex(-1),
    pinocchio::FrameIndex(-1)
  }};
  
  bool probe_frame_ids_ready_{false};
  
  std::array<Eigen::Vector3d, 20> probe_points_w_{};
  
  // Obstacle cylinder parameters, equivalent to IsaacLab cfg
  Eigen::Vector3d obstacle_pos_w_{0.5, 0.0, 0.2};
  double obstacle_radius_{0.05};
  double obstacle_height_{0.20};
  double link_radius_{0.05};
  
  void initProbeFrameIds();
  void updateClearObsFromPinocchio();
  double computeCylinderClearance(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& obstacle_pos,
    double obstacle_radius,
    double obstacle_height,
    double link_radius) const;

  std::array<float, 20> clear_obs_{};

  Eigen::Vector3d target_p_gear_fixed_{0.5, 0.0, 0.0};  
  Eigen::Matrix<double, 7, 1> fingertip_midpoint_p_quaterion{Eigen::Matrix<double, 7, 1>::Zero()};

  Eigen::Vector3d fingertip_pos_rel_fixed{Eigen::Matrix<double, 3, 1>::Zero()};
  Eigen::Vector4d fingertip_quat{Eigen::Matrix<double, 4, 1>::Zero()};
  Eigen::Vector3d ee_linvel{Eigen::Matrix<double, 3, 1>::Zero()};
  Eigen::Vector3d ee_angvel{Eigen::Matrix<double, 3, 1>::Zero()};
  Eigen::Vector3d fingertip_pos{Eigen::Matrix<double, 3, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> hand_wrench{Eigen::Matrix<double, 6, 1>::Zero()};



  // --- FD buffers (match IsaacLab) ---
  bool fd_inited_{false};
  Eigen::Vector3d prev_fingertip_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond prev_fingertip_quat_{1.0, 0.0, 0.0, 0.0}; // w,x,y,z  

  void updateEeVelFd(const Eigen::Vector3d& fingertip_pos_now,const Eigen::Quaterniond& fingertip_quat_now,
      double dt,Eigen::Vector3d& ee_linvel_fd_out,Eigen::Vector3d& ee_angvel_fd_out);


  RlActionProcessor rl_action_processor;
  Eigen::Vector3d rl_ctrl_target_fingertip_midpoint_pos = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rl_ctrl_target_fingertip_midpoint_quat = Eigen::Quaterniond::Identity();
  double rl_ctrl_target_gripper_{0.0};

  Eigen::Matrix<double, 6, 1> poseToXyzRpy(const Eigen::Vector3d& pos,const Eigen::Quaterniond& quat);

  double rl_dt_ = 0.0667;       // 15Hz
  double rl_timer_ = 0.0;
  bool rl_target_valid_ = false;  
  double rt_infer_count = 0;

  Eigen::Vector3d target_p_3_{0.5, 0.0, 0.4};
  Eigen::Vector4d target_q_xyzw_3_{0.0, 0.0, 0.0, 1.0}; 
  Vector7d q_goal_3_{Vector7d::Zero()}; 

  pinocchio::SE3 target;
  double q_interp_alpha_ = 0.00;
  Eigen::Matrix<double, 6, 1> hand_wrench_default{Eigen::Matrix<double, 6, 1>::Zero()};


  std::string arm_id_{"fr3"};

  std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state_;
  franka_msgs::msg::FrankaRobotState franka_state_msg_;

  double tau_ext_last_joint_{0.0};
  std::array<double, 6> wrench_ext_k_{{0, 0, 0, 0, 0, 0}};
  std::array<double, 6> wrench_ext_o_{{0, 0, 0, 0, 0, 0}};
  bool tau_ext_measure = false;
  bool rl_target_collect_ = true;


  // ------------------------------
  // cached interface index maps
  // ------------------------------
  bool state_map_built_{false};
  std::array<int, 7> pos_idx_{};
  std::array<int, 7> vel_idx_{};
  std::array<int, 7> eff_idx_{};

  bool cmd_map_built_{false};
  std::array<int, 7> eff_cmd_idx_{};

  void reset_interface_maps();

  /// variable
  pinocchio::SE3 ee_real;
  Vector7d q_ref_arm{Vector7d::Zero()};
  pinocchio::SE3 ee_ref;
  pinocchio::SE3 ee_refx;
  Vector7d q_arm_rt_{Vector7d::Zero()};

  Eigen::Quaterniond q_now{1.0, 0.0, 0.0, 0.0};


  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;
  Eigen::Quaterniond ee_orientation_current_{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d ee_position_current_{Eigen::Vector3d::Zero()};

  Eigen::Vector3d ee_position_dbg_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond ee_orientation_dbg_{1.0, 0.0, 0.0, 0.0};  

  Eigen::Vector3d ee_position_msg_dbg_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond ee_orientation_msg_dbg_{1.0, 0.0, 0.0, 0.0};  
  Eigen::Matrix<double, 6, 1> hand_wrench_dbg_{Eigen::Matrix<double, 6, 1>::Zero()};
  
  rclcpp::TimerBase::SharedPtr debug_timer_;
  

  // fk test
  double ik_timer_ = 0.0;


  /// nrt loop: ik for RL test ///
  rclcpp::TimerBase::SharedPtr nrt_ik_timer_;
  
  std::atomic<bool> rl_ik_request_{false};
  std::atomic<bool> rl_ik_solution_ready_{false};
  
  Eigen::Vector3d rl_ik_target_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rl_ik_target_quat_{1.0, 0.0, 0.0, 0.0};
  Vector7d rl_ik_q_init_{Vector7d::Zero()};

  Vector7d rl_ik_q_sol_{Vector7d::Zero()};
  double rl_ik_final_err_{-1.0};
  bool rl_ik_last_ok_{false};

  void runRlIkNonRt();


  int rl_ik_request_count_{0};
  int rl_ik_success_count_{0};
  int rl_ik_fail_count_{0};
  Vector7d q_interp_start_{Vector7d::Zero()};

  /// rt print
  std::array<float, 9> action_rl_bg_{};
  bool action_rl_bg_valid_{false};

  Eigen::Vector3d rl_ctrl_target_fingertip_midpoint_pos_bg_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rl_ctrl_target_fingertip_midpoint_quat_bg_ = Eigen::Quaterniond::Identity();

  Vector7d tau_pd_osc_diff_bg_{Vector7d::Zero()};

  Eigen::Vector3d pos_err_bg_{Eigen::Vector3d::Zero()};
  double rot_err_angle_bg_ = 0.0;

  double osc_blend_alpha_gain_{0};
  std::array<double, 6> wrench_ext_k_bg_{{0, 0, 0, 0, 0, 0}};
  std::array<double, 6> wrench_ext_o_bg_{{0, 0, 0, 0, 0, 0}};


  static constexpr size_t kWrenchDim = 6;
  static constexpr size_t kBiasWindowSize = 1000;   // 1s @ 1kHz

  std::vector<std::array<double, kWrenchDim>> wrench_ext_o_buffer_;
  size_t wrench_ext_o_buffer_idx_ = 0;
  bool wrench_ext_o_buffer_full_ = false;

  std::array<double, kWrenchDim> wrench_ext_o_bias_{};


  std::array<double, kWrenchDim> computeWrenchExtOBias() const;

  rclcpp_action::Client<Grasp>::SharedPtr grasp_client_;
  rclcpp::TimerBase::SharedPtr gripper_nonrt_timer_;

  // RT -> nonRT 请求发送一次 grasp
  std::atomic<bool> request_send_grasp_{false};

  // nonRT 侧状态
  std::atomic<bool> grasp_goal_sent_{false};
  std::atomic<bool> grasp_action_active_{false};
  std::atomic<bool> grasp_action_done_{false};
  std::atomic<bool> grasp_action_success_{false};

  // 可选：防止重复打印
  std::atomic<bool> grasp_result_reported_{false};

  // grasp 参数
  double grasp_width_{0.035};
  double grasp_speed_{0.03};
  double grasp_force_{1.0};
  double grasp_epsilon_inner_{0.005};
  double grasp_epsilon_outer_{0.005};
  std::string grasp_action_name_;


  /// grasp 
  // =========================
  // RT-friendly logging
  // =========================
  std::vector<std::string> log_var_names_;
  std::vector<double> log_buffer_;   // row-major: [step0 var0..varN-1, step1 ...]
  size_t log_num_vars_{0};
  size_t log_capacity_steps_{200000};   // can change
  std::atomic<size_t> log_steps_written_{0};

  std::atomic<bool> log_started_{false};      // start after entering MOVE_POSE_1
  std::atomic<bool> log_save_done_{false};    // avoid repeated save
  std::string log_save_path_{"move_pose_1_log.txt"};

  void init_log_schema_and_buffer();
  void append_log_sample_rt();
  void save_log_to_txt_nonrt(const std::string& reason);

  std::string make_log_file_path() const;

  double log_time_start_{0.0};


  std::atomic<bool> shutting_down_{false};

  std::atomic<bool> save_requested_{false};
  std::atomic<bool> rl_finish_latched_{false};

  void save_log_snapshot_nonrt(const std::string& reason);
  std::string make_log_snapshot_file_path(int snapshot_idx) const;
  int manual_snapshot_seq_{0};

  // MOVE_POSE_1 Cartesian trajectory state
  Eigen::Vector3d move1_start_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d move1_lift_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond move1_start_quat_{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond move1_target_quat_{Eigen::Quaterniond::Identity()};

  rclcpp_action::Client<Move>::SharedPtr move_client_;

  double move_open_width_{0.04};
  double move_open_speed_{0.03};
  std::string move_action_name_{"/franka_gripper/move"};

  std::atomic<bool> request_send_move_{false};
  std::atomic<bool> move_action_done_{false};
  std::atomic<bool> move_action_success_{false};
  std::atomic<bool> move_action_active_{false};
  std::atomic<bool> move_goal_sent_{false};

  // warmup only-once latch
  std::atomic<bool> warmup_open_requested_{false};

  Eigen::Vector3d move1_hover_pos_{Eigen::Vector3d::Zero()};
  double lift_duration = 2.0;
  double hover_move_duration = 2.0;
  double descend_duration = 2.0;
    
  void save_log_autosave_nonrt(const std::string& reason);

};

}  // namespace franka_example_controllers
