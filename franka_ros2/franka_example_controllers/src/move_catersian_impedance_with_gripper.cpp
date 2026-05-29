#include "franka_example_controllers/move_catersian_impedance_with_gripper.hpp"
#include <franka_example_controllers/default_robot_behavior_utils.hpp>
#include <franka_example_controllers/robot_utils.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include <pinocchio/parsers/urdf.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <onnxruntime_cxx_api.h>
#include "franka_example_controllers/onnx_policy.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// [FT PATCH] begin
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <realtime_tools/realtime_buffer.hpp>
// [FT PATCH] end
// #include <chrono>
#include <fstream>
#include <filesystem>

namespace franka_example_controllers
{

controller_interface::CallbackReturn MoveCatersianImpWithGripper::on_init()
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
  declare_if_needed("q_goal_grasp", std::vector<double>{});
  declare_if_needed("move_duration", 10.0);
  declare_if_needed("finish_tolerance", 0.01);
  declare_if_needed("hold_position", true);

  declare_if_needed("move_duration_grasp", 10.0);
  declare_if_needed("move_duration_2", 10.0);

  // Switch between joint PD and Cartesian impedance (OSC)
  // NOTE: keep the user's original spelling: "Catersian_imp"
  declare_if_needed("Catersian_imp", false);
  declare_if_needed("osc_blend_alpha_gain", 0.0);

  // Test gravity
  declare_if_needed("test_compensation", false);

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

  // --- joint PD gains (arm 7 + gripper 1)
  // If YAML provides 7 values, the 8th (gripper) will reuse the 7th.
  declare_if_needed(
    "k_gains",
    std::vector<double>{24.0, 24.0, 24.0, 24.0, 10.0, 6.0, 2.0});
  declare_if_needed(
    "d_gains",
    std::vector<double>{2.0, 2.0, 2.0, 1.0, 1.0, 1.0, 0.5});

  // state marker
  declare_if_needed("process_finished", false);

  // NEW: warmup_seconds parameter declared here (so update() never declares/gets it)
  declare_if_needed("warmup_seconds", 0.0);

  node->get_parameter("joint_names", joint_names_);

  if (joint_names_.empty()) {
    RCLCPP_WARN(node->get_logger(),
                "joint_names is empty in on_init(); will be read again in on_configure().");
  }


  // [FT PATCH] begin
  declare_if_needed("ft_topic", std::string("/fr3/wrist_ft"));
  // [FT PATCH] end


  declare_if_needed("arm_id", std::string("fr3"));

  declare_if_needed("ft_sensor_data", false);
  declare_if_needed("RL_target_collect", false);

  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
          franka_semantic_components::FrankaCartesianPoseInterface(false));

  declare_if_needed("gripper.grasp.width", 0.035);
  declare_if_needed("gripper.grasp.speed", 0.03);
  declare_if_needed("gripper.grasp.force", 20.0);
  declare_if_needed("gripper.grasp.epsilon_inner", 0.005);
  declare_if_needed("gripper.grasp.epsilon_outer", 0.005);
  declare_if_needed("gripper.grasp.action_name", std::string("/franka_gripper/grasp"));
  
  declare_if_needed("policy_name", std::string("policy.onnx")); 

  declare_if_needed("gripper.move.width", 0.05);
  declare_if_needed("gripper.move.speed", 0.03);
  declare_if_needed("gripper.move.action_name", std::string("/franka_gripper/move"));

  declare_if_needed("pick_place.action_scale", 0.01);
  declare_if_needed("pick_place.dof_velocity_scale", 0.1);
  declare_if_needed("pick_place.r_in", 0.02);

  declare_if_needed("pick_place.finger_open_pos", 0.04);
  declare_if_needed("pick_place.finger_closed_pos", 0.0);

  declare_if_needed("pick_place.object_init_z", 0.0);
  declare_if_needed("pick_place.success_height", 0.10);

  declare_if_needed(
    "pick_place.robot_dof_lower_limit",
    std::vector<double>{
      -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
       0.0, 0.0
    });

  declare_if_needed(
    "pick_place.robot_dof_upper_limit",
    std::vector<double>{
       2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973,
       0.04, 0.04
    });

  declare_if_needed(
    "pick_place.robot_dof_speed_scales",
    std::vector<double>{
      1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
      1.0, 1.0
    });


  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MoveCatersianImpWithGripper::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& j : joint_names_) {
    cfg.names.push_back(j + "/effort");
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
MoveCatersianImpWithGripper::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& j : joint_names_) {
    cfg.names.push_back(j + "/position");
    cfg.names.push_back(j + "/velocity");
    cfg.names.push_back(j + "/effort");
  }

  cfg.names.push_back(arm_id_ + "/robot_state");

  const auto cartesian_pose_names = franka_cartesian_pose_->get_state_interface_names();
  cfg.names.insert(cfg.names.end(),
                  cartesian_pose_names.begin(),
                  cartesian_pose_names.end());

  return cfg;
}

controller_interface::CallbackReturn
MoveCatersianImpWithGripper::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  auto node = get_node();

  // Set default Franka collision behavior, following CartesianPoseExampleController.
  auto client = node->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");

  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

  const double collision_scale =1.5;

  auto scale_vec = [collision_scale](auto& values) {
    for (auto& v : values) {
      v *= collision_scale;
    }
  };

  scale_vec(request->lower_torque_thresholds_nominal);
  scale_vec(request->upper_torque_thresholds_nominal);
  scale_vec(request->lower_torque_thresholds_acceleration);
  scale_vec(request->upper_torque_thresholds_acceleration);

  scale_vec(request->lower_force_thresholds_nominal);
  scale_vec(request->upper_force_thresholds_nominal);
  scale_vec(request->lower_force_thresholds_acceleration);
  scale_vec(request->upper_force_thresholds_acceleration);



  RCLCPP_INFO(node->get_logger(),
            "Set collision behavior to %.2fx default.", collision_scale);

  auto future_result = client->async_send_request(request);
  future_result.wait_for(robot_utils::time_out);

  auto success = future_result.get();
  if (!success) {
    RCLCPP_FATAL(node->get_logger(), "Failed to set scaled collision behavior.");
    return controller_interface::CallbackReturn::ERROR;
  } else {
    // RCLCPP_INFO(node->get_logger(), "Default collision behavior set.");
    RCLCPP_INFO(node->get_logger(), "Scaled collision behavior set.");
  }


  // read params
  node->get_parameter("joint_names", joint_names_);

  
  node->get_parameter("q_goal", q_goal_vec);
  
  node->get_parameter("q_goal_grasp", q_goal_vec_grasp);


  node->get_parameter("move_duration", move_duration_);
  node->get_parameter("finish_tolerance", finish_tolerance_);
  node->get_parameter("hold_position", hold_position_);

  // OSC enable flag
  node->get_parameter("Catersian_imp", catersian_imp_);
  node->get_parameter("osc_blend_alpha_gain", osc_blend_alpha_gain_);

  node->get_parameter("test_compensation", test_compensation_);

  node->get_parameter("use_pinocchio_ik", use_pinocchio_ik_);
  node->get_parameter("robot_description_topic", robot_description_topic_);
  node->get_parameter("use_robot_hand", use_robot_hand_);
  node->get_parameter("ee_frame_override", ee_frame_override_);

  std::vector<double> target_pos, target_quat;
  node->get_parameter("target_position", target_pos);
  node->get_parameter("target_orientation_xyzw", target_quat);

  std::vector<double> target_pos2, target_quat2;
  node->get_parameter("target_position_plugin", target_pos2);
  node->get_parameter("target_orientation_xyzw_plugin", target_quat2);

  std::vector<double> target_pos2_gear;
  node->get_parameter("fixed_pos_action_frame", target_pos2_gear);


  node->get_parameter("ik.max_iters", ik_opt_.max_iters);
  node->get_parameter("ik.eps", ik_opt_.eps);
  node->get_parameter("ik.step_size", ik_opt_.step_size);
  node->get_parameter("ik.damping", ik_opt_.damping);
  node->get_parameter("ik.clamp_to_limits", ik_opt_.clamp_to_limits);
  node->get_parameter("ik.w_rot", ik_opt_.w_rot);

  // NEW: cache warmup_seconds here (update() will only use warmup_seconds_)
  node->get_parameter("warmup_seconds", warmup_seconds_);

  // Read joint PD gains from YAML
  std::vector<double> k_gains_vec, d_gains_vec;
  node->get_parameter("k_gains", k_gains_vec);
  node->get_parameter("d_gains", d_gains_vec);

  //====================================================================================================================================================
  // Added by francesco
  node->get_parameter("pick_place.action_scale", action_scale_);
  node->get_parameter("pick_place.dof_velocity_scale", dof_velocity_scale_);
  node->get_parameter("pick_place.r_in", r_in_);

  node->get_parameter("pick_place.finger_open_pos", finger_open_pos_);
  node->get_parameter("pick_place.finger_closed_pos", finger_closed_pos_);

  node->get_parameter("pick_place.object_init_z", object_init_z_);
  node->get_parameter("pick_place.success_height", success_height_);

  std::vector<double> lower_vec;
  std::vector<double> upper_vec;
  std::vector<double> speed_vec;

  node->get_parameter("pick_place.robot_dof_lower_limit", lower_vec);
  node->get_parameter("pick_place.robot_dof_upper_limit", upper_vec);
  node->get_parameter("pick_place.robot_dof_speed_scales", speed_vec);

  if (lower_vec.size() != 9 ||
      upper_vec.size() != 9 ||
      speed_vec.size() != 9)
  {
    RCLCPP_ERROR(
        node->get_logger(),
        "pick_place.robot_dof_lower_limit, upper_limit and speed_scales must have size %d.",
        9);
    return controller_interface::CallbackReturn::ERROR;
  }

  for (int i = 0; i < 9; ++i)
  {
    robot_dof_lower_limit_[i] = lower_vec[i];
    robot_dof_upper_limit_[i] = upper_vec[i];
    robot_dof_speed_scales_[i] = speed_vec[i];
  }

  clear_obs_.fill(0.0f);
  //====================================================================================================================================================

  auto fill_gains = [node](const std::vector<double>& src, Vector7d& dst, const char* name) {
    if (src.size() == 7) {
      for (size_t i = 0; i < 7; ++i) {
        dst(static_cast<int>(i)) = src[i];
      }
      // // gripper uses the last (7th) value
      // dst(7) = src[6];
    } else {
      RCLCPP_ERROR(
        node->get_logger(),
        "%s must have 7 (arm) values, got %zu",
        name, src.size());
      throw std::runtime_error("invalid gain vector size");
    }
  };

  try {
    fill_gains(k_gains_vec, k_gains_, "k_gains");
    fill_gains(d_gains_vec, d_gains_, "d_gains");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "gain parse failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Require 7 joints
  if (joint_names_.size() != 7) {
    RCLCPP_ERROR(node->get_logger(), "joint_names must have 7 joints, got %zu", joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Require 7 q_goal values
  if (q_goal_vec.size() != 7) {
    RCLCPP_ERROR(node->get_logger(), "q_goal must have exactly 7 values (7 arm + 0 gripper), got %zu", q_goal_vec.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  // if (q_goal_vec_grasp.size() != 1) {
  //   RCLCPP_ERROR(node->get_logger(), "q_goal must have exactly 1 values (1 gripper), got %zu", q_goal_vec_grasp.size());
  //   return controller_interface::CallbackReturn::ERROR;
  // }

  for (size_t i = 0; i < 7; ++i) {
    q_goal_(static_cast<int>(i)) = q_goal_vec[i];
  }
  q_goal_2_ = q_goal_;
  q_goal_3_ = q_goal_;
  q_des_    = q_goal_;

  q_interp_start_ = q_goal_;



  if (target_pos.size() == 3) {
    target_p_ = Eigen::Vector3d(target_pos[0], target_pos[1], target_pos[2]);
  }
  else{
    RCLCPP_WARN(node->get_logger(), "target_position size != 3, keep previous/default.");
  }
  if (target_quat.size() == 4) {
    target_q_xyzw_ = Eigen::Vector4d(target_quat[0], target_quat[1], target_quat[2], target_quat[3]);
  }
  else
  {
    RCLCPP_WARN(node->get_logger(), "target_orientation_xyzw size != 4, keep previous/default.");
  }

  
  if (target_pos2.size() == 3) {
    target_p_2_ = Eigen::Vector3d(target_pos2[0], target_pos2[1], target_pos2[2]);
  }
  else{
    RCLCPP_WARN(node->get_logger(), "target_position (before plugin) size != 3, keep previous/default.");
  }
  if (target_quat2.size() == 4) {
    target_q_xyzw_2_ = Eigen::Vector4d(target_quat2[0], target_quat2[1], target_quat2[2], target_quat2[3]);
  }
  else
  {
    RCLCPP_WARN(node->get_logger(), "target_orientation_xyzw (before plugin) size != 4, keep previous/default.");
  }  

  target_p_3_ = target_p_2_;
  target_q_xyzw_3_ = target_q_xyzw_2_;

  rl_ctrl_target_fingertip_midpoint_pos = target_p_3_;
  rl_ctrl_target_fingertip_midpoint_quat.x() = target_q_xyzw_2_[0];
  rl_ctrl_target_fingertip_midpoint_quat.y() = target_q_xyzw_2_[1];
  rl_ctrl_target_fingertip_midpoint_quat.z() = target_q_xyzw_2_[2];  
  rl_ctrl_target_fingertip_midpoint_quat.w() = target_q_xyzw_2_[3];




  if (target_pos2_gear.size() == 3) {
    target_p_gear_fixed_ = Eigen::Vector3d(target_pos2_gear[0], target_pos2_gear[1], target_pos2_gear[2]);
  }
  else{
    RCLCPP_WARN(node->get_logger(), "target_position (gear bolt) size != 3, keep previous/default.");
  }


  // Subscribe robot_description (URDF string) via topic
  urdf_received_.store(false);
  {
    std::lock_guard<std::mutex> lk(urdf_mutex_);
    urdf_xml_.clear();
  }
  robot_description_sub_.reset();
  // ===============================

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
      // NOTE: this callback is NOT RT; logging here is fine.
      RCLCPP_INFO(get_node()->get_logger(), "Received robot_description from topic '%s' (len=%zu)",
                  robot_description_topic_.c_str(), msg->data.size());
    });

  // reset
  elapsed_time_ = 0.0;
  move_1_finished_ = true;
  grasp_finished_ = true;
  kin_initialized_ = false;

  node->get_parameter("move_duration_grasp", move_duration_grasp_);
  node->get_parameter("move_duration_2", move_duration_2_);
  node->get_parameter("RL_duration", rl_duration_);
  node->get_parameter("RL_test", rl_test_);
  node->get_parameter("ft_sensor_data", tau_ext_measure); 
  node->get_parameter("RL_target_collect", rl_target_collect_); 

  std::cout<<"=====================Is RL_target_collect mode? "<<rl_target_collect_<<std::endl;

  // reset OSC state
  osc_initialized_ = false;
  osc_.reset();

  // NEW: ensure marker reset in non-RT
  get_node()->set_parameter(rclcpp::Parameter("process_finished", false));
  request_set_process_finished_param_.store(false);

  // NEW: reset RT flags
  rt_missing_warmup_eff_joint_.store(-1);
  rt_missing_eff_joint_.store(-1);
  rt_missing_state_joint_.store(-1);
  rt_move_finished_event_.store(false);
  rt_cmd_map_ready_.store(false);

  // NEW: reset warmup runtime state (so lifecycle restart behaves predictably)
  warmup_inited_ = false;
  warmup_elapsed_ = 0.0;
  warmup_q_hold_.setZero();
  warmup_cmd_map_built_ = false;
  warmup_eff_cmd_idx_.fill(-1);

  gait_data_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>("franka3_data", 10);

  joint2simulation_.position.resize(100, 0.0);
  joint2simulation_.velocity.resize(100, 0.0);

  joint2simulation_.name.resize(100);
  for (int i = 0; i < 100; ++i) {
    joint2simulation_.name[i] = "dbg_" + std::to_string(i);
  }

  // NEW: non-RT housekeeping timer:
  // - handle set_parameter(process_finished=true) requested by RT
  // - print warnings/errors that RT detected (without logging in RT)
  nonrt_housekeeping_timer_.reset();
  nonrt_housekeeping_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() {

      // periodic autosave snapshot every 0.5 s, non-RT
      static int autosave_counter = 0;
      autosave_counter++;

      if (autosave_counter >= 5) {  // 100 ms * 5 = 0.5 s
        autosave_counter = 0;

        if (log_started_.load(std::memory_order_acquire) &&
            !log_save_done_.load(std::memory_order_acquire)) {
          try {
            save_log_autosave_nonrt("autosave");
          } catch (const std::exception& e) {
            std::cerr << "[log] exception in autosave: " << e.what() << std::endl;
          } catch (...) {
            std::cerr << "[log] unknown exception in autosave" << std::endl;
          }
        }
      }




      // auto save after RL finished
      if (save_requested_.exchange(false, std::memory_order_acq_rel)) {
        try {
          save_log_to_txt_nonrt("rl_finished");
        } catch (const std::exception& e) {
          std::cerr << "[log] exception in auto save after rl_finished: "
                    << e.what() << std::endl;
        } catch (...) {
          std::cerr << "[log] unknown exception in auto save after rl_finished"
                    << std::endl;
        }
      }  

      // 1) handle process_finished parameter setting (non-RT)
      if (request_set_process_finished_param_.exchange(false)) {
        get_node()->set_parameter(rclcpp::Parameter("process_finished", true));
        RCLCPP_INFO(get_node()->get_logger(), "Move-to-start finished. (process_finished parameter set true in non-RT)");
      }

      // 2) report RT-detected mapping/state errors (non-RT)
      const int miss_warmup = rt_missing_warmup_eff_joint_.exchange(-1);
      if (miss_warmup >= 0 && miss_warmup < static_cast<int>(joint_names_.size())) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "warmup command iface map error: cannot find '%s/effort'",
                     joint_names_[static_cast<size_t>(miss_warmup)].c_str());
      }

      const int miss_cmd = rt_missing_eff_joint_.exchange(-1);
      if (miss_cmd >= 0 && miss_cmd < static_cast<int>(joint_names_.size())) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "command iface map error: cannot find '%s/effort'",
                     joint_names_[static_cast<size_t>(miss_cmd)].c_str());
      }

      const int miss_state = rt_missing_state_joint_.exchange(-1);
      if (miss_state >= 0 && miss_state < static_cast<int>(joint_names_.size())) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "read_state(): cannot find state interfaces for joint '%s' (pos/vel/eff). Check joint names and interfaces.",
                     joint_names_[static_cast<size_t>(miss_state)].c_str());
      }

      // 3) optional: log warmup_seconds once, after warmup init happened in RT
      if (warmup_inited_ && rt_move_finished_event_.exchange(false)) {
        // (reserved) - currently used as a generic edge event flag; keep minimal
      }
      // ==============================
      // [FT PATCH] 1 Hz print wrench (NON-RT SAFE)
      // ==============================
      static int print_counter = 0;
      print_counter++;

      // if (print_counter >= 100)  // 1ms * 1000 = 1s
      // {
      //   print_counter = 0;

      //   // 推荐：从 RT buffer 读一次（确保真的收到过消息）
      //   auto msg_ptr = *(ft_rt_buffer_.readFromRT());
      //   if (msg_ptr) {
      //     RCLCPP_INFO(
      //       get_node()->get_logger(),
      //       "[FT] Fx=%.3f Fy=%.3f Fz=%.3f  Tx=%.3f Ty=%.3f Tz=%.3f",
      //       msg_ptr->wrench.force.x,  msg_ptr->wrench.force.y,  msg_ptr->wrench.force.z,
      //       msg_ptr->wrench.torque.x, msg_ptr->wrench.torque.y, msg_ptr->wrench.torque.z);
      //   } else {
      //     RCLCPP_WARN(get_node()->get_logger(), "[FT] no wrench msg yet on %s", ft_topic_.c_str());
      //   }
      // }

    });

  task_cmd_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
    "/task_cmd",
    rclcpp::QoS(10).best_effort(),   // 或 keep_last(10) reliable，看你需求
    [this](const std_msgs::msg::String::SharedPtr msg) {
      // 非RT线程：允许字符串比较
      const auto & s = msg->data;

      TaskCmd cmd = TaskCmd::NONE;
      if (s == "move_pose_1") cmd = TaskCmd::MOVE_POSE_1;
      else if (s == "move_pose_2") cmd = TaskCmd::MOVE_POSE_2;
      else if (s == "grasp") cmd = TaskCmd::GRASP;
      else if (s == "open_gripper") cmd = TaskCmd::OPEN_GRIPPER;
      else if (s == "close_gripper") cmd = TaskCmd::CLOSE_GRIPPER;
      else if (s == "run_rl") cmd = TaskCmd::RUN_RL;
      else if (s == "save_log") cmd = TaskCmd::SAVE_LOG;
      else if (s == "stop") cmd = TaskCmd::STOP;

      if (cmd == TaskCmd::SAVE_LOG) {
        try {
          save_log_snapshot_nonrt("manual_save");
        } catch (const std::exception& e) {
          std::cerr << "[log] exception in manual snapshot save: " << e.what() << std::endl;
        } catch (...) {
          std::cerr << "[log] unknown exception in manual snapshot save" << std::endl;
        }
        return;
      }

      task_cmd_.store(static_cast<int>(cmd), std::memory_order_relaxed);
      task_cmd_new_.store(true, std::memory_order_release);
    });

  allow_move_pose_1 = false;


  ///================ load the NN model =====================///
  // Ort::Env ort_env(ORT_LOGGING_LEVEL_WARNING, "test");
  node->get_parameter("policy_name", policy_name_);
  try {
    auto share = ament_index_cpp::get_package_share_directory("franka_example_controllers");
    std::string onnx_path = share + "/models/" + policy_name_;
    policy_ = std::make_unique<franka_example_controllers::OnnxPolicy>(onnx_path);
    RCLCPP_INFO(node->get_logger(), "Loaded ONNX policy: %s (in=%s, out=%s)",
                onnx_path.c_str(), policy_->input_name().c_str(), policy_->output_name().c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load ONNX policy: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  rl_obs_.fill(0.0f);
  rl_prev_action_.fill(0.0f);
  ///-------------------------


  // [FT PATCH] begin
  node->get_parameter("ft_topic", ft_topic_);

  // reset on reconfigure
  ft_sub_.reset();
  ft_rt_buffer_.writeFromNonRT(geometry_msgs::msg::WrenchStamped::SharedPtr());

  // Use SensorDataQoS for high-rate sensor stream
  auto ft_qos = rclcpp::SensorDataQoS();

  ft_sub_ = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
    ft_topic_, ft_qos,
    [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
      // non-RT callback
      ft_rt_buffer_.writeFromNonRT(msg);
    });
  // [FT PATCH] end


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

  reset_interface_maps();


  wrench_ext_o_buffer_.resize(kBiasWindowSize);
  for (auto& v : wrench_ext_o_buffer_) {
    v.fill(0.0);
  }
  wrench_ext_o_bias_.fill(0.0);



  debug_timer_ = get_node()->create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() {
        const Eigen::Vector3d dp = ee_position_dbg_ - ee_position_msg_dbg_;
        const double qdot = std::abs(
            ee_orientation_dbg_.w() * ee_orientation_msg_dbg_.w() +
            ee_orientation_dbg_.x() * ee_orientation_msg_dbg_.x() +
            ee_orientation_dbg_.y() * ee_orientation_msg_dbg_.y() +
            ee_orientation_dbg_.z() * ee_orientation_msg_dbg_.z());


        RCLCPP_INFO(
            get_node()->get_logger(),
            "ee mesure: pos = [%.4f, %.4f, %.4f], quat(wxyz) = [%.4f, %.4f, %.4f, %.4f]",
            ee_position_dbg_.x(), ee_position_dbg_.y(), ee_position_dbg_.z(),
            ee_orientation_dbg_.w(), ee_orientation_dbg_.x(),
            ee_orientation_dbg_.y(), ee_orientation_dbg_.z());

        // RCLCPP_INFO(
        //     get_node()->get_logger(),
        //     "o_t_ee:    pos = [%.4f, %.4f, %.4f], quat(wxyz) = [%.4f, %.4f, %.4f, %.4f]",
        //     ee_position_msg_dbg_.x(), ee_position_msg_dbg_.y(), ee_position_msg_dbg_.z(),
        //     ee_orientation_msg_dbg_.w(), ee_orientation_msg_dbg_.x(),
        //     ee_orientation_msg_dbg_.y(), ee_orientation_msg_dbg_.z());

        // RCLCPP_INFO(
        //     get_node()->get_logger(),
        //     "wrench_ext_k_ = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
        //     wrench_ext_k_bg_[0],
        //     wrench_ext_k_bg_[1],
        //     wrench_ext_k_bg_[2],
        //     wrench_ext_k_bg_[3],
        //     wrench_ext_k_bg_[4],
        //     wrench_ext_k_bg_[5]); 

        // RCLCPP_INFO(
        //     get_node()->get_logger(),
        //     "wrench_ext_o_ = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
        //     wrench_ext_o_bg_[0],
        //     wrench_ext_o_bg_[1],
        //     wrench_ext_o_bg_[2],
        //     wrench_ext_o_bg_[3],
        //     wrench_ext_o_bg_[4],
        //     wrench_ext_o_bg_[5]);             

        // RCLCPP_INFO(
        //     get_node()->get_logger(),
        //     "cart_pose vs o_t_ee: dp = [%.6f, %.6f, %.6f], |dp| = %.6f, quat_dot = %.6f",
        //     dp.x(), dp.y(), dp.z(), dp.norm(), qdot);  
        
        // RCLCPP_INFO(
        //     get_node()->get_logger(),
        //     "estimated hand wrench = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
        //     hand_wrench_dbg_(0), hand_wrench_dbg_(1),
        //     hand_wrench_dbg_(2), hand_wrench_dbg_(3),
        //     hand_wrench_dbg_(4), hand_wrench_dbg_(5));           

        // RCLCPP_INFO(
        //     get_node()->get_logger(),
        //     "RL IK stats: request=%d, success=%d, fail=%d",
        //     rl_ik_request_count_,
        //     rl_ik_success_count_,
        //     rl_ik_fail_count_);
        
        if(action_rl_bg_valid_)
        {
          // RCLCPP_INFO(
          //     get_node()->get_logger(),
          //     "==rl  action_ = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
          //     action_rl_bg_[0],
          //     action_rl_bg_[1],
          //     action_rl_bg_[2],
          //     action_rl_bg_[3],
          //     action_rl_bg_[4],
          //     action_rl_bg_[5]);

          RCLCPP_INFO(
              get_node()->get_logger(),
              "DEBUG RL FRAME:\n"
              "fingertip_pos      = [%.4f, %.4f, %.4f]\n"
              "fixed_pos_obs_frame= [%.4f, %.4f, %.4f]\n"
              "rel_fixed          = [%.4f, %.4f, %.4f]",
              fingertip_pos.x(), fingertip_pos.y(), fingertip_pos.z(),
              target_p_gear_fixed_.x(), target_p_gear_fixed_.y(), target_p_gear_fixed_.z(),
              fingertip_pos_rel_fixed.x(), fingertip_pos_rel_fixed.y(), fingertip_pos_rel_fixed.z()
          );


          RCLCPP_INFO(
              get_node()->get_logger(),
              "===rl output taraget pos[xyz]=[%.4f, %.4f, %.4f], quat[xyzw]=[%.4f, %.4f, %.4f, %.4f]",
              rl_ctrl_target_fingertip_midpoint_pos_bg_.x(),
              rl_ctrl_target_fingertip_midpoint_pos_bg_.y(),
              rl_ctrl_target_fingertip_midpoint_pos_bg_.z(),
              rl_ctrl_target_fingertip_midpoint_quat_bg_.x(),
              rl_ctrl_target_fingertip_midpoint_quat_bg_.y(),
              rl_ctrl_target_fingertip_midpoint_quat_bg_.z(),
              rl_ctrl_target_fingertip_midpoint_quat_bg_.w()); 
              
          //===========================================================================================================================================
          RCLCPP_INFO(
            get_node()->get_logger(),
            "====DEBUG JOINT ANGLES ======\n"
            "q_               = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n"
            "q_des            = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n"
            "err_             = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n"
            "fingertip_pos    = [%.4f, %.4f, %.4f]\n"
            "fingertip_pos_des= [%.4f, %.4f, %.4f]\n"
            "fingertip_err    = [%.4f, %.4f, %.4f]",
            q_(0), q_(1), q_(2), q_(3), q_(4), q_(5), q_(6),
            q_des_(0), q_des_(1), q_des_(2), q_des_(3), q_des_(4), q_des_(5), q_des_(6),
            q_(0) - q_des_(0), q_(1) - q_des_(1), q_(2) - q_des_(2), q_(3) - q_des_(3), q_(4) - q_des_(4), q_(5) - q_des_(5), q_(6) - q_des_(6),
            fingertip_pos.x(), fingertip_pos.y(), fingertip_pos.z(),
            fingertip_pos_fk_dbg_.x(), fingertip_pos_fk_dbg_.y(), fingertip_pos_fk_dbg_.z(),
            fingertip_pos.x()-fingertip_pos_fk_dbg_.x(), fingertip_pos.y()-fingertip_pos_fk_dbg_.y(), fingertip_pos.z()-fingertip_pos_fk_dbg_.z()
          );
          //==========================================================================================================================================
        }

        // RCLCPP_INFO(
        //     get_node()->get_logger(),
        //     "tau_pd_osc_diff_bg_ = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
        //     tau_pd_osc_diff_bg_(0),
        //     tau_pd_osc_diff_bg_(1),
        //     tau_pd_osc_diff_bg_(2),
        //     tau_pd_osc_diff_bg_(3),
        //     tau_pd_osc_diff_bg_(4),
        //     tau_pd_osc_diff_bg_(5),
        //     tau_pd_osc_diff_bg_(6));        
          
        // RCLCPP_INFO(
        //     get_node()->get_logger(),
        //     "ee_err: pos_err = [%.6f, %.6f, %.6f], |pos_err| = %.6f, rot_err = %.6f rad",
        //     pos_err_bg_.x(),
        //     pos_err_bg_.y(),
        //     pos_err_bg_.z(),
        //     pos_err_bg_.norm(),
        //     rot_err_angle_bg_);

      });

  // in the release build mode, do not use it
  // nrt_ik_timer_ = get_node()->create_wall_timer(
  //     std::chrono::milliseconds(50),   // 20 Hz
  //     std::bind(&MoveCatersianImpWithGripper::runRlIkNonRt, this));

  // nrt_ik_timer_ = get_node()->create_wall_timer(
  //     std::chrono::microseconds(33333),   // ~30 Hz
  //     std::bind(&MoveCatersianImpWithGripper::runRlIkNonRt, this));

  node->get_parameter("gripper.grasp.width", grasp_width_);
  node->get_parameter("gripper.grasp.speed", grasp_speed_);
  node->get_parameter("gripper.grasp.force", grasp_force_);
  node->get_parameter("gripper.grasp.epsilon_inner", grasp_epsilon_inner_);
  node->get_parameter("gripper.grasp.epsilon_outer", grasp_epsilon_outer_);


  node->get_parameter("gripper.grasp.action_name", grasp_action_name_);

  grasp_client_ = rclcpp_action::create_client<Grasp>(node, grasp_action_name_);


  node->get_parameter("gripper.move.width", move_open_width_);
  node->get_parameter("gripper.move.speed", move_open_speed_);
  node->get_parameter("gripper.move.action_name", move_action_name_);

  move_client_ = rclcpp_action::create_client<Move>(node, move_action_name_);  
    
    


  gripper_nonrt_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(20),
    [this]() {
      
      if (request_send_move_.exchange(false)) {
        if (!move_client_) {
          RCLCPP_ERROR(get_node()->get_logger(), "move_client_ is null");
          move_action_done_.store(true);
          move_action_success_.store(false);
          move_action_active_.store(false);
          return;
        }

        if (!move_client_->wait_for_action_server(std::chrono::milliseconds(1))) {
          RCLCPP_WARN(get_node()->get_logger(), "%s action server not available", move_action_name_.c_str());
          request_send_move_.store(true);
          return;
        }

        Move::Goal goal;
        goal.width = move_open_width_;
        goal.speed = move_open_speed_;

        move_goal_sent_.store(true);
        move_action_active_.store(true);
        move_action_done_.store(false);
        move_action_success_.store(false);

        rclcpp_action::Client<Move>::SendGoalOptions options;

        options.goal_response_callback =
          [this](const MoveGoalHandle::SharedPtr & goal_handle) {
            if (!goal_handle) {
              RCLCPP_ERROR(get_node()->get_logger(), "Move goal rejected");
              move_action_active_.store(false);
              move_action_done_.store(true);
              move_action_success_.store(false);
            } else {
              RCLCPP_INFO(get_node()->get_logger(), "Move goal accepted");
            }
          };

        options.result_callback =
          [this](const MoveGoalHandle::WrappedResult & result) {
            move_action_active_.store(false);
            move_action_done_.store(true);

            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
              move_action_success_.store(true);
              RCLCPP_INFO(get_node()->get_logger(), "Move finished successfully");
            } else {
              move_action_success_.store(false);
              RCLCPP_WARN(
                get_node()->get_logger(),
                "Move action finished with code %d",
                static_cast<int>(result.code));
            }
          };

        move_client_->async_send_goal(goal, options);
        return;
      }      
           
      if (!request_send_grasp_.exchange(false)) {
        return;
      }

      if (!grasp_client_) {
        RCLCPP_ERROR(get_node()->get_logger(), "grasp_client_ is null");
        grasp_action_done_.store(true);
        grasp_action_success_.store(false);
        grasp_action_active_.store(false);
        return;
      }

      if (!grasp_client_->wait_for_action_server(std::chrono::milliseconds(1))) {
        RCLCPP_WARN(get_node()->get_logger(),"%s action server not available",grasp_action_name_.c_str());
        // 这里不要阻塞太久
        request_send_grasp_.store(true);  // 下次 timer 再试
        return;
      }

      Grasp::Goal goal;
      goal.width = grasp_width_;
      goal.speed = grasp_speed_;
      goal.force = grasp_force_;
      goal.epsilon.inner = grasp_epsilon_inner_;
      goal.epsilon.outer = grasp_epsilon_outer_;

      grasp_goal_sent_.store(true);
      grasp_action_active_.store(true);
      grasp_action_done_.store(false);
      grasp_action_success_.store(false);
      grasp_result_reported_.store(false);

      rclcpp_action::Client<Grasp>::SendGoalOptions options;

      options.goal_response_callback =
        [this](const GraspGoalHandle::SharedPtr & goal_handle) {
          if (!goal_handle) {
            RCLCPP_ERROR(get_node()->get_logger(), "Grasp goal rejected");
            grasp_action_active_.store(false);
            grasp_action_done_.store(true);
            grasp_action_success_.store(false);
          } else {
            RCLCPP_INFO(get_node()->get_logger(), "Grasp goal accepted");
          }
        };

      options.result_callback =
        [this](const GraspGoalHandle::WrappedResult & result) {
          grasp_action_active_.store(false);
          grasp_action_done_.store(true);

          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            // franka_msgs/action/Grasp 的 result.success 才是抓取是否真正成功
            grasp_action_success_.store(result.result->success);
            RCLCPP_INFO(
              get_node()->get_logger(),
              "Grasp finished. success=%d",
              static_cast<int>(result.result->success));
          } else {
            grasp_action_success_.store(false);
            RCLCPP_WARN(
              get_node()->get_logger(),
              "Grasp action finished with code %d",
              static_cast<int>(result.code));
          }
        };

      grasp_client_->async_send_goal(goal, options);
    }
  );

  

  
  
  
  
  
  // =========================
  // Logging init (non-RT)
  // =========================
  init_log_schema_and_buffer();
  manual_snapshot_seq_ = 0;


  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MoveCatersianImpWithGripper::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{

  reset_interface_maps();
  // Read initial state
  if (!read_state()) {
    RCLCPP_ERROR(get_node()->get_logger(), "read_state() failed in on_activate()");
    return controller_interface::CallbackReturn::ERROR;
  }
  q_start_ = q_;
  dq_filtered_.setZero();

  // If Cartesian impedance enabled, init Pinocchio (FK) + OSC model (non-RT)
  const bool ok_fk = init_pinocchio_if_needed_with_wait();
  //==================================================================================================================================================
  if (ok_fk) {initProbeFrameIds();}
  //==================================================================================================================================================
  const bool ok_osc = init_cartesian_osc_if_needed_with_wait();
  if (!ok_fk || !ok_osc) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Pinocchio/OSC init failed. Will fallback to joint PD torques.");
  }

  // logging state
  log_elapsed_time_ = 0.0;
  printed_last_loop_ = false;

  // warmup runtime state reset on activate (non-RT)
  warmup_inited_ = false;
  warmup_elapsed_ = 0.0;
  warmup_cmd_map_built_ = false;
  warmup_eff_cmd_idx_.fill(-1);

  // compute q_goal_ and q_goal_2_ via Pinocchio IK (arm only: first 7 joints)?
  {
    if (ok_fk) 
    {
      /// target 1: for grasping 
      target = make_target_se3(1);
    
      if (use_pinocchio_ik_)
      {
        q_init_arm = q_.head<7>();
        q_sol = q_init_arm;
        double final_err = -1.0;

        const auto t0 = std::chrono::steady_clock::now();
        bool ok = pino_.ik(target, q_init_arm, q_sol, ik_opt_, &final_err);
        const auto t1 = std::chrono::steady_clock::now();

        const double ik_elapsed_ms =
            std::chrono::duration<double, std::milli>(t1 - t0).count();        
        if (!ok) {
          RCLCPP_WARN(get_node()->get_logger(),
                      "Pinocchio IK(target1) failed (final_err=%.6f), elapsed=%.3f ms",
                      final_err, ik_elapsed_ms);
        } else {
          q_goal_.head<7>() = q_sol;
          RCLCPP_INFO(get_node()->get_logger(),
                      "Pinocchio IK(target1) success, elapsed=%.3f ms",
                      ik_elapsed_ms);
        }
      }
      else {
        RCLCPP_INFO(get_node()->get_logger(),
                    "use_pinocchio_ik=false; skip IK and use configured q_goal.");
      }

      /// target 2: for plugin
      target = make_target_se3(2);
    
      if (use_pinocchio_ik_)
      {
        q_init_arm = q_.head<7>();
        q_sol = q_init_arm;
        double final_err2 = -1.0;

        const auto t2 = std::chrono::steady_clock::now();
        bool ok2 = pino_.ik(target, q_init_arm, q_sol, ik_opt_, &final_err2);
        const auto t3 = std::chrono::steady_clock::now();

        const double ik_elapsed_ms_2 =
            std::chrono::duration<double, std::milli>(t3 - t2).count();


        if (!ok2) {
          RCLCPP_WARN(get_node()->get_logger(),
                      "Pinocchio IK(target2) failed (final_err=%.6f), elapsed=%.3f ms",
                      final_err2, ik_elapsed_ms_2);
        } else {
          q_goal_2_.head<7>() = q_sol;
          q_goal_3_ = q_goal_2_;
          q_interp_start_ = q_goal_2_;
          RCLCPP_INFO(get_node()->get_logger(),
                      "Pinocchio IK(target2) success, elapsed=%.3f ms",
                      ik_elapsed_ms_2);
        }
      }
      else {
        RCLCPP_INFO(get_node()->get_logger(),
                    "use_pinocchio_ik=false; skip IK and use configured q_goal.");
      }



    } else {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Pinocchio init not ready (URDF not received). Using configured q_goal directly.");
    }
  }

  last_cmd_.store(static_cast<int>(TaskCmd::NONE), std::memory_order_relaxed);
  task_cmd_.store(static_cast<int>(TaskCmd::NONE), std::memory_order_relaxed); // 可选，但推荐
  

  if (!franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                "Failed to assign franka robot state interface.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                "Failed to assign franka cartesian pose interface.");
    return controller_interface::CallbackReturn::ERROR;
  }


  warmup_open_requested_.store(false, std::memory_order_relaxed);
  request_send_move_.store(false, std::memory_order_relaxed);
  move_goal_sent_.store(false, std::memory_order_relaxed);
  move_action_done_.store(false, std::memory_order_relaxed);
  move_action_success_.store(false, std::memory_order_relaxed);
  move_action_active_.store(false, std::memory_order_relaxed);



  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MoveCatersianImpWithGripper::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{

  reset_interface_maps();
  // stop timer (non-RT)
  nonrt_housekeeping_timer_.reset();


  if (franka_robot_state_) {
    franka_robot_state_->release_interfaces();
  }

  warmup_inited_ = false;
  warmup_elapsed_ = 0.0;
  warmup_cmd_map_built_ = false;
  warmup_eff_cmd_idx_.fill(-1);


  if (franka_cartesian_pose_) {
    franka_cartesian_pose_->release_interfaces();
  }  

  gripper_nonrt_timer_.reset();


  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
MoveCatersianImpWithGripper::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  // cmd: latest command from subscriber
  const TaskCmd cmd  = static_cast<TaskCmd>(task_cmd_.load(std::memory_order_relaxed));
  // last cmd : last effective mode used by RT loop
  const TaskCmd last = static_cast<TaskCmd>(last_cmd_.load(std::memory_order_relaxed));

  // This is the mode we actually execute in THIS control cycle
  TaskCmd effective_cmd = last;

  // ------------------------------------------------------------
  // Gate + Lock policy:
  //
  // (A1) Entry gate: MOVE_POSE_1 can ONLY be entered from NONE (at the switching edge).
  // (B1) Lock: once in MOVE_POSE_1, ignore ANY other commands until move_1_finished_==true.
  //
  // (A2) Entry gate: GRASP can ONLY be entered if last==MOVE_POSE_1 AND move_1_finished_==true (at switching edge).
  // (B2) Lock: once in GRASP, ignore ANY other commands until grasp_finished_==true.
  //
  // (A3) Entry gate: MOVE_POSE_2 can ONLY be entered if last==GRASP AND grasp_finished_==true (at switching edge).
  // (B3) Lock: once in MOVE_POSE_2, ignore ANY other commands until move_2_finished_==true.
  
  // (A4) Entry gate: RUN_RL can ONLY be entered if last==MOVE_POSE_2 AND move_2_finished_==true (at switching edge).
  // (B4) Lock: once in RUN_RL, ignore ANY other commands until rl_finished_==true.
  // ------------------------------------------------------------
  
  // (B4) Lock : 
  if (last == TaskCmd::RUN_RL && !rl_finished_)   
  {
    effective_cmd = TaskCmd::RUN_RL;
  }
  // (B3) Lock : if currently in MOVE_POSE_2 and not finished, force stay in MOVE_POSE_2
  else if (last == TaskCmd::MOVE_POSE_2 && !move_2_finished_) 
  {
    effective_cmd = TaskCmd::MOVE_POSE_2;
  } 
  // (B2) Lock: if currently in GRASP and not finished, force stay in GRASP
  else if (last == TaskCmd::GRASP && !grasp_finished_) 
  {
    effective_cmd = TaskCmd::GRASP;
  } 
  // (B1) Lock: if currently in MOVE_POSE_1 and not finished, force stay in MOVE_POSE_1
  else if (last == TaskCmd::MOVE_POSE_1 && !move_1_finished_) 
  {
    effective_cmd = TaskCmd::MOVE_POSE_1;
  } 
  else 
  {
    // Not locked -> handle switching edge rules
    if (cmd != last) 
    {
      // (A1) Entry gate for MOVE_POSE_1
      if (cmd == TaskCmd::MOVE_POSE_1) 
      {
        if (last != TaskCmd::NONE) 
        {
          // Reject entering MOVE_POSE_1 from non-NONE: keep last mode
          effective_cmd = last;
        } 
        else 
        {
          // Accept entering MOVE_POSE_1 from NONE: one-time init
          q_start_ = q_;
          elapsed_time_ = 0.0;
          move_1_finished_ = false;
          effective_cmd = TaskCmd::MOVE_POSE_1;

          // initialize Cartesian move-pose-1 trajectory
          if (kin_initialized_) {
            const pinocchio::SE3 ee_start = pino_.fk(q_.head<7>());
            move1_start_pos_ = ee_start.translation();
            move1_lift_pos_ = move1_start_pos_;
            move1_lift_pos_.z() += 0.05;   // lift 5 cm

            Eigen::Quaterniond q_start(ee_start.rotation());
            q_start.normalize();
            move1_start_quat_ = q_start;

            Eigen::Quaterniond q_target(
                target_q_xyzw_[3],
                target_q_xyzw_[0],
                target_q_xyzw_[1],
                target_q_xyzw_[2]);   // w,x,y,z
            q_target.normalize();

            // shortest-path slerp
            if (move1_start_quat_.dot(q_target) < 0.0) {
              q_target.coeffs() *= -1.0;
            }

            move1_target_quat_ = q_target;

            move1_hover_pos_ = target_p_;
            move1_hover_pos_.z() += 0.05;

          } else {
            move1_start_pos_.setZero();
            move1_lift_pos_.setZero();
            move1_lift_pos_.z() = 0.1;

            move1_start_quat_.setIdentity();

            Eigen::Quaterniond q_target(
                target_q_xyzw_[3],
                target_q_xyzw_[0],
                target_q_xyzw_[1],
                target_q_xyzw_[2]);   // w,x,y,z
            q_target.normalize();

            if (move1_start_quat_.dot(q_target) < 0.0) {
              q_target.coeffs() *= -1.0;
            }

            move1_target_quat_ = q_target;
          }




        }
      }
      // (A2) Entry gate for GRASP
      else if (cmd == TaskCmd::GRASP)
      {
        // Only allow GRASP right after MOVE_POSE_1 has finished
        if (!(last == TaskCmd::MOVE_POSE_1 && move_1_finished_))
        {
          // Reject entering GRASP unless MOVE_POSE_1 finished
          effective_cmd = last;
        }
        else
        {
          // Accept entering GRASP: one-time init for grasp stage
          q_start_ = q_des_; 
          elapsed_time_ = 0.0;
          grasp_finished_ = false;  // 进入即未完成 -> 触发 lock

          request_send_grasp_.store(true);
          grasp_goal_sent_.store(false);
          grasp_action_active_.store(false);
          grasp_action_done_.store(false);
          grasp_action_success_.store(false);
          grasp_result_reported_.store(false);


          effective_cmd = TaskCmd::GRASP;
        }
      }
      // (A3) Entry gate for MOVE_POSE_2 (manual switching after grasp finished)
      else if (cmd == TaskCmd::MOVE_POSE_2)
      {
        // Only allow MOVE_POSE_2 right after GRASP has finished
        if (!(last == TaskCmd::GRASP && grasp_finished_))
        {
          // Reject entering MOVE_POSE_2 unless GRASP finished
          effective_cmd = last;
        }
        else
        {
          // Accept entering MOVE_POSE_2: one-time init for move2 stage
          // q_start_ = q_des_;
          // elapsed_time_ = 0.0;
          // move_2_finished_ = false;
          // effective_cmd = TaskCmd::MOVE_POSE_2;
          q_start_ = q_;   // 用真实当前关节，避免速度跳变
          elapsed_time_ = 0.0;
          move_2_finished_ = false;
          effective_cmd = TaskCmd::MOVE_POSE_2;

          // initialize Cartesian move-pose-2 trajectory
          if (kin_initialized_) {
            const pinocchio::SE3 ee_start = pino_.fk(q_.head<7>());

            move1_start_pos_ = ee_start.translation();   // 复用已有变量，最小改动
            move1_lift_pos_ = move1_start_pos_;
            move1_lift_pos_.z() += 0.10;                 // 先向上抬 10 cm

            Eigen::Quaterniond q_start(ee_start.rotation());
            q_start.normalize();
            move1_start_quat_ = q_start;

            Eigen::Quaterniond q_target(
                target_q_xyzw_2_[3],
                target_q_xyzw_2_[0],
                target_q_xyzw_2_[1],
                target_q_xyzw_2_[2]);   // w,x,y,z
            q_target.normalize();

            if (move1_start_quat_.dot(q_target) < 0.0) {
              q_target.coeffs() *= -1.0;
            }

            move1_target_quat_ = q_target;
          }




        }
      }
      // (A4) Entry gate for RUN_RL
      else if (cmd == TaskCmd::RUN_RL)
      {
        // Only allow RUN_RL right after MOVE_POSE_2 has finished
        if (!(last == TaskCmd::MOVE_POSE_2 && move_2_finished_))
        {
          // Reject entering RUN_RL unless MOVE_POSE_2 finished
          effective_cmd = last;
        }
        else
        {
          // Accept entering RUN_RL: one-time init for RL stage (only state init here; do NOT touch switch-case)
          q_start_ = q_des_;
          elapsed_time_ = 0.0;
          rl_finished_ = false;
          effective_cmd = TaskCmd::RUN_RL;

          // start logging from here
          log_steps_written_.store(0, std::memory_order_relaxed);
          log_save_done_.store(false, std::memory_order_relaxed);
          log_time_start_ = time_total_;
          log_save_path_ = make_log_file_path();
          log_started_.store(true, std::memory_order_release);
          // reset auto-save flags for this rollout
          save_requested_.store(false, std::memory_order_relaxed);
          rl_finish_latched_.store(false, std::memory_order_relaxed);

        }
      }      
      /// TBD add another stop, for exmpal, STOP //////
      // You can add other entry rules here later (e.g. STOP...)
    }
  }

  // Update last_cmd_ to reflect what we actually execute this cycle
  last_cmd_.store(static_cast<int>(effective_cmd), std::memory_order_relaxed);
 





  N_rt_ += 1;

  const double dt = period.seconds();
  time_total_ += dt;

  if (!read_state()) {
    RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "read_state() failed in update()");
    return controller_interface::return_type::ERROR;
  }

  if (!franka_robot_state_->get_values_as_message(franka_state_msg_)) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Failed to read franka robot state.");
    return controller_interface::return_type::ERROR;
  }

  tau_ext_last_joint_ = franka_state_msg_.tau_ext_hat_filtered.effort[6];
  
  /// I found k_f_ext_hat_k coincidences with the ft definition in IssacLab.
  wrench_ext_k_bg_[0] =wrench_ext_k_[0] = franka_state_msg_.k_f_ext_hat_k.wrench.force.x;
  wrench_ext_k_bg_[1] =wrench_ext_k_[1] = franka_state_msg_.k_f_ext_hat_k.wrench.force.y;
  wrench_ext_k_bg_[2] =wrench_ext_k_[2] = franka_state_msg_.k_f_ext_hat_k.wrench.force.z;
  wrench_ext_k_bg_[3] =wrench_ext_k_[3] = franka_state_msg_.k_f_ext_hat_k.wrench.torque.x;
  wrench_ext_k_bg_[4] =wrench_ext_k_[4] = franka_state_msg_.k_f_ext_hat_k.wrench.torque.y;
  wrench_ext_k_bg_[5] =wrench_ext_k_[5] = franka_state_msg_.k_f_ext_hat_k.wrench.torque.z;

  wrench_ext_o_bg_[0] = wrench_ext_o_[0] = franka_state_msg_.o_f_ext_hat_k.wrench.force.x;
  wrench_ext_o_bg_[1] = wrench_ext_o_[1] = franka_state_msg_.o_f_ext_hat_k.wrench.force.y;
  wrench_ext_o_bg_[2] = wrench_ext_o_[2] = franka_state_msg_.o_f_ext_hat_k.wrench.force.z;
  wrench_ext_o_bg_[3] = wrench_ext_o_[3] = franka_state_msg_.o_f_ext_hat_k.wrench.torque.x;
  wrench_ext_o_bg_[4] = wrench_ext_o_[4] = franka_state_msg_.o_f_ext_hat_k.wrench.torque.y;
  wrench_ext_o_bg_[5] = wrench_ext_o_[5] = franka_state_msg_.o_f_ext_hat_k.wrench.torque.z;

  std::array<double, MoveCatersianImpWithGripper::kWrenchDim> current_wrench_o = {
      franka_state_msg_.k_f_ext_hat_k.wrench.force.x,
      franka_state_msg_.k_f_ext_hat_k.wrench.force.y,
      franka_state_msg_.k_f_ext_hat_k.wrench.force.z,
      franka_state_msg_.k_f_ext_hat_k.wrench.torque.x,
      franka_state_msg_.k_f_ext_hat_k.wrench.torque.y,
      franka_state_msg_.k_f_ext_hat_k.wrench.torque.z
  };

  wrench_ext_o_buffer_[wrench_ext_o_buffer_idx_] = current_wrench_o;
  wrench_ext_o_buffer_idx_++;

  if (wrench_ext_o_buffer_idx_ >= kBiasWindowSize) {
    wrench_ext_o_buffer_idx_ = 0;
    wrench_ext_o_buffer_full_ = true;
  }





  // WARMUP HOLD (optional) --- RT-safe: no parameter get/declare here
  if (!warmup_inited_) 
  {
    warmup_elapsed_ = 0.0;
    warmup_q_hold_ = q_;  // capture current pose at first update()
    warmup_inited_ = true;

    // NOTE: logging moved out of RT. (We keep behavior; warmup_seconds_ already cached.)
  }



  // --------------------------update rl obs----
  if (kin_initialized_) {

    // /// the following code is not rt-friendly, we need to sate release build type..
    ik_timer_ += dt;
    if (ik_timer_ >= 0.000) {   // 500 Hz
      ik_timer_ = 0.0;
      // call IK once
      q_arm_rt_ = q_.head<7>();
      // std::cout<<" fk_rt function, 500 hz start"<<std::endl;  
      pino_.fk_rt(q_arm_rt_, ee_real);
      ee6_mea_ = se3ToXyzRpy(ee_real);
      fingertip_midpoint_p_quaterion = se3ToXyzQuat(ee_real);  
      // std::cout<<"fk_rt function, 50 hz success"<<std::endl;
      ee_position_msg_dbg_ = Eigen::Vector3d(ee6_mea_(0), ee6_mea_(1), ee6_mea_(2));
      Eigen::Quaterniond quat(
          fingertip_midpoint_p_quaterion(3),
          fingertip_midpoint_p_quaterion(4),
          fingertip_midpoint_p_quaterion(5),
          fingertip_midpoint_p_quaterion(6));     
      ee_orientation_msg_dbg_ = quat;          
    }
    
    ////--------- TO check if the ee  is point to the fingertip ee------
    // // Franka 官方状态里直接给出 O_T_EE
    // const auto& pose = franka_state_msg_.o_t_ee.pose;

    // const double px = pose.position.x;
    // const double py = pose.position.y;
    // const double pz = pose.position.z;

    // // geometry_msgs quaternion: x y z w
    // Eigen::Quaterniond quat(
    //     pose.orientation.w,
    //     pose.orientation.x,
    //     pose.orientation.y,
    //     pose.orientation.z);

    // quat.normalize();

    // const Eigen::Matrix3d R_fr3 = quat.toRotationMatrix();
    // const Eigen::Vector3d rpy_fr3 = R_fr3.eulerAngles(0, 1, 2);

    // // ee6_mea_ = [x, y, z, roll, pitch, yaw]
    // ee6_mea_[0] = px;
    // ee6_mea_[1] = py;
    // ee6_mea_[2] = pz;
    // ee6_mea_[3] = rpy_fr3[0];
    // ee6_mea_[4] = rpy_fr3[1];
    // ee6_mea_[5] = rpy_fr3[2];

    // // fingertip_midpoint_p_quaterion = [x, y, z, qw, qx, qy, qz]
    // fingertip_midpoint_p_quaterion[0] = px;
    // fingertip_midpoint_p_quaterion[1] = py;
    // fingertip_midpoint_p_quaterion[2] = pz;
    // fingertip_midpoint_p_quaterion[3] = quat.w();
    // fingertip_midpoint_p_quaterion[4] = quat.x();
    // fingertip_midpoint_p_quaterion[5] = quat.y();
    // fingertip_midpoint_p_quaterion[6] = quat.z();

    std::tie(ee_orientation_current_, ee_position_current_) =
    franka_cartesian_pose_->getCurrentOrientationAndTranslation();

    ee_position_dbg_ = ee_position_current_;
    ee_orientation_dbg_ = ee_orientation_current_;


    // ee_position_msg_dbg_ = Eigen::Vector3d(px, py, pz);
    // ee_orientation_msg_dbg_ = quat;
    

    const Eigen::Matrix3d R_fr3 = ee_orientation_current_.toRotationMatrix();
    const Eigen::Vector3d rpy_fr3 = R_fr3.eulerAngles(0, 1, 2);

    // ee6_mea_ = [x, y, z, roll, pitch, yaw]
    ee6_mea_[0] = ee_position_current_.x();
    ee6_mea_[1] = ee_position_current_.y();
    ee6_mea_[2] = ee_position_current_.z();
    ee6_mea_[3] = rpy_fr3[0];
    ee6_mea_[4] = rpy_fr3[1];
    ee6_mea_[5] = rpy_fr3[2];   
    // fingertip_midpoint_p_quaterion = [x, y, z, qw, qx, qy, qz]
    fingertip_midpoint_p_quaterion[0] = ee_position_current_.x();
    fingertip_midpoint_p_quaterion[1] = ee_position_current_.y();
    fingertip_midpoint_p_quaterion[2] = ee_position_current_.z();
    fingertip_midpoint_p_quaterion[3] = ee_orientation_current_.w();
    fingertip_midpoint_p_quaterion[4] = ee_orientation_current_.x();
    fingertip_midpoint_p_quaterion[5] = ee_orientation_current_.y();
    fingertip_midpoint_p_quaterion[6] = ee_orientation_current_.z();



  } else {
    ee6_mea_.setZero();
    fingertip_midpoint_p_quaterion.setZero();
    fingertip_midpoint_p_quaterion[0] = target_p_2_[0];
    fingertip_midpoint_p_quaterion[1] = target_p_2_[1];
    fingertip_midpoint_p_quaterion[2] = target_p_2_[2];

    fingertip_midpoint_p_quaterion[4] = 1.0;  // // fallback quat [qw,qx,qy,qz]=[0,1,0,0], i.e. 180 deg about x
  }


  // --------------------------
  // 1) 计算 fingertip 当前位姿（你需要已有的 SE3 / 或能拿到 R,t）
  // 这里用你已有的 7x1 缓存变量 fingertip_midpoint_p_quaterion：
  // 约定：0..2 是 pos, 3..6 是 quat(wxyz)
  fingertip_pos = fingertip_midpoint_p_quaterion.segment<3>(0);
  fingertip_quat = fingertip_midpoint_p_quaterion.segment<4>(3);


  fingertip_pos_rel_fixed = fingertip_pos - target_p_gear_fixed_;

  --------------------------
  // 3) ee_linvel / ee_angvel
  //    你训练用的是 finite-diff velocity（ee_linvel_fd / ee_angvel_fd）
  // --------------------------
  q_now.w() = fingertip_quat[0];
  q_now.x() = fingertip_quat[1];
  q_now.y() = fingertip_quat[2];
  q_now.z() = fingertip_quat[3];
  q_now.normalize();


  updateEeVelFd(fingertip_pos, q_now, dt, ee_linvel, ee_angvel);
  // --------------------------
  // 4) hand_wrench： 
  // [FT PATCH] begin
  // RT-safe: read latest wrench message from buffer
  if(tau_ext_measure)/// if you have the ft sensor
  {
    if (auto msg_ptr = *(ft_rt_buffer_.readFromRT()); msg_ptr) {
      hand_wrench(0) = msg_ptr->wrench.force.x;
      hand_wrench(1) = msg_ptr->wrench.force.y;
      hand_wrench(2) = msg_ptr->wrench.force.z;
      hand_wrench(3) = msg_ptr->wrench.torque.x;
      hand_wrench(4) = msg_ptr->wrench.torque.y;
      hand_wrench(5) = msg_ptr->wrench.torque.z;
    } else {
      // no message yet: keep previous value (or setZero if你更想要)
      hand_wrench.setZero();
    }
    // [FT PATCH] end
  }
  else //fr3 built-in estimation 
  {
    hand_wrench(0) = franka_state_msg_.k_f_ext_hat_k.wrench.force.x;
    hand_wrench(1) = franka_state_msg_.k_f_ext_hat_k.wrench.force.y;
    hand_wrench(2) = franka_state_msg_.k_f_ext_hat_k.wrench.force.z;
    hand_wrench(3) = franka_state_msg_.k_f_ext_hat_k.wrench.torque.x;
    hand_wrench(4) = franka_state_msg_.k_f_ext_hat_k.wrench.torque.y;
    hand_wrench(5) = franka_state_msg_.k_f_ext_hat_k.wrench.torque.z;    
  }
  hand_wrench_dbg_ = hand_wrench;



  if (warmup_seconds_ > 0.0 && warmup_elapsed_ < warmup_seconds_) 
  {
    warmup_elapsed_ += dt;

    if (warmup_elapsed_ > 0.2 &&
        !warmup_open_requested_.exchange(true, std::memory_order_acq_rel)) {
      request_send_move_.store(true, std::memory_order_release);
    }


    // Hold desired posture as the captured pose
    q_des_ = warmup_q_hold_;

    // Ensure command interface index map exists before writing commands.
    if (!warmup_cmd_map_built_) 
    {
      warmup_eff_cmd_idx_.fill(-1);

      for (size_t k = 0; k < command_interfaces_.size(); ++k) {
        const auto & ci = command_interfaces_[k];
        const std::string full = ci.get_name();  // e.g. "fr3_joint1/effort"

        for (size_t j = 0; j < 7; ++j) {
          const std::string & jn = joint_names_[j];
          if (full == (jn + "/effort")) {
            warmup_eff_cmd_idx_[j] = static_cast<int>(k);
          }
        }
      }

      for (size_t j = 0; j < 7; ++j) {
        if (warmup_eff_cmd_idx_[j] < 0) {
          // NO logging in RT: record which joint is missing, then return ERROR
          rt_missing_warmup_eff_joint_.store(static_cast<int>(j));
          return controller_interface::return_type::ERROR;
        }
      }

      warmup_cmd_map_built_ = true;
    }

    // Joint-space PD torque for all 7 joints
    for (size_t i = 0; i < 7; ++i) 
    {
      const double kp = k_gains_(static_cast<int>(i));
      const double kd = d_gains_(static_cast<int>(i));
      tau_joint_pd_(static_cast<int>(i)) =
        kp * (q_des_(static_cast<int>(i)) - q_(static_cast<int>(i)))
        - kd * dq_(static_cast<int>(i));
    }

    // Build EE reference from FK of interpolated joints (arm only)
    /// no rt friendly, fixed it later, we need to set release build mode /////
    if (kin_initialized_) 
    {
      q_ref_arm = q_des_.head<7>();
      ee_refx = pino_.fk(q_ref_arm);
      ee6_ref_ = se3ToXyzRpy(ee_refx);
    } 
    else 
    {
      ee6_ref_.setZero();
    }


    // During warmup: define tau_osc_cmd_ for publishing only (no OSC computation)
    tau_osc_cmd_ = tau_joint_pd_;

    tau_to_send = tau_joint_pd_;

 
    // /// rt test
    if(rl_target_collect_)
    {
      tau_to_send.setZero();
    }

    // Send joint PD torque to hold pose
    for (size_t i = 0; i < 7; ++i) {
      command_interfaces_[warmup_eff_cmd_idx_[i]].set_value(tau_to_send(i));
    }

  } 
  else
  {

    N_torque_ +=1;

    if (!cmd_map_built_) {
      eff_cmd_idx_.fill(-1);

      for (size_t k = 0; k < command_interfaces_.size(); ++k) {
        const auto& ci = command_interfaces_[k];
        const std::string full_name = ci.get_name();

        for (size_t j = 0; j < 7; ++j) {
          if (full_name == joint_names_[j] + "/effort") {
            eff_cmd_idx_[j] = static_cast<int>(k);
          }
        }
      }

      for (size_t j = 0; j < 7; ++j) {
        if (eff_cmd_idx_[j] < 0) {
          RCLCPP_ERROR(
              get_node()->get_logger(),
              "Command interface mapping failed for joint %s: effort index=%d",
              joint_names_[j].c_str(),
              eff_cmd_idx_[j]);
          return controller_interface::return_type::ERROR;
        }
      }

      cmd_map_built_ = true;
    }


    // ------------------------------------------------------------
    // Command-driven reference generation (switch-case)
    // ------------------------------------------------------------
    switch (effective_cmd) 
    {
      case TaskCmd::MOVE_POSE_1: 
      {
        if(abs(elapsed_time_)<=1e-6)
        {
          std::cout<<"switch to the move_pose_1, move to the grasp pose" <<std::endl;
          std::cout<<"Real q:"<<q_.transpose() <<std::endl;
          std::cout<<"Initial q for motion generation:"<<q_start_.transpose() <<std::endl;
          std::cout<<"kin_initialized_:"<<kin_initialized_<<std::endl;
          std::cout<<"osc_initialized_:"<<osc_initialized_<<std::endl;
          std::cout<<"catersian_imp_:"<<catersian_imp_<<std::endl;
          std::cout<<"osc_blend_alpha_gain_:"<<osc_blend_alpha_gain_<<std::endl;

          //============================================================================================================================================
          q_policy_target_ = q_;

          // Resetting policy timer
          rl_timer_ = 0.0;
          // Resetting observations
          rl_obs_.fill(0.0f);
          rl_prev_action_.fill(0.0f);
          action_rl_bg_.fill(0.0f);
          action_rl_bg_valid_ = false;
          //============================================================================================================================================
        }

        elapsed_time_ += dt;
        rl_timer_ +=dt;
        // Only in MOVE_POSE_1: generate q_des_ by linear interpolation
        // if (!move_1_finished_) 
        // {
        //   const double s = std::clamp(elapsed_time_ / std::max(1e-6, move_duration_), 0.0, 1.0);
        //   q_des_ = (1.0 - s) * q_start_ + s * q_goal_;

        //   if (((q_des_ - q_goal_).norm() < finish_tolerance_) || (elapsed_time_ >= 1.5*move_duration_)) 
        //   {
        //     move_1_finished_ = true;
        //     std::cout<<"!!! move_1_finished !!!"<<std::endl;
        //   }
        // }

        // auto quintic = [](double tau) -> double {
        //   tau = std::clamp(tau, 0.0, 1.0);
        //   const double tau2 = tau * tau;
        //   const double tau3 = tau2 * tau;
        //   const double tau4 = tau3 * tau;
        //   const double tau5 = tau4 * tau;
        //   return 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
        // };

        if (!move_1_finished_)
        {
          // const double lift_duration = move_duration_ * 0.5;
          // const double move_duration_remain = move_duration_ * 0.5;
          //============================================================================================================================================
          constexpr float phase_mask = 0.0f;
          
          // ids[0:6] = normalized arm joint position 
          // ids[9:15] = normalized arm joint velocities
          for (int i=0; i<7; i++){
            const double lower = robot_dof_lower_limit_[i];
            const double upper = robot_dof_upper_limit_[i];
            const double denom = std::max(1e-9, upper - lower);

            double q_norm = 2.0 * (q_(i) - lower)/denom - 1.0;
            if(!std::isfinite(q_norm)){q_norm = 0.0;}

            double dq_norm = dq_(i) * dof_velocity_scale_;
            if(!std::isfinite(dq_norm)){dq_norm = 0.0;}

            rl_obs_[i] = static_cast<float>(std::clamp(q_norm, -5.0, 5.0));
            rl_obs_[9 + i] = static_cast<float>(std::clamp(dq_norm, -5.0, 5.0));
          }

          // hardcoded finger position and velocity -> open with vel = 0
          const double finger_pos[2] = {finger_open_pos_, finger_open_pos_};
          const double finger_vel[2] = {0.0, 0.0};

          for (int i=0; i<2; i++) {
            const int idx = 7 + i;

            const double lower = robot_dof_lower_limit_[idx];
            const double upper = robot_dof_upper_limit_[idx];
            const double denom = std::max(1e-9, upper - lower);

            double q_norm = 2.0 * (finger_pos[i] - lower)/denom - 1.0;
            if(!std::isfinite(q_norm)){q_norm = 0.0;}

            double dq_norm = finger_vel[i] * dof_velocity_scale_;
            if(!std::isfinite(dq_norm)){dq_norm = 0.0;}

            rl_obs_[idx] = static_cast<float>(std::clamp(q_norm, -5.0, 5.0));
            rl_obs_[9 + idx] = static_cast<float>(std::clamp(dq_norm, -5.0, 5.0));
          }

          // reaching phase flag = 0
          rl_obs_[18] = phase_mask;

          // 3D error vector to the target
          const Eigen::Vector3d to_target = target_p_ - fingertip_pos;
          rl_obs_[19] = static_cast<float>(to_target.x());
          rl_obs_[20] = static_cast<float>(to_target.y());
          rl_obs_[21] = static_cast<float>(to_target.z());

          // orientation error
          Eigen::Quaterniond q_hand(
            fingertip_quat[0],
            fingertip_quat[1],
            fingertip_quat[2],
            fingertip_quat[3]
          );
          q_hand.normalize();

          Eigen::Quaterniond q_target(
            target_q_xyzw_[3],
            target_q_xyzw_[0],
            target_q_xyzw_[1],
            target_q_xyzw_[2]
          );
          q_target.normalize();
          const double quat_dot = std::abs(
            move1_target_quat_.w() * fingertip_quat[0] +
            move1_target_quat_.x() * fingertip_quat[1] +
            move1_target_quat_.y() * fingertip_quat[2] +
            move1_target_quat_.z() * fingertip_quat[3]);
          const double rot_err = 2.0 * std::acos(std::clamp(quat_dot, 0.0, 1.0));
            

          Eigen::Quaterniond q_rel = q_target * q_hand.conjugate();
          q_rel.normalize();

          if (q_rel.w() < 0.0) {q_rel.coeffs() *= -1.0;}

          const Eigen::Vector3d qv(q_rel.x(), q_rel.y(), q_rel.z());
          const double qw = std::max(1e-8, q_rel.w());
          const double nv = qv.norm();

          Eigen::Vector3d ori_err = Eigen::Vector3d::Zero();

          if (nv > 1e-6) {
            const double angle = 2.0 * std::atan2(nv, qw);
            ori_err = angle * qv / nv;
          }
          else {
            ori_err = 2.0 * qv;
          }
          ori_err /= M_PI;
          
          rl_obs_[22] = static_cast<float>(std::clamp(ori_err.x(), -1.0, 1.0));
          rl_obs_[23] = static_cast<float>(std::clamp(ori_err.y(), -1.0, 1.0));
          rl_obs_[24] = static_cast<float>(std::clamp(ori_err.z(), -1.0, 1.0));

          // lift_height == 0, before the grasp
          rl_obs_[25] = 0.0f;

          if (policy_ && rl_timer_ >= rl_dt_) {
            const double policy_dt = rl_timer_;
            rl_timer_ = 0.0;
            // obs[26:46] = clear_obs
            //
            // In the simulations there where the probe points, recreated in this section with added pinocchio functions
            // ------------------------------------------------------------
            updateClearObsFromPinocchio();

            for (int i = 0; i < 20; ++i)
            {
              float v = clear_obs_[i];
            
              if (!std::isfinite(v)) {
                v = 0.0f;
              }
            
              rl_obs_[26 + i] = std::clamp(v, -5.0f, 5.0f);
            }
          
            // Protezione finale equivalente a torch.nan_to_num + clamp.
            for (float& v : rl_obs_)
            {
              if (!std::isfinite(v)) {
                v = 0.0f;
              }
            
              v = std::clamp(v, -5.0f, 5.0f);
            }
            auto action = policy_->infer(rl_obs_);
            action_rl_bg_ = action;
            action_rl_bg_valid_ = true;
            rl_prev_action_ = action;

            // IsaacLab:
            // targets_arm =
            //   robot_dof_targets[:, :-2]
            //   + robot_dof_speed_scales[:-2] * dt * arm_actions * action_scale
            for (int i = 0; i < 7; i++){
              double a = static_cast<double>(action[i]);
              if (!std::isfinite(a)) {a = 0.0;}
              a = std::clamp(a, -1.0, 1.0);
              q_policy_target_(i) += robot_dof_speed_scales_[i] * policy_dt * a * action_scale_;
              q_policy_target_(i) = std::clamp(q_policy_target_(i), robot_dof_lower_limit_[i], robot_dof_upper_limit_[i]);
              // the action for the gripper is ignored becouse it will be closed with GRASP phase
            }
          }

          q_des_.head<7>() = q_policy_target_;
          q_goal_.head<7>() = q_des_.head<7>();

          // Se stai usando solo joint PD, questa parte non è critica.
          if (kin_initialized_)
          {
            const pinocchio::SE3 ee_ref_policy = pino_.fk(q_des_.head<7>());
            ee6_ref_ = se3ToXyzRpy(ee_ref_policy);
          }
          else {
            ee6_ref_.setZero();
          }

          const double dist_pick = (target_p_ - fingertip_pos).norm();

          if (N_torque_ % 1000 == 0)
          {
            std::cout << "[MOVE_POSE_1 PICK POLICY]" << std::endl;
            std::cout << "q: " << q_.transpose() << std::endl;
            std::cout << "q_des: " << q_des_.transpose() << std::endl;
            std::cout << "target_p_: " << target_p_.transpose() << std::endl;
            std::cout << "fingertip_pos: " << fingertip_pos.transpose() << std::endl;
            std::cout << "to_target: " << to_target.transpose() << std::endl;
            std::cout << "dist_pick: " << dist_pick << std::endl;
          }

          if ((elapsed_time_ >= move_duration_) || ((dist_pick < r_in_) && (rot_err < 0.05) && (elapsed_time_ >= 1.0 * move_duration_))) {
            
            move_1_finished_ = true;

            q_goal_.head<7>() = q_des_.head<7>();

            std::cout << "!!! move_1_finished: pick policy stopped before grasp !!!" << std::endl;
            std::cout << "dist_pick = " << dist_pick << std::endl;
            std::cout << "q_goal_ = " << q_goal_.transpose() << std::endl;
          }
          //==========================================================================================================================================

          // const double s_total = quintic(
          //     std::clamp(elapsed_time_ / std::max(1e-6, move_duration_), 0.0, 1.0));

          // Eigen::Vector3d pos_ref = move1_start_pos_;

          // if (elapsed_time_ <= lift_duration)
          // {
          //   const double s1 = quintic(elapsed_time_ / std::max(1e-6, lift_duration));
          //   pos_ref = move1_start_pos_ + s1 * (move1_lift_pos_ - move1_start_pos_);
          // }
          // else
          // {
          //   const double t2 = elapsed_time_ - lift_duration;
          //   const double s2 = quintic(t2 / std::max(1e-6, move_duration_remain));
          //   pos_ref = move1_lift_pos_ + s2 * (target_p_ - move1_lift_pos_);
          // }

          // segment 1: first 2s, move upward
          // if (elapsed_time_ <= lift_duration)
          // {
          //   const double s1 = quintic(elapsed_time_ / std::max(1e-6, lift_duration));
          //   pos_ref = move1_start_pos_ + s1 * (move1_lift_pos_ - move1_start_pos_);
          // }
          // // segment 2: next 2s, move to target xy while keeping 5cm above target z
          // else if (elapsed_time_ <= lift_duration + hover_move_duration)
          // {
          //   const double t2 = elapsed_time_ - lift_duration;
          //   const double s2 = quintic(t2 / std::max(1e-6, hover_move_duration));
          //   pos_ref = move1_lift_pos_ + s2 * (move1_hover_pos_ - move1_lift_pos_);
          // }
          // // segment 3: remaining time, move downward to target pose
          // else
          // {
          //   const double t3 = elapsed_time_ - lift_duration - hover_move_duration;
          //   const double s3 = quintic(t3 / std::max(1e-6, descend_duration));
          //   pos_ref = move1_hover_pos_ + s3 * (target_p_ - move1_hover_pos_);
          // }




          // Eigen::Quaterniond quat_ref =
          //     move1_start_quat_.slerp(s_total, move1_target_quat_);
          // quat_ref.normalize();

          // pinocchio::SE3 ee_ref_move1(quat_ref.toRotationMatrix(), pos_ref);

          // ee6_ref_ = poseToXyzRpy(pos_ref, quat_ref);

          // if (kin_initialized_)
          // {
          //   Vector7d q_seed = q_des_.head<7>();
          //   if (elapsed_time_ <= 2.0 * dt) {
          //     q_seed = q_.head<7>();
          //   }

          //   Vector7d q_sol = q_seed;
          //   double final_err = -1.0;
          //   bool ok = pino_.ik(ee_ref_move1, q_seed, q_sol, ik_opt_, &final_err);

          //   if (ok) {
          //     q_des_.head<7>() = q_sol;
          //   }
          // }

          // const double quat_dot = std::abs(
          //     move1_target_quat_.w() * fingertip_quat[0] +
          //     move1_target_quat_.x() * fingertip_quat[1] +
          //     move1_target_quat_.y() * fingertip_quat[2] +
          //     move1_target_quat_.z() * fingertip_quat[3]);
          // const double rot_err = 2.0 * std::acos(std::clamp(quat_dot, 0.0, 1.0));

          // if(N_torque_%1000==0)
          // {
          //   std::cout<<"q_des_:"<<q_des_.transpose() << std::endl;
          //   std::cout<<"q_goal_:"<<q_goal_.transpose() << std::endl;
          //   std::cout<<"pos_ref: :"<<pos_ref.transpose() << std::endl;
          //   std::cout<<"target_p_: :"<<target_p_.transpose() << std::endl;
          // }

          // if ((elapsed_time_ >= move_duration_) ||
          //     (((fingertip_pos - target_p_).norm() < finish_tolerance_) &&
          //     (rot_err < 0.05) &&
          //     (elapsed_time_ >= 1.0 * move_duration_)))
          // {
          //   move_1_finished_ = true;
          //   q_goal_.head<7>() = q_des_.head<7>();   // 建议补这一句
          //   std::cout << "!!! move_1_finished !!!" << std::endl;
          // }
        }
        else 
        {
          // When finished: keep previous behavior
          if (hold_position_) {
            q_des_ = q_goal_;
          } else {
            q_des_ = q_;
          }
        }
        break;
      }
      case TaskCmd::GRASP:
      {
        if(abs(elapsed_time_)<=1e-6)
        {
          std::cout<<"switch to the grasp" <<std::endl;
          std::cout<<"Real q:"<<q_.transpose() <<std::endl;
          std::cout<<"des for move_pose_1:"<<q_goal_.transpose() <<std::endl;          
          std::cout<<"-----Call grasp service to close the gripper Now----"<<std::endl;
        }

        elapsed_time_ += dt;

        // When finished: keep previous behavior
        if (hold_position_) {
          q_des_ = q_goal_;
        } else {
          q_des_ = q_;
        }

        // 真正依据 action 结果来结束 grasp
        if(!grasp_finished_)
        {
          if (grasp_action_done_.load(std::memory_order_relaxed)) {
            if (grasp_action_success_.load(std::memory_order_relaxed)) 
            {
              grasp_finished_ = true;
              if (!grasp_result_reported_.exchange(true)) {
                std::cout << "!!! grasp_finished by action success !!!" << std::endl;
              }
            } 
            else 
            {
              // 失败时你可以选择：
              // 1) 一直卡在 GRASP
              // 2) 或设置一个失败标志然后 STOP
              if (!grasp_result_reported_.exchange(true)) {
                std::cout << "!!! grasp action failed !!!" << std::endl;
              }
              else
              {
                if( elapsed_time_ > move_duration_grasp_)
                {
                  grasp_finished_ = true;
                  std::cout << "!!! grasp action failed but time out !!!" << std::endl;

                }
              }
            }
          }
          else
          {
            if( elapsed_time_ > move_duration_grasp_)
            {
              grasp_finished_ = true;
              std::cout << "!!! grasp action_node does not run, then time out !!!" << std::endl;

            }
          }
        }





        break;
      }
      case TaskCmd::MOVE_POSE_2:
      {
        if(abs(elapsed_time_)<=1e-6)
        {
          // std::cout<<"switch to the MOVE_POSE_2, move to the plugin init pose" <<std::endl;
          std::cout<<"switch to the MOVE_POSE_2, move to the placing pose" <<std::endl;
          std::cout<<"Real q:"<<q_.transpose() <<std::endl;
          std::cout<<"q_goal_ for move pose 2:"<<q_goal_2_.transpose() <<std::endl;
          // std::cout<<"kin_initialized_:"<<kin_initialized_<<std::endl;
          // std::cout<<"osc_initialized_:"<<osc_initialized_<<std::endl;
          // =========================================================================================================================================
          q_policy_target_ = q_;

          // Resetting policy timer
          rl_timer_ = 0.0;
          // Resetting observations
          rl_obs_.fill(0.0f);
          rl_prev_action_.fill(0.0f);
          action_rl_bg_.fill(0.0f);
          action_rl_bg_valid_ = false;
        }

        elapsed_time_ += dt;
        rl_timer_ += dt;
        // In MOVE_POSE_2: generate EE pos by linear interpolation
        if (!move_2_finished_) 
        {
          constexpr float phase_mask = 1.0f;
          
          // ids[0:6] = normalized arm joint position 
          // ids[9:15] = normalized arm joint velocities
          for (int i=0; i<7; i++){
            const double lower = robot_dof_lower_limit_[i];
            const double upper = robot_dof_upper_limit_[i];
            const double denom = std::max(1e-9, upper - lower);

            double q_norm = 2.0 * (q_(i) - lower)/denom - 1.0;
            if(!std::isfinite(q_norm)){q_norm = 0.0;}

            double dq_norm = dq_(i) * dof_velocity_scale_;
            if(!std::isfinite(dq_norm)){dq_norm = 0.0;}

            rl_obs_[i] = static_cast<float>(std::clamp(q_norm, -5.0, 5.0));
            rl_obs_[9 + i] = static_cast<float>(std::clamp(dq_norm, -5.0, 5.0));
          }

          // hardcoded finger position and velocity -> open with vel = 0
          const double finger_pos[2] = {finger_closed_pos_, finger_closed_pos_};
          const double finger_vel[2] = {0.0, 0.0};

          for (int i=0; i<2; i++) {
            const int idx = 7 + i;

            const double lower = robot_dof_lower_limit_[idx];
            const double upper = robot_dof_upper_limit_[idx];
            const double denom = std::max(1e-9, upper - lower);

            double q_norm = 2.0 * (finger_pos[i] - lower)/denom - 1.0;
            if(!std::isfinite(q_norm)){q_norm = 0.0;}

            double dq_norm = finger_vel[i] * dof_velocity_scale_;
            if(!std::isfinite(dq_norm)){dq_norm = 0.0;}

            rl_obs_[idx] = static_cast<float>(std::clamp(q_norm, -5.0, 5.0));
            rl_obs_[9 + idx] = static_cast<float>(std::clamp(dq_norm, -5.0, 5.0));
          }

          // reaching phase flag = 0
          rl_obs_[18] = phase_mask;

          // 3D error vector to the target
          const Eigen::Vector3d to_target = target_p_2_ - fingertip_pos;
          rl_obs_[19] = static_cast<float>(to_target.x());
          rl_obs_[20] = static_cast<float>(to_target.y());
          rl_obs_[21] = static_cast<float>(to_target.z());

          // orientation error
          Eigen::Quaterniond q_hand(
            fingertip_quat[0],
            fingertip_quat[1],
            fingertip_quat[2],
            fingertip_quat[3]
          );
          q_hand.normalize();

          Eigen::Quaterniond q_target(
            target_q_xyzw_2_[3],
            target_q_xyzw_2_[0],
            target_q_xyzw_2_[1],
            target_q_xyzw_2_[2]
          );
          q_target.normalize();
          const double quat_dot = std::abs(
            move1_target_quat_.w() * fingertip_quat[0] +
            move1_target_quat_.x() * fingertip_quat[1] +
            move1_target_quat_.y() * fingertip_quat[2] +
            move1_target_quat_.z() * fingertip_quat[3]);
          const double rot_err = 2.0 * std::acos(std::clamp(quat_dot, 0.0, 1.0));

          Eigen::Quaterniond q_rel = q_target * q_hand.conjugate();
          q_rel.normalize();
          

          if (q_rel.w() < 0.0) {q_rel.coeffs() *= -1.0;}

          const Eigen::Vector3d qv(q_rel.x(), q_rel.y(), q_rel.z());
          const double qw = std::max(1e-8, q_rel.w());
          const double nv = qv.norm();

          Eigen::Vector3d ori_err = Eigen::Vector3d::Zero();

          if (nv > 1e-6) {
            const double angle = 2.0 * std::atan2(nv, qw);
            ori_err = angle * qv / nv;
          }
          else {
            ori_err = 2.0 * qv;
          }
          ori_err /= M_PI;
          
          rl_obs_[22] = static_cast<float>(std::clamp(ori_err.x(), -1.0, 1.0));
          rl_obs_[23] = static_cast<float>(std::clamp(ori_err.y(), -1.0, 1.0));
          rl_obs_[24] = static_cast<float>(std::clamp(ori_err.z(), -1.0, 1.0));

          // lift_height == 0, before the grasp
          rl_obs_[25] = static_cast<float>(fingertip_pos.z());

          if (policy_ && rl_timer_ >= rl_dt_) {
            const double policy_dt = rl_timer_;
            rl_timer_ = 0.0;
            // obs[26:46] = clear_obs
            //
            // In the simulations there where the probe points, recreated in this section with added pinocchio functions
            // ------------------------------------------------------------
            updateClearObsFromPinocchio();

            for (int i = 0; i < 20; ++i)
            {
              float v = clear_obs_[i];
            
              if (!std::isfinite(v)) {
                v = 0.0f;
              }
            
              rl_obs_[26 + i] = std::clamp(v, -5.0f, 5.0f);
            }
          
            // Protezione finale equivalente a torch.nan_to_num + clamp.
            for (float& v : rl_obs_)
            {
              if (!std::isfinite(v)) {
                v = 0.0f;
              }
            
              v = std::clamp(v, -5.0f, 5.0f);
            }
            auto action = policy_->infer(rl_obs_);
            action_rl_bg_ = action;
            action_rl_bg_valid_ = true;
            rl_prev_action_ = action;

            // IsaacLab:
            // targets_arm =
            //   robot_dof_targets[:, :-2]
            //   + robot_dof_speed_scales[:-2] * dt * arm_actions * action_scale
            for (int i = 0; i < 7; i++){
              double a = static_cast<double>(action[i]);
              if (!std::isfinite(a)) {a = 0.0;}
              a = std::clamp(a, -1.0, 1.0);
              q_policy_target_(i) += robot_dof_speed_scales_[i] * policy_dt * a * action_scale_;
              q_policy_target_(i) = std::clamp(q_policy_target_(i), robot_dof_lower_limit_[i], robot_dof_upper_limit_[i]);
              // the action for the gripper is ignored becouse it will be closed with GRASP phase
            }
          }

          q_des_.head<7>() = q_policy_target_;
          q_goal_.head<7>() = q_des_.head<7>();

          // Se stai usando solo joint PD, questa parte non è critica.
          if (kin_initialized_)
          {
            const pinocchio::SE3 ee_ref_policy = pino_.fk(q_des_.head<7>());
            ee6_ref_ = se3ToXyzRpy(ee_ref_policy);
          }
          else {
            ee6_ref_.setZero();
          }

          const double dist_pick = (target_p_2_ - fingertip_pos).norm();

          if (N_torque_ % 1000 == 0)
          {
            std::cout << "[MOVE_POSE_2 PICK POLICY]" << std::endl;
            std::cout << "q: " << q_.transpose() << std::endl;
            std::cout << "q_des: " << q_des_.transpose() << std::endl;
            std::cout << "target_p_: " << target_p_.transpose() << std::endl;
            std::cout << "fingertip_pos: " << fingertip_pos.transpose() << std::endl;
            std::cout << "to_target: " << to_target.transpose() << std::endl;
            std::cout << "dist_pick: " << dist_pick << std::endl;
          }

          if ((elapsed_time_ >= move_duration_) || ((dist_pick < r_in_) && (rot_err < 0.05) && (elapsed_time_ >= 1.0 * move_duration_))) {
            
            move_2_finished_ = true;

            q_goal_.head<7>() = q_des_.head<7>();

            std::cout << "!!! move_2_finished: !!!" << std::endl;
            std::cout << "dist_pick = " << dist_pick << std::endl;
            std::cout << "q_goal_ = " << q_goal_.transpose() << std::endl;
          }
          // const double s = std::clamp(elapsed_time_ / std::max(1e-6, move_duration_2_), 0.0, 1.0);
          // q_des_ = (1.0 - s) * q_start_ + s * q_goal_2_;

          // if (((q_des_ - q_goal_2_).norm() < finish_tolerance_) || (elapsed_time_ >= move_duration_2_)) 
          // {
          //   move_2_finished_ = true;
          //   hand_wrench_default = hand_wrench; 
          //   std::cout<<"!!! move_2_finished !!!"<<std::endl;
          // }
          // auto quintic = [](double tau) -> double {
          //   tau = std::clamp(tau, 0.0, 1.0);
          //   const double tau2 = tau * tau;
          //   const double tau3 = tau2 * tau;
          //   const double tau4 = tau3 * tau;
          //   const double tau5 = tau4 * tau;
          //   return 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
          // };

          // const double lift_duration_2 = 0.5 * move_duration_2_;
          // const double move_duration_2_remain = 0.5 * move_duration_2_;

          // Eigen::Vector3d pos_ref = move1_start_pos_;

          // if (elapsed_time_ <= lift_duration_2) {
          //   const double s1 = quintic(elapsed_time_ / std::max(1e-6, lift_duration_2));
          //   pos_ref = move1_start_pos_ + s1 * (move1_lift_pos_ - move1_start_pos_);
          // } else {
          //   const double t2 = elapsed_time_ - lift_duration_2;
          //   const double s2 = quintic(t2 / std::max(1e-6, move_duration_2_remain));
          //   pos_ref = move1_lift_pos_ + s2 * (target_p_2_ - move1_lift_pos_);
          // }

          // const double s_total = quintic(
          //     std::clamp(elapsed_time_ / std::max(1e-6, move_duration_2_), 0.0, 1.0));

          // Eigen::Quaterniond quat_ref =
          //     move1_start_quat_.slerp(s_total, move1_target_quat_);
          // quat_ref.normalize();

          // pinocchio::SE3 ee_ref_move2(quat_ref.toRotationMatrix(), pos_ref);
          // ee6_ref_ = poseToXyzRpy(pos_ref, quat_ref);

          // if (kin_initialized_) {
          //   Vector7d q_seed = q_des_.head<7>();
          //   if (elapsed_time_ <= 2.0 * dt) {
          //     q_seed = q_.head<7>();
          //   }

          //   Vector7d q_sol = q_seed;
          //   double final_err = -1.0;
          //   bool ok = pino_.ik(ee_ref_move2, q_seed, q_sol, ik_opt_, &final_err);

          //   if (ok) {
          //     q_des_.head<7>() = q_sol;
          //   }
          // }

          // const double quat_dot = std::abs(
          //     move1_target_quat_.w() * fingertip_quat[0] +
          //     move1_target_quat_.x() * fingertip_quat[1] +
          //     move1_target_quat_.y() * fingertip_quat[2] +
          //     move1_target_quat_.z() * fingertip_quat[3]);

          // const double rot_err = 2.0 * std::acos(std::clamp(quat_dot, 0.0, 1.0));

          // if ((elapsed_time_ >= move_duration_2_) ||
          //     (((fingertip_pos - target_p_2_).norm() < finish_tolerance_) &&
          //     (rot_err < 0.05))) {
          //   move_2_finished_ = true;
          //   q_goal_2_.head<7>() = q_des_.head<7>();
          //   hand_wrench_default = hand_wrench;
          //   std::cout << "!!! move_2_finished !!!" << std::endl;
          // }

          //=====================================================================================================================================




        }
        else 
        {
          // When finished: keep previous behavior
          if (hold_position_) {
            q_des_ = q_goal_2_;
          } else {
            q_des_ = q_;
          }
        }
        break;
      }   
      case TaskCmd::RUN_RL:
      {
        // if(abs(elapsed_time_)<=1e-6)
        // {
        //   std::cout<<"switch to the RL_POLICY, Gear meshing" <<std::endl;
        //   std::cout<<"Real q:"<<q_.transpose() <<std::endl;
        //   std::cout<<"des for move_pose_2:"<<q_goal_2_.transpose() <<std::endl;

        //   q_interp_start_ = q_des_;


        //   // Reset RL Cartesian target to the actually reached fingertip pose
        //   rl_ctrl_target_fingertip_midpoint_pos = fingertip_pos;

        //   rl_ctrl_target_fingertip_midpoint_quat.w() = fingertip_quat[0];
        //   rl_ctrl_target_fingertip_midpoint_quat.x() = fingertip_quat[1];
        //   rl_ctrl_target_fingertip_midpoint_quat.y() = fingertip_quat[2];
        //   rl_ctrl_target_fingertip_midpoint_quat.z() = fingertip_quat[3];

        //   target_p_3_ = rl_ctrl_target_fingertip_midpoint_pos;
        //   target_q_xyzw_3_[0] = rl_ctrl_target_fingertip_midpoint_quat.x();
        //   target_q_xyzw_3_[1] = rl_ctrl_target_fingertip_midpoint_quat.y();
        //   target_q_xyzw_3_[2] = rl_ctrl_target_fingertip_midpoint_quat.z();
        //   target_q_xyzw_3_[3] = rl_ctrl_target_fingertip_midpoint_quat.w();

        //   rl_ctrl_target_fingertip_midpoint_pos_bg_ = rl_ctrl_target_fingertip_midpoint_pos;
        //   rl_ctrl_target_fingertip_midpoint_quat_bg_ = rl_ctrl_target_fingertip_midpoint_quat;

        //   std::cout<<"fingertip_pos entering RL loop:"<<fingertip_pos.transpose() <<std::endl;
        //   std::cout<<"rl_ctrl_target_fingertip_midpoint_pos entering RL loop:"<<rl_ctrl_target_fingertip_midpoint_pos.transpose()<<std::endl;
        //   std::cout << "rl_ctrl_target_fingertip_midpoint_quat[xyzw] entering RL loop: ["
        //   << rl_ctrl_target_fingertip_midpoint_quat.x() << ", "
        //   << rl_ctrl_target_fingertip_midpoint_quat.y() << ", "
        //   << rl_ctrl_target_fingertip_midpoint_quat.z() << ", "
        //   << rl_ctrl_target_fingertip_midpoint_quat.w() << "]"
        //   << std::endl;
          
        //   wrench_ext_o_bias_ = computeWrenchExtOBias();

        //   for (int k = 0; k < 6; ++k) {
        //     hand_wrench_default[k] = wrench_ext_o_bias_[k];
        //   }          
        //   RCLCPP_INFO(
        //     get_node()->get_logger(),
        //     "wrench_ext_o bias = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
        //     wrench_ext_o_bias_[0], wrench_ext_o_bias_[1], wrench_ext_o_bias_[2],
        //     wrench_ext_o_bias_[3], wrench_ext_o_bias_[4], wrench_ext_o_bias_[5]);
        // }

        // rl_target_valid_ = false;
        // elapsed_time_ += dt;
        // rl_timer_ += dt;
        // // In RUN_RL: generate joint angle by RL policy
        // if (!rl_finished_) 
        // {

        //   if(rl_test_)
        //   {
        //     q_des_ = q_goal_2_;

        //   }
        //   else
        //   {
        //     // q_des_ = q_goal_3_;

        //     // // achieving smooth behavior
        //     q_interp_alpha_ = std::clamp(rl_timer_ / rl_dt_, 0.0, 1.0);
        //     q_des_ = q_interp_start_ + q_interp_alpha_ * (q_goal_3_ - q_interp_start_);


        //   }  


        //   if (policy_ && rl_timer_ >= rl_dt_ ) 
        //   {

        //     // updateEeVelFd(fingertip_pos, q_now, rl_dt_, ee_linvel, ee_angvel);
        //     rt_infer_count += 1;     
        //     rl_timer_ = 0.0;
        //     // --------------------------
        //     // 5) 组装 rl_obs_ (float[28])，严格按训练顺序
        //     // --------------------------
        //     rl_obs_.fill(0.0f);

        //     int idx = 0;

        //     // (1) fingertip_pos_rel_fixed (3)
        //     rl_obs_[idx++] = static_cast<float>(fingertip_pos_rel_fixed.x());
        //     rl_obs_[idx++] = static_cast<float>(fingertip_pos_rel_fixed.y());
        //     rl_obs_[idx++] = static_cast<float>(fingertip_pos_rel_fixed.z());

        //     // (2) fingertip_quat (4)  w,x,y,z
        //     rl_obs_[idx++] = static_cast<float>(fingertip_quat[0]); // w
        //     rl_obs_[idx++] = static_cast<float>(fingertip_quat[1]); // x
        //     rl_obs_[idx++] = static_cast<float>(fingertip_quat[2]); // y
        //     rl_obs_[idx++] = static_cast<float>(fingertip_quat[3]); // z

        //     // (3) ee_linvel (3)
        //     rl_obs_[idx++] = static_cast<float>(ee_linvel.x());
        //     rl_obs_[idx++] = static_cast<float>(ee_linvel.y());
        //     rl_obs_[idx++] = static_cast<float>(ee_linvel.z());

        //     // (4) ee_angvel (3)
        //     rl_obs_[idx++] = static_cast<float>(ee_angvel.x());
        //     rl_obs_[idx++] = static_cast<float>(ee_angvel.y());
        //     rl_obs_[idx++] = static_cast<float>(ee_angvel.z());

        //     // (5) fingertip_pos (3)
        //     rl_obs_[idx++] = static_cast<float>(fingertip_pos.x());
        //     rl_obs_[idx++] = static_cast<float>(fingertip_pos.y());
        //     rl_obs_[idx++] = static_cast<float>(fingertip_pos.z());

        //     // (6) hand_wrench (6) [Fx,Fy,Fz,Tx,Ty,Tz]
        //     for (int k = 0; k < 6; ++k) {
        //       rl_obs_[idx++] = static_cast<float>((hand_wrench[k]-hand_wrench_default[k]));
        //     }

        //     // (7) prev_actions (6)
        //     for (int k = 0; k < 6; ++k) {
        //       rl_obs_[idx++] = rl_prev_action_[k];
        //     }
            


        //     /// compare with python model
        //     const std::array<std::array<float, 28>, 10> rl_obs_debug_seq = {{
        //         {{
        //             0.0133499f,  0.0132065f,  0.0356440f,
        //             5.57733e-05f, 0.948364f,  0.317183f,  0.000496372f,
        //             0.0f,        0.0f,        0.0f,
        //             0.0f,        0.0f,        0.0f,
        //             0.653535f,   0.0533694f,  0.159303f,
        //             -0.0225031f, -0.00610229f, -0.0186993f,
        //             -0.000634027f, 0.000454222f, -0.00051686f,
        //             0.266998f,   0.264131f,   0.71288f,
        //             0.0f,        0.0f,        0.0593679f
        //         }},
        //         {{
        //             0.0114145f,  0.0122277f,  0.0344917f,
        //             7.04028e-05f, 0.949035f,  0.315171f, -0.000628459f,
        //             -0.0385594f, -0.0225806f, -0.0334167f,
        //             -0.00983859f, 0.029358f,  -0.061512f,
        //             0.6516f,     0.0523905f,  0.15815f,
        //             -0.297489f,   0.0438133f,  0.41248f,
        //             -0.00717521f, 0.00444819f, 0.00159193f,
        //             -0.934255f,  -0.760142f,  -1.6584f,
        //             0.976882f,  -0.780937f,  -0.206983f
        //         }},
        //         {{
        //             0.00815636f, 0.00964827f, 0.0299656f,
        //             0.000178994f, 0.949337f, 0.314256f, -0.00145593f,
        //             -0.0562191f, -0.0492565f, -0.0934446f,
        //             -0.00812596f, 0.017919f,  -0.0297402f,
        //             0.648342f,   0.0498112f, 0.153624f,
        //             -0.297962f,   0.10795f,   0.642282f,
        //             -0.00541552f, 0.00636913f, 0.000916491f,
        //             -0.577084f,  -1.14015f,   -3.28605f,
        //             0.426549f,  -1.49844f,   -0.146642f
        //         }},
        //         {{
        //             0.00429499f, 0.00571864f, 0.021645f,
        //             0.000237286f, 0.948491f, 0.3168f, -0.00148267f,
        //             -0.0597382f, -0.0665197f, -0.146089f,
        //             -0.0010804f, -0.00502843f, 0.0761658f,
        //             0.64448f,    0.0458815f, 0.145304f,
        //             -0.168989f,   0.115991f,  0.533401f,
        //             -0.00378188f,-0.00468064f, 8.03585e-05f,
        //             -6.51479e-05f, -1.00539f, -4.26262f,
        //             -0.538824f,  -2.36427f,   0.151123f
        //         }},
        //         {{
        //             0.000654221f, 0.00214989f, 0.0131826f,
        //             -0.000287906f, 0.946651f, 0.32226f, -0.000812189f,
        //             -0.0499248f, -0.0312267f, -0.0630909f,
        //             0.0357707f, -0.0204555f, 0.169956f,
        //             0.640839f,   0.0423128f, 0.136841f,
        //             3.49794f,    4.17594f,   23.9741f,
        //             -0.329715f,   0.298579f,  0.0175709f,
        //             0.595454f,  -0.562321f, -4.88331f,
        //             -1.5109f,    -3.1349f,    0.399982f
        //         }},
        //         {{
        //             -0.000657082f, 0.00106381f, 0.0140826f,
        //             -0.000296223f, 0.944961f, 0.32718f, 0.000953141f,
        //             -0.0262499f, -0.021421f,  0.00568271f,
        //             -0.00473291f, 0.0069089f, 0.155786f,
        //             0.639528f,   0.0412267f, 0.137741f,
        //             -0.225041f,   0.104224f,  0.832875f,
        //             0.00663477f,-0.0334866f, 0.00189754f,
        //             0.647551f,  -0.512717f, -4.01465f,
        //             -1.65751f,   -2.51048f,   0.449527f
        //         }},
        //         {{
        //             -0.00176132f, -0.000352737f, 0.0127941f,
        //             -0.000364046f, 0.944224f, 0.329299f, 0.00170345f,
        //             0.000901222f,-0.011854f, -0.0149775f,
        //             0.0341645f, -0.0701962f, 0.0728566f,
        //             0.638424f,   0.0398102f, 0.136453f,
        //             -0.712159f,   1.6639f,    14.374f,
        //             -0.175807f,  -0.0973861f, 0.0307293f,
        //             1.32038f,   -0.167674f, -3.18339f,
        //             -1.61465f,   -2.02266f,   0.175758f
        //         }},
        //         {{
        //             -0.00210047f, -0.000486623f, 0.0128114f,
        //             -0.000777814f, 0.94333f,  0.33185f, 0.00204509f,
        //             -0.00836849f, -0.000962913f, -0.0093019f,
        //             0.00256343f, 0.00545443f, 0.0799136f,
        //             0.638085f,   0.0396763f,  0.13647f,
        //             0.219305f,   0.392154f,    5.15621f,
        //             -0.025052f,   0.0124259f,   0.00445469f,
        //             0.486676f,   0.14202f,    -2.87769f,
        //             -1.0872f,    -1.73855f,     0.226202f
        //         }},
        //         {{
        //             -0.00202984f, -0.000422459f, 0.0127556f,
        //             -0.000639233f, 0.943233f, 0.332124f, 0.00226928f,
        //             0.00275373f, -0.000731349f, -0.00316501f,
        //             -0.00767754f, -0.00261773f, 0.010203f,
        //             0.638155f,   0.0397404f,   0.136414f,
        //             1.27219f,   -0.261368f,    6.17333f,
        //             0.0408175f,  0.0907637f,  -0.00845671f,
        //             0.8045f,     0.283839f,   -2.43659f,
        //             -1.00014f,   -1.44077f,    -0.0127086f
        //         }},
        //         {{
        //             -0.00170988f, -0.00026533f, 0.0129406f,
        //             -0.000621466f, 0.943648f, 0.330944f, 0.00217249f,
        //             0.00457764f, 0.00636622f, -0.00131428f,
        //             0.00434273f, 0.00624766f, -0.0358968f,
        //             0.638475f,   0.0398976f,  0.136599f,
        //             1.38619f,    0.812975f,   3.70114f,
        //             -0.034688f,   0.0633193f,  0.00123114f,
        //             0.757081f,   0.467729f,  -1.98892f,
        //             -0.746301f,  -1.15563f,   -0.173511f
        //         }}
        //     }};      

        //     const std::array<std::array<float, 6>, 10> py_action_ref = {{
        //         {{ -5.73927f, -4.85723f, -11.1435f,  4.88441f, -3.90469f, -1.27239f }},
        //         {{  0.851601f, -2.6602f,  -9.79666f, -1.77479f, -4.36845f,  0.0947218f }},
        //         {{  2.30801f,  -0.466343f,-8.16888f, -4.40031f, -5.82761f,  1.34219f }},
        //         {{  2.97753f,   1.20996f, -7.36609f, -5.39921f, -6.21743f,  1.39542f }},
        //         {{  0.855937f, -0.3143f,  -0.540019f,-2.24392f, -0.0127724f, 0.647707f }},
        //         {{  4.0117f,    1.2125f,   0.141655f,-1.44323f, -0.0713745f,-0.919321f }},
        //         {{ -2.84814f,   1.3808f,  -1.65487f, 1.02258f, -0.60213f,   0.427979f }},
        //         {{  2.0758f,    0.851115f,-0.672206f,-0.651879f,-0.249624f,-0.968351f }},
        //         {{  0.567406f,  1.20329f, -0.198211f, 0.269049f,-0.0150624f,-0.816721f }},
        //         {{  0.41514f,   0.655045f,-1.42871f,  1.85815f, -0.471627f,-1.06583f }}
        //     }};



        //     // sanity check
        //     if (idx != 28) {
        //       RCLCPP_ERROR(get_node()->get_logger(), "rl_obs_ fill error: idx=%d (expect 28)", idx);
        //     }

        //     bool obs_bad = false;
        //     for (size_t i = 0; i < rl_obs_.size(); ++i) {
        //       if (!std::isfinite(rl_obs_[i])) {
        //         std::cout << "Invalid rl_obs_ at " << i << ": " << rl_obs_[i] << std::endl;
        //         obs_bad = true;
        //       }
        //     }

        //     // --------------------------
        //     // 6) 推理
        //     // --------------------------
        //     if (obs_bad) {
        //       std::cout << "Skip inference because rl_obs_ contains invalid values." << std::endl;
        //     } 
        //     else 
        //     {
            
        //       auto action_rl = policy_->infer(rl_obs_);
              
        //       // if(elapsed_time_ <5*rl_dt_)
        //       // {
        //       //   // for (int t = 0; t < 10; ++t) {
        //       //   //   auto action_test = policy_->infer(rl_obs_);
        //       //   //   std::cout << "test " << t << ": ";
        //       //   //   for (int k = 0; k < 6; ++k) {
        //       //   //     std::cout << action_test[k] << " ";
        //       //   //   }
        //       //   //   std::cout << std::endl;
        //       //   // }
        //       //   std::cout << "raw action: ";
        //       //   for (int k = 0; k < 6; ++k) {
        //       //     std::cout << action_rl[k] << " ";
        //       //   }
        //       //   std::cout << std::endl;

        //       //   std::cout << "tanh action: ";
        //       //   for (int k = 0; k < 6; ++k) {
        //       //     std::cout << std::tanh(action_rl[k]) << " ";
        //       //   }
        //       //   std::cout << std::endl;
        //       // }

        //       if(rt_infer_count<=1)
        //       {
        //         std::cout << "rt_infer_count"<<rt_infer_count<<std::endl;
        //         std::cout << "rl_obs_ from the c++, check if they align with the python = [";
        //         for (size_t i = 0; i < 28; ++i) {
        //             std::cout << rl_obs_[i];
        //             if (i + 1 < 28) {
        //                 std::cout << ", ";
        //             }
        //         }
        //         std::cout << "]" << std::endl;


        //         for (size_t t = 0; t < rl_obs_debug_seq.size(); ++t) {
        //           rl_obs_ = rl_obs_debug_seq[t];

        //           auto action_rlx = policy_->infer(rl_obs_);

        //           std::cout << "=== test " << t << " ===" << std::endl;

        //           std::cout << "obs: ";
        //           for (int i = 0; i < 28; ++i) {
        //             std::cout << rl_obs_[i] << " ";
        //           }
        //           std::cout << std::endl;

        //           std::cout << "onnx raw action: ";
        //           for (int k = 0; k < 6; ++k) {
        //             std::cout << action_rlx[k] << " ";
        //           }
        //           std::cout << std::endl;

        //           std::cout << "python ref raw action: ";
        //           for (int k = 0; k < 6; ++k) {
        //             std::cout << py_action_ref[t][k] << " ";
        //           }
        //           std::cout << std::endl;

        //           std::cout << "abs diff: ";
        //           for (int k = 0; k < 6; ++k) {
        //             std::cout << std::abs(action_rlx[k] - py_action_ref[t][k]) << " ";
        //           }
        //           std::cout << std::endl;
        //         }
        //       }



        //       for (int k = 0; k < 6; ++k) {
        //         action_rl_bg_[k] = action_rl[k];
        //       }
        //       action_rl_bg_valid_ = true;



        //     }

        //     // post-process 
        //     // 1. 写输入
        //     for (int k = 0; k < 6; ++k) 
        //     {
        //       if(k!=2)
        //       {
        //         rl_action_processor.policy_action[k] = action_rl_bg_[k];
        //       }
        //       else
        //       {
        //         rl_action_processor.policy_action[k] = 0.06*action_rl_bg_[k];//// for ppo_eal 0.04: sometimes can not explore; better 0.06 ////   for ppo_only: 0.06
        //       }
              

        //       rl_action_processor.prev_used_action[k] = rl_prev_action_[k];
        //     }
            
        //     rl_action_processor.fingertip_pos = fingertip_pos;
        //     /// check this 
        //     rl_action_processor.fingertip_quat.w() = fingertip_quat[0];
        //     rl_action_processor.fingertip_quat.x() = fingertip_quat[1];
        //     rl_action_processor.fingertip_quat.y() = fingertip_quat[2];
        //     rl_action_processor.fingertip_quat.z() = fingertip_quat[3];


        //     rl_action_processor.fixed_pos_action_frame = target_p_gear_fixed_;

        //     // 2. 处理
        //     rl_action_processor.process();

        //     // 3. 直接读输出
        //     rl_ctrl_target_fingertip_midpoint_pos = rl_action_processor.ctrl_target_fingertip_midpoint_pos;
        //     rl_ctrl_target_fingertip_midpoint_quat = rl_action_processor.ctrl_target_fingertip_midpoint_quat;
        //     rl_ctrl_target_gripper_ = rl_action_processor.ctrl_target_gripper_dof_pos; ///// gripper angle, always zeros;

        //     target_p_3_ = rl_ctrl_target_fingertip_midpoint_pos;
        //     target_q_xyzw_3_[0] = rl_ctrl_target_fingertip_midpoint_quat.x();
        //     target_q_xyzw_3_[1] = rl_ctrl_target_fingertip_midpoint_quat.y();
        //     target_q_xyzw_3_[2] = rl_ctrl_target_fingertip_midpoint_quat.z();
        //     target_q_xyzw_3_[3] = rl_ctrl_target_fingertip_midpoint_quat.w();

        //     rl_ctrl_target_fingertip_midpoint_pos_bg_ = rl_ctrl_target_fingertip_midpoint_pos;
        //     rl_ctrl_target_fingertip_midpoint_quat_bg_ = rl_ctrl_target_fingertip_midpoint_quat;


        //     // 4. 更新缓存
        //     for (int k = 0; k < 6; ++k) 
        //     {          
        //       rl_prev_action_[k] = rl_action_processor.used_action[k];  
        //     }        
            

        //     rl_target_valid_ = true;
      

        //     if (kin_initialized_ && rl_target_valid_ && (!rl_test_)) {

        //       rl_ik_request_count_ += 1;
        //       /// target 3: for plugin
        //       target = make_target_se3(3);
            

        //       q_init_arm = q_.head<7>();
        //       q_sol = q_init_arm;
        //       double final_err2 = -1.0;
        //       bool ok2 = pino_.ik(target, q_init_arm, q_sol, ik_opt_, &final_err2);

        //       if (!ok2) {
        //         rl_ik_fail_count_ += 1;
        //         RCLCPP_WARN(get_node()->get_logger(),
        //                     "RL Pinocchio IK failed (final_err=%.6f); keep configured q_goal.", final_err2);

        //       } else {
        //         // only overwrite first 7 arm joints; keep 8th (gripper) unchanged
        //         q_goal_3_.head<7>() = q_sol;
        //         rl_ik_success_count_ += 1;
        //         // RCLCPP_INFO(get_node()->get_logger(), "RL Pinocchio IK success. Using IK solution as q_goal (arm only).");
        //       }
        //     }

        //     // rl_ik_target_pos_ = rl_ctrl_target_fingertip_midpoint_pos;
        //     // rl_ik_target_quat_ = rl_ctrl_target_fingertip_midpoint_quat;
        //     // rl_ik_q_init_ = q_.head<7>();

        //     // rl_ik_request_.store(true, std::memory_order_release);
        //     // rl_ik_request_count_ += 1;

        //     // if (rl_ik_solution_ready_.load(std::memory_order_acquire)) {
        //     //   rl_ik_solution_ready_.store(false, std::memory_order_release);
        //     //   q_goal_3_.head<7>() = rl_ik_q_sol_;
        //     // }

        //     q_interp_start_ = q_des_;

        //   }

        
          

 

        //   if(elapsed_time_ >= rl_duration_)
        //   {
        //     rl_finished_ = true;

        //     if (!rl_finish_latched_.exchange(true, std::memory_order_relaxed)) {
        //       save_requested_.store(true, std::memory_order_release);
        //     }

        //     std::cout<<"!!! rl_policy_finished !!!"<<std::endl;
        //   }
        // }
        // else 
        // {
        //   // When finished: keep previous behavior
        //   if (hold_position_) {
        //     q_des_ = q_goal_3_;
        //   } else {
        //     q_des_ = q_;
        //   }
        // }
        // break;        
      }   
      default: 
      {
        // For all other commands: keep current reference unchanged.
        // That is, keep tracking q_des_ already defined in warmup_stage (or previous cycles).
        // NOTE: Do not modify q_des_ here.
        break;
      }
    
    }
    // 1) Joint-space PD torque for all 7 joints (ALWAYS computed)
    for (size_t i = 0; i < 7; ++i) 
    {
      const double kp = k_gains_(static_cast<int>(i));
      const double kd = d_gains_(static_cast<int>(i));
      tau_joint_pd_(static_cast<int>(i)) =
        kp * (q_des_(static_cast<int>(i)) - q_(static_cast<int>(i)))
        - kd * dq_(static_cast<int>(i));
    }

    // 2) Cartesian OSC torque for arm (7 joints)
    tau_osc_cmd_.setZero();

    q_ref_arm = q_des_.head<7>();

    if (kin_initialized_) {
      ee_ref = pino_.fk(q_ref_arm);
      ee6_ref_ = se3ToXyzRpy(ee_ref);

      fingertip_pos_fk_dbg_ = ee_ref.translation();
    }
    else
    {
      ee6_ref_.setZero();
    }
    /// in the RUN_RL policy, we have the following ee pos////
    if ((effective_cmd == TaskCmd::RUN_RL) && (!rl_test_))
    {
      ee6_ref_ = poseToXyzRpy(rl_ctrl_target_fingertip_midpoint_pos,rl_ctrl_target_fingertip_midpoint_quat);
      ee_ref = pinocchio::SE3(
          rl_ctrl_target_fingertip_midpoint_quat.normalized().toRotationMatrix(),
          rl_ctrl_target_fingertip_midpoint_pos
      );      
    }
    
    if (kin_initialized_ && osc_initialized_) {
      Vector7d tau_arm;
      osc_->computeTorque(q_.head<7>(), dq_.head<7>(), ee_ref, tau_arm, elapsed_time_, nullptr);
      tau_osc_cmd_.head<7>() = tau_arm;
      // std::cout << "[computing] tau_osc:"
      //           << std::endl;   
    } else {
      tau_osc_cmd_.head<7>() = tau_joint_pd_.head<7>();
    }

    // 3) Select which torque to SEND (with ramp when switching to OSC)
    const bool use_osc_to_send = (catersian_imp_ && kin_initialized_ && osc_initialized_);
    // Detect switching edge
    if (use_osc_to_send && !prev_use_osc) {
      // Rising edge: start ramp from PD
      osc_blend_alpha = 0.0;
    }
    if ((!use_osc_to_send) || ((effective_cmd == TaskCmd::NONE))) {
      // If OSC not used, keep alpha at 0 (pure PD)
      osc_blend_alpha = 0.0;
    } else {
      // Increase alpha towards 1
      if (osc_blend_duration <= 1e-6) {
        osc_blend_alpha = 1.0;
      } else {
        osc_blend_alpha += dt / osc_blend_duration;
        if (osc_blend_alpha > 1.0) osc_blend_alpha = 1.0;
      }
    }
    
    /// for safety test ////
    osc_blend_alpha = std::min(osc_blend_alpha, osc_blend_alpha_gain_);

    prev_use_osc = use_osc_to_send;

    // Blend torques
    if (use_osc_to_send) {
      tau_to_send = (1.0 - osc_blend_alpha) * tau_joint_pd_ + osc_blend_alpha * tau_osc_cmd_;
    } else {
      tau_to_send = tau_joint_pd_;
    }

    tau_pd_osc_diff_bg_ = tau_osc_cmd_ - tau_joint_pd_;
    {
      // // debug==============torque command ============
      // if ((effective_cmd == TaskCmd::MOVE_POSE_1) &&
      // (!move_1_finished_) &&
      // (elapsed_time_<1)&&(N_rt_ % 20==0))
      // {
        // pos/rot error norms
        pos_err_bg_ = ee_ref.translation() - ee_real.translation();
        const Eigen::Matrix3d R_err = ee_ref.rotation() * ee_real.rotation().transpose();
        const Eigen::AngleAxisd aa(R_err);
        rot_err_angle_bg_ = aa.angle();  // rad
      // }      
    }
  

    // /// rt test
    if(rl_target_collect_)
    {
      tau_to_send.setZero();
    }


    for (size_t i = 0; i < 7; ++i) {
      command_interfaces_[eff_cmd_idx_[i]]
        .set_value(tau_to_send(i));
    }
  }



  append_log_sample_rt();
  // // publish data (per your request: do not change 3/4, keep as-is)
  // joint2simulation_.header.stamp = get_node()->now();
  // joint2simulation_.header.frame_id = "fr3";

  // for (int j = 0; j < 7; ++j) {
  //   joint2simulation_.position[j] = q_[j];
  // }
  // for (int j = 0; j < 7; ++j) {
  //   joint2simulation_.position[j + 9] = q_des_[j];
  // }
  // for (int j = 0; j < 7; ++j) {
  //   joint2simulation_.position[j + 18] = dq_[j];
  // }
  // for (int j = 0; j < 7; ++j) {
  //   joint2simulation_.position[j + 27] = dq_[j];
  // }
  // for (int j = 0; j < 7; ++j) {
  //   joint2simulation_.position[j + 36] = tau_joint_pd_[j];
  // }
  // for (int j = 0; j < 7; ++j) {
  //   joint2simulation_.position[j + 45] = tau_osc_cmd_[j];
  // }
  // for (int j = 0; j < 7; ++j) {
  //   joint2simulation_.position[j + 54] = tau_meas_[j];
  // }
  // for (int j = 0; j < 6; ++j) {
  //   joint2simulation_.position[j + 63] = ee6_ref_[j];
  // }
  // for (int j = 0; j < 6; ++j) {
  //   joint2simulation_.position[j + 69] = ee6_mea_[j];
  // }

  // joint2simulation_.position[75] = kin_initialized_ ? 1.0 : 0.0;
  // joint2simulation_.position[76] = osc_initialized_ ? 1.0 : 0.0;
  // joint2simulation_.position[77] = elapsed_time_;

  // for (int j = 0; j < 100; ++j) {
  //   joint2simulation_.velocity[j] = elapsed_time_;
  // }

  // gait_data_pub_->publish(joint2simulation_);



  /// minimal test for rt loop
  // if (!read_state()) {
  //   return controller_interface::return_type::ERROR;
  // }

  // if (!cmd_map_built_) {
  //   eff_cmd_idx_.fill(-1);
  //   for (size_t k = 0; k < command_interfaces_.size(); ++k) {
  //     const auto& ci = command_interfaces_[k];
  //     const std::string full_name = ci.get_name();
  //     for (size_t j = 0; j < 7; ++j) {
  //       if (full_name == joint_names_[j] + "/effort") {
  //         eff_cmd_idx_[j] = static_cast<int>(k);
  //       }
  //     }
  //   }
  //   for (size_t j = 0; j < 7; ++j) {
  //     if (eff_cmd_idx_[j] < 0) {
  //       return controller_interface::return_type::ERROR;
  //     }
  //   }
  //   cmd_map_built_ = true;
  // }

  // for (size_t i = 0; i < 7; ++i) {
  //   command_interfaces_[eff_cmd_idx_[i]].set_value(0.0);
  // }




  return controller_interface::return_type::OK;
}

bool MoveCatersianImpWithGripper::read_state()
{
  // static bool map_built = false;
  // static std::array<int, 7> pos_idx;
  // static std::array<int, 7> vel_idx;
  // static std::array<int, 7> eff_idx;

  // if (!map_built) {
  //   pos_idx.fill(-1);
  //   vel_idx.fill(-1);
  //   eff_idx.fill(-1);

  //   for (size_t k = 0; k < state_interfaces_.size(); ++k) {
  //     const auto & si = state_interfaces_[k];
  //     const std::string full = si.get_name();

  //     for (size_t j = 0; j < 7; ++j) {
  //       const std::string & jn = joint_names_[j];
  //       if (full == (jn + "/position")) pos_idx[j] = static_cast<int>(k);
  //       if (full == (jn + "/velocity")) vel_idx[j] = static_cast<int>(k);
  //       if (full == (jn + "/effort"))   eff_idx[j] = static_cast<int>(k);
  //     }
  //   }

  //   for (size_t j = 0; j < 7; ++j) {
  //     if (pos_idx[j] < 0 || vel_idx[j] < 0 || eff_idx[j] < 0) {
  //       // NO logging in RT path: record and return
  //       rt_missing_state_joint_.store(static_cast<int>(j));
  //       return;
  //     }
  //   }

  //   map_built = true;

  //   // NO RCLCPP_INFO mapping prints here (RT path)
  // }

  // for (size_t j = 0; j < 7; ++j) {
  //   q_(static_cast<int>(j))  = state_interfaces_[static_cast<size_t>(pos_idx[j])].get_value();
  //   dq_(static_cast<int>(j)) = state_interfaces_[static_cast<size_t>(vel_idx[j])].get_value();
  //   tau_meas_(static_cast<int>(j)) = state_interfaces_[static_cast<size_t>(eff_idx[j])].get_value();
  // }

  if (!state_map_built_) {
    pos_idx_.fill(-1);
    vel_idx_.fill(-1);
    eff_idx_.fill(-1);

    for (size_t k = 0; k < state_interfaces_.size(); ++k) {
      const auto& si = state_interfaces_[k];
      const std::string full_name = si.get_name();

      for (size_t j = 0; j < 7; ++j) {
        const auto& jn = joint_names_[j];
        if (full_name == jn + "/position") {
          pos_idx_[j] = static_cast<int>(k);
        } else if (full_name == jn + "/velocity") {
          vel_idx_[j] = static_cast<int>(k);
        } else if (full_name == jn + "/effort") {
          eff_idx_[j] = static_cast<int>(k);
        }
      }
    }

    for (size_t j = 0; j < 7; ++j) {
      if (pos_idx_[j] < 0 || vel_idx_[j] < 0 || eff_idx_[j] < 0) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "State interface mapping failed for joint %s: pos=%d vel=%d eff=%d",
            joint_names_[j].c_str(),
            pos_idx_[j], vel_idx_[j], eff_idx_[j]);
        rt_missing_state_joint_.store(static_cast<int>(j), std::memory_order_relaxed);
        return false;
      }
    }

    state_map_built_ = true;
  }

  for (size_t j = 0; j < 7; ++j) {
    q_(j) = state_interfaces_[pos_idx_[j]].get_value();
    dq_(j) = state_interfaces_[vel_idx_[j]].get_value();
    tau_meas_(j) = state_interfaces_[eff_idx_[j]].get_value();
  }


  dq_filtered_ = 0.9 * dq_filtered_ + 0.1 * dq_;

  return true;
}

std::string MoveCatersianImpWithGripper::resolve_ee_frame_name() const
{
  if (!ee_frame_override_.empty()) {
    return ee_frame_override_;
  }
  if (use_robot_hand_) {
    return std::string("fr3_hand");
  }
  return std::string("fr3_flange");
}

Eigen::Matrix<double, 6, 1>
MoveCatersianImpWithGripper::se3ToXyzRpy(const pinocchio::SE3& T) const
{
  Eigen::Matrix<double, 6, 1> out;
  out.setZero();

  // position
  out.segment<3>(0) = T.translation();

  // rotation -> RPY (ZYX)
  const Eigen::Matrix3d R = T.rotation();

  const double yaw   = std::atan2(R(1,0), R(0,0));
  const double pitch = std::atan2(-R(2,0), std::sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));
  const double roll  = std::atan2(R(2,1), R(2,2));

  out(3) = roll;
  out(4) = pitch;
  out(5) = yaw;
  return out;
}

Eigen::Matrix<double, 7, 1>
MoveCatersianImpWithGripper::se3ToXyzQuat(const pinocchio::SE3& T) const
{
  Eigen::Matrix<double, 7, 1> out;
  out.setZero();

  // position
  out.segment<3>(0) = T.translation();

  // rotation matrix
  const Eigen::Matrix3d R = T.rotation();

  // convert to quaternion
  Eigen::Quaterniond q(R);

  // ⚠️ Eigen 默认内部是 (x,y,z,w)
  // 但 q.w(), q.x(), q.y(), q.z() 返回的是正确顺序

  q.normalize();

  out(3) = q.w();
  out(4) = q.x();
  out(5) = q.y();
  out(6) = q.z();

  return out;
}


static Eigen::Vector3d quatToAxisAngle(const Eigen::Quaterniond& q_in)
{
  // q_in assumed normalized
  Eigen::Quaterniond q = q_in;

  // Ensure shortest rotation: make w >= 0  (same as torch sign trick)
  if (q.w() < 0.0) q.coeffs() *= -1.0;   // coeffs() = (x,y,z,w)

  // Convert to axis-angle vector = axis * angle
  // angle in [0, pi]
  const double w = std::clamp(q.w(), -1.0, 1.0);
  const double angle = 2.0 * std::acos(w);

  // For small angles, avoid division by near-zero
  const double s = std::sqrt(std::max(0.0, 1.0 - w*w)); // = sin(angle/2)
  Eigen::Vector3d axis;
  if (s < 1e-8) {
    axis.setZero();
  } else {
    axis << q.x() / s, q.y() / s, q.z() / s;
  }

  return axis * angle; // axis-angle vector (3)
}

void MoveCatersianImpWithGripper::updateEeVelFd(
    const Eigen::Vector3d& fingertip_pos_now,
    const Eigen::Quaterniond& fingertip_quat_now,
    double dt,
    Eigen::Vector3d& ee_linvel_fd_out,
    Eigen::Vector3d& ee_angvel_fd_out)
{
  // Safety: dt must be > 0
  if (dt <= 1e-9) {
    ee_linvel_fd_out.setZero();
    ee_angvel_fd_out.setZero();
    return;
  }

  if (!fd_inited_) {
    prev_fingertip_pos_ = fingertip_pos_now;
    prev_fingertip_quat_ = fingertip_quat_now;
    fd_inited_ = true;

    // Match training: initial fd velocity = 0
    ee_linvel_fd_out.setZero();
    ee_angvel_fd_out.setZero();
    return;
  }

  // 1) linear velocity fd
  ee_linvel_fd_out = (fingertip_pos_now - prev_fingertip_pos_) / dt;

  // 2) angular velocity fd (axis-angle / dt)
  // rot_diff = q_now * conj(q_prev)
  Eigen::Quaterniond q_prev_conj = prev_fingertip_quat_.conjugate();
  Eigen::Quaterniond q_diff = fingertip_quat_now * q_prev_conj;
  q_diff.normalize();

  const Eigen::Vector3d rot_diff_aa = quatToAxisAngle(q_diff);
  ee_angvel_fd_out = rot_diff_aa / dt;

  // update buffers
  prev_fingertip_pos_ = fingertip_pos_now;
  prev_fingertip_quat_ = fingertip_quat_now;
}




pinocchio::SE3 MoveCatersianImpWithGripper::make_target_se3(int i) const
{
  Eigen::Quaterniond q(1, 0, 0, 0);
  Eigen::Vector3d p = Eigen::Vector3d::Zero();

  if (i == 1) {
    q = Eigen::Quaterniond(target_q_xyzw_[3],
                           target_q_xyzw_[0],
                           target_q_xyzw_[1],
                           target_q_xyzw_[2]);  // w,x,y,z
    p = target_p_;
  } 
  else 
  {
    if (i == 2) 
    {
      q = Eigen::Quaterniond(target_q_xyzw_2_[3],
                            target_q_xyzw_2_[0],
                            target_q_xyzw_2_[1],
                            target_q_xyzw_2_[2]);  // w,x,y,z
      p = target_p_2_;
    }
    else
    {
      q = Eigen::Quaterniond(target_q_xyzw_3_[3],
                            target_q_xyzw_3_[0],
                            target_q_xyzw_3_[1],
                            target_q_xyzw_3_[2]);  // w,x,y,z
      p = target_p_3_;      
    }
  }

  q.normalize();
  const Eigen::Matrix3d R = q.toRotationMatrix();
  return pinocchio::SE3(R, p);

}


Eigen::Matrix<double, 6, 1> MoveCatersianImpWithGripper::poseToXyzRpy(
    const Eigen::Vector3d& pos,
    const Eigen::Quaterniond& quat)
{
    Eigen::Matrix<double, 6, 1> xyzrpy;
    Eigen::Vector3d rpy = quat.toRotationMatrix().eulerAngles(0, 1, 2);

    xyzrpy << pos.x(), pos.y(), pos.z(),
              rpy[0], rpy[1], rpy[2];
    return xyzrpy;
}


bool MoveCatersianImpWithGripper::init_pinocchio_if_needed_with_wait()
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

bool MoveCatersianImpWithGripper::init_cartesian_osc_if_needed_with_wait()
{
  if (osc_initialized_) {
    return true;
  }

  const auto t0 = std::chrono::steady_clock::now();
  while (!urdf_received_.load()) {
    const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0);
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

  // Build model from URDF XML
  pinocchio::Model model;
  try {
    pinocchio::urdf::buildModelFromXML(urdf, model);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to build Pinocchio model from URDF: %s", e.what());
    return false;
  }

  osc_ = std::make_unique<CartesianImpedanceOscRT>(model, 7);

  CartesianImpedanceOscConfig cfg = CartesianImpedanceOscConfig::Default();
  cfg.ee_frame_name = resolve_ee_frame_name();
  if (!osc_->configure(cfg)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "OSC configure failed. ee_frame='%s'",
      cfg.ee_frame_name.c_str());
    osc_.reset();
    return false;
  }
  
  osc_->printCurrentConfig();
  osc_initialized_ = true;
  RCLCPP_INFO(get_node()->get_logger(), "OSC initialized. ee_frame='%s'", cfg.ee_frame_name.c_str());
  return true;
}


void MoveCatersianImpWithGripper::reset_interface_maps()
{
  state_map_built_ = false;
  pos_idx_.fill(-1);
  vel_idx_.fill(-1);
  eff_idx_.fill(-1);

  cmd_map_built_ = false;
  eff_cmd_idx_.fill(-1);
}


void MoveCatersianImpWithGripper::runRlIkNonRt()
{
  // 没有请求，就不做任何事
  if (!rl_ik_request_.load(std::memory_order_acquire)) {
    return;
  }

  // 清掉请求标志，避免重复算同一份
  rl_ik_request_.store(false, std::memory_order_release);

  if (!kin_initialized_) {
    return;
  }

  // 由缓存的目标位置/姿态构造 SE3
  pinocchio::SE3 target_se3(
      rl_ik_target_quat_.normalized().toRotationMatrix(),
      rl_ik_target_pos_);

  Vector7d q_sol = rl_ik_q_init_;
  double final_err = -1.0;

  const bool ok = pino_.ik(target_se3, rl_ik_q_init_, q_sol, ik_opt_, &final_err);

  rl_ik_last_ok_ = ok;
  rl_ik_final_err_ = final_err;

  if (!ok) {
    rl_ik_fail_count_ += 1;
    RCLCPP_WARN(
        get_node()->get_logger(),
        "NRT RL IK failed (final_err=%.6f). Keep previous q_goal_3_.",
        final_err);
    return;
  }

  rl_ik_q_sol_ = q_sol;
  rl_ik_solution_ready_.store(true, std::memory_order_release);
  rl_ik_success_count_ += 1;
}

std::array<double, MoveCatersianImpWithGripper::kWrenchDim> MoveCatersianImpWithGripper::computeWrenchExtOBias() const
{
  std::array<double, MoveCatersianImpWithGripper::kWrenchDim> mean{};
  mean.fill(0.0);

  size_t valid_size = wrench_ext_o_buffer_full_ ? kBiasWindowSize : wrench_ext_o_buffer_idx_;
  if (valid_size == 0) {
    return mean;
  }

  for (size_t i = 0; i < valid_size; ++i) {
    for (size_t j = 0; j < MoveCatersianImpWithGripper::kWrenchDim; ++j) {
      mean[j] += wrench_ext_o_buffer_[i][j];
    }
  }

  for (size_t j = 0; j < MoveCatersianImpWithGripper::kWrenchDim; ++j) {
    mean[j] /= static_cast<double>(valid_size);
  }

  return mean;
}

void MoveCatersianImpWithGripper::init_log_schema_and_buffer()
{
  // ========== define variable names here ==========

  log_var_names_.clear();

  log_var_names_.push_back("t");

  for (int i = 0; i < 7; ++i) log_var_names_.push_back("q" + std::to_string(i));
  for (int i = 0; i < 7; ++i) log_var_names_.push_back("q_des" + std::to_string(i));
  for (int i = 0; i < 7; ++i) log_var_names_.push_back("dq" + std::to_string(i));
  for (int i = 0; i < 7; ++i) log_var_names_.push_back("tau_meas" + std::to_string(i));
  for (int i = 0; i < 7; ++i) log_var_names_.push_back("tau_cmd" + std::to_string(i));
  for (int i = 0; i < 6; ++i) log_var_names_.push_back("hand_contact_wrench" + std::to_string(i));
  for (int i = 0; i < 6; ++i) log_var_names_.push_back("ee_ref" + std::to_string(i));                                                        
  for (int i = 0; i < 6; ++i) log_var_names_.push_back("ee6_meas" + std::to_string(i));

  log_var_names_.push_back("cmd");
  log_var_names_.push_back("move_1_finished");
  log_var_names_.push_back("grasp_finished");
  log_var_names_.push_back("move_2_finished");
  log_var_names_.push_back("rl_finished");

  log_num_vars_ = log_var_names_.size();

  // pre-allocate once in non-RT
  log_buffer_.assign(log_capacity_steps_ * log_num_vars_, 0.0);

  log_steps_written_.store(0, std::memory_order_relaxed);
  log_started_.store(false, std::memory_order_relaxed);
  log_save_done_.store(false, std::memory_order_relaxed);
  log_time_start_ = 0.0;
}

void MoveCatersianImpWithGripper::append_log_sample_rt()
{
  if (!log_started_.load(std::memory_order_relaxed)) {
    return;
  }

  const size_t step = log_steps_written_.load(std::memory_order_relaxed);
  if (step >= log_capacity_steps_) {
    return;  // buffer full: silently stop appending
  }

  const size_t base = step * log_num_vars_;
  size_t k = 0;

  log_buffer_[base + k++] = time_total_ - log_time_start_;

  for (int i = 0; i < 7; ++i) log_buffer_[base + k++] = q_[i];
  for (int i = 0; i < 7; ++i) log_buffer_[base + k++] = q_des_[i];
  for (int i = 0; i < 7; ++i) log_buffer_[base + k++] = dq_[i];
  for (int i = 0; i < 7; ++i) log_buffer_[base + k++] = tau_meas_[i];
  for (int i = 0; i < 7; ++i) log_buffer_[base + k++] = tau_to_send[i];
  for (int i = 0; i < 6; ++i) log_buffer_[base + k++] = hand_wrench[i];
  for (int i = 0; i < 6; ++i) log_buffer_[base + k++] = ee6_ref_[i];
  for (int i = 0; i < 6; ++i) log_buffer_[base + k++] = ee6_mea_[i]; 


  log_buffer_[base + k++] = static_cast<double>(last_cmd_.load(std::memory_order_relaxed));
  log_buffer_[base + k++] = move_1_finished_ ? 1.0 : 0.0;
  log_buffer_[base + k++] = grasp_finished_ ? 1.0 : 0.0;
  log_buffer_[base + k++] = move_2_finished_ ? 1.0 : 0.0;
  log_buffer_[base + k++] = rl_finished_ ? 1.0 : 0.0;

  log_steps_written_.store(step + 1, std::memory_order_relaxed);
}


void MoveCatersianImpWithGripper::save_log_to_txt_nonrt(const std::string& reason)
{
  if (log_save_done_.exchange(true)) {
    return;  // already saved
  }

  const size_t steps = log_steps_written_.load(std::memory_order_acquire);
  if (steps == 0 || log_num_vars_ == 0) {
    std::cerr << "[log] No log data to save. reason=" << reason << std::endl;
    return;
  }

  // stop further logging
  log_started_.store(false, std::memory_order_release);

  std::filesystem::create_directories("logs");

  std::ofstream ofs(log_save_path_);
  if (!ofs.is_open()) {
    std::cerr << "[log] Failed to open log file: " << log_save_path_ << std::endl;
    return;
  }

  // header
  for (size_t j = 0; j < log_num_vars_; ++j) {
    ofs << log_var_names_[j];
    if (j + 1 < log_num_vars_) ofs << " ";
  }
  ofs << "\n";

  // T rows, N columns
  for (size_t i = 0; i < steps; ++i) {
    const size_t base = i * log_num_vars_;
    for (size_t j = 0; j < log_num_vars_; ++j) {
      ofs << std::setprecision(10) << log_buffer_[base + j];
      if (j + 1 < log_num_vars_) ofs << " ";
    }
    ofs << "\n";
  }

  ofs.close();

  std::cout << "[log] Saved log to " << log_save_path_
            << ", reason=" << reason
            << ", steps=" << steps
            << ", vars=" << log_num_vars_
            << std::endl;

  std::cout << "[log dbg] reason=" << reason
            << ", steps=" << steps
            << ", t_first=" << (steps > 0 ? log_buffer_[0] : -1.0)
            << ", t_last=" << (steps > 0 ? log_buffer_[(steps - 1) * log_num_vars_] : -1.0)
            << ", move1=" << (move_1_finished_ ? 1 : 0)
            << ", grasp=" << (grasp_finished_ ? 1 : 0)
            << ", move2=" << (move_2_finished_ ? 1 : 0)
            << ", rl=" << (rl_finished_ ? 1 : 0)
            << ", last_cmd=" << static_cast<int>(last_cmd_.load(std::memory_order_relaxed))
            << std::endl;
}

void MoveCatersianImpWithGripper::save_log_snapshot_nonrt(const std::string& reason)
{
  const size_t steps = log_steps_written_.load(std::memory_order_acquire);
  if (steps == 0 || log_num_vars_ == 0) {
    std::cerr << "[log] No log data to snapshot. reason=" << reason << std::endl;
    return;
  }

  std::filesystem::create_directories("logs");

  const std::string snapshot_path = make_log_snapshot_file_path(manual_snapshot_seq_++);
  std::ofstream ofs(snapshot_path);
  if (!ofs.is_open()) {
    std::cerr << "[log] Failed to open snapshot file: " << snapshot_path << std::endl;
    return;
  }

  // header
  for (size_t j = 0; j < log_num_vars_; ++j) {
    ofs << log_var_names_[j];
    if (j + 1 < log_num_vars_) ofs << " ";
  }
  ofs << "\n";

  // T rows, N columns
  for (size_t i = 0; i < steps; ++i) {
    const size_t base = i * log_num_vars_;
    for (size_t j = 0; j < log_num_vars_; ++j) {
      ofs << std::setprecision(10) << log_buffer_[base + j];
      if (j + 1 < log_num_vars_) ofs << " ";
    }
    ofs << "\n";
  }

  ofs.close();

  std::cout << "[log] Snapshot saved to " << snapshot_path
            << ", reason=" << reason
            << ", steps=" << steps
            << ", vars=" << log_num_vars_
            << std::endl;
}

void MoveCatersianImpWithGripper::save_log_autosave_nonrt(const std::string& reason)
{
  const size_t steps = log_steps_written_.load(std::memory_order_acquire);
  if (steps == 0 || log_num_vars_ == 0) {
    return;
  }

  std::filesystem::create_directories("logs");

  std::string autosave_path = log_save_path_;
  const std::string suffix = "_RL_log.txt";
  const auto pos = autosave_path.rfind(suffix);
  if (pos != std::string::npos) {
    autosave_path.replace(pos, suffix.size(), "_autosave_RL_log.txt");
  } else {
    autosave_path += "_autosave.txt";
  }

  std::ofstream ofs(autosave_path);
  if (!ofs.is_open()) {
    std::cerr << "[log] Failed to open autosave file: " << autosave_path << std::endl;
    return;
  }

  for (size_t j = 0; j < log_num_vars_; ++j) {
    ofs << log_var_names_[j];
    if (j + 1 < log_num_vars_) ofs << " ";
  }
  ofs << "\n";

  for (size_t i = 0; i < steps; ++i) {
    const size_t base = i * log_num_vars_;
    for (size_t j = 0; j < log_num_vars_; ++j) {
      ofs << std::setprecision(10) << log_buffer_[base + j];
      if (j + 1 < log_num_vars_) ofs << " ";
    }
    ofs << "\n";
  }

  ofs.close();

  // keep this light; do not print every 0.5 s unless debugging
  // std::cout << "[log] Autosave updated: " << autosave_path
  //           << ", reason=" << reason
  //           << ", steps=" << steps << std::endl;
}







std::string MoveCatersianImpWithGripper::make_log_file_path() const
{
  // create a cleaner stem from policy_name_
  std::string policy_stem = policy_name_;
  const std::string ext = ".onnx";
  if (policy_stem.size() >= ext.size() &&
      policy_stem.substr(policy_stem.size() - ext.size()) == ext) {
    policy_stem = policy_stem.substr(0, policy_stem.size() - ext.size());
  }

  // optional: replace '/' if user later passes subdir-like names
  std::replace(policy_stem.begin(), policy_stem.end(), '/', '_');
  std::replace(policy_stem.begin(), policy_stem.end(), ' ', '_');

  // current local time
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);

  std::tm tm_buf;
  #ifdef _WIN32
    localtime_s(&tm_buf, &t);
  #else
    localtime_r(&t, &tm_buf);
  #endif

  std::ostringstream oss;
  oss << "logs/"
      << policy_stem << "_"
      << std::put_time(&tm_buf, "%Y%m%d_%H%M%S")
      << "_RL_log.txt";

  return oss.str();
}

std::string MoveCatersianImpWithGripper::make_log_snapshot_file_path(int snapshot_idx) const
{
  std::string policy_stem = policy_name_;
  const std::string ext = ".onnx";
  if (policy_stem.size() >= ext.size() &&
      policy_stem.substr(policy_stem.size() - ext.size()) == ext) {
    policy_stem = policy_stem.substr(0, policy_stem.size() - ext.size());
  }

  std::replace(policy_stem.begin(), policy_stem.end(), '/', '_');
  std::replace(policy_stem.begin(), policy_stem.end(), ' ', '_');

  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);

  std::tm tm_buf;
  #ifdef _WIN32
    localtime_s(&tm_buf, &t);
  #else
    localtime_r(&t, &tm_buf);
  #endif

  std::ostringstream oss;
  oss << "logs/"
      << policy_stem << "_"
      << std::put_time(&tm_buf, "%Y%m%d_%H%M%S")
      << "_snapshot_" << snapshot_idx
      << "_RL_log.txt";

  return oss.str();
}

// =====================================================================================================================================================
// Additional functions to compute the probe points and clearance
void MoveCatersianImpWithGripper::initProbeFrameIds()
{
  probe_frame_ids_ready_ = false;

  if (!kin_initialized_ || !pino_.isInitialized())
  {
    return;
  }

  for (size_t i = 0; i < probe_frame_names_.size(); ++i)
  {
    probe_frame_ids_[i] = pino_.frameIdByName(probe_frame_names_[i]);

    if (probe_frame_ids_[i] == pinocchio::FrameIndex(-1))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Probe frame '%s' not found in Pinocchio model.",
        probe_frame_names_[i].c_str());

      probe_frame_ids_ready_ = false;
      return;
    }

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Probe frame '%s' id=%lu",
      probe_frame_names_[i].c_str(),
      static_cast<unsigned long>(probe_frame_ids_[i]));
  }

  probe_frame_ids_ready_ = true;
}

double MoveCatersianImpWithGripper::computeCylinderClearance(
  const Eigen::Vector3d& p,
  const Eigen::Vector3d& obstacle_pos,
  double obstacle_radius,
  double obstacle_height,
  double link_radius) const
{
  const double cyl_radius = obstacle_radius;
  const double cyl_half_h = 0.5 * obstacle_height;

  const double dx = p.x() - obstacle_pos.x();
  const double dy = p.y() - obstacle_pos.y();

  const double radial_dist = std::sqrt(dx * dx + dy * dy + 1e-8);

  // Signed radial distance from infinite cylinder surface
  const double radial_out = radial_dist - cyl_radius;

  // Vertical distance from finite cylinder slab
  const double dz = std::abs(p.z() - obstacle_pos.z()) - cyl_half_h;

  const double outside_radial = std::max(radial_out, 0.0);
  const double outside_vertical = std::max(dz, 0.0);

  const double outside_dist =
      std::sqrt(
          outside_radial * outside_radial +
          outside_vertical * outside_vertical +
          1e-8);

  const double inside_dist =
      std::min(std::max(radial_out, dz), 0.0);

  const double sdf_cylinder = outside_dist + inside_dist;

  const double clearance = sdf_cylinder - link_radius;

  return clearance;
}

void MoveCatersianImpWithGripper::updateClearObsFromPinocchio()
{
  if (!kin_initialized_ || !probe_frame_ids_ready_)
  {
    clear_obs_.fill(0.0f);
    return;
  }

  std::array<pinocchio::SE3, 4> T;

  const bool ok = pino_.FramePoses4Rt(
      q_.head<7>(),
      probe_frame_ids_,
      T);

  if (!ok)
  {
    clear_obs_.fill(0.0f);
    return;
  }

  const Eigen::Vector3d p2 = T[0].translation();
  const Eigen::Vector3d p4 = T[1].translation();
  const Eigen::Vector3d p6 = T[2].translation();
  const Eigen::Vector3d p7 = T[3].translation();

  // pg = robot_grasp_pos in IsaacLab.
  // Qui usiamo il fingertip/midpoint già calcolato nel controller.
  const Eigen::Vector3d pg = fingertip_pos;

  const Eigen::Quaterniond hand_quat(T[3].rotation());

  auto lerp = [](const Eigen::Vector3d& a,
                 const Eigen::Vector3d& b,
                 double t) -> Eigen::Vector3d {
    return (1.0 - t) * a + t * b;
  };

  int k = 0;

  // anchors = [p2, p4, p6, p7, pg]
  probe_points_w_[k++] = p2;
  probe_points_w_[k++] = p4;
  probe_points_w_[k++] = p6;
  probe_points_w_[k++] = p7;
  probe_points_w_[k++] = pg;

  // seg_24 = lerp(p2, p4, [0.25, 0.5, 0.75])
  probe_points_w_[k++] = lerp(p2, p4, 0.25);
  probe_points_w_[k++] = lerp(p2, p4, 0.50);
  probe_points_w_[k++] = lerp(p2, p4, 0.75);

  // seg_46 = lerp(p4, p6, [0.25, 0.5, 0.75])
  probe_points_w_[k++] = lerp(p4, p6, 0.25);
  probe_points_w_[k++] = lerp(p4, p6, 0.50);
  probe_points_w_[k++] = lerp(p4, p6, 0.75);

  // seg_67 = lerp(p6, p7, [0.33, 0.66])
  probe_points_w_[k++] = lerp(p6, p7, 0.33);
  probe_points_w_[k++] = lerp(p6, p7, 0.66);

  // hand offsets local, same as IsaacLab
  const double sx = 0.04;
  const double sy = 0.04;
  const double sz = 0.03;
  const double tip = 0.22;

  const std::array<Eigen::Vector3d, 7> hand_offsets_local = {{
    Eigen::Vector3d(+sx, 0.0, 0.0),
    Eigen::Vector3d(-sx, 0.0, 0.0),
    Eigen::Vector3d(0.0, +sy, 0.0),
    Eigen::Vector3d(0.0, -sy, 0.0),
    Eigen::Vector3d(0.0, 0.0, +sz),
    Eigen::Vector3d(0.0, 0.0, -sz),
    Eigen::Vector3d(0.0, 0.0, +tip),
  }};

  const Eigen::Matrix3d R_hand = hand_quat.normalized().toRotationMatrix();

  for (const auto& off_local : hand_offsets_local)
  {
    probe_points_w_[k++] = p7 + R_hand * off_local;
  }

  // Safety check: should be exactly 20
  if (k != 20)
  {
    clear_obs_.fill(0.0f);
    return;
  }

  constexpr double scale = 0.2;

  for (int i = 0; i < 20; ++i)
  {
    double clearance = computeCylinderClearance(
        probe_points_w_[i],
        obstacle_pos_w_,
        obstacle_radius_,
        obstacle_height_,
        link_radius_);

    if (!std::isfinite(clearance))
    {
      clearance = 0.0;
    }

    const double obs = std::tanh(clearance / scale);

    clear_obs_[i] =
        static_cast<float>(std::clamp(obs, -5.0, 5.0));
  }
}







}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(
  franka_example_controllers::MoveCatersianImpWithGripper,
  controller_interface::ControllerInterface)
