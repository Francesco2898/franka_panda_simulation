#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>
#include <unordered_map>

class TaskPipelineNode : public rclcpp::Node {
public:
  TaskPipelineNode() : Node("task_pipeline_node") {
    cmd_topic_ = this->declare_parameter<std::string>("cmd_topic", "/task_cmd");
    state_timeout_sec_ = this->declare_parameter<double>("state_timeout_sec", 0.0);  
    // state_timeout_sec=0 表示不自动超时回到 IDLE（2A阶段先不开）

    sub_ = this->create_subscription<std_msgs::msg::String>(
      cmd_topic_, 10,
      std::bind(&TaskPipelineNode::onCmd, this, std::placeholders::_1));

    // 用 timer 做一个可选的超时机制（2A 默认不开）
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TaskPipelineNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "TaskPipelineNode started. Subscribing '%s'\n"
      "State machine (2A): only prints + gating.\n"
      "Commands: move_pose_1, move_pose_2, grasp, open_gripper, close_gripper, run_rl, stop\n",
      cmd_topic_.c_str());
  }

private:
  enum class State {
    IDLE = 0,
    MOVING,
    GRASPING,
    RUNNING_RL
  };

  static const char* stateName(State s) {
    switch (s) {
      case State::IDLE: return "IDLE";
      case State::MOVING: return "MOVING";
      case State::GRASPING: return "GRASPING";
      case State::RUNNING_RL: return "RUNNING_RL";
      default: return "UNKNOWN";
    }
  }

  void setState(State s, const std::string& reason) {
    if (state_ == s) {
      RCLCPP_INFO(get_logger(), "State stays %s (%s)", stateName(state_), reason.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "State: %s -> %s (%s)",
                stateName(state_), stateName(s), reason.c_str());
    state_ = s;
    last_state_change_time_ = now();
  }

  bool allowCmdInState(const std::string& cmd, State s) {
    // 2A：简单 gating 规则（你后面可以按需要调）
    // IDLE: 允许任何动作开始
    // MOVING: 只允许 stop
    // GRASPING: 只允许 stop
    // RUNNING_RL: 只允许 stop
    if (s == State::IDLE) return true;
    if (cmd == "stop") return true;
    return false;
  }

  void onCmd(const std_msgs::msg::String::SharedPtr msg) {
    const std::string cmd = msg->data;

    RCLCPP_INFO(get_logger(), "[cmd] '%s' received in state=%s", cmd.c_str(), stateName(state_));

    if (!allowCmdInState(cmd, state_)) {
      RCLCPP_WARN(get_logger(),
                  "Command '%s' rejected because current state=%s. (Only 'stop' allowed now)",
                  cmd.c_str(), stateName(state_));
      return;
    }

    // 2A：这里只做打印 + 状态切换（模拟动作开始）
    if (cmd == "move_pose_1") {
      setState(State::MOVING, "start move to pose 1 (simulated)");
      // 2A阶段：不真正调用 move，先假设你之后会在这里触发 move_to_start
    } else if (cmd == "move_pose_2") {
      setState(State::MOVING, "start move to pose 2 (simulated)");
    } else if (cmd == "grasp") {
      setState(State::GRASPING, "start grasp (simulated)");
    } else if (cmd == "open_gripper") {
      setState(State::GRASPING, "start open_gripper (simulated)");
    } else if (cmd == "close_gripper") {
      setState(State::GRASPING, "start close_gripper (simulated)");
    } else if (cmd == "run_rl") {
      setState(State::RUNNING_RL, "start RL policy (simulated)");
    } else if (cmd == "stop") {
      setState(State::IDLE, "stop requested");
    } else {
      RCLCPP_WARN(get_logger(), "Unknown command '%s' (ignored)", cmd.c_str());
    }

    // 2A阶段：为了方便你测试流程，这里提供一个“手动完成”命令也可以加
    // 但我们先不加，保持最小。
  }

  void onTimer() {
    if (state_timeout_sec_ <= 0.0) return;

    const double dt = (now() - last_state_change_time_).seconds();
    if (state_ != State::IDLE && dt > state_timeout_sec_) {
      RCLCPP_WARN(get_logger(),
                  "State %s timeout (%.2fs > %.2fs). Auto back to IDLE.",
                  stateName(state_), dt, state_timeout_sec_);
      setState(State::IDLE, "auto timeout");
    }
  }

private:
  std::string cmd_topic_{"/task_cmd"};
  double state_timeout_sec_{0.0};

  State state_{State::IDLE};
  rclcpp::Time last_state_change_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskPipelineNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
