#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

#include <string>

class KeyboardCmdNode : public rclcpp::Node {
public:
  KeyboardCmdNode() : Node("keyboard_cmd_node") {
    topic_ = this->declare_parameter<std::string>("topic", "/task_cmd");
    print_help_ = this->declare_parameter<bool>("print_help_on_start", true);

    pub_ = this->create_publisher<std_msgs::msg::String>(topic_, 10);

    if (print_help_) {
      printHelp();
    }

    enableRawMode();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(30),
      std::bind(&KeyboardCmdNode::pollKeyboard, this));
  }

  ~KeyboardCmdNode() override {
    disableRawMode();
  }

private:
  void printHelp() {
    RCLCPP_INFO(this->get_logger(),
      "KeyboardCmdNode 已启动，发布 std_msgs/String 到 '%s'\n"
      "keyboard mapping:\n"
      "  0: NONE\n"
      "  1: move_pose_1\n"
      "  2: move_pose_2\n"
      "  g: grasp\n"
      "  o: open_gripper\n"
      "  c: close_gripper\n"
      "  r: run_rl\n"
      "  e: save_log\n"
      "  s: stop\n"
      "  h: help\n",
      //"  q: quit node\n",
      topic_.c_str());
  }

  void enableRawMode() {
    if (raw_mode_enabled_) return;

    if (tcgetattr(STDIN_FILENO, &orig_termios_) == -1) {
      RCLCPP_WARN(this->get_logger(), "tcgetattr 失败，键盘读取可能不可用。");
      return;
    }

    termios raw = orig_termios_;
    raw.c_lflag &= ~(ECHO | ICANON);  // 关闭回显 & 非规范模式
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == -1) {
      RCLCPP_WARN(this->get_logger(), "tcsetattr 失败，键盘读取可能不可用。");
      return;
    }

    raw_mode_enabled_ = true;
  }

  void disableRawMode() {
    if (!raw_mode_enabled_) return;
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
    raw_mode_enabled_ = false;
  }

  bool stdinHasData() {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    const int rv = select(STDIN_FILENO + 1, &set, nullptr, nullptr, &tv);
    return (rv > 0) && FD_ISSET(STDIN_FILENO, &set);
  }

  void publishCmd(const std::string& cmd) {
    std_msgs::msg::String msg;
    msg.data = cmd;
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "cmd -> %s", cmd.c_str());
  }

  void pollKeyboard() {
    if (!stdinHasData()) return;

    char c;
    const ssize_t n = read(STDIN_FILENO, &c, 1);
    if (n <= 0) return;

    switch (c) {
      case '0': publishCmd("NONE"); break;
      case '1': publishCmd("move_pose_1"); break;
      case '2': publishCmd("move_pose_2"); break;

      case 'g': publishCmd("grasp"); break;
      case 'o': publishCmd("open_gripper"); break;
      case 'c': publishCmd("close_gripper"); break;

      case 'r': publishCmd("run_rl"); break;
      case 'e': publishCmd("save_log"); break;
      case 's': publishCmd("stop"); break;

      case 'h':
        printHelp();
        break;

      case 'q':
        RCLCPP_INFO(this->get_logger(), "收到 quit，准备退出。");
        rclcpp::shutdown();
        break;

      default:
        // ignore
        break;
    }
  }

private:
  std::string topic_{"/task_cmd"};
  bool print_help_{true};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  termios orig_termios_{};
  bool raw_mode_enabled_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardCmdNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
