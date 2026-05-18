#!/usr/bin/env python3
"""
Watch fr3 gripper finger joint position from /joint_states.
Print only when value changes.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

TARGET_JOINT = "fr3_finger_joint1"


class WatchFinger(Node):
    def __init__(self):
        super().__init__("watch_finger1")
        self.last_pos = None
        self.sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.cb,
            10,
        )
        self.get_logger().info(
            f"Watching joint '{TARGET_JOINT}' on /joint_states ..."
        )

    def cb(self, msg: JointState):
        if TARGET_JOINT not in msg.name:
            return

        idx = msg.name.index(TARGET_JOINT)
        if idx >= len(msg.position):
            return

        pos = float(msg.position[idx])

        if self.last_pos is None or abs(pos - self.last_pos) > 1e-6:
            print(f"{TARGET_JOINT}: {pos:.6f}")
            self.last_pos = pos


def main():
    rclpy.init()
    node = WatchFinger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
