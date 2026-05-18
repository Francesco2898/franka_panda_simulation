#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

TARGETS = ["fr3_finger_joint1", "fr3_finger_joint2"]

class Watch(Node):
    def __init__(self):
        super().__init__("watch_fingers")
        self.sub = self.create_subscription(JointState, "/joint_states", self.cb, 10)
        self.last = {}

    def cb(self, msg: JointState):
        data = {}
        for j in TARGETS:
            if j in msg.name:
                i = msg.name.index(j)
                p = msg.position[i] if i < len(msg.position) else None
                data[j] = p
            else:
                data[j] = None

        # 打印变化（或首次）
        changed = False
        for j, p in data.items():
            if j not in self.last or self.last[j] != p:
                changed = True
        if changed:
            print("joint_states:", ", ".join([f"{j}={data[j]}" for j in TARGETS]))
            self.last = data

def main():
    rclpy.init()
    node = Watch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
