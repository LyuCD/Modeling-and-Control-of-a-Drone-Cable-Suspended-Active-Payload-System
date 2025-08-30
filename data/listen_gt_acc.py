#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class AccLogger(Node):
    def __init__(self):
        super().__init__('acc_logger')
        self.max_horizontal_acc = 0.0
        self.subscription = self.create_subscription(
            Twist,
            '/gt_acc',  # 订阅插件发布的加速度消息
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        ax = msg.linear.x
        ay = msg.linear.y
        a_horiz = math.sqrt(ax**2 + ay**2)
        if a_horiz > self.max_horizontal_acc:
            self.max_horizontal_acc = a_horiz
            self.get_logger().info(f'New max horizontal acc: {a_horiz:.3f} m/s²')

def main(args=None):
    rclpy.init(args=args)
    node = AccLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
