import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Wrench
import time

class PayloadPIDController(Node):
    def __init__(self):
        super().__init__('payload_controller')

        # 订阅位置数据
        self.create_subscription(Odometry, '/simple_drone/odom', self.odom_callback, 10)
        self.create_subscription(Point, '/simple_drone/payload_position', self.payload_callback, 10)

        # 发布控制力
        self.force_pub = self.create_publisher(Wrench, '/payload_force_cmd', 10)

        # PID 参数
        self.kp = 10.0
        self.ki = 0.0
        self.kd = 2.0

        # PID 状态
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0

        self.last_time = time.time()

        # 最新位置缓存
        self.drone_pos = None
        self.payload_pos = None

        # 循环定时器
        self.create_timer(0.01, self.control_loop)

    def odom_callback(self, msg):
        self.drone_pos = msg.pose.pose.position

    def payload_callback(self, msg):
        self.payload_pos = msg

    def control_loop(self):
        if self.drone_pos is None or self.payload_pos is None:
            return

        now = time.time()
        dt = now - self.last_time
        if dt <= 0.0:
            dt = 1e-6  # 避免除0
        self.last_time = now

        # 误差计算
        error_x = self.drone_pos.x - self.payload_pos.x
        error_y = self.drone_pos.y - self.payload_pos.y

        # PID计算
        self.integral_x += error_x * dt
        self.integral_y += error_y * dt

        derivative_x = (error_x - self.prev_error_x) / dt
        derivative_y = (error_y - self.prev_error_y) / dt

        force_x = self.kp * error_x + self.ki * self.integral_x + self.kd * derivative_x
        force_y = self.kp * error_y + self.ki * self.integral_y + self.kd * derivative_y

        max_force = 1.0
        force_x = max(min(force_x, max_force), -max_force)
        force_y = max(min(force_y, max_force), -max_force)

        self.prev_error_x = error_x
        self.prev_error_y = error_y

        # 发布力指令
        wrench = Wrench()
        wrench.force.x = force_x
        wrench.force.y = force_y
        wrench.force.z = 0.0  # 不控制z方向
        self.force_pub.publish(wrench)

def main():
    rclpy.init()
    node = PayloadPIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
