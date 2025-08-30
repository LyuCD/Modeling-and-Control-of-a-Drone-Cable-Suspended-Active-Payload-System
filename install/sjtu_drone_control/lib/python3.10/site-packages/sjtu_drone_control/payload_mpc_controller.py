#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist, Wrench
import numpy as np
import cvxpy as cp

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')

        # === 控制参数 ===
        self.dt = 0.1     # 控制周期
        self.N = 10       # 预测步长
        self.L = 0.6      # 绳长 (m)
        self.m = 2.0      # 负载质量 (kg)
        self.g = 9.81     # 重力加速度

        # === 状态变量 ===
        self.drone_pos = None
        self.payload_pos = None
        self.payload_vel = None
        self.goal_pos = np.array([0.0, 0.0])  # UAV 目标位置

        # === ROS 接口 ===
        self.create_subscription(Odometry, '/simple_drone/odom', self.drone_cb, 10)
        self.create_subscription(Point, '/simple_drone/payload_position', self.payload_pos_cb, 10)
        self.create_subscription(Twist, '/simple_drone/payload_velocity', self.payload_vel_cb, 10)
        self.create_subscription(Twist, '/simple_drone/cmd_vel', self.cmd_vel_cb, 10)
        self.force_pub = self.create_publisher(Wrench, '/payload_force_cmd', 10)

        self.create_timer(self.dt, self.control_loop)

    def drone_cb(self, msg):
        self.drone_pos = msg.pose.pose.position

    def payload_pos_cb(self, msg):
        self.payload_pos = msg

    def payload_vel_cb(self, msg):
        self.payload_vel = msg.linear

    def cmd_vel_cb(self, msg):
        # 接收 UAV 目标位置（不是速度）
        self.goal_pos = np.array([msg.linear.x, msg.linear.y])

    def control_loop(self):
        if self.drone_pos is None or self.payload_pos is None or self.payload_vel is None:
            return

        # === 当前状态 ===
        px, py = self.payload_pos.x, self.payload_pos.y
        vx, vy = self.payload_vel.x, self.payload_vel.y
        dx = self.drone_pos.x - px
        dy = self.drone_pos.y - py

        theta_x = dx / self.L
        theta_y = dy / self.L
        dtheta_x = -vx / self.L
        dtheta_y = -vy / self.L

        x0 = np.array([theta_x, theta_y, dtheta_x, dtheta_y, px, py, vx, vy])

        # === 构建 UAV 未来参考轨迹（固定目标点）===
        ref_traj = np.tile(self.goal_pos.reshape(2, 1), (1, self.N))
        ref_vel = np.array([0.0, 0.0])  # UAV 目标是静止的

        # === 状态空间模型 ===
        A = np.eye(8)
        A[0, 2] = self.dt
        A[1, 3] = self.dt
        A[2, 0] = -(self.g / self.L) * self.dt
        A[3, 1] = -(self.g / self.L) * self.dt
        A[4, 6] = self.dt
        A[5, 7] = self.dt

        B = np.zeros((8, 2))
        B[6, 0] = self.dt / self.m
        B[7, 1] = self.dt / self.m

        # === 优化变量 ===
        x = cp.Variable((8, self.N + 1))
        u = cp.Variable((2, self.N))

        cost = 0
        constraints = [x[:, 0] == x0]

        for t in range(self.N):
            pos_error = x[4:6, t] - ref_traj[:, t]
            vel_error = x[6:8, t] - ref_vel
            theta_error = x[0:2, t]

            cost += 5.0 * cp.sum_squares(pos_error)
            cost += 1.0 * cp.sum_squares(vel_error)
            cost += 1.0 * cp.sum_squares(theta_error)
            cost += 0.01 * cp.sum_squares(u[:, t])

            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t]]
            constraints += [cp.norm(u[:, t], 'inf') <= 1.0]

        # 终端惩罚项（增强收敛）
        cost += 20.0 * cp.sum_squares(x[4:6, self.N] - ref_traj[:, -1])

        prob = cp.Problem(cp.Minimize(cost), constraints)
        try:
            prob.solve(solver=cp.OSQP)
        except Exception as e:
            self.get_logger().warn(f"[MPC] Solver error: {e}")
            return

        if u.value is None:
            self.get_logger().warn("MPC solver returned no solution.")
            return

        fx, fy = u.value[:, 0]
        wrench = Wrench()
        wrench.force.x = float(fx)
        wrench.force.y = float(fy)
        wrench.force.z = 0.0
        self.force_pub.publish(wrench)
        # self.get_logger().info(f"[MPC] fx={fx:.2f}, fy={fy:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = MPCController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
