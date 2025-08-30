#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist, Wrench

import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
import sys
import time
import csv
from datetime import datetime
import numpy as np


def smooth(data, window_size=5):
    if len(data) < window_size:
        return data
    out = []
    s = 0.0
    for i, v in enumerate(data):
        s += v
        if i >= window_size:
            s -= data[i - window_size]
            out.append(s / window_size)
        else:
            out.append(s / (i + 1))
    return out


class PositionPlotter(Node):
    def __init__(self):
        super().__init__('position_plotter')

        # ===== ROS 参数 =====
        self.declare_parameter('fmax', 1.0)             # 力限幅 (N)
        self.declare_parameter('rope_length', 0.6)      # 绳长 (m)
        self.declare_parameter('err_eps', 0.05)         # 收敛阈值：|e| (m)
        self.declare_parameter('hold_steps', 10)        # 连续 N 步进入阈值判收敛
        self.declare_parameter('record_duration', 20.0) # 记录时长 (s)

        self.fmax = float(self.get_parameter('fmax').value)
        self.rope_L = float(self.get_parameter('rope_length').value)
        self.err_eps = float(self.get_parameter('err_eps').value)
        self.required_consecutive_steps = int(self.get_parameter('hold_steps').value)
        self.record_duration = float(self.get_parameter('record_duration').value)

        # ===== 滚动显示缓存 =====
        self.time_data = []
        self.drone_x, self.drone_y, self.drone_z = [], [], []
        self.payload_x, self.payload_y, self.payload_z = [], [], []
        self.error_x, self.error_y = [], []
        self.fx_hist, self.fy_hist = [], []

        self.start_time = time.time()
        self.latest_drone_pos = None   # geometry_msgs/Point
        self.latest_payload_pos = None # geometry_msgs/Point
        self.latest_force = (0.0, 0.0)

        # ===== recording 状态 / 指标 =====
        self.recording = False
        self.record_start_time = None
        self.error_log_rows = []

        self.dt_log = 0.1  # 和 timer 一致
        self.iae = 0.0
        self.ise = 0.0
        self.effort_l1 = 0.0
        self.effort_l2 = 0.0
        self.sat_count = 0
        self.slack_count = 0
        self.total_count = 0

        self.convergence_counter = 0
        self.convergence_duration = 0.0
        self.record_converged_time_once = False

        # ===== 订阅 ROS 话题 =====
        self.create_subscription(Odometry, '/simple_drone/odom', self.odom_callback, 10)
        self.create_subscription(Point, '/simple_drone/payload_position', self.payload_callback, 10)
        self.create_subscription(Twist, '/simple_drone/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Wrench, '/payload_force_cmd', self.force_callback, 10)

        # 0.1s 刷新
        self.timer = self.create_timer(self.dt_log, self.update_plot)

    # ----------- 回调 -----------
    def odom_callback(self, msg: Odometry):
        self.latest_drone_pos = msg.pose.pose.position

    def payload_callback(self, msg: Point):
        self.latest_payload_pos = msg

    def cmd_vel_callback(self, msg: Twist):
        self.get_logger().info('✅ Received /cmd_vel, start recording...')
        self.recording = True
        self.record_start_time = time.time()
        self.error_log_rows = []

        # 重置统计
        self.iae = self.ise = 0.0
        self.effort_l1 = self.effort_l2 = 0.0
        self.sat_count = self.slack_count = self.total_count = 0
        self.convergence_counter = 0
        self.convergence_duration = 0.0
        self.record_converged_time_once = False

    def force_callback(self, msg: Wrench):
        self.latest_force = (float(msg.force.x), float(msg.force.y))
        # 短历史用于显示
        self.fx_hist.append(self.latest_force[0])
        self.fy_hist.append(self.latest_force[1])
        self.fx_hist = self.fx_hist[-300:]
        self.fy_hist = self.fy_hist[-300:]
        if hasattr(self, 'plot5') and hasattr(self, 'plot6'):
            n = len(self.fx_hist)
            self.plot5.setData(self.time_data[-n:], self.fx_hist)
            self.plot6.setData(self.time_data[-n:], self.fy_hist)

    # ----------- 主循环 -----------
    def update_plot(self):
        if not (self.latest_drone_pos and self.latest_payload_pos):
            return

        t = time.time() - self.start_time

        # 位置
        x_d = float(self.latest_drone_pos.x)
        y_d = float(self.latest_drone_pos.y)
        z_d = float(self.latest_drone_pos.z)
        x_p = float(self.latest_payload_pos.x)
        y_p = float(self.latest_payload_pos.y)
        z_p = float(self.latest_payload_pos.z)

        # 误差（与你原图标题保持一致：Drone - Payload）
        err_x = x_d - x_p
        err_y = y_d - y_p
        err_norm = float(np.hypot(err_x, err_y))

        # 用于摆角：payload - drone（符号不影响角度绝对值）
        ex = x_p - x_d
        ey = y_p - y_d
        dz = z_d - z_p
        dz_abs = abs(dz) if abs(dz) > 1e-6 else 1e-6
        d3 = float(np.sqrt(ex**2 + ey**2 + dz**2))

        # 摆角（deg）
        theta_x_deg = float(np.degrees(np.arctan2(ex, dz)))
        theta_y_deg = float(np.degrees(np.arctan2(ey, dz)))
        theta_deg   = float(np.degrees(np.arctan2(np.hypot(ex, ey), dz_abs)))

        # 力 / 饱和 / 松绳
        fx, fy = self.latest_force
        fmag = float(np.hypot(fx, fy))
        saturated = int(fmag >= self.fmax - 1e-9)
        slack = int(d3 < (self.rope_L - 0.005))  # 5mm 余量

        # 滚动显示缓存
        self.time_data.append(t)
        self.drone_x.append(x_d); self.drone_y.append(y_d); self.drone_z.append(z_d)
        self.payload_x.append(x_p); self.payload_y.append(y_p); self.payload_z.append(z_p)
        self.error_x.append(err_x); self.error_y.append(err_y)

        max_len = 300
        for attr in ['time_data','drone_x','drone_y','drone_z',
                     'payload_x','payload_y','payload_z',
                     'error_x','error_y']:
            lst = getattr(self, attr)
            setattr(self, attr, lst[-max_len:])

        # === 记录到内存（到时写 CSV）===
        if self.recording:
            elapsed = time.time() - self.record_start_time

            self.error_log_rows.append([
                t,
                x_d, y_d, z_d,
                x_p, y_p, z_p,
                err_x, err_y, err_norm,
                theta_x_deg, theta_y_deg, theta_deg,
                fx, fy, fmag, saturated,
                d3, slack
            ])

            # 累计指标（离散积分）
            dt = self.dt_log
            self.iae += err_norm * dt
            self.ise += (err_norm ** 2) * dt
            self.effort_l1 += fmag * dt
            self.effort_l2 += (fmag ** 2) * dt
            self.sat_count += saturated
            self.slack_count += slack
            self.total_count += 1

            # 收敛判据（基于 |e|）
            if err_norm < self.err_eps:
                self.convergence_counter += 1
                if self.convergence_counter >= self.required_consecutive_steps and not self.record_converged_time_once:
                    self.convergence_duration = time.time() - self.record_start_time
                    self.record_converged_time_once = True
            else:
                self.convergence_counter = 0

            # 底部标签
            sat_ratio = (self.sat_count / self.total_count) if self.total_count > 0 else 0.0
            slack_ratio = (self.slack_count / self.total_count) if self.total_count > 0 else 0.0
            self.label_x.setText(f"IAE(|e|): {self.iae:.3f} | ISE(|e|): {self.ise:.3f} | Effort_L1: {self.effort_l1:.3f}")
            self.label_y.setText(f"Settling: {self.convergence_duration:.2f}s | SatRatio: {sat_ratio:.1%} | SlackRatio: {slack_ratio:.1%}")

            # 到时写盘
            if elapsed >= self.record_duration:
                filename = f"error_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
                with open(filename, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        'time',
                        'drone_x','drone_y','drone_z',
                        'payload_x','payload_y','payload_z',
                        'err_x','err_y','err_norm',
                        'theta_x_deg','theta_y_deg','theta_deg',
                        'fx','fy','fmag','saturated',
                        'endpoint_dist','slack'
                    ])
                    writer.writerows(self.error_log_rows)

                    # Summary
                    writer.writerow([])
                    writer.writerow(['SUMMARY'])
                    writer.writerow(['IAE(|e|)', f"{self.iae:.6f}"])
                    writer.writerow(['ISE(|e|)', f"{self.ise:.6f}"])
                    writer.writerow(['Effort_L1', f"{self.effort_l1:.6f}"])
                    writer.writerow(['Effort_L2', f"{self.effort_l2:.6f}"])
                    writer.writerow(['SatRatio', f"{(self.sat_count / max(1,self.total_count)):.6f}"])
                    writer.writerow(['SlackRatio', f"{(self.slack_count / max(1,self.total_count)):.6f}"])
                    writer.writerow(['SettlingTime', f"{self.convergence_duration:.6f}" if self.record_converged_time_once else '--'])
                    writer.writerow(['Params', f"fmax={self.fmax}, rope_L={self.rope_L}, eps={self.err_eps}, hold={self.required_consecutive_steps}, T={self.record_duration}"])

                self.get_logger().info(f'💾 {self.record_duration:.0f}s data written to: {filename}')
                self.recording = False

        # === 更新图像 ===
        if hasattr(self, 'plot1'):
            self.plot1.setData(self.time_data, self.drone_x)
            self.plot1_b.setData(self.time_data, self.payload_x)
            self.plot2.setData(self.time_data, self.drone_y)
            self.plot2_b.setData(self.time_data, self.payload_y)
            self.plot3.setData(self.time_data, smooth(self.error_x))
            self.plot4.setData(self.time_data, smooth(self.error_y))


def main():
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    win = pg.GraphicsLayoutWidget(show=True, title="Drone vs Payload Visualization")
    win.resize(1000, 1000)

    # === 创建图表 ===
    p1 = win.addPlot(title="X Position")
    p1.addLegend()
    p1.showGrid(x=True, y=True)
    curve1 = p1.plot(pen='r', name="Drone X")
    curve1_b = p1.plot(pen='b', name="Payload X")

    win.nextRow()
    p2 = win.addPlot(title="Y Position")
    p2.addLegend()
    p2.showGrid(x=True, y=True)
    curve2 = p2.plot(pen='r', name="Drone Y")
    curve2_b = p2.plot(pen='b', name="Payload Y")

    win.nextRow()
    p3 = win.addPlot(title="X Error (Drone - Payload)")
    p3.showGrid(x=True, y=True)
    curve3 = p3.plot(pen='m')

    win.nextRow()
    p4 = win.addPlot(title="Y Error (Drone - Payload)")
    p4.showGrid(x=True, y=True)
    curve4 = p4.plot(pen='c')

    win.nextRow()
    p5 = win.addPlot(title="Payload Force fx")
    p5.showGrid(x=True, y=True)
    curve5 = p5.plot(pen='y')

    win.nextRow()
    p6 = win.addPlot(title="Payload Force fy")
    p6.showGrid(x=True, y=True)
    curve6 = p6.plot(pen='g')

    win.nextRow()
    label_x = pg.LabelItem(justify='left')
    win.addItem(label_x)
    label_x.setText("IAE(|e|): --- | ISE(|e|): --- | Effort_L1: ---")

    win.nextRow()
    label_y = pg.LabelItem(justify='left')
    win.addItem(label_y)
    label_y.setText("Settling: --- | SatRatio: --- | SlackRatio: ---")

    # === 启动 ROS 节点 ===
    node = PositionPlotter()
    node.plot1 = curve1
    node.plot1_b = curve1_b
    node.plot2 = curve2
    node.plot2_b = curve2_b
    node.plot3 = curve3
    node.plot4 = curve4
    node.plot5 = curve5
    node.plot6 = curve6
    node.label_x = label_x
    node.label_y = label_y

    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(5)

    exit_code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
