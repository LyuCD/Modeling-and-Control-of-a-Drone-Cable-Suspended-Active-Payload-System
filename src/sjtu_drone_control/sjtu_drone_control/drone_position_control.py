import rclpy
from rclpy.node import Timer
from sjtu_drone_control.drone_utils.drone_object import DroneObject

class DronePositionControl(DroneObject):
    def __init__(self):
        super().__init__('drone_position_control')

        self.takeOff()
        self.get_logger().info('Drone takeoff')

        # Set the m_posCtrl flag to True
        self.posCtrl(True)
        self.get_logger().info('Position control mode set to True')

        # Move to first pose
        self.move_drone_to_pose(0.0, 0.0, 10.0)

        # After 15 seconds, move to the next pose
        self.create_timer(15.0, self.move_to_next_pose)

    def move_drone_to_pose(self, x, y, z):
        super().moveTo(x, y, z)
        self.get_logger().info(f'Moving drone to pose: x={x}, y={y}, z={z}')

    def move_to_next_pose(self):
        # This function is called once after 15 seconds
        self.move_drone_to_pose(50.0, 50.0, 10.0)
        self.get_logger().info('Moved to next pose after 15 seconds')

        # Optionally cancel the timer if only needed once
        # But in ROS2, this timer is one-shot only if set with a flag (not available directly here)
        # So instead, we manually shut down to stop spinning if no further commands needed
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    drone_position_control_node = DronePositionControl()
    rclpy.spin(drone_position_control_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
