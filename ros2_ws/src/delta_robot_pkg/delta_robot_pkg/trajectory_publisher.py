"""
ROS 2 Trajectory Publisher Node for Delta Robot
Publishes target 3D Cartesian position, velocity, and acceleration references to /delta/cmd_pos.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import String


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('delta_trajectory_publisher')

        self.publisher_pos = self.create_publisher(Point, '/delta/cmd_pos', 10)
        self.publisher_vel = self.create_publisher(Vector3, '/delta/cmd_vel', 10)
        self.publisher_acc = self.create_publisher(Vector3, '/delta/cmd_acc', 10)

        self.subscription_traj = self.create_subscription(
            String,
            '/delta/select_trajectory',
            self.trajectory_callback,
            10
        )

        self.traj_type = 'ellipse'
        self.t = 0.0
        self.dt = 0.01  # 100Hz loop

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info('Delta Trajectory Publisher started operating at 100Hz.')

    def trajectory_callback(self, msg):
        selected = msg.data.lower()
        if selected in ['ellipse', 'circle', 'figure8', 'spiral']:
            self.traj_type = selected
            self.t = 0.0
            self.get_logger().info(f'Switched trajectory reference to: {self.traj_type}')

    def timer_callback(self):
        self.t += self.dt
        t = self.t
        omega = 1.25

        pos = Point()
        vel = Vector3()
        acc = Vector3()

        if self.traj_type == 'ellipse':
            pos.x = 0.05 * math.cos(omega * t)
            pos.y = 0.03 * math.sin(omega * t)
            pos.z = -0.18 + 0.01 * math.sin(2.0 * omega * t)

            vel.x = -0.05 * omega * math.sin(omega * t)
            vel.y = 0.03 * omega * math.cos(omega * t)
            vel.z = 0.02 * omega * math.cos(2.0 * omega * t)

            acc.x = -0.05 * (omega**2) * math.cos(omega * t)
            acc.y = -0.03 * (omega**2) * math.sin(omega * t)
            acc.z = -0.04 * (omega**2) * math.sin(2.0 * omega * t)

        elif self.traj_type == 'circle':
            pos.x = 0.04 * math.cos(omega * t)
            pos.y = 0.04 * math.sin(omega * t)
            pos.z = -0.18

            vel.x = -0.04 * omega * math.sin(omega * t)
            vel.y = 0.04 * omega * math.cos(omega * t)
            vel.z = 0.0

            acc.x = -0.04 * (omega**2) * math.cos(omega * t)
            acc.y = -0.04 * (omega**2) * math.sin(omega * t)
            acc.z = 0.0

        elif self.traj_type == 'figure8':
            pos.x = 0.05 * math.sin(omega * t)
            pos.y = 0.03 * math.sin(2.0 * omega * t)
            pos.z = -0.18

            vel.x = 0.05 * omega * math.cos(omega * t)
            vel.y = 0.06 * omega * math.cos(2.0 * omega * t)
            vel.z = 0.0

            acc.x = -0.05 * (omega**2) * math.sin(omega * t)
            acc.y = -0.12 * (omega**2) * math.sin(2.0 * omega * t)
            acc.z = 0.0

        elif self.traj_type == 'spiral':
            r = 0.04 * (1.0 + 0.2 * math.sin(0.5 * t))
            pos.x = r * math.cos(2.0 * omega * t)
            pos.y = r * math.sin(2.0 * omega * t)
            pos.z = -0.18 + 0.015 * math.cos(omega * t)

            # Approximated derivatives
            vel.x = -r * 2.0 * omega * math.sin(2.0 * omega * t)
            vel.y = r * 2.0 * omega * math.cos(2.0 * omega * t)
            vel.z = -0.015 * omega * math.sin(omega * t)

            acc.x = -r * (2.0 * omega)**2 * math.cos(2.0 * omega * t)
            acc.y = -r * (2.0 * omega)**2 * math.sin(2.0 * omega * t)
            acc.z = -0.015 * (omega**2) * math.cos(omega * t)

        self.publisher_pos.publish(pos)
        self.publisher_vel.publish(vel)
        self.publisher_acc.publish(acc)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
