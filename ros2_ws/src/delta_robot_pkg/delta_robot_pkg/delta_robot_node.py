"""
Main Dual Delta Robot ROS 2 Controller & Solver Node
Runs Ghost (IK Reference) and Real (Dynamics + SMC Controller) side-by-side.
Publishes joint states to /ghost/joint_states and /real/joint_states.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import String

from .kinematics import DeltaKinematics
from .dynamics import DeltaDynamics


class DeltaRobotNode(Node):
    def __init__(self):
        super().__init__('delta_robot_node')

        self.kin = DeltaKinematics()
        self.dyn = DeltaDynamics()
        self.controller_type = 's1sat'

        # Reference trajectory targets
        self.ref_pos = {'x': 0.0, 'y': 0.0, 'z': -0.18}
        self.ref_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.ref_acc = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        # Publishers
        self.pub_ghost_js = self.create_publisher(JointState, '/ghost/joint_states', 10)
        self.pub_real_js = self.create_publisher(JointState, '/real/joint_states', 10)
        self.pub_torques = self.create_publisher(Vector3, '/delta/control_effort', 10)

        # Subscriptions
        self.sub_pos = self.create_subscription(Point, '/delta/cmd_pos', self.pos_cb, 10)
        self.sub_vel = self.create_subscription(Vector3, '/delta/cmd_vel', self.vel_cb, 10)
        self.sub_acc = self.create_subscription(Vector3, '/delta/cmd_acc', self.acc_cb, 10)
        self.sub_ctrl = self.create_subscription(String, '/delta/select_controller', self.ctrl_cb, 10)

        # Physics & publisher loop at 200Hz
        self.dt = 0.005
        self.timer = self.create_timer(self.dt, self.physics_loop)
        self.get_logger().info('Dual Delta Robot Controller Node started at 200Hz.')

    def pos_cb(self, msg):
        self.ref_pos['x'] = msg.x
        self.ref_pos['y'] = msg.y
        self.ref_pos['z'] = msg.z

    def vel_cb(self, msg):
        self.ref_vel['x'] = msg.x
        self.ref_vel['y'] = msg.y
        self.ref_vel['z'] = msg.z

    def acc_cb(self, msg):
        self.ref_acc['x'] = msg.x
        self.ref_acc['y'] = msg.y
        self.ref_acc['z'] = msg.z

    def ctrl_cb(self, msg):
        selected = msg.data.lower()
        if selected in ['s1sat', 's1dhrl', 's2', 'pdf', 'pd']:
            self.controller_type = selected
            self.get_logger().info(f'Active controller switched to: {self.controller_type}')

    def physics_loop(self):
        now = self.get_clock().now().to_msg()

        # 1. Ghost IK solver (Ideal Kinematic Target)
        ik_ghost = self.kin.solve_ik(self.ref_pos['x'], self.ref_pos['y'], self.ref_pos['z'])
        ghost_rad = ik_ghost['theta_rad'] if ik_ghost['ok'] else [0.0, 0.0, 0.0]

        # Publish Ghost JointState
        js_ghost = JointState()
        js_ghost.header.stamp = now
        js_ghost.name = ['arm1_joint', 'arm2_joint', 'arm3_joint']
        js_ghost.position = [float(g) for g in ghost_rad]
        self.pub_ghost_js.publish(js_ghost)

        # 2. Real Robot Dynamics Step
        real_state = self.dyn.step(
            self.controller_type,
            self.ref_pos,
            self.ref_vel,
            self.ref_acc,
            self.dt
        )

        # Publish Real JointState
        js_real = JointState()
        js_real.header.stamp = now
        js_real.name = ['arm1_joint', 'arm2_joint', 'arm3_joint']
        js_real.position = [float(r) for r in real_state['theta_rad']]
        self.pub_real_js.publish(js_real)

        # Publish Torque Effort
        tau_msg = Vector3()
        tau_msg.x = float(real_state['torques'][0])
        tau_msg.y = float(real_state['torques'][1])
        tau_msg.z = float(real_state['torques'][2])
        self.pub_torques.publish(tau_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DeltaRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
