"""
Main Dual Delta Robot ROS 2 Controller & Solver Node
Runs Ghost (IK Reference) and Real (Dynamics + SMC Controller) side-by-side.
Publishes exact 3D delta robot geometry matching the WebGL simulator (robot3d.js)
using MarkerArray with twin parallel rods, ball joints, moving nacelle, tool cone,
motors, and trajectory trails.
"""

from collections import deque
import math

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, TransformStamped, Vector3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .dynamics import DeltaDynamics
from .kinematics import D2R, DeltaKinematics, PHI_DEG


def make_cylinder_marker(marker_id, ns, p1, p2, radius, rgba, stamp, frame_id="world"):
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.type = Marker.CYLINDER
    m.action = Marker.ADD
    m.lifetime = Duration(sec=0, nanosec=0)

    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dz = p2[2] - p1[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length < 1e-6:
        length = 1e-6

    m.pose.position.x = (p1[0] + p2[0]) / 2.0
    m.pose.position.y = (p1[1] + p2[1]) / 2.0
    m.pose.position.z = (p1[2] + p2[2]) / 2.0

    m.scale.x = radius * 2.0
    m.scale.y = radius * 2.0
    m.scale.z = length

    vx, vy, vz = dx / length, dy / length, dz / length
    dot = vz
    if dot > 0.999999:
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0
    elif dot < -0.999999:
        m.pose.orientation.x = 1.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 0.0
    else:
        ax = -vy
        ay = vx
        az = 0.0
        w = 1.0 + dot
        inv_norm = 1.0 / math.sqrt(ax * ax + ay * ay + az * az + w * w)
        m.pose.orientation.x = ax * inv_norm
        m.pose.orientation.y = ay * inv_norm
        m.pose.orientation.z = az * inv_norm
        m.pose.orientation.w = w * inv_norm

    m.color.r = float(rgba[0])
    m.color.g = float(rgba[1])
    m.color.b = float(rgba[2])
    m.color.a = float(rgba[3])
    return m


def make_sphere_marker(marker_id, ns, center, radius, rgba, stamp, frame_id="world"):
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.lifetime = Duration(sec=0, nanosec=0)

    m.pose.position.x = center[0]
    m.pose.position.y = center[1]
    m.pose.position.z = center[2]
    m.pose.orientation.w = 1.0

    m.scale.x = radius * 2.0
    m.scale.y = radius * 2.0
    m.scale.z = radius * 2.0

    m.color.r = float(rgba[0])
    m.color.g = float(rgba[1])
    m.color.b = float(rgba[2])
    m.color.a = float(rgba[3])
    return m


def make_platform_marker(marker_id, ns, center, radius, height, rgba, stamp, frame_id="world"):
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.type = Marker.CYLINDER
    m.action = Marker.ADD
    m.lifetime = Duration(sec=0, nanosec=0)

    m.pose.position.x = center[0]
    m.pose.position.y = center[1]
    m.pose.position.z = center[2]
    m.pose.orientation.w = 1.0

    m.scale.x = radius * 2.0
    m.scale.y = radius * 2.0
    m.scale.z = height

    m.color.r = float(rgba[0])
    m.color.g = float(rgba[1])
    m.color.b = float(rgba[2])
    m.color.a = float(rgba[3])
    return m


def make_tool_cone_marker(marker_id, ns, ee_pos, rgba, stamp, frame_id="world"):
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.lifetime = Duration(sec=0, nanosec=0)

    p_start = Point(x=ee_pos[0], y=ee_pos[1], z=ee_pos[2] - 0.0035)
    p_end = Point(x=ee_pos[0], y=ee_pos[1], z=ee_pos[2] - 0.0235)
    m.points = [p_start, p_end]

    m.scale.x = 0.002
    m.scale.y = 0.012
    m.scale.z = 0.020

    m.color.r = float(rgba[0])
    m.color.g = float(rgba[1])
    m.color.b = float(rgba[2])
    m.color.a = float(rgba[3])
    return m


def make_line_strip_marker(marker_id, ns, points, width, rgba, stamp, frame_id="world"):
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.lifetime = Duration(sec=0, nanosec=0)

    m.pose.orientation.w = 1.0
    m.scale.x = width
    m.points = [Point(x=p[0], y=p[1], z=p[2]) for p in points]

    m.color.r = float(rgba[0])
    m.color.g = float(rgba[1])
    m.color.b = float(rgba[2])
    m.color.a = float(rgba[3])
    return m


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

        # History trails
        self.ghost_trail = deque(maxlen=250)
        self.real_trail = deque(maxlen=250)

        # Publishers
        self.pub_markers = self.create_publisher(MarkerArray, '/delta/markers', 10)
        self.pub_ghost_js = self.create_publisher(JointState, '/ghost/joint_states', 10)
        self.pub_real_js = self.create_publisher(JointState, '/real/joint_states', 10)
        self.pub_torques = self.create_publisher(Vector3, '/delta/control_effort', 10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriptions
        self.sub_pos = self.create_subscription(Point, '/delta/cmd_pos', self.pos_cb, 10)
        self.sub_vel = self.create_subscription(Vector3, '/delta/cmd_vel', self.vel_cb, 10)
        self.sub_acc = self.create_subscription(Vector3, '/delta/cmd_acc', self.acc_cb, 10)
        self.sub_ctrl = self.create_subscription(String, '/delta/select_controller', self.ctrl_cb, 10)

        # Loop at 100Hz
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.physics_loop)
        self.get_logger().info('Dual Delta Robot Controller Node started at 100Hz with exact WebGL mechanism markers.')

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

    def publish_tf(self, now, p_ghost, p_real):
        # 1. Base link
        t_base = TransformStamped()
        t_base.header.stamp = now
        t_base.header.frame_id = 'world'
        t_base.child_frame_id = 'base_link'
        t_base.transform.translation.x = 0.0
        t_base.transform.translation.y = 0.0
        t_base.transform.translation.z = 0.0
        t_base.transform.rotation.w = 1.0

        # 2. Ghost End-Effector
        t_ghost_ee = TransformStamped()
        t_ghost_ee.header.stamp = now
        t_ghost_ee.header.frame_id = 'world'
        t_ghost_ee.child_frame_id = 'ghost/ee'
        t_ghost_ee.transform.translation.x = p_ghost[0]
        t_ghost_ee.transform.translation.y = p_ghost[1]
        t_ghost_ee.transform.translation.z = p_ghost[2]
        t_ghost_ee.transform.rotation.w = 1.0

        # 3. Real End-Effector
        t_real_ee = TransformStamped()
        t_real_ee.header.stamp = now
        t_real_ee.header.frame_id = 'world'
        t_real_ee.child_frame_id = 'real/ee'
        t_real_ee.transform.translation.x = p_real[0]
        t_real_ee.transform.translation.y = p_real[1]
        t_real_ee.transform.translation.z = p_real[2]
        t_real_ee.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform([t_base, t_ghost_ee, t_real_ee])

    def physics_loop(self):
        now = self.get_clock().now().to_msg()

        # 1. Ghost IK solver (Ideal Kinematic Target)
        ik_ghost = self.kin.solve_ik(self.ref_pos['x'], self.ref_pos['y'], self.ref_pos['z'])
        ghost_rad = ik_ghost['theta_rad'] if ik_ghost['ok'] else [0.0, 0.0, 0.0]

        # Publish Ghost JointState
        js_ghost = JointState()
        js_ghost.header.stamp = now
        js_ghost.name = ['motor_joint_1', 'motor_joint_2', 'motor_joint_3']
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
        js_real.name = ['motor_joint_1', 'motor_joint_2', 'motor_joint_3']
        js_real.position = [float(r) for r in real_state['theta_rad']]
        self.pub_real_js.publish(js_real)

        # Publish Torque Effort
        tau_msg = Vector3()
        tau_msg.x = float(real_state['torques'][0])
        tau_msg.y = float(real_state['torques'][1])
        tau_msg.z = float(real_state['torques'][2])
        self.pub_torques.publish(tau_msg)

        # Real IK for exact 3D mechanism geometry
        ik_real = self.kin.solve_ik(real_state['pos']['x'], real_state['pos']['y'], real_state['pos']['z'])

        p_ghost = (self.ref_pos['x'], self.ref_pos['y'], self.ref_pos['z'])
        p_real = (real_state['pos']['x'], real_state['pos']['y'], real_state['pos']['z'])

        # Record trail points
        self.ghost_trail.append(p_ghost)
        self.real_trail.append(p_real)

        # Broadcast TF
        self.publish_tf(now, p_ghost, p_real)

        # Build MarkerArray matching WebGL simulator (robot3d.js)
        markers = []

        # Color definitions from WebGL simulator
        c_base = (0.706, 0.729, 0.769, 1.0)
        c_motor = (0.208, 0.231, 0.275, 1.0)
        c_joint_grey = (0.420, 0.447, 0.502, 1.0)
        c_dark_slate = (0.208, 0.231, 0.275, 1.0)
        c_plat_real = (0.180, 0.204, 0.251, 1.0)
        c_orange = (1.0, 0.34, 0.13, 1.0)

        c_ghost_cyan = (0.0, 0.953, 1.0, 0.35)
        c_ghost_rod = (0.0, 0.70, 0.85, 0.30)
        c_ghost_joint = (0.0, 1.0, 1.0, 0.45)

        # A. Base Plate (Rf + 30mm radius = 0.1105m, 8mm thickness)
        markers.append(make_platform_marker(
            0, "base_plate", (0.0, 0.0, 0.0), self.kin.Rf + 0.030, 0.008, c_base, now
        ))

        # B. 3 Stepper Motors (mounted at base periphery)
        for i in range(3):
            phi = PHI_DEG[i] * D2R
            bx = self.kin.Rf * math.cos(phi)
            by = self.kin.Rf * math.sin(phi)
            # Motor cylinder along tangent
            tx = -math.sin(phi) * 0.013
            ty = math.cos(phi) * 0.013
            m_p1 = (bx - tx, by - ty, 0.0)
            m_p2 = (bx + tx, by + ty, 0.0)
            markers.append(make_cylinder_marker(
                1 + i, "motors", m_p1, m_p2, 0.014, c_motor, now
            ))

        # C. Ghost Robot (Ideal Reference IK)
        if ik_ghost['ok']:
            for i in range(3):
                phi = PHI_DEG[i] * D2R
                bx = self.kin.Rf * math.cos(phi)
                by = self.kin.Rf * math.sin(phi)
                base_pt = (bx, by, 0.0)
                elbow_pt = (ik_ghost['elbows'][i]['x'], ik_ghost['elbows'][i]['y'], ik_ghost['elbows'][i]['z'])
                plat_pt = (ik_ghost['platform_pts'][i]['x'], ik_ghost['platform_pts'][i]['y'], ik_ghost['platform_pts'][i]['z'])

                # Ghost Bicep (radius 7mm)
                markers.append(make_cylinder_marker(
                    10 + i, "ghost_biceps", base_pt, elbow_pt, 0.007, c_ghost_cyan, now
                ))
                # Ghost Elbow joint (radius 6.5mm)
                markers.append(make_sphere_marker(
                    13 + i, "ghost_elbows", elbow_pt, 0.0065, c_ghost_joint, now
                ))

                # Tangential offset of 9mm for twin parallel rods
                tang_x = -math.sin(phi) * 0.009
                tang_y = math.cos(phi) * 0.009

                # Twin Parallel Forearm Rods (radius 3.5mm each)
                e_a = (elbow_pt[0] + tang_x, elbow_pt[1] + tang_y, elbow_pt[2])
                p_a = (plat_pt[0] + tang_x, plat_pt[1] + tang_y, plat_pt[2])
                e_b = (elbow_pt[0] - tang_x, elbow_pt[1] - tang_y, elbow_pt[2])
                p_b = (plat_pt[0] - tang_x, plat_pt[1] - tang_y, plat_pt[2])

                markers.append(make_cylinder_marker(
                    16 + 2 * i, "ghost_rods", e_a, p_a, 0.0035, c_ghost_rod, now
                ))
                markers.append(make_cylinder_marker(
                    16 + 2 * i + 1, "ghost_rods", e_b, p_b, 0.0035, c_ghost_rod, now
                ))

                # Ghost Platform joints (radius 5mm)
                markers.append(make_sphere_marker(
                    22 + i, "ghost_plat_joints", plat_pt, 0.005, c_ghost_joint, now
                ))

            # Ghost Nacelle Moving Platform (radius Re + 14mm = 0.0491m, 7mm thickness)
            markers.append(make_platform_marker(
                25, "ghost_platform", p_ghost, self.kin.Re + 0.014, 0.007, c_ghost_cyan, now
            ))
            # Ghost Tool Tip Cone
            markers.append(make_tool_cone_marker(
                26, "ghost_tool", p_ghost, c_ghost_cyan, now
            ))

        # D. Real Robot (Dynamic Closed-Loop Tracked Robot)
        if ik_real['ok']:
            for i in range(3):
                phi = PHI_DEG[i] * D2R
                bx = self.kin.Rf * math.cos(phi)
                by = self.kin.Rf * math.sin(phi)
                base_pt = (bx, by, 0.0)
                elbow_pt = (ik_real['elbows'][i]['x'], ik_real['elbows'][i]['y'], ik_real['elbows'][i]['z'])
                plat_pt = (ik_real['platform_pts'][i]['x'], ik_real['platform_pts'][i]['y'], ik_real['platform_pts'][i]['z'])

                # Real Bicep (radius 7mm, metallic orange)
                markers.append(make_cylinder_marker(
                    30 + i, "real_biceps", base_pt, elbow_pt, 0.007, c_orange, now
                ))
                # Real Elbow joint (radius 6.5mm, grey)
                markers.append(make_sphere_marker(
                    33 + i, "real_elbows", elbow_pt, 0.0065, c_joint_grey, now
                ))

                # Tangential offset of 9mm for twin parallel rods
                tang_x = -math.sin(phi) * 0.009
                tang_y = math.cos(phi) * 0.009

                # Twin Parallel Forearm Rods (radius 3.5mm each, dark slate)
                e_a = (elbow_pt[0] + tang_x, elbow_pt[1] + tang_y, elbow_pt[2])
                p_a = (plat_pt[0] + tang_x, plat_pt[1] + tang_y, plat_pt[2])
                e_b = (elbow_pt[0] - tang_x, elbow_pt[1] - tang_y, elbow_pt[2])
                p_b = (plat_pt[0] - tang_x, plat_pt[1] - tang_y, plat_pt[2])

                markers.append(make_cylinder_marker(
                    36 + 2 * i, "real_rods", e_a, p_a, 0.0035, c_dark_slate, now
                ))
                markers.append(make_cylinder_marker(
                    36 + 2 * i + 1, "real_rods", e_b, p_b, 0.0035, c_dark_slate, now
                ))

                # Real Platform joints (radius 5mm, grey)
                markers.append(make_sphere_marker(
                    42 + i, "real_plat_joints", plat_pt, 0.005, c_joint_grey, now
                ))

            # Real Nacelle Moving Platform (radius Re + 14mm, 7mm thickness, dark slate)
            markers.append(make_platform_marker(
                45, "real_platform", p_real, self.kin.Re + 0.014, 0.007, c_plat_real, now
            ))
            # Real Tool Tip Cone (metallic orange)
            markers.append(make_tool_cone_marker(
                46, "real_tool", p_real, c_orange, now
            ))

        # E. Trajectory Trails & Dynamic Error Vector
        if len(self.ghost_trail) > 1:
            markers.append(make_line_strip_marker(
                50, "ghost_trail", list(self.ghost_trail), 0.002, (0.0, 0.953, 1.0, 0.75), now
            ))
        if len(self.real_trail) > 1:
            markers.append(make_line_strip_marker(
                51, "real_trail", list(self.real_trail), 0.0025, (1.0, 0.34, 0.13, 0.90), now
            ))

        # Dynamic Error Vector between Ghost and Real
        markers.append(make_line_strip_marker(
            52, "error_vector", [p_ghost, p_real], 0.002, (0.937, 0.267, 0.267, 0.90), now
        ))

        # Publish all markers
        msg_array = MarkerArray()
        msg_array.markers = markers
        self.pub_markers.publish(msg_array)


def main(args=None):
    rclpy.init(args=args)
    node = DeltaRobotNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
