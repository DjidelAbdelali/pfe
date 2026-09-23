"""
Delta Robot Dynamic Solver & Controllers in Python
Controllers: s1sat, s1dhrl, s2, pdf, pd
Matching PFE MATLAB & Web Simulator dynamic parameters.
"""

import math
from .kinematics import DeltaKinematics, D2R, PHI_DEG


class DeltaDynamics:
    def __init__(self, m_platform=0.8, m_arm=0.15, g=9.81):
        self.kin = DeltaKinematics()
        self.m_total = m_platform + 3.0 * m_arm
        self.g = g

        # Current actual state in Cartesian space
        self.pos = {'x': 0.0, 'y': 0.0, 'z': -0.18}
        self.vel = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        # Integrated sliding surface integral state for S2
        self.s2_integral = [0.0, 0.0, 0.0]

        # Controller gains
        self.gains = {
            'c': 25.0,
            'K': 12.0,
            'kp': 180.0,
            'kd': 15.0,
            'ki': 5.0,
            'eps': 0.02
        }

    def compute_jacobian(self, x, y, z, theta_rad):
        """
        Computes inverse Jacobian Matrix J_inv (3x3) mapping task space velocity to joint space velocity.
        d(theta) / dt = J_inv * d(X) / dt
        """
        J = [[0.0]*3 for _ in range(3)]

        for i in range(3):
            phi = PHI_DEG[i] * D2R
            c = math.cos(phi)
            s = math.sin(phi)
            th = theta_rad[i]

            # Elbow position
            ex = (self.kin.Rf + self.kin.L * math.cos(th)) * c
            ey = (self.kin.Rf + self.kin.L * math.cos(th)) * s
            ez = self.kin.L * math.sin(th)

            # Platform joint position
            px = x + self.kin.Re * c
            py = y + self.kin.Re * s
            pz = z

            # Vector from elbow to platform joint (forearm)
            vx = px - ex
            vy = py - ey
            vz = pz - ez

            # Partial derivatives
            # Forearm constraint: |P_i - E_i|^2 = l^2
            # Differentiating w.r.t X: 2*(P-E)^T * dP = 2*(P-E)^T * dE
            # dE/dtheta_i = [-L*sin(th)*c, -L*sin(th)*s, L*cos(th)]^T
            dE_dth_x = -self.kin.L * math.sin(th) * c
            dE_dth_y = -self.kin.L * math.sin(th) * s
            dE_dth_z = self.kin.L * math.cos(th)

            dot_theta = vx * dE_dth_x + vy * dE_dth_y + vz * dE_dth_z
            if abs(dot_theta) < 1e-6:
                dot_theta = 1e-6 if dot_theta >= 0 else -1e-6

            J[i][0] = vx / dot_theta
            J[i][1] = vy / dot_theta
            J[i][2] = vz / dot_theta

        return J

    def compute_control(self, controller_type, pos_ref, vel_ref, acc_ref, dt=0.002):
        """
        Computes control torque [tau1, tau2, tau3] for given controller type.
        """
        ik = self.kin.solve_ik(self.pos['x'], self.pos['y'], self.pos['z'])
        if not ik['ok']:
            return [0.0, 0.0, 0.0]

        ik_ref = self.kin.solve_ik(pos_ref['x'], pos_ref['y'], pos_ref['z'])
        if not ik_ref['ok']:
            return [0.0, 0.0, 0.0]

        q = ik['theta_rad']
        q_ref = ik_ref['theta_rad']

        # Errors in task space
        ex = pos_ref['x'] - self.pos['x']
        ey = pos_ref['y'] - self.pos['y']
        ez = pos_ref['z'] - self.pos['z']

        evx = vel_ref['x'] - self.vel['x']
        evy = vel_ref['y'] - self.vel['y']
        evz = vel_ref['z'] - self.vel['z']

        c = self.gains['c']
        K = self.gains['K']
        kp = self.gains['kp']
        kd = self.gains['kd']
        eps = self.gains['eps']

        # Task space sliding surface S
        Sx = evx + c * ex
        Sy = evy + c * ey
        Sz = evz + c * ez

        # Control force F_cart (3D task space force)
        if controller_type == 's1sat':
            # Saturation continuous SMC
            sat_x = max(-1.0, min(1.0, Sx / eps))
            sat_y = max(-1.0, min(1.0, Sy / eps))
            sat_z = max(-1.0, min(1.0, Sz / eps))
            Fx = self.m_total * (acc_ref['x'] + c * evx) + K * sat_x
            Fy = self.m_total * (acc_ref['y'] + c * evy) + K * sat_y
            Fz = self.m_total * (acc_ref['z'] + c * evz + self.g) + K * sat_z

        elif controller_type == 's1dhrl':
            # Hyperbolic tangent smoothing SMC
            Fx = self.m_total * (acc_ref['x'] + c * evx) + K * math.tanh(Sx / eps)
            Fy = self.m_total * (acc_ref['y'] + c * evy) + K * math.tanh(Sy / eps)
            Fz = self.m_total * (acc_ref['z'] + c * evz + self.g) + K * math.tanh(Sz / eps)

        elif controller_type == 's2':
            # Integral sliding mode
            self.s2_integral[0] += ex * dt
            self.s2_integral[1] += ey * dt
            self.s2_integral[2] += ez * dt

            S2x = evx + 2.0 * c * ex + (c * c) * self.s2_integral[0]
            S2y = evy + 2.0 * c * ey + (c * c) * self.s2_integral[1]
            S2z = evz + 2.0 * c * ez + (c * c) * self.s2_integral[2]

            Fx = self.m_total * (acc_ref['x'] + 2.0 * c * evx + (c * c) * ex) + K * math.tanh(S2x / eps)
            Fy = self.m_total * (acc_ref['y'] + 2.0 * c * evy + (c * c) * ey) + K * math.tanh(S2y / eps)
            Fz = self.m_total * (acc_ref['z'] + 2.0 * c * evz + (c * c) * ez + self.g) + K * math.tanh(S2z / eps)

        elif controller_type == 'pdf':
            # PD with feedforward gravity & acceleration
            Fx = self.m_total * acc_ref['x'] + kp * ex + kd * evx
            Fy = self.m_total * acc_ref['y'] + kp * ey + kd * evy
            Fz = self.m_total * (acc_ref['z'] + self.g) + kp * ez + kd * evz

        else:  # 'pd' classic
            # Pure PD control without exact dynamic feedforward
            Fx = (kp * 0.6) * ex + (kd * 0.6) * evx
            Fy = (kp * 0.6) * ey + (kd * 0.6) * evy
            Fz = self.m_total * self.g * 0.8 + (kp * 0.6) * ez + (kd * 0.6) * evz

        # Convert Cartesian Force to Joint Torques via J^T
        J = self.compute_jacobian(self.pos['x'], self.pos['y'], self.pos['z'], q)

        # tau = J^T * F
        tau1 = J[0][0] * Fx + J[0][1] * Fy + J[0][2] * Fz
        tau2 = J[1][0] * Fx + J[1][1] * Fy + J[1][2] * Fz
        tau3 = J[2][0] * Fx + J[2][1] * Fy + J[2][2] * Fz

        return [tau1, tau2, tau3], [Fx, Fy, Fz]

    def step(self, controller_type, pos_ref, vel_ref, acc_ref, dt=0.002):
        """
        Advances real robot physics simulation by dt seconds.
        """
        torques, F_cart = self.compute_control(controller_type, pos_ref, vel_ref, acc_ref, dt)

        # Acceleration in Cartesian space: F_net = m * a  =>  a = F / m - g
        ax = F_cart[0] / self.m_total
        ay = F_cart[1] / self.m_total
        az = F_cart[2] / self.m_total - self.g

        # Semi-implicit Euler integration
        self.vel['x'] += ax * dt
        self.vel['y'] += ay * dt
        self.vel['z'] += az * dt

        self.pos['x'] += self.vel['x'] * dt
        self.pos['y'] += self.vel['y'] * dt
        self.pos['z'] += self.vel['z'] * dt

        ik = self.kin.solve_ik(self.pos['x'], self.pos['y'], self.pos['z'])
        return {
            'pos': dict(self.pos),
            'vel': dict(self.vel),
            'torques': torques,
            'theta_deg': ik['theta_deg'] if ik['ok'] else [0.0, 0.0, 0.0],
            'theta_rad': ik['theta_rad'] if ik['ok'] else [0.0, 0.0, 0.0]
        }
