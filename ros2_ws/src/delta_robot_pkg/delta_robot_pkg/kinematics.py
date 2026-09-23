"""
3-RSS Parallel Delta Robot Kinematics Solver (Python / ROS 2)
Geometry from PFE model:
Rf = 0.0805 m, L = 0.0801 m, l = 0.1600 m, Re = 0.0351 m
Motor Base Angles: [95.7°, 216.3°, 336.4°]
"""

import math

PHI_DEG = [95.7, 216.3, 336.4]
D2R = math.pi / 180.0


def circle_intersect(c1x, c1z, r1, c2x, c2z, r2):
    dx = c2x - c1x
    dz = c2z - c1z
    d = math.hypot(dx, dz)
    if d < 1e-9 or d > r1 + r2 + 1e-6 or d < abs(r1 - r2) - 1e-6:
        return []
    a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d)
    h2 = r1 * r1 - a * a
    h = math.sqrt(max(h2, 0.0))
    mx = c1x + a * dx / d
    mz = c1z + a * dz / d
    ox = -dz / d * h
    oz = dx / d * h
    return [{'x': mx + ox, 'z': mz + oz}, {'x': mx - ox, 'z': mz - oz}]


class DeltaKinematics:
    def __init__(self, Rf=0.0805, L=0.0801, l=0.1600, Re=0.0351):
        self.Rf = Rf
        this_L = L
        this_l = l
        this_Re = Re
        self.L = this_L
        self.l = this_l
        self.Re = this_Re

    def solve_ik(self, x, y, z):
        """
        Inverse Kinematics: (x, y, z) in meters -> [theta1, theta2, theta3] (in radians & degrees)
        """
        thetas_deg = []
        thetas_rad = []
        elbows = []
        platform_pts = []
        ok = True

        for i in range(3):
            phi = PHI_DEG[i] * D2R
            c = math.cos(phi)
            s = math.sin(phi)

            tx = x * c + y * s
            ty = -x * s + y * c
            tz = z

            Px = tx + self.Re
            Pz = tz
            l_eff2 = self.l * self.l - ty * ty

            if l_eff2 < 0:
                ok = False
                thetas_deg.append(0.0)
                thetas_rad.append(0.0)
                continue

            l_eff = math.sqrt(l_eff2)
            sols = circle_intersect(self.Rf, 0.0, self.L, Px, Pz, l_eff)
            if not sols:
                ok = False
                thetas_deg.append(0.0)
                thetas_rad.append(0.0)
                continue

            e = sols[0] if sols[0]['x'] > sols[1]['x'] else sols[1]
            theta_rad = math.atan2(e['z'], e['x'] - self.Rf)
            theta_deg = theta_rad / D2R

            thetas_deg.append(theta_deg)
            thetas_rad.append(theta_rad)
            elbows.append({'x': e['x'] * c, 'y': e['x'] * s, 'z': e['z']})
            platform_pts.append({'x': x + self.Re * c, 'y': y + self.Re * s, 'z': z})

        return {
            'ok': ok,
            'theta_deg': thetas_deg,
            'theta_rad': thetas_rad,
            'elbows': elbows,
            'platform_pts': platform_pts,
        }

    def solve_fk(self, theta_deg):
        """
        Forward Kinematics: [theta1, theta2, theta3] in degrees -> (x, y, z) in meters
        """
        C = []
        for i in range(3):
            phi = PHI_DEG[i] * D2R
            c = math.cos(phi)
            s = math.sin(phi)
            th = theta_deg[i] * D2R
            ex = self.Rf + self.L * math.cos(th)
            ez = self.L * math.sin(th)
            C.append({'x': ex * c - self.Re * c, 'y': ex * s - self.Re * s, 'z': ez})

        C1, C2, C3 = C
        Ax = 2.0 * (C2['x'] - C1['x'])
        Ay = 2.0 * (C2['y'] - C1['y'])
        Az = 2.0 * (C2['z'] - C1['z'])

        Bx = 2.0 * (C3['x'] - C1['x'])
        By = 2.0 * (C3['y'] - C1['y'])
        Bz = 2.0 * (C3['z'] - C1['z'])

        dA = (C2['x']**2 + C2['y']**2 + C2['z']**2) - (C1['x']**2 + C1['y']**2 + C1['z']**2)
        dB = (C3['x']**2 + C3['y']**2 + C3['z']**2) - (C1['x']**2 + C1['y']**2 + C1['z']**2)

        det = Ax * By - Ay * Bx
        if abs(det) < 1e-9:
            return None

        x0 = (dA * By - Ay * dB) / det
        x1 = (Ay * Bz - Az * By) / det

        y0 = (Ax * dB - dA * Bx) / det
        y1 = (Az * Bx - Ax * Bz) / det

        p = x0 - C1['x']
        q = y0 - C1['y']

        aq = x1 * x1 + y1 * y1 + 1.0
        bq = 2.0 * p * x1 + 2.0 * q * y1 - 2.0 * C1['z']
        cq = p * p + q * q + C1['z']**2 - self.l**2

        disc = bq * bq - 4.0 * aq * cq
        if disc < 0:
            return None

        sq = math.sqrt(disc)
        z1 = (-bq + sq) / (2.0 * aq)
        z2 = (-bq - sq) / (2.0 * aq)
        z = min(z1, z2)
        x = x0 + x1 * z
        y = y0 + y1 * z

        return {'x': x, 'y': y, 'z': z}
