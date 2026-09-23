/**
 * 3-RSS Parallel Delta Robot Kinematics Solver
 * Exact Geometry & Outward Elbow Solver from official ASSEMBLY_1 URDF
 */

const D2R = Math.PI / 180;
const PHI_DEG = [95.7, 216.3, 336.4];

function circleIntersect(c1x, c1z, r1, c2x, c2z, r2) {
    const dx = c2x - c1x, dz = c2z - c1z;
    const d = Math.hypot(dx, dz);
    if (d < 1e-9 || d > r1 + r2 + 1e-6 || d < Math.abs(r1 - r2) - 1e-6) return [];
    const a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
    const h2 = r1 * r1 - a * a;
    const h = h2 > 0 ? Math.sqrt(h2) : 0;
    const mx = c1x + a * dx / d, mz = c1z + a * dz / d;
    const ox = -dz / d * h, oz = dx / d * h;
    return [{ x: mx + ox, z: mz + oz }, { x: mx - ox, z: mz - oz }];
}

window.DeltaKinematics = class DeltaKinematics {
    constructor(params = {}) {
        this.Rf = params.Rf !== undefined ? params.Rf : 80.5; // mm
        this.L  = params.L  !== undefined ? params.L  : 80.1;
        this.l  = params.l  !== undefined ? params.l  : 160.0;
        this.Re = params.Re !== undefined ? params.Re : 35.1;
    }

    /**
     * Solve Inverse Kinematics: target {x, y, z} in mm -> {ok, theta: [deg, deg, deg], elbows, platformPts}
     */
    solveIK(target) {
        const thetas = [], elbows = [], platformPts = [];
        let ok = true;

        for (let i = 0; i < 3; i++) {
            const phi = PHI_DEG[i] * D2R;
            const c = Math.cos(phi), s = Math.sin(phi);

            const tx = target.x * c + target.y * s;
            const ty = -target.x * s + target.y * c;
            const tz = target.z;

            const Px = tx + this.Re, Pz = tz;
            const lEff2 = this.l * this.l - ty * ty;

            if (lEff2 < 0) {
                ok = false;
                thetas.push(null); elbows.push(null); platformPts.push(null);
                continue;
            }

            const lEff = Math.sqrt(lEff2);
            const sols = circleIntersect(this.Rf, 0, this.L, Px, Pz, lEff);
            if (sols.length === 0) {
                ok = false;
                thetas.push(null); elbows.push(null); platformPts.push(null);
                continue;
            }

            const e = sols[0].x > sols[1].x ? sols[0] : sols[1];
            const theta = Math.atan2(e.z, e.x - this.Rf) / D2R;

            thetas.push(theta);
            elbows.push({ x: e.x * c, y: e.x * s, z: e.z });
            platformPts.push({ x: target.x + this.Re * c, y: target.y + this.Re * s, z: target.z });
        }

        return { ok, theta: thetas, elbows, platformPts };
    }

    /**
     * Solve Forward Kinematics: motor angles thetaDeg[3] -> target {x, y, z} in mm
     */
    solveFK(thetaDeg) {
        const C = [];
        for (let i = 0; i < 3; i++) {
            const phi = PHI_DEG[i] * D2R, c = Math.cos(phi), s = Math.sin(phi);
            const th = thetaDeg[i] * D2R;
            const ex = this.Rf + this.L * Math.cos(th), ez = this.L * Math.sin(th);
            C.push({ x: ex * c - this.Re * c, y: ex * s - this.Re * s, z: ez });
        }

        const [C1, C2, C3] = C;
        const Ax = 2 * (C2.x - C1.x), Ay = 2 * (C2.y - C1.y), Az = 2 * (C2.z - C1.z);
        const Bx = 2 * (C3.x - C1.x), By = 2 * (C3.y - C1.y), Bz = 2 * (C3.z - C1.z);

        const dA = (C2.x ** 2 + C2.y ** 2 + C2.z ** 2) - (C1.x ** 2 + C1.y ** 2 + C1.z ** 2);
        const dB = (C3.x ** 2 + C3.y ** 2 + C3.z ** 2) - (C1.x ** 2 + C1.y ** 2 + C1.z ** 2);

        const det = Ax * By - Ay * Bx;
        if (Math.abs(det) < 1e-9) return null;

        const x0 = (dA * By - Ay * dB) / det, x1 = (Ay * Bz - Az * By) / det;
        const y0 = (Ax * dB - dA * Bx) / det, y1 = (Az * Bx - Ax * Bz) / det;

        const p = x0 - C1.x, x1c = x1;
        const q = y0 - C1.y, y1c = y1;

        const aq = x1c * x1c + y1c * y1c + 1;
        const bq = 2 * p * x1c + 2 * q * y1c - 2 * C1.z;
        const cq = p * p + q * q + C1.z * C1.z - this.l * this.l;

        const disc = bq * bq - 4 * aq * cq;
        if (disc < 0) return null;

        const sq = Math.sqrt(disc);
        const z1 = (-bq + sq) / (2 * aq), z2 = (-bq - sq) / (2 * aq);
        const z = Math.min(z1, z2);
        const x = x0 + x1c * z, y = y0 + y1c * z;

        return { x, y, z };
    }
};
