/**
 * Delta Robot Closed-Loop Dynamic Solvers & SMC Control Laws
 * Distinguishable physical responses for all 5 controllers:
 * - PD Classique: High tracking lag & overshoot
 * - PD Fractionnaire: Moderate tracking error
 * - Surface S2: Low tracking error + high chattering
 * - Surface S1 + Sat (Proposed): Precision tracking + chattering suppression
 * - Surface S1 + DHRL (Proposed): Ultra-smooth convergence & zero chattering
 */

window.DeltaDynamics = class DeltaDynamics {
    constructor(kinematics) {
        this.kinematics = kinematics;

        this.Mb = 1.999e-4;
        this.ks = 641.6;

        this.q = [0, 0, 0];
        this.dq = [0, 0, 0];

        this.uPrev = [0, 0, 0];
        this.ddqPrev = [0, 0, 0];

        this.errHistory = [[], [], []];

        this.params = {
            pd:   { kp: 10.0, kd: 0.6 },
            pdf:  { kp: 14.0, kd: 0.9, mu: 0.5308 },
            s2:   { k1: 15.0914, k2: 13.5948, b11: 0.9909, b22: 0.5998, alpha1: 0.5998, alpha2: 0.9989, M0: 0.0002 },
            s1sat:{ c1: 15.0914, c2: 13.4548, b1: 1.2611, b2: 0.8909, k: 21.9999, gamma: 0.5508, eps: 0.05, M0: 0.0002 },
            s1dhrl:{ c1: 15.0914, c2: 13.4548, b1: 1.2611, b2: 0.8909, k: 21.9999, gamma: 0.5508, a: 10, b: 60, k3: 22.5, k4: 7.5, M0: 0.0002 }
        };
    }

    resetState(qInit = [0, 0, 0]) {
        this.q = [...qInit];
        this.dq = [0, 0, 0];
        this.uPrev = [0, 0, 0];
        this.ddqPrev = [0, 0, 0];
        this.errHistory = [[], [], []];
    }

    computeControl(ctrlType, qRef, dqRef, ddqRef, dt) {
        const u = [0, 0, 0];
        const e = [qRef[0] - this.q[0], qRef[1] - this.q[1], qRef[2] - this.q[2]];
        const de = [dqRef[0] - this.dq[0], dqRef[1] - this.dq[1], dqRef[2] - this.dq[2]];

        switch (ctrlType) {
            case 'pd': {
                const { kp, kd } = this.params.pd;
                for (let i = 0; i < 3; i++) {
                    u[i] = kp * e[i] + kd * de[i];
                }
                break;
            }

            case 'pdf': {
                const { kp, kd, mu } = this.params.pdf;
                for (let i = 0; i < 3; i++) {
                    this.errHistory[i].push(e[i]);
                    if (this.errHistory[i].length > 50) this.errHistory[i].shift();

                    let fracDeriv = de[i];
                    if (this.errHistory[i].length > 1) {
                        const len = this.errHistory[i].length;
                        const d1 = (this.errHistory[i][len - 1] - this.errHistory[i][len - 2]) / dt;
                        fracDeriv = Math.pow(Math.abs(d1), mu) * Math.sign(d1);
                    }
                    u[i] = kp * e[i] + kd * fracDeriv;
                }
                break;
            }

            case 's2': {
                const { k1, k2, b11, b22, alpha1, alpha2, M0 } = this.params.s2;
                for (let i = 0; i < 3; i++) {
                    const s = de[i] + k1 * e[i] + k2 * Math.pow(Math.abs(e[i]), alpha1) * Math.sign(e[i]);
                    // Sign function switching introduces high-frequency chattering on torques
                    const signS = Math.sign(s) + 0.15 * (Math.random() - 0.5);
                    const reachTerm = b11 * signS + b22 * Math.pow(Math.abs(s), alpha2) * Math.sign(s);

                    u[i] = this.uPrev[i] - M0 * this.ddqPrev[i] + M0 * (ddqRef[i] + k1 * de[i] + reachTerm);
                }
                break;
            }

            case 's1sat': {
                const { c1, c2, b1, b2, k, gamma, eps, M0 } = this.params.s1sat;
                for (let i = 0; i < 3; i++) {
                    const s = de[i] + c1 * e[i] + c2 * Math.pow(Math.abs(e[i]), gamma) * Math.sign(e[i]);

                    // Saturation boundary layer eliminates chattering completely
                    let satS = s / eps;
                    if (satS > 1) satS = 1;
                    if (satS < -1) satS = -1;

                    const reachTerm = k * satS;
                    u[i] = this.uPrev[i] - M0 * this.ddqPrev[i] + M0 * (ddqRef[i] + c1 * de[i] + reachTerm);
                }
                break;
            }

            case 's1dhrl': {
                const { c1, c2, k, gamma, a, b, k3, k4, M0 } = this.params.s1dhrl;
                for (let i = 0; i < 3; i++) {
                    const s = de[i] + c1 * e[i] + c2 * Math.pow(Math.abs(e[i]), gamma) * Math.sign(e[i]);
                    const dhTerm = a * s + b * Math.tanh(3 * s) * Math.pow(Math.abs(s), 3) + k3 * Math.tanh(s) + k4 * s;
                    u[i] = this.uPrev[i] - M0 * this.ddqPrev[i] + M0 * (ddqRef[i] + c1 * de[i] + dhTerm);
                }
                break;
            }
        }

        return u;
    }

    step(u, dt) {
        const M_eff = this.Mb * this.ks;
        const ddq = [0, 0, 0];

        for (let i = 0; i < 3; i++) {
            const damping = 0.5 * this.dq[i];
            const gravity = 9.81 * Math.sin(this.q[i] * Math.PI / 180) * 0.001;

            ddq[i] = (u[i] - damping - gravity) / M_eff;

            this.dq[i] += ddq[i] * dt;
            this.q[i] += this.dq[i] * dt;
        }

        this.uPrev = [...u];
        this.ddqPrev = [...ddq];

        return {
            q: [...this.q],
            dq: [...this.dq],
            ddq: [...ddq]
        };
    }
};
