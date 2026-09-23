/**
 * Dual Delta Robot 3D Simulator Main Application Controller
 * Interactive Surface & Controller Switching with Real-Time Comparison (Ghost IK vs Real Robot Dynamics)
 */

class DualRobotApp {
    constructor() {
        this.container = document.getElementById('viewport-container');
        this.canvas = document.getElementById('canvas3d');

        this.isRunning = true;
        this.simTime = 0.0;
        this.dt = 0.001;
        this.speedMultiplier = 1.0;

        this.selectedController = 's1sat';
        this.selectedTrajectory = 'ellipse';
        this.ghostVisible = true;
        this.trailsEnabled = true;

        const DeltaKinematics = window.DeltaKinematics;
        const DeltaDynamics = window.DeltaDynamics;

        this.kinematics = new DeltaKinematics();
        this.dynamics = new DeltaDynamics(this.kinematics);

        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;

        this.ghostRobot = null;
        this.realRobot = null;

        this.ghostTrailLine = null;
        this.realTrailLine = null;
        this.errorLine = null;

        this.ghostTrailPoints = [];
        this.realTrailPoints = [];

        this.historyLength = 300;
        this.chartData = {
            t: [],
            ghostX: [], realX: [],
            ghostZ: [], realZ: [],
            errorNorm: [],
            u1: [], u2: [], u3: []
        };

        try {
            this.init3DScene();
            this.initRobots();
            this.initVisualizers();
            this.initEventListeners();
            this.initCharts();

            this.animate = this.animate.bind(this);
            requestAnimationFrame(this.animate);
        } catch (e) {
            console.error('Error initializing Dual Robot Simulator App:', e);
        }
    }

    init3DScene() {
        const THREE = window.THREE;
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0b0f19);

        // Grid floor at Y = -350 mm
        const gridHelper = new THREE.GridHelper(700, 24, 0x00f3ff, 0x1f293d);
        gridHelper.position.y = -350;
        this.scene.add(gridHelper);

        // Camera setup
        this.camera = new THREE.PerspectiveCamera(42, this.container.clientWidth / this.container.clientHeight, 1, 5000);
        this.camera.position.set(300, 180, 420);

        // Lights
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.85);
        this.scene.add(ambientLight);

        const dirLight1 = new THREE.DirectionalLight(0x00f3ff, 1.2);
        dirLight1.position.set(400, 500, 350);
        this.scene.add(dirLight1);

        const dirLight2 = new THREE.DirectionalLight(0xff5722, 0.85);
        dirLight2.position.set(-400, -150, -300);
        this.scene.add(dirLight2);

        // Renderer
        this.renderer = new THREE.WebGLRenderer({ canvas: this.canvas, antialias: true });
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

        // OrbitControls
        const OrbitControlsClass = THREE.OrbitControls || window.OrbitControls;
        if (OrbitControlsClass) {
            this.controls = new OrbitControlsClass(this.camera, this.renderer.domElement);
            this.controls.enableDamping = true;
            this.controls.dampingFactor = 0.05;
            this.controls.target.set(0, -140, 0);
        }

        window.addEventListener('resize', () => this.onWindowResize());
    }

    initRobots() {
        const DeltaRobot3D = window.DeltaRobot3D;
        if (!DeltaRobot3D) return;

        // Ghost Robot (Semi-transparent cyan, IK ideal movement)
        this.ghostRobot = new DeltaRobot3D(this.scene, this.kinematics, {
            isGhost: true,
            color: 0x00f3ff,
            opacity: 0.35
        });

        // Real Robot (Solid metallic, dynamic solver + SMC controller)
        this.realRobot = new DeltaRobot3D(this.scene, this.kinematics, {
            isGhost: false,
            color: 0xff5722,
            opacity: 1.0
        });

        const ik0 = this.kinematics.solveIK({ x: 0, y: 0, z: -200 });
        if (ik0 && ik0.ok) {
            this.dynamics.resetState(ik0.theta);
        }
    }

    initVisualizers() {
        const THREE = window.THREE;

        // 1. Ghost Trail Line
        const ghostMat = new THREE.LineBasicMaterial({ color: 0x00f3ff, linewidth: 2, transparent: true, opacity: 0.7 });
        const ghostGeo = new THREE.BufferGeometry();
        this.ghostTrailLine = new THREE.Line(ghostGeo, ghostMat);
        this.scene.add(this.ghostTrailLine);

        // 2. Real Trail Line
        const realMat = new THREE.LineBasicMaterial({ color: 0xff5722, linewidth: 2.5 });
        const realGeo = new THREE.BufferGeometry();
        this.realTrailLine = new THREE.Line(realGeo, realMat);
        this.scene.add(this.realTrailLine);

        // 3. Dynamic Error Vector Line connecting Ghost EE and Real EE
        const errorMat = new THREE.LineDashedMaterial({
            color: 0xef4444,
            linewidth: 2,
            dashSize: 5,
            gapSize: 3
        });
        const errorGeo = new THREE.BufferGeometry();
        this.errorLine = new THREE.Line(errorGeo, errorMat);
        this.scene.add(this.errorLine);
    }

    restartSimulation() {
        this.simTime = 0.0;
        this.chartData = { t: [], ghostX: [], realX: [], ghostZ: [], realZ: [], errorNorm: [], u1: [], u2: [], u3: [] };
        this.ghostTrailPoints = [];
        this.realTrailPoints = [];

        const refMM = this.getReferenceTrajectory(0);
        const ik0 = this.kinematics.solveIK(refMM);
        if (ik0 && ik0.ok) {
            this.dynamics.resetState(ik0.theta);
        }

        this.isRunning = true;
        const btnPlay = document.getElementById('btn-play-pause');
        if (btnPlay) btnPlay.textContent = '⏸';
    }

    initEventListeners() {
        const btnPlay = document.getElementById('btn-play-pause');
        if (btnPlay) {
            btnPlay.addEventListener('click', () => {
                this.isRunning = !this.isRunning;
                btnPlay.textContent = this.isRunning ? '⏸' : '▶';
            });
        }

        const btnStop = document.getElementById('btn-stop');
        if (btnStop) {
            btnStop.addEventListener('click', () => {
                this.restartSimulation();
                this.isRunning = false;
                if (btnPlay) btnPlay.textContent = '▶';
            });
        }

        const btnGhost = document.getElementById('btn-toggle-ghost');
        if (btnGhost) {
            btnGhost.addEventListener('click', () => {
                this.ghostVisible = !this.ghostVisible;
                this.ghostRobot.setGhostOpacity(this.ghostVisible ? 0.35 : 0.0);
                btnGhost.classList.toggle('active', this.ghostVisible);
            });
        }

        const btnTrails = document.getElementById('btn-toggle-trails');
        if (btnTrails) {
            btnTrails.addEventListener('click', () => {
                this.trailsEnabled = !this.trailsEnabled;
                this.ghostTrailLine.visible = this.trailsEnabled;
                this.realTrailLine.visible = this.trailsEnabled;
                btnTrails.classList.toggle('active', this.trailsEnabled);
            });
        }

        const btnResetView = document.getElementById('btn-reset-view');
        if (btnResetView) {
            btnResetView.addEventListener('click', () => {
                this.camera.position.set(300, 180, 420);
                if (this.controls) this.controls.target.set(0, -140, 0);
            });
        }

        // Controller Chip Click Handler -> Run & Compare Immediately
        const ctrlChips = document.querySelectorAll('#ctrl-selector .chip-item');
        ctrlChips.forEach(chip => {
            chip.addEventListener('click', () => {
                ctrlChips.forEach(c => c.classList.remove('active'));
                chip.classList.add('active');
                this.selectedController = chip.dataset.ctrl;
                this.restartSimulation();
            });
        });

        // Trajectory Chip Click Handler
        const trajChips = document.querySelectorAll('#traj-selector .chip-item');
        trajChips.forEach(chip => {
            chip.addEventListener('click', () => {
                trajChips.forEach(c => c.classList.remove('active'));
                chip.classList.add('active');
                this.selectedTrajectory = chip.dataset.traj;
                this.restartSimulation();
            });
        });
    }

    getReferenceTrajectory(t) {
        let x = 0, y = 0, z = -200; // in mm
        const omega = 2 * Math.PI * 1.2;

        switch (this.selectedTrajectory) {
            case 'ellipse':
                x = 55 * Math.cos(omega * t);
                y = 35 * Math.sin(omega * t);
                z = -200 + 15 * Math.sin(2 * omega * t);
                break;

            case 'circle':
                x = 55 * Math.cos(omega * t);
                y = 55 * Math.sin(omega * t);
                z = -200;
                break;

            case 'figure8':
                x = 55 * Math.sin(omega * t);
                y = 45 * Math.sin(2 * omega * t);
                z = -200;
                break;

            case 'spiral':
                const r = 10 + 45 * (t % 3) / 3;
                x = r * Math.cos(omega * t);
                y = r * Math.sin(omega * t);
                z = -200 - 15 * (t % 3) / 3;
                break;
        }

        return { x, y, z };
    }

    animate() {
        requestAnimationFrame(this.animate);

        if (this.isRunning) {
            const substeps = 5;
            const dtSub = (this.dt * this.speedMultiplier) / substeps;

            for (let s = 0; s < substeps; s++) {
                this.simTime += dtSub;

                const refMM = this.getReferenceTrajectory(this.simTime);
                const ghostIK = this.kinematics.solveIK(refMM);

                if (ghostIK && ghostIK.ok) {
                    const qRef = ghostIK.theta; // degrees
                    const dqRef = [0, 0, 0];
                    const ddqRef = [0, 0, 0];

                    const u = this.dynamics.computeControl(this.selectedController, qRef, dqRef, ddqRef, dtSub);
                    this.dynamics.step(u, dtSub);

                    if (s === substeps - 1) {
                        const realPosMM = this.kinematics.solveFK(this.dynamics.q);
                        const realIK = realPosMM ? this.kinematics.solveIK(realPosMM) : null;

                        this.updateVisualizers(refMM, ghostIK, realPosMM, realIK, u);
                    }
                }
            }
        }

        if (this.controls) this.controls.update();
        if (this.renderer && this.scene && this.camera) {
            this.renderer.render(this.scene, this.camera);
        }
    }

    updateVisualizers(ghostPosMM, ghostIK, realPosMM, realIK, u) {
        const THREE = window.THREE;
        const wGhostPos = this.ghostRobot.updatePose(ghostPosMM, ghostIK);
        let wRealPos = null;
        if (realPosMM && realIK && realIK.ok) {
            wRealPos = this.realRobot.updatePose(realPosMM, realIK);
        }

        if (!wGhostPos || !wRealPos) return;

        // UI Telemetry Card Updates
        const elGx = document.getElementById('ghost-x');
        const elGy = document.getElementById('ghost-y');
        const elGz = document.getElementById('ghost-z');
        if (elGx) elGx.textContent = `${(ghostPosMM.x * 0.001).toFixed(3)} m`;
        if (elGy) elGy.textContent = `${(ghostPosMM.y * 0.001).toFixed(3)} m`;
        if (elGz) elGz.textContent = `${(ghostPosMM.z * 0.001).toFixed(3)} m`;

        const elRx = document.getElementById('real-x');
        const elRy = document.getElementById('real-y');
        const elRz = document.getElementById('real-z');
        if (elRx) elRx.textContent = `${(realPosMM.x * 0.001).toFixed(3)} m`;
        if (elRy) elRy.textContent = `${(realPosMM.y * 0.001).toFixed(3)} m`;
        if (elRz) elRz.textContent = `${(realPosMM.z * 0.001).toFixed(3)} m`;

        const errX = ghostPosMM.x - realPosMM.x;
        const errY = ghostPosMM.y - realPosMM.y;
        const errZ = ghostPosMM.z - realPosMM.z;
        const errorNorm = Math.sqrt(errX * errX + errY * errY + errZ * errZ); // mm
        const elErr = document.getElementById('error-norm');
        if (elErr) elErr.textContent = `${errorNorm.toFixed(3)} mm`;

        const elTime = document.getElementById('sim-time');
        if (elTime) elTime.textContent = `t = ${this.simTime.toFixed(2)} s`;

        // Update Error Line Vector
        const errorLinePositions = new Float32Array([
            wGhostPos.x, wGhostPos.y, wGhostPos.z,
            wRealPos.x, wRealPos.y, wRealPos.z
        ]);
        this.errorLine.geometry.setAttribute('position', new THREE.BufferAttribute(errorLinePositions, 3));
        this.errorLine.geometry.computeBoundingSphere();

        // Update Trail Lines
        if (this.trailsEnabled) {
            this.ghostTrailPoints.push(wGhostPos.clone());
            this.realTrailPoints.push(wRealPos.clone());

            if (this.ghostTrailPoints.length > 200) this.ghostTrailPoints.shift();
            if (this.realTrailPoints.length > 200) this.realTrailPoints.shift();

            this.ghostTrailLine.geometry.setFromPoints(this.ghostTrailPoints);
            this.realTrailLine.geometry.setFromPoints(this.realTrailPoints);
        }

        this.pushChartData({
            t: this.simTime,
            ghostX: ghostPosMM.x * 0.001, realX: realPosMM.x * 0.001,
            ghostZ: ghostPosMM.z * 0.001, realZ: realPosMM.z * 0.001,
            errorNorm,
            u1: u[0], u2: u[1], u3: u[2]
        });
    }

    pushChartData(data) {
        this.chartData.t.push(data.t);
        this.chartData.ghostX.push(data.ghostX);
        this.chartData.realX.push(data.realX);
        this.chartData.ghostZ.push(data.ghostZ);
        this.chartData.realZ.push(data.realZ);
        this.chartData.errorNorm.push(data.errorNorm);
        this.chartData.u1.push(data.u1);
        this.chartData.u2.push(data.u2);
        this.chartData.u3.push(data.u3);

        if (this.chartData.t.length > this.historyLength) {
            for (let key in this.chartData) {
                this.chartData[key].shift();
            }
        }

        this.renderCharts();
    }

    initCharts() {
        const cPosX = document.getElementById('chart-pos-x');
        const cPosZ = document.getElementById('chart-pos-z');
        const cErr  = document.getElementById('chart-error');
        const cTorque = document.getElementById('chart-torque');

        if (cPosX) this.ctxPosX = cPosX.getContext('2d');
        if (cPosZ) this.ctxPosZ = cPosZ.getContext('2d');
        if (cErr)  this.ctxErr  = cErr.getContext('2d');
        if (cTorque) this.ctxTorque = cTorque.getContext('2d');
    }

    renderCharts() {
        if (this.ctxPosX) this.drawChart(this.ctxPosX, [this.chartData.ghostX, this.chartData.realX], ['#00f3ff', '#ff5722']);
        if (this.ctxPosZ) this.drawChart(this.ctxPosZ, [this.chartData.ghostZ, this.chartData.realZ], ['#00f3ff', '#ff5722']);
        if (this.ctxErr)  this.drawChart(this.ctxErr,  [this.chartData.errorNorm], ['#ef4444']);
        if (this.ctxTorque) this.drawChart(this.ctxTorque, [this.chartData.u1, this.chartData.u2, this.chartData.u3], ['#00f3ff', '#8b5cf6', '#10b981']);
    }

    drawChart(ctx, seriesArray, colors) {
        const width = ctx.canvas.width;
        const height = ctx.canvas.height;
        ctx.clearRect(0, 0, width, height);

        if (this.chartData.t.length < 2) return;

        let minVal = Infinity, maxVal = -Infinity;
        seriesArray.forEach(series => {
            series.forEach(v => {
                if (v < minVal) minVal = v;
                if (v > maxVal) maxVal = v;
            });
        });

        if (minVal === maxVal) { minVal -= 0.01; maxVal += 0.01; }
        const margin = (maxVal - minVal) * 0.1;
        minVal -= margin;
        maxVal += margin;

        const dx = width / (this.historyLength - 1);

        seriesArray.forEach((series, sIndex) => {
            ctx.beginPath();
            ctx.strokeStyle = colors[sIndex];
            ctx.lineWidth = 1.5;

            series.forEach((val, i) => {
                const x = i * dx;
                const y = height - ((val - minVal) / (maxVal - minVal)) * height;
                if (i === 0) ctx.moveTo(x, y);
                else ctx.lineTo(x, y);
            });

            ctx.stroke();
        });
    }

    onWindowResize() {
        if (this.camera && this.container) {
            this.camera.aspect = this.container.clientWidth / this.container.clientHeight;
            this.camera.updateProjectionMatrix();
        }
        if (this.renderer && this.container) {
            this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        }
    }
}

window.addEventListener('DOMContentLoaded', () => {
    window.app = new DualRobotApp();
});
