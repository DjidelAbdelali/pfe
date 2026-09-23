/**
 * Three.js 3D Delta Robot Visualizer with Native Millimeter Scale
 * Official ASSEMBLY_1 URDF Mated Kinematic Chains
 */

function worldFromMM(p) {
  // Convert mm to 3D world vector: map (x, y, z_kin) -> (x, z_kin, y)
  return new THREE.Vector3(p.x, p.z, p.y);
}

function placeCylinderBetween(mesh, a, b) {
  const dir = new THREE.Vector3().subVectors(b, a);
  const len = dir.length();
  const mid = new THREE.Vector3().addVectors(a, b).multiplyScalar(0.5);
  mesh.position.copy(mid);
  mesh.scale.set(1, Math.max(len, 0.1), 1);
  const quat = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
  mesh.quaternion.copy(quat);
}

window.DeltaRobot3D = class DeltaRobot3D {
    constructor(scene, kinematics, options = {}) {
        this.scene = scene;
        this.kinematics = kinematics;
        this.isGhost = options.isGhost || false;

        this.colorAccent = options.color || (this.isGhost ? 0x00f3ff : 0xd9720f);
        this.opacity = options.opacity !== undefined ? options.opacity : (this.isGhost ? 0.35 : 1.0);

        this.group = new THREE.Group();
        this.scene.add(this.group);

        this.bicepMeshes = [];
        this.rodMeshes = [];
        this.elbowJointMeshes = [];
        this.platJointMeshes = [];
        this.platformMesh = null;

        this.initMaterials();
        this.buildRobot();
    }

    initMaterials() {
        const isTransparent = this.isGhost;

        this.armMat = new THREE.MeshStandardMaterial({
            color: this.colorAccent,
            metalness: 0.4,
            roughness: 0.35,
            transparent: isTransparent,
            opacity: this.opacity
        });

        this.darkMat = new THREE.MeshStandardMaterial({
            color: this.isGhost ? 0x005577 : 0x353b46,
            metalness: 0.5,
            roughness: 0.4,
            transparent: isTransparent,
            opacity: this.opacity * 0.9
        });

        this.jointMat = new THREE.MeshStandardMaterial({
            color: this.isGhost ? 0x00ffff : 0x6b7280,
            metalness: 0.6,
            roughness: 0.3,
            transparent: isTransparent,
            opacity: this.opacity
        });

        this.platformMat = new THREE.MeshStandardMaterial({
            color: this.isGhost ? 0x00f3ff : 0x2e3440,
            metalness: 0.5,
            roughness: 0.4,
            transparent: isTransparent,
            opacity: this.opacity
        });

        this.baseMat = new THREE.MeshStandardMaterial({
            color: this.isGhost ? 0x00aacc : 0xb4bac4,
            metalness: 0.65,
            roughness: 0.35,
            transparent: isTransparent,
            opacity: this.opacity
        });
    }

    makeRod(radiusMM) {
        const geo = new THREE.CylinderGeometry(radiusMM, radiusMM, 1, 12);
        return new THREE.Mesh(geo, this.darkMat);
    }

    buildRobot() {
        const { Rf, Re } = this.kinematics;

        // 1. Base Plate & Stepper Motors
        this.baseGroup = new THREE.Group();
        const basePlateGeo = new THREE.CylinderGeometry(Rf + 30, Rf + 30, 8, 48);
        const baseMesh = new THREE.Mesh(basePlateGeo, this.baseMat);
        baseMesh.position.set(0, 0, 0);
        this.baseGroup.add(baseMesh);

        const phiDeg = [95.7, 216.3, 336.4];
        const d2r = Math.PI / 180;

        for (let i = 0; i < 3; i++) {
            const phi = phiDeg[i] * d2r;
            const bx = Rf * Math.cos(phi), by = Rf * Math.sin(phi);
            const motor = new THREE.Mesh(new THREE.CylinderGeometry(14, 14, 26, 20), this.darkMat);
            const wp = worldFromMM({ x: bx, y: by, z: 0 });
            motor.position.copy(wp).add(new THREE.Vector3(0, -13, 0));
            motor.rotation.z = Math.PI / 2;
            motor.rotation.y = -phi;
            this.baseGroup.add(motor);
        }
        this.group.add(this.baseGroup);

        // 2. Arms (3 Biceps + 3 Forearm Rod Pairs)
        for (let i = 0; i < 3; i++) {
            const bicep = this.makeRod(7);
            bicep.material = this.armMat;
            this.group.add(bicep);
            this.bicepMeshes.push(bicep);

            const rodA = this.makeRod(3.5);
            const rodB = this.makeRod(3.5);
            this.group.add(rodA);
            this.group.add(rodB);
            this.rodMeshes.push(rodA, rodB);

            const ej = new THREE.Mesh(new THREE.SphereGeometry(6.5, 16, 12), this.jointMat);
            this.group.add(ej);
            this.elbowJointMeshes.push(ej);

            const pj = new THREE.Mesh(new THREE.SphereGeometry(5, 16, 12), this.jointMat);
            this.group.add(pj);
            this.platJointMeshes.push(pj);
        }

        // 3. Moving End-Effector Platform
        const platGeo = new THREE.CylinderGeometry(Re + 14, Re + 14, 7, 40);
        this.platformMesh = new THREE.Mesh(platGeo, this.platformMat);

        // Tool Tip Cone
        const toolGeo = new THREE.ConeGeometry(6, 20, 16);
        toolGeo.rotateX(Math.PI);
        toolGeo.translate(0, -15, 0);
        const toolMesh = new THREE.Mesh(toolGeo, this.armMat);
        this.platformMesh.add(toolMesh);

        this.group.add(this.platformMesh);
    }

    /**
     * Update 3D pose of robot given target point in mm {x, y, z} and ikResult
     */
    updatePose(targetMM, ikResult) {
        if (!targetMM || !ikResult || !ikResult.ok) return;
        const { Rf } = this.kinematics;

        const wTarget = worldFromMM(targetMM);
        this.platformMesh.position.copy(wTarget);

        const phiDeg = [95.7, 216.3, 336.4];
        const d2r = Math.PI / 180;

        for (let i = 0; i < 3; i++) {
            const phi = phiDeg[i] * d2r;
            const baseMM = { x: Rf * Math.cos(phi), y: Rf * Math.sin(phi), z: 0 };
            const elbowMM = ikResult.elbows[i];
            const platMM = ikResult.platformPts[i];

            if (!elbowMM || !platMM) continue;

            const base = worldFromMM(baseMM);
            const elbow = worldFromMM(elbowMM);
            const plat = worldFromMM(platMM);

            // Bicep Link
            placeCylinderBetween(this.bicepMeshes[i], base, elbow);
            this.elbowJointMeshes[i].position.copy(elbow);
            this.platJointMeshes[i].position.copy(plat);

            // Parallel Forearm Rods (offset along local tangential direction)
            const tangent = new THREE.Vector3(-Math.sin(phi), Math.cos(phi), 0).multiplyScalar(9);
            const tOffset = worldFromMM({ x: tangent.x, y: tangent.y, z: 0 }).sub(worldFromMM({ x: 0, y: 0, z: 0 }));

            placeCylinderBetween(this.rodMeshes[i * 2], elbow.clone().add(tOffset), plat.clone().add(tOffset));
            placeCylinderBetween(this.rodMeshes[i * 2 + 1], elbow.clone().sub(tOffset), plat.clone().sub(tOffset));
        }

        return wTarget;
    }

    setGhostOpacity(opacity) {
        this.opacity = opacity;
        this.group.traverse((child) => {
            if (child.isMesh && child.material) {
                child.material.opacity = opacity;
                child.material.transparent = opacity < 1.0;
            }
        });
    }
};
