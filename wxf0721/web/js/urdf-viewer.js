// urdf-viewer.js
// 背景 3D 模型查看器（基于 Three.js + urdf-loader）
// 始终显示，透明度 30%（由 CSS 控制）

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import URDFLoader from 'urdf-loader';

export class UrdfViewer {
    constructor(canvas) {
        this.canvas = canvas;

        // ── 场景 ──────────────────────────────────
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0xf0f4f8);

        // ── 相机 ──────────────────────────────────
        this.camera = new THREE.PerspectiveCamera(
            45,
            window.innerWidth / window.innerHeight,
            0.01, 100
        );
        this.camera.position.set(3, 1.5, 3);
        this.camera.lookAt(0, 0.8, 0);

        // ── 渲染器 ────────────────────────────────
        this.renderer = new THREE.WebGLRenderer({
            canvas: canvas,
            antialias: true,
            alpha: true
        });
        this.renderer.setSize(window.innerWidth, window.innerHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.shadowMap.enabled = true;

        // ── 灯光 ──────────────────────────────────
        this.scene.add(new THREE.AmbientLight(0xffffff, 0.75));
        const dirLight = new THREE.DirectionalLight(0xffffff, 1.5);
        dirLight.position.set(5, 10, 7);
        this.scene.add(dirLight);
        const fillLight = new THREE.DirectionalLight(0x88aaff, 0.6);
        fillLight.position.set(-5, 3, -5);
        this.scene.add(fillLight);
        const rimLight = new THREE.DirectionalLight(0xffffff, 0.5);
        rimLight.position.set(0, 5, -10);
        this.scene.add(rimLight);

        // ── 网格地面 ───────────────────────────────
        const grid = new THREE.GridHelper(10, 20, 0x409eff, 0xe4ecf4);
        grid.material.opacity = 0.3;
        grid.material.transparent = true;
        this.scene.add(grid);

        // ── 控制器（指针事件由前景处理，这里仅动画） ──
        this.controls = new OrbitControls(this.camera, canvas);
        this.controls.target.set(0, 0.8, 0);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.08;

        // ── URDF 加载器 ───────────────────────────
        this.stlLoader = new STLLoader();
        this.urdfLoader = new URDFLoader();
        this.urdfLoader.loadMeshCb = (path, manager, done) => {
            // model.urdf 中的路径是相对 meshes/ 的，如 G2/T2/base_link.stl
            // 文件名大小写可能不匹配（Linux 大小写敏感），统一转小写 .stl
            const url = 'meshes/' + path.replace(/^\.?\//, '').replace(/^meshes\//, '')
                .replace(/\.stl$/i, '.STL');
            this.stlLoader.load(url, (geom) => {
                console.log('[STL] 加载成功:', url, '顶点数:', geom.attributes.position?.count);
                const mat = new THREE.MeshPhongMaterial({
                    color: 0xcccccc,
                    specular: 0x444444,
                    shininess: 50
                });
                const mesh = new THREE.Mesh(geom, mat);
                mesh.castShadow = true;
                mesh.receiveShadow = true;
                done(mesh);
            }, undefined, (err) => {
                console.warn('STL 加载失败:', url, err);
                done(new THREE.Mesh());
            });
        };

        this.robot = null;
        this._onResize = this._onResize.bind(this);
        window.addEventListener('resize', this._onResize);

        this._animate = this._animate.bind(this);
        this._animate();
    }

    loadUrdf(url) {
        if (this.robot) {
            this.scene.remove(this.robot);
            this.robot = null;
        }
        this.urdfLoader.load(url, (robot) => {
            this.robot = robot;
            robot.traverse(c => {
                if (c.isMesh) {
                    c.castShadow = true;
                    c.receiveShadow = true;
                }
            });
            this.scene.add(robot);

            // URDF 默认 Z 轴朝上，Three.js 默认 Y 轴朝上
            // 绕 X 轴逆时针旋转 90°（-π/2），使 Z→Y，让机器人立正
            robot.rotation.x = -Math.PI / 2;

            // 调试：输出模型信息
            let meshCount = 0;
            robot.traverse(c => { if (c.isMesh) meshCount++; });
            console.log('[URDF] 模型已加载, mesh 数量:', meshCount);

            // 自动调整相机
            const box = new THREE.Box3().setFromObject(robot);
            const size = box.getSize(new THREE.Vector3());
            const center = box.getCenter(new THREE.Vector3());
            console.log('[URDF] 包围盒 size:', size, 'center:', center);

            const maxDim = Math.max(size.x, size.y, size.z) || 1;
            const dist = maxDim * 2.0;
            this.camera.position.set(center.x + dist, center.y + dist * 0.6, center.z + dist);
            this.camera.near = 0.01;
            this.camera.far = dist * 10;
            this.camera.updateProjectionMatrix();
            this.controls.target.copy(center);
            this.controls.update();
        }, undefined, (err) => {
            console.error('URDF 加载失败:', err);
        });
    }

    setJointAngle(jointName, angleRad) {
        if (!this.robot) return;
        const j = this.robot.joints[jointName];
        if (j) j.setJointValue(angleRad);
    }

    /**
     * 根据机器人状态消息批量设置关节角
     * @param {Object} joints - { 关节名: 弧度值 }
     */
    setJointsFromStatus(joints) {
        if (!this.robot) return;
        for (const [name, rad] of Object.entries(joints)) {
            const j = this.robot.joints[name];
            if (j) j.setJointValue(rad);
        }
    }

    _onResize() {
        this.camera.aspect = window.innerWidth / window.innerHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(window.innerWidth, window.innerHeight);
    }

    _animate() {
        requestAnimationFrame(this._animate);
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }
}
