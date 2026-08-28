// chassis-control.js
// 底盘控制：画布展示点位 + 遥控 + 到位导航

import { mqttClient } from './mqtt-client.js';

export default {
    name: 'ChassisControl',
    inject: ['getRobotStatus'],
    template: `
    <div class="panel cc-panel">
        <!-- 画布区域 -->
        <div class="cc-canvas-wrap">
            <canvas ref="canvas" class="cc-canvas"></canvas>

            <!-- 右上角漂浮遥控面板 -->
            <div class="cc-remote">
                <div class="cc-remote-title">底盘遥控</div>
                <!-- XY 移动: X 前后 / Y 左右 -->
                <div class="cc-xy-pad">
                    <div class="cc-xy-row">
                        <button class="cc-dir-btn" @click="moveX(1)">X+ 前</button>
                    </div>
                    <div class="cc-xy-row">
                        <button class="cc-dir-btn" @click="moveY(1)">Y+ 左</button>
                        <button class="cc-dir-btn cc-center" disabled>底盘</button>
                        <button class="cc-dir-btn" @click="moveY(-1)">Y- 右</button>
                    </div>
                    <div class="cc-xy-row">
                        <button class="cc-dir-btn" @click="moveX(-1)">X- 后</button>
                    </div>
                </div>
                <!-- 旋转控制 -->
                <div class="cc-rot-row">
                    <button class="cc-rot-btn" @click="rotate(1)">↺ 左转</button>
                    <button class="cc-rot-btn" @click="rotate(-1)">右转 ↻</button>
                </div>
            </div>
        </div>

        <!-- 底部 bar -->
        <div class="cc-bottombar">
            <!-- 左侧：步进设置 + 地图选择 -->
            <div class="cc-bar-left">
                <div class="cc-stepper">
                    <label>移动距离</label>
                    <div class="cc-stepper-ctrl">
                        <button class="cc-step-btn" @click="adjustMoveStep(-10)">−</button>
                        <input type="number" class="cc-step-input" v-model.number="moveStep" min="1" step="1" />
                        <span class="cc-unit">mm</span>
                        <button class="cc-step-btn" @click="adjustMoveStep(10)">+</button>
                    </div>
                </div>
                <div class="cc-stepper">
                    <label>旋转步长</label>
                    <div class="cc-stepper-ctrl">
                        <button class="cc-step-btn" @click="adjustRotStep(-0.01)">−</button>
                        <input type="number" class="cc-step-input" v-model.number="rotStep" min="0.01" step="0.01" />
                        <span class="cc-unit">rad</span>
                        <button class="cc-step-btn" @click="adjustRotStep(0.01)">+</button>
                    </div>
                </div>
                <div class="cc-stepper">
                    <label>保存点位</label>
                    <div class="cc-stepper-ctrl">
                        <input type="text" class="cc-name-input" v-model="pointName" placeholder="名称" />
                        <button class="cc-btn cc-btn-save" @click="savePoint">保存</button>
                        <button class="cc-btn cc-btn-refresh" @click="loadPoints">刷新</button>
                    </div>
                </div>
                <div class="cc-stepper cc-map-stepper">
                    <label>地图选择</label>
                    <div class="cc-stepper-ctrl">
                        <select class="cc-select cc-map-select" v-model="selectedMapId" @change="switchMap">
                            <option value="">-- 加载中 --</option>
                            <option v-for="m in mapList" :key="m.id" :value="m.id">
                                {{ m.displayName }}{{ m.is_current ? ' (当前)' : '' }}
                            </option>
                        </select>
                    </div>
                </div>
            </div>

            <!-- 中间：底盘状态 -->
            <div class="cc-bar-center">
                <div class="cc-pose" v-if="chassis">
                    <span class="cc-pose-item">X: <b>{{ chassis.x.toFixed(3) }}</b></span>
                    <span class="cc-pose-item">Y: <b>{{ chassis.y.toFixed(3) }}</b></span>
                    <span class="cc-pose-item">Yaw: <b>{{ chassis.yaw.toFixed(3) }}</b> ({{ (chassis.yaw * 180 / Math.PI).toFixed(1) }}°)</span>
                </div>
                <div class="cc-pose" v-else>
                    <span class="cc-pose-item">等待底盘状态...</span>
                </div>
            </div>

            <!-- 右侧：点位到位 -->
            <div class="cc-bar-right">
                <div class="cc-goto-row">
                    <select class="cc-select" v-model="selectedPoint">
                        <option value="">-- 选择点位 --</option>
                        <option v-for="pt in mapPoints" :key="pt.name" :value="pt.name">
                            {{ pt.name }} ({{ pt.source }}) [{{ pt.position[0].toFixed(2) }}, {{ pt.position[1].toFixed(2) }}]
                        </option>
                    </select>
                    <button class="cc-btn cc-btn-go" @click="gotoPoint" :disabled="!selectedPoint">到位</button>
                </div>
            </div>
        </div>
    </div>
    `,
    data() {
        return {
            moveStep: 100,        // 毫米
            rotStep: 0.1,         // 弧度
            pointName: '',
            mapPoints: [],
            selectedPoint: '',
            mapList: [],
            selectedMapId: '',
            // 画布相关
            ctx: null,
            rafId: null,
            scale: 40,            // 1 米 = 40 像素
            viewX: 0,             // 画布中心对应的地图坐标 X
            viewY: 0,             // 画布中心对应的地图坐标 Y
            _onMapInfo: null,
        };
    },
    computed: {
        chassis() {
            const status = this.getRobotStatus();
            return status && status.chassis ? status.chassis : null;
        }
    },
    methods: {
        adjustMoveStep(delta) {
            this.moveStep = Math.max(1, this.moveStep + delta);
        },
        adjustRotStep(delta) {
            this.rotStep = Math.max(0.01, Math.round((this.rotStep + delta) * 100) / 100);
        },
        // X 是前后（正=前）
        moveX(dir) {
            const meters = (this.moveStep / 1000) * dir;
            mqttClient.publishCommand('go_rel', { x: meters, y: 0, yaw_rad: 0 });
            console.log('[底盘] X移动', meters, 'm');
        },
        // Y 是左右（正=左）
        moveY(dir) {
            const meters = (this.moveStep / 1000) * dir;
            mqttClient.publishCommand('go_rel', { x: 0, y: meters, yaw_rad: 0 });
            console.log('[底盘] Y移动', meters, 'm');
        },
        // 旋转：左转=正（逆时针），右转=负（顺时针）
        rotate(dir) {
            const rad = this.rotStep * dir;
            mqttClient.publishCommand('go_rel', { x: 0, y: 0, yaw_rad: rad });
            console.log('[底盘] 旋转', rad, 'rad');
        },
        savePoint() {
            if (!this.pointName.trim()) {
                alert('请输入点位名称');
                return;
            }
            mqttClient.publishMapControl('save_point', { name: this.pointName.trim() });
            console.log('[底盘] 保存点位', this.pointName);
            this.pointName = '';
            setTimeout(() => this.loadPoints(), 500);
        },
        loadPoints() {
            mqttClient.publishMapControl('read_points');
            console.log('[底盘] 请求地图点位');
        },
        switchMap() {
            if (!this.selectedMapId) return;
            mqttClient.publishMapControl('switch_map', { map_id: this.selectedMapId });
            console.log('[底盘] 切换地图', this.selectedMapId);
            setTimeout(() => this.loadPoints(), 1000);
        },
        gotoPoint() {
            if (!this.selectedPoint) return;
            const pt = this.mapPoints.find(p => p.name === this.selectedPoint);
            if (!pt) return;
            if (pt.source === 'map' && /^\\d+$/.test(pt.name)) {
                mqttClient.publishCommand('go', parseInt(pt.name));
            } else {
                mqttClient.publishCommand('go', pt.name);
            }
            console.log('[底盘] 到位', this.selectedPoint);
        },
        onMapPoints(data) {
            if (data && data.command === 'map_points' && Array.isArray(data.data)) {
                this.mapPoints = data.data;
                console.log('[底盘] 收到', this.mapPoints.length, '个点位');
                // 自动居中：计算所有点位的中心
                if (this.mapPoints.length > 0) {
                    let sx = 0, sy = 0;
                    for (const p of this.mapPoints) {
                        sx += p.position[0];
                        sy += p.position[1];
                    }
                    this.viewX = sx / this.mapPoints.length;
                    this.viewY = sy / this.mapPoints.length;
                }
            }
        },
        onMapInfo(data) {
            if (data && data.command === 'maps' && Array.isArray(data.data)) {
                this.mapList = data.data.map((m, i) => {
                    const n = m.name ? m.name.trim() : '';
                    return { ...m, displayName: n || `地图${i + 1}` };
                });
                const curr = this.mapList.find(m => m.is_current);
                if (curr) {
                    this.selectedMapId = curr.id;
                }
                console.log('[底盘] 收到', this.mapList.length, '个地图');
            }
        },

        // ── 画布渲染 ─────────────────────────────────
        resizeCanvas() {
            const c = this.$refs.canvas;
            if (!c) return;
            const rect = c.parentElement.getBoundingClientRect();
            c.width = rect.width;
            c.height = Math.max(400, rect.height);
            this.render();
        },
        // 地图坐标 → 画布像素
        worldToCanvas(wx, wy) {
            const c = this.$refs.canvas;
            const cx = c.width / 2 + (wx - this.viewX) * this.scale;
            // 画布 Y 向下，地图 Y 向上 → 翻转
            const cy = c.height / 2 - (wy - this.viewY) * this.scale;
            return [cx, cy];
        },
        render() {
            const c = this.$refs.canvas;
            if (!c || !this.ctx) return;
            const ctx = this.ctx;
            const W = c.width, H = c.height;

            // 清屏 - 白色背景
            ctx.fillStyle = '#f8fafc';
            ctx.fillRect(0, 0, W, H);

            // 画网格
            this.drawGrid(ctx, W, H);

            // 画坐标轴
            this.drawAxes(ctx, W, H);

            // 画地图点位
            for (const pt of this.mapPoints) {
                if (!pt.position || pt.position.length < 2) continue;
                const [px, py] = this.worldToCanvas(pt.position[0], pt.position[1]);
                this.drawPoint(ctx, px, py, pt.name, pt.source === 'map' ? '#409eff' : '#e6a23c');
            }

            // 画底盘位置
            if (this.chassis) {
                const [px, py] = this.worldToCanvas(this.chassis.x, this.chassis.y);
                this.drawChassis(ctx, px, py, this.chassis.yaw);
            }
        },
        drawGrid(ctx, W, H) {
            ctx.strokeStyle = '#e4ecf4';
            ctx.lineWidth = 1;
            const step = this.scale; // 1 米一格
            const [ox, _] = this.worldToCanvas(0, 0);
            // 竖线
            for (let x = ox % step; x < W; x += step) {
                ctx.beginPath();
                ctx.moveTo(x, 0);
                ctx.lineTo(x, H);
                ctx.stroke();
            }
            // 横线
            const [__, oy] = this.worldToCanvas(0, 0);
            for (let y = oy % step; y < H; y += step) {
                ctx.beginPath();
                ctx.moveTo(0, y);
                ctx.lineTo(W, y);
                ctx.stroke();
            }
        },
        drawAxes(ctx, W, H) {
            const [ox, oy] = this.worldToCanvas(0, 0);
            // X 轴（水平）
            ctx.strokeStyle = '#409eff';
            ctx.lineWidth = 1.5;
            ctx.beginPath();
            ctx.moveTo(0, oy);
            ctx.lineTo(W, oy);
            ctx.stroke();
            // Y 轴（垂直）
            ctx.beginPath();
            ctx.moveTo(ox, 0);
            ctx.lineTo(ox, H);
            ctx.stroke();
            // 标签
            ctx.fillStyle = '#909399';
            ctx.font = '12px monospace';
            ctx.fillText('X+', W - 24, oy - 6);
            ctx.fillText('Y+', ox + 6, 16);
        },
        drawPoint(ctx, px, py, label, color) {
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.arc(px, py, 5, 0, Math.PI * 2);
            ctx.fill();
            ctx.strokeStyle = '#fff';
            ctx.lineWidth = 1.5;
            ctx.stroke();
            // 标签
            ctx.fillStyle = '#606266';
            ctx.font = '11px monospace';
            ctx.fillText(label, px + 8, py - 8);
        },
        drawChassis(ctx, px, py, yaw) {
            // 画三角形表示朝向
            const size = 10;
            ctx.save();
            ctx.translate(px, py);
            // 画布 Y 翻转，所以角度取反
            ctx.rotate(-yaw);
            ctx.fillStyle = '#67c23a';
            ctx.strokeStyle = '#409eff';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(size, 0);          // 前端
            ctx.lineTo(-size * 0.7, size * 0.7);   // 左后
            ctx.lineTo(-size * 0.7, -size * 0.7);  // 右后
            ctx.closePath();
            ctx.fill();
            ctx.stroke();
            ctx.restore();

            // 标签
            ctx.fillStyle = '#67c23a';
            ctx.font = 'bold 12px monospace';
            ctx.fillText('底盘', px + 12, py + 4);
        },
        // 动画循环：持续重绘（底盘位置会变化）
        startRenderLoop() {
            const loop = () => {
                this.render();
                this.rafId = requestAnimationFrame(loop);
            };
            loop();
        }
    },
    mounted() {
        mqttClient.addMapPointsCallback(this.onMapPoints);
        mqttClient.addMapInfoCallback(this.onMapInfo);
        this.ctx = this.$refs.canvas.getContext('2d');
        this.resizeCanvas();
        window.addEventListener('resize', this.resizeCanvas);
        this.loadPoints();
        mqttClient.publishMapControl('read_maps');
        this.startRenderLoop();
    },
    beforeUnmount() {
        mqttClient.removeMapPointsCallback(this.onMapPoints);
        mqttClient.removeMapInfoCallback(this.onMapInfo);
        window.removeEventListener('resize', this.resizeCanvas);
        if (this.rafId) cancelAnimationFrame(this.rafId);
    }
};
