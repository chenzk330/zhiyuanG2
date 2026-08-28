// teach-coords.js
// 示教（坐标）组件：左右手末端坐标 XYZ + RX RY RZ
// 左上角：左手/右手坐标系下拉菜单
// 右下角：保存末端位姿按钮

import { mqttClient } from './mqtt-client.js';

export default {
    name: 'TeachCoords',
    inject: ['getRobotStatus'],
    template: `
    <div class="panel tc-panel">
        <h5 class="tc-title">示教（坐标） · 末端位姿</h5>

        <div class="tc-content">
        <!-- 坐标系选择 -->
        <div class="tc-coord-sys-bar">
            <div class="coord-sys-row">
                <label class="coord-sys-label">左手坐标系</label>
                <select v-model="leftFrame" class="coord-sys-select">
                    <option value="base">base</option>
                    <option value="ltool0">ltool0</option>
                    <option value="lobj0">lobj0</option>
                </select>
            </div>
            <div class="coord-sys-row">
                <label class="coord-sys-label">右手坐标系</label>
                <select v-model="rightFrame" class="coord-sys-select">
                    <option value="base">base</option>
                    <option value="rtool0">rtool0</option>
                    <option value="robj0">robj0</option>
                </select>
            </div>
        </div>

        <div class="tc-hands-wrap">
            <!-- 左手 -->
            <div class="tc-hand-group tc-hand-left">
                <h6 class="tc-hand-title">左手</h6>
                <div class="coord-row" v-for="ax in axes" :key="'L_'+ax">
                    <span class="axis-label">{{ ax.toUpperCase() }}</span>
                    <button class="minus" @click="step('left', ax, -stepSize)">−</button>
                    <span class="val">{{ format(left[ax]) }}</span>
                    <button class="plus"  @click="step('left', ax,  stepSize)">+</button>
                </div>
                <!-- 左手夹爪 -->
                <div class="gripper-row">
                    <span class="gripper-label">夹爪</span>
                    <button class="gripper-btn open" @click="gripper('left', 'open')">张开</button>
                    <button class="gripper-btn close" @click="gripper('left', 'close')">闭合</button>
                </div>
            </div>

            <!-- 右手 -->
            <div class="tc-hand-group tc-hand-right">
                <h6 class="tc-hand-title">右手</h6>
                <div class="coord-row" v-for="ax in axes" :key="'R_'+ax">
                    <span class="axis-label">{{ ax.toUpperCase() }}</span>
                    <button class="minus" @click="step('right', ax, -stepSize)">−</button>
                    <span class="val">{{ format(right[ax]) }}</span>
                    <button class="plus"  @click="step('right', ax,  stepSize)">+</button>
                </div>
                <!-- 右手夹爪 -->
                <div class="gripper-row">
                    <span class="gripper-label">夹爪</span>
                    <button class="gripper-btn open" @click="gripper('right', 'open')">张开</button>
                    <button class="gripper-btn close" @click="gripper('right', 'close')">闭合</button>
                </div>
            </div>
        </div>
        </div>

        <!-- 底部操作栏：按钮组 + 右侧功能描述 -->
        <div class="tc-actionbar">
            <div class="tc-actions">
                <button class="djc-btn djc-btn-read" @click="showSaveDialog = true">
                    <span style="margin-right:4px;">💾</span> 保存
                </button>
            </div>
            <div class="tc-info">
                <span class="tc-info-item">
                    <span class="tc-info-label">步长</span>
                    <input type="number" v-model.number="stepSize" step="0.01" min="0.001" class="tc-step-input">
                </span>
                <span class="tc-info-desc">(XYZ:米, RX/RY/RZ:弧度)</span>
                <span class="tc-status">
                    <span class="tc-conn-dot"></span>
                    <span class="tc-conn-text">已连接实时状态</span>
                </span>
            </div>
        </div>

        <!-- 保存弹窗 -->
        <div v-if="showSaveDialog" class="save-overlay" @click.self="showSaveDialog = false">
            <div class="save-dialog">
                <h6 style="color:#409eff; margin-bottom:16px;">保存末端位姿</h6>

                <div class="form-row">
                    <label>名称</label>
                    <input type="text" v-model="saveName" placeholder="例如 pick"
                           @keyup.enter="doSave" />
                </div>

                <div style="margin-bottom:16px;">
                    <label style="display:block; color:#606266; font-size:14px; margin-bottom:8px;">类型</label>
                    <div class="radio-group">
                        <label v-for="t in saveTypes" :key="t">
                            <input type="radio" :value="t" v-model="saveType" /> {{ t }}
                        </label>
                    </div>
                </div>

                <div class="step-actions">
                    <button class="nav-btn" @click="showSaveDialog = false">取消</button>
                    <button class="nav-btn start-btn" @click="doSave" :disabled="!saveName">保存</button>
                </div>
            </div>
        </div>
    </div>
    `,
    data() {
        return {
            stepSize: 0.02,
            axes: ['x', 'y', 'z', 'rx', 'ry', 'rz'],
            left:  { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 },
            right: { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 },
            // 坐标系
            leftFrame: 'base',
            rightFrame: 'base',
            // 保存弹窗
            showSaveDialog: false,
            saveName: '',
            saveType: 'both',
            saveTypes: ['left', 'right', 'both'],
            _unwatch: null
        };
    },
    methods: {
        step(side, axis, delta) {
            this[side][axis] += delta;
            // 发布 offset_move 命令
            this.publishOffset(side, axis, delta);
        },
        format(v) {
            return v.toFixed(3);
        },
        // 发布末端相对移动命令
        publishOffset(side, axis, delta) {
            const isTranslation = ['x', 'y', 'z'].includes(axis);
            const value_mm = isTranslation ? delta * 1000 : delta * 1000;
            const key = side === 'left' ? 'l' + axis : 'r' + axis;
            const data = { [key]: value_mm };
            mqttClient.publishCommand('offset_move', data);
        },
        // 控制夹爪开合
        gripper(side, action) {
            // open: -0.7, close: -0.0
            const pos = action === 'open' ? -0.7 : -0.0;
            const data = side === 'left' ? { left: pos } : { right: pos };
            mqttClient.publishCommand('grab', data);
            console.log(`[夹爪] ${side} ${action} (pos=${pos})`);
        },
        // 从 MQTT 状态刷新末端坐标
        syncFromStatus(status) {
            if (!status) return;
            if (status.left_ee) {
                const p = status.left_ee.position || [];
                const o = status.left_ee.orientation || [];
                this.left.x  = p[0] || 0;
                this.left.y  = p[1] || 0;
                this.left.z  = p[2] || 0;
                this.left.rx = o[0] || 0;
                this.left.ry = o[1] || 0;
                this.left.rz = o[2] || 0;
            }
            if (status.right_ee) {
                const p = status.right_ee.position || [];
                const o = status.right_ee.orientation || [];
                this.right.x  = p[0] || 0;
                this.right.y  = p[1] || 0;
                this.right.z  = p[2] || 0;
                this.right.rx = o[0] || 0;
                this.right.ry = o[1] || 0;
                this.right.rz = o[2] || 0;
            }
        },
        // 收集当前末端位姿
        collectData() {
            if (this.saveType === 'left') {
                return { ...this.left };
            } else if (this.saveType === 'right') {
                return { ...this.right };
            } else {
                // both
                return {
                    left:  { ...this.left },
                    right: { ...this.right }
                };
            }
        },
        // 保存末端位姿到服务端
        doSave() {
            if (!this.saveName) return;
            const data = this.collectData();
            mqttClient.publishDataSave({
                command: 'save_position',
                type: this.saveType,
                name: this.saveName,
                data: data
            });
            console.log(`[保存位姿] type=${this.saveType}, name=${this.saveName}`);
            this.showSaveDialog = false;
            this.saveName = '';
        }
    },
    mounted() {
        this._unwatch = this.$watch(
            () => this.getRobotStatus(),
            (newStatus) => {
                this.syncFromStatus(newStatus);
            },
            { immediate: true }
        );
    },
    beforeUnmount() {
        if (this._unwatch) this._unwatch();
    }
};
