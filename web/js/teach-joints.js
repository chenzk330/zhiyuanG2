// teach-joints.js
// 示教（角）组件：通过 +/- 按钮调整各关节角度

import { mqttClient } from './mqtt-client.js';

export default {
    name: 'TeachJoints',
    inject: ['getUrdfViewer', 'getRobotStatus'],
    template: `
    <div class="panel tj-panel">
        <h5 class="tj-title">示教（角） · 关节角度</h5>

        <div class="tj-content">
        <div class="teach-grid">
            <!-- 头部 -->
            <div class="joint-group joint-group-head">
                <div class="group-title"><span class="group-icon">◉</span> 头部</div>
                <div class="joint-row" v-for="j in headJoints" :key="j.name">
                    <span class="j-label">{{ j.label }}</span>
                    <button class="j-btn minus" @mousedown="startStep(j,-1)" @mouseup="stopStep" @mouseleave="stopStep">−</button>
                    <span class="j-angle">{{ format(j.value) }}°</span>
                    <button class="j-btn plus"  @mousedown="startStep(j, 1)" @mouseup="stopStep" @mouseleave="stopStep">+</button>
                </div>
            </div>

            <!-- 腰部 -->
            <div class="joint-group joint-group-waist">
                <div class="group-title"><span class="group-icon">◆</span> 腰部</div>
                <div class="joint-row" v-for="j in waistJoints" :key="j.name">
                    <span class="j-label">{{ j.label }}</span>
                    <button class="j-btn minus" @mousedown="startStep(j,-1)" @mouseup="stopStep" @mouseleave="stopStep">−</button>
                    <span class="j-angle">{{ format(j.value) }}°</span>
                    <button class="j-btn plus"  @mousedown="startStep(j, 1)" @mouseup="stopStep" @mouseleave="stopStep">+</button>
                </div>
            </div>

            <!-- 腿部 -->
            <div class="joint-group joint-group-leg">
                <div class="group-title"><span class="group-icon">▼</span> 腿部</div>
                <div class="joint-row" v-for="j in legJoints" :key="j.name">
                    <span class="j-label">{{ j.label }}</span>
                    <button class="j-btn minus" @mousedown="startStep(j,-1)" @mouseup="stopStep" @mouseleave="stopStep">−</button>
                    <span class="j-angle">{{ format(j.value) }}°</span>
                    <button class="j-btn plus"  @mousedown="startStep(j, 1)" @mouseup="stopStep" @mouseleave="stopStep">+</button>
                </div>
            </div>

            <!-- 左臂 -->
            <div class="joint-group joint-group-arm">
                <div class="group-title group-title-left"><span class="group-icon">◀</span> 左臂</div>
                <div class="joint-row" v-for="j in leftArmJoints" :key="j.name">
                    <span class="j-label j-label-num">{{ j.label }}</span>
                    <button class="j-btn minus" @mousedown="startStep(j,-1)" @mouseup="stopStep" @mouseleave="stopStep">−</button>
                    <span class="j-angle">{{ format(j.value) }}°</span>
                    <button class="j-btn plus"  @mousedown="startStep(j, 1)" @mouseup="stopStep" @mouseleave="stopStep">+</button>
                </div>
            </div>

            <!-- 右臂 -->
            <div class="joint-group joint-group-arm">
                <div class="group-title group-title-right"><span class="group-icon">▶</span> 右臂</div>
                <div class="joint-row" v-for="j in rightArmJoints" :key="j.name">
                    <span class="j-label j-label-num">{{ j.label }}</span>
                    <button class="j-btn minus" @mousedown="startStep(j,-1)" @mouseup="stopStep" @mouseleave="stopStep">−</button>
                    <span class="j-angle">{{ format(j.value) }}°</span>
                    <button class="j-btn plus"  @mousedown="startStep(j, 1)" @mouseup="stopStep" @mouseleave="stopStep">+</button>
                </div>
            </div>
        </div>
        </div>

        <!-- 底部操作栏：按钮组 + 右侧功能描述 -->
        <div class="tj-actionbar">
            <div class="tj-actions">
                <button class="djc-btn djc-btn-read" @click="showSaveDialog = true">
                    <span style="margin-right:4px;">💾</span> 保存
                </button>
            </div>
            <div class="tj-info">
                <span class="tj-info-item">
                    <span class="tj-info-label">步长</span>
                    <input type="number" v-model.number="stepSize" step="0.5" min="0.1" class="tj-step-input">
                    <span class="tj-info-unit">°</span>
                </span>
                <span class="tj-status">
                    <span class="tj-conn-dot"></span>
                    <span class="tj-conn-text">实时连接</span>
                </span>
            </div>
        </div>

        <!-- 保存弹窗 -->
        <div v-if="showSaveDialog" class="save-overlay" @click.self="showSaveDialog = false">
            <div class="save-dialog">
                <h6 style="color:#409eff; margin-bottom:16px;">保存关节角</h6>

                <div class="form-row">
                    <label>名称</label>
                    <input type="text" v-model="saveName" placeholder="例如 hold"
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
            stepSize: 1.0,
            showSaveDialog: false,
            saveName: '',
            saveType: 'WBC',
            saveTypes: ['WBC', 'arms', 'left', 'right', 'head', 'waist'],
            _repeatTimer: null,
            _repeatInterval: null,
            headJoints: [
                { name: 'idx11_head_joint1', label: '头转', value: 0, urdfName: 'idx11_head_joint1' },
                { name: 'idx12_head_joint2', label: '头侧', value: 0, urdfName: 'idx12_head_joint2' },
                { name: 'idx13_head_joint3', label: '头仰', value: 0, urdfName: 'idx13_head_joint3' }
            ],
            leftArmJoints: [
                { name: 'idx21_arm_l_joint1', label: 'J1', value: 0, urdfName: 'idx21_arm_l_joint1' },
                { name: 'idx22_arm_l_joint2', label: 'J2', value: 0, urdfName: 'idx22_arm_l_joint2' },
                { name: 'idx23_arm_l_joint3', label: 'J3', value: 0, urdfName: 'idx23_arm_l_joint3' },
                { name: 'idx24_arm_l_joint4', label: 'J4', value: 0, urdfName: 'idx24_arm_l_joint4' },
                { name: 'idx25_arm_l_joint5', label: 'J5', value: 0, urdfName: 'idx25_arm_l_joint5' },
                { name: 'idx26_arm_l_joint6', label: 'J6', value: 0, urdfName: 'idx26_arm_l_joint6' },
                { name: 'idx27_arm_l_joint7', label: 'J7', value: 0, urdfName: 'idx27_arm_l_joint7' }
            ],
            rightArmJoints: [
                { name: 'idx61_arm_r_joint1', label: 'J1', value: 0, urdfName: 'idx61_arm_r_joint1' },
                { name: 'idx62_arm_r_joint2', label: 'J2', value: 0, urdfName: 'idx62_arm_r_joint2' },
                { name: 'idx63_arm_r_joint3', label: 'J3', value: 0, urdfName: 'idx63_arm_r_joint3' },
                { name: 'idx64_arm_r_joint4', label: 'J4', value: 0, urdfName: 'idx64_arm_r_joint4' },
                { name: 'idx65_arm_r_joint5', label: 'J5', value: 0, urdfName: 'idx65_arm_r_joint5' },
                { name: 'idx66_arm_r_joint6', label: 'J6', value: 0, urdfName: 'idx66_arm_r_joint6' },
                { name: 'idx67_arm_r_joint7', label: 'J7', value: 0, urdfName: 'idx67_arm_r_joint7' }
            ],
            waistJoints: [
                { name: 'idx03_body_joint3', label: '腰仰', value: 0, urdfName: 'idx03_body_joint3' },
                { name: 'idx04_body_joint4', label: '腰侧', value: 0, urdfName: 'idx04_body_joint4' },
                { name: 'idx05_body_joint5', label: '腰转', value: 0, urdfName: 'idx05_body_joint5' }
            ],
            legJoints: [
                { name: 'idx01_body_joint1', label: '腿1', value: 0, urdfName: 'idx01_body_joint1' },
                { name: 'idx02_body_joint2', label: '腿2', value: 0, urdfName: 'idx02_body_joint2' }
            ],
            _unwatch: null
        };
    },
    methods: {
        step(j, dir) {
            j.value += dir * this.stepSize;
            this.publishJoints();
        },
        startStep(j, dir) {
            this.step(j, dir);
            this._repeatTimer = setTimeout(() => {
                this._repeatInterval = setInterval(() => this.step(j, dir), 80);
            }, 400);
        },
        stopStep() {
            if (this._repeatTimer) { clearTimeout(this._repeatTimer); this._repeatTimer = null; }
            if (this._repeatInterval) { clearInterval(this._repeatInterval); this._repeatInterval = null; }
        },
        format(v) {
            return v.toFixed(1);
        },
        publishJoints() {
            const joints = {};
            const allGroups = [this.headJoints, this.leftArmJoints, this.rightArmJoints, this.waistJoints, this.legJoints];
            allGroups.forEach(group => {
                group.forEach(joint => {
                    joints[joint.name] = joint.value * Math.PI / 180;
                });
            });
            mqttClient.publishJointCommand('WBC', joints);
        },
        syncFromStatus(status) {
            if (!status || !status.joints) return;
            const j = status.joints;
            const allGroups = [this.headJoints, this.leftArmJoints, this.rightArmJoints, this.waistJoints, this.legJoints];
            allGroups.forEach(group => {
                group.forEach(joint => {
                    if (j[joint.name] !== undefined) {
                        joint.value = j[joint.name] * 180 / Math.PI;
                    }
                });
            });
        },
        doSave() {
            if (!this.saveName) return;
            const typeGroups = {
                WBC:   [this.headJoints, this.leftArmJoints, this.rightArmJoints, this.waistJoints, this.legJoints],
                arms:  [this.leftArmJoints, this.rightArmJoints],
                left:  [this.leftArmJoints],
                right: [this.rightArmJoints],
                head:  [this.headJoints],
                waist: [this.waistJoints, this.legJoints],
            };
            const groups = typeGroups[this.saveType] || typeGroups.WBC;
            const joints = {};
            groups.forEach(group => {
                group.forEach(joint => {
                    joints[joint.name] = joint.value * Math.PI / 180;
                });
            });
            mqttClient.publishDataSave({
                command: 'save_joints',
                type: this.saveType,
                name: this.saveName,
                data: joints
            });
            console.log(`[保存] type=${this.saveType}, name=${this.saveName}, joints=${Object.keys(joints).length}`);
            this.showSaveDialog = false;
            this.saveName = '';
        }
    },
    mounted() {
        this._unwatch = this.$watch(
            () => this.getRobotStatus(),
            (newStatus) => { this.syncFromStatus(newStatus); },
            { immediate: true }
        );
    },
    beforeUnmount() {
        if (this._unwatch) this._unwatch();
        this.stopStep();
    }
};
