// data-joints-coords.js
// 关节/坐标数据管理页面：读取、到位、更新、删除

import { mqttClient } from './mqtt-client.js';

export default {
    name: 'DataJointsCoords',
    inject: ['getRobotStatus'],
    template: `
    <div class="panel djc-panel">
        <div class="djc-table-wrap" v-if="items.length > 0">
            <table class="djc-table">
                <thead>
                    <tr>
                        <th>分类</th>
                        <th>类型</th>
                        <th>名称</th>
                        <th>数据摘要</th>
                    </tr>
                </thead>
                <tbody>
                    <tr v-for="(item, idx) in items" :key="idx"
                        :class="{ 'djc-row-selected': selectedIdx === idx }"
                        @click="selectRow(idx)">
                        <td>{{ item.category === 'joints' ? '关节' : '坐标' }}</td>
                        <td>{{ item.type }}</td>
                        <td>{{ item.name }}</td>
                        <td class="djc-summary">{{ summarize(item) }}</td>
                    </tr>
                </tbody>
            </table>
        </div>

        <div v-else class="djc-empty">
            点击右下角「读取」加载数据
        </div>

        <!-- 底部固定操作栏 -->
        <div class="djc-actionbar">
            <div class="djc-left-info">
                <span v-if="items.length > 0" class="djc-count-bar">共 {{ items.length }} 条</span>
                <span class="djc-sel-info" v-if="selectedItem">
                    选中: {{ selectedItem.category === 'joints' ? '关节' : '坐标' }} ·
                    {{ selectedItem.type }} · {{ selectedItem.name }}
                </span>
            </div>
            <div class="djc-actions">
                <button class="djc-btn djc-btn-read" @click="readData">读取</button>
                <button class="djc-btn djc-btn-go" :disabled="!selectedItem" @click="confirmGoTo">到位</button>
                <button class="djc-btn djc-btn-update" :disabled="!selectedItem" @click="confirmUpdate">更新</button>
                <button class="djc-btn djc-btn-delete" :disabled="!selectedItem" @click="confirmDelete">删除</button>
            </div>
        </div>

        <!-- 确认弹窗 -->
        <div v-if="confirmDialog.visible" class="djc-overlay" @click.self="cancelConfirm">
            <div class="djc-dialog">
                <div class="djc-dialog-header">
                    <span :class="['djc-dialog-icon', confirmDialog.iconClass]">{{ confirmDialog.icon }}</span>
                    <span class="djc-dialog-title">{{ confirmDialog.title }}</span>
                </div>
                <div class="djc-dialog-body">
                    {{ confirmDialog.message }}
                </div>
                <div class="djc-dialog-footer">
                    <button class="djc-btn djc-btn-cancel" @click="cancelConfirm">取消</button>
                    <button :class="['djc-btn', confirmDialog.confirmClass]" @click="executeConfirm">确定</button>
                </div>
            </div>
        </div>
    </div>
    `,
    data() {
        return {
            items: [],
            selectedIdx: -1,
            confirmDialog: {
                visible: false,
                title: '',
                message: '',
                icon: '',
                iconClass: '',
                confirmClass: '',
                action: null,
            },
        };
    },
    computed: {
        selectedItem() {
            return this.selectedIdx >= 0 ? this.items[this.selectedIdx] : null;
        }
    },
    methods: {
        readData() {
            mqttClient.publishDataReq('read');
            console.log('[数据] 已请求读取');
        },
        selectRow(idx) {
            this.selectedIdx = this.selectedIdx === idx ? -1 : idx;
        },
        summarize(item) {
            const v = item.value;
            if (!v) return '-';
            if (typeof v === 'object') {
                const keys = Object.keys(v);
                return `${keys.length} 项: ${keys.slice(0, 3).join(', ')}${keys.length > 3 ? '...' : ''}`;
            }
            return String(v);
        },
        onDataResp(data) {
            if (data && data.command === 'response' && Array.isArray(data.data)) {
                this.items = data.data;
                this.selectedIdx = -1;
                console.log('[数据] 收到', this.items.length, '条数据');
            }
        },
        showConfirm(title, message, icon, iconClass, confirmClass, action) {
            this.confirmDialog = {
                visible: true,
                title,
                message,
                icon,
                iconClass,
                confirmClass,
                action,
            };
        },
        cancelConfirm() {
            this.confirmDialog.visible = false;
            this.confirmDialog.action = null;
        },
        executeConfirm() {
            if (this.confirmDialog.action) {
                this.confirmDialog.action();
            }
            this.cancelConfirm();
        },
        confirmGoTo() {
            const item = this.selectedItem;
            if (!item) return;
            const categoryName = item.category === 'joints' ? '关节' : '坐标';
            this.showConfirm(
                '确认到位',
                `确定要让机器人运动到 ${categoryName}「${item.type}/${item.name}」吗？请确保周围环境安全。`,
                '▶',
                'djc-icon-go',
                'djc-btn-go',
                () => this.doGoTo()
            );
        },
        doGoTo() {
            const item = this.selectedItem;
            if (!item) return;
            if (item.category === 'joints') {
                mqttClient.publishJointCommand(item.type, item.name);
                console.log('[到位] 关节', item.type, item.name);
            } else {
                console.log('[到位] 坐标运动尚未实现', item.type, item.name);
                alert('坐标运动服务端尚未实现');
            }
        },
        confirmUpdate() {
            const item = this.selectedItem;
            if (!item) return;
            const categoryName = item.category === 'joints' ? '关节' : '坐标';
            this.showConfirm(
                '确认更新',
                `确定要用当前机器人姿态更新 ${categoryName}「${item.type}/${item.name}」吗？原有数据将被覆盖。`,
                '↻',
                'djc-icon-update',
                'djc-btn-update',
                () => this.doUpdate()
            );
        },
        doUpdate() {
            const item = this.selectedItem;
            if (!item) return;
            const status = this.getRobotStatus();
            if (!status || !status.joints) {
                alert('未收到机器人状态数据');
                return;
            }
            const joints = status.joints;
            const typeJointKeys = {
                WBC:   null,
                arms:  ['idx21_arm_l_joint1','idx22_arm_l_joint2','idx23_arm_l_joint3','idx24_arm_l_joint4','idx25_arm_l_joint5','idx26_arm_l_joint6','idx27_arm_l_joint7',
                        'idx61_arm_r_joint1','idx62_arm_r_joint2','idx63_arm_r_joint3','idx64_arm_r_joint4','idx65_arm_r_joint5','idx66_arm_r_joint6','idx67_arm_r_joint7'],
                left:  ['idx21_arm_l_joint1','idx22_arm_l_joint2','idx23_arm_l_joint3','idx24_arm_l_joint4','idx25_arm_l_joint5','idx26_arm_l_joint6','idx27_arm_l_joint7'],
                right: ['idx61_arm_r_joint1','idx62_arm_r_joint2','idx63_arm_r_joint3','idx64_arm_r_joint4','idx65_arm_r_joint5','idx66_arm_r_joint6','idx67_arm_r_joint7'],
                head:  ['idx11_head_joint1','idx12_head_joint2','idx13_head_joint3'],
                waist: ['idx03_body_joint3','idx04_body_joint4','idx05_body_joint5','idx01_body_joint1','idx02_body_joint2'],
            };
            let dataToSend;
            if (item.category === 'joints') {
                const keys = typeJointKeys[item.type];
                if (keys === null) {
                    dataToSend = { ...joints };
                } else {
                    dataToSend = {};
                    keys.forEach(k => { if (joints[k] !== undefined) dataToSend[k] = joints[k]; });
                }
            } else {
                dataToSend = {};
            }
            mqttClient.publishDataReq('update', {
                category: item.category,
                type: item.type,
                name: item.name,
                data: dataToSend
            });
            item.value = dataToSend;
            console.log('[更新]', item.type, item.name, Object.keys(dataToSend).length, '项');
        },
        confirmDelete() {
            const item = this.selectedItem;
            if (!item) return;
            const categoryName = item.category === 'joints' ? '关节' : '坐标';
            this.showConfirm(
                '确认删除',
                `确定要删除 ${categoryName}「${item.type}/${item.name}」吗？此操作不可恢复。`,
                '✕',
                'djc-icon-delete',
                'djc-btn-delete',
                () => this.doDelete()
            );
        },
        doDelete() {
            const item = this.selectedItem;
            if (!item) return;
            mqttClient.publishDataReq('delete', {
                category: item.category,
                type: item.type,
                name: item.name
            });
            this.items.splice(this.selectedIdx, 1);
            this.selectedIdx = -1;
            console.log('[删除]', item.type, item.name);
        }
    },
    mounted() {
        mqttClient.addDataRespCallback(this.onDataResp);
        // 自动读取一次
        this.readData();
    },
    beforeUnmount() {
        mqttClient.removeDataRespCallback(this.onDataResp);
    }
};
