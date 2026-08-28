// modbus-view.js
// Modbus 数据视图：显示 holding registers 的读取/写入数据

import { mqttClient } from './mqtt-client.js';

export default {
    name: 'ModbusView',
    template: `
    <div class="panel mv-panel">
        <!-- 设备选择栏 -->
        <div class="mv-toolbar">
            <div class="mv-device-select">
                <label>设备</label>
                <select class="cc-select" v-model="selectedIp" @change="onDeviceChange">
                    <option value="">-- 无设备 --</option>
                    <option v-for="d in deviceList" :key="d.ip" :value="d.ip">
                        {{ d.ip }}:{{ d.port }}
                    </option>
                </select>
            </div>
            <button class="mv-add-device-btn" @click="openAddDeviceDialog">+ 增加设备</button>
            <div class="mv-status">
                <span :class="['mv-dot', connected ? 'mv-dot-on' : 'mv-dot-off']"></span>
                {{ connected ? '已连接' : '未连接' }}
            </div>
        </div>

        <!-- 数据表格区 -->
        <div class="mv-tables" v-if="currentDevice">
            <!-- Read 区 -->
            <div class="mv-table-block">
                <div class="mv-block-header mv-read-header">
                    <span class="mv-block-icon">📖</span> 读取区 (Read Holdings)
                    <div class="mv-block-actions">
                        <button class="mv-small-btn mv-small-btn-del"
                                :disabled="selectedReadAddr === null"
                                @click="deleteReadAddr">删除</button>
                        <button class="mv-small-btn mv-small-btn-add" @click="openAddRangeDialog('read')">增加</button>
                    </div>
                </div>
                <table class="mv-table">
                    <thead>
                        <tr>
                            <th class="mv-col-addr">地址</th>
                            <th class="mv-col-val">当前值</th>
                            <th class="mv-col-hex">十六进制</th>
                        </tr>
                    </thead>
                    <tbody>
                        <tr v-for="item in currentDevice.read" :key="'r' + item.address"
                            :class="{ 'mv-row-selected': selectedReadAddr === item.address }"
                            @click="selectReadRow(item.address)">
                            <td class="mv-addr">{{ item.address }}</td>
                            <td class="mv-val" :class="{ 'mv-null': item.value === null }">
                                {{ item.value === null ? '--' : item.value }}
                            </td>
                            <td class="mv-hex">
                                {{ item.value === null ? '--' : '0x' + item.value.toString(16).toUpperCase().padStart(4, '0') }}
                            </td>
                        </tr>
                        <tr v-if="currentDevice.read.length === 0">
                            <td colspan="3" class="mv-empty">暂无读取配置，点击"增加"添加地址</td>
                        </tr>
                    </tbody>
                </table>
            </div>

            <!-- Write 区 -->
            <div class="mv-table-block">
                <div class="mv-block-header mv-write-header">
                    <span class="mv-block-icon">✏️</span> 写入区 (Write Holdings)
                    <div class="mv-block-actions">
                        <button class="mv-small-btn mv-small-btn-edit"
                                :disabled="selectedWriteAddr === null"
                                @click="openEditDialog">修改</button>
                        <button class="mv-small-btn mv-small-btn-del"
                                :disabled="selectedWriteAddr === null"
                                @click="deleteWriteAddr">删除</button>
                        <button class="mv-small-btn mv-small-btn-add" @click="openAddRangeDialog('write')">增加</button>
                    </div>
                </div>
                <table class="mv-table">
                    <thead>
                        <tr>
                            <th class="mv-col-addr">地址</th>
                            <th class="mv-col-val">当前值</th>
                            <th class="mv-col-hex">十六进制</th>
                        </tr>
                    </thead>
                    <tbody>
                        <tr v-for="item in currentDevice.write"
                            :key="'w' + item.address"
                            :class="{ 'mv-row-selected': selectedWriteAddr === item.address }"
                            @click="selectWriteRow(item.address)">
                            <td class="mv-addr">{{ item.address }}</td>
                            <td class="mv-val" :class="{ 'mv-null': item.value === null }">
                                {{ item.value === null ? '--' : item.value }}
                            </td>
                            <td class="mv-hex">
                                {{ item.value === null ? '--' : '0x' + item.value.toString(16).toUpperCase().padStart(4, '0') }}
                            </td>
                        </tr>
                        <tr v-if="currentDevice.write.length === 0">
                            <td colspan="3" class="mv-empty">暂无写入配置，点击"增加"添加地址</td>
                        </tr>
                    </tbody>
                </table>
            </div>
        </div>

        <div v-else class="mv-no-device">请先添加或选择一个设备</div>

        <!-- 删除确认弹窗 -->
        <div v-if="deleteDialog.visible" class="save-overlay" @click.self="closeDeleteDialog">
            <div class="save-dialog" style="width:360px;">
                <h6 style="color:#f56c6c; margin-bottom:16px;">🗑️ 确认删除</h6>
                <div class="mv-edit-form">
                    <div class="mv-form-row">
                        <label>设备</label>
                        <span class="mv-form-val">{{ deleteDialog.ip }}</span>
                    </div>
                    <div class="mv-form-row">
                        <label>区域</label>
                        <span class="mv-form-val">{{ deleteDialog.type === 'read' ? '读取区' : '写入区' }}</span>
                    </div>
                    <div class="mv-form-row">
                        <label>地址</label>
                        <span class="mv-form-val">{{ deleteDialog.address }}</span>
                    </div>
                </div>
                <div class="step-actions">
                    <button class="nav-btn" @click="closeDeleteDialog">取消</button>
                    <button class="nav-btn" style="background:#f56c6c; color:#fff;" @click="confirmDelete">删除</button>
                </div>
            </div>
        </div>

        <!-- 修改值弹窗 -->
        <div v-if="editDialog.visible" class="save-overlay" @click.self="closeEditDialog">
            <div class="save-dialog" style="width:380px;">
                <h6 style="color:#e6a23c; margin-bottom:16px;">✏️ 修改 Holding Register</h6>
                <div class="mv-edit-form">
                    <div class="mv-form-row">
                        <label>设备</label>
                        <span class="mv-form-val">{{ editDialog.ip }}</span>
                    </div>
                    <div class="mv-form-row">
                        <label>地址</label>
                        <span class="mv-form-val">{{ editDialog.address }}</span>
                    </div>
                    <div class="mv-form-row">
                        <label>当前值</label>
                        <span class="mv-form-val">{{ editDialog.oldValue === null ? '--' : editDialog.oldValue }}</span>
                    </div>
                    <div class="mv-form-row">
                        <label>新值</label>
                        <input class="mv-input" type="number"
                               v-model="editDialog.newValue"
                               placeholder="输入 0-65535 的整数"
                               @keyup.enter="submitEdit" />
                    </div>
                </div>
                <div class="step-actions">
                    <button class="nav-btn" @click="closeEditDialog">取消</button>
                    <button class="nav-btn" style="background:#e6a23c; color:#fff;" @click="submitEdit">写入</button>
                </div>
            </div>
        </div>

        <!-- 增加设备弹窗 -->
        <div v-if="addDeviceDialog.visible" class="save-overlay" @click.self="closeAddDeviceDialog">
            <div class="save-dialog" style="width:380px;">
                <h6 style="color:#409eff; margin-bottom:16px;">➕ 增加 Modbus 设备</h6>
                <div class="mv-edit-form">
                    <div class="mv-form-row">
                        <label>IP 地址</label>
                        <input class="mv-input" type="text"
                               v-model="addDeviceDialog.ip"
                               placeholder="例如 10.20.15.120"
                               @keyup.enter="submitAddDevice"
                               ref="addDeviceIpInput" />
                    </div>
                    <div class="mv-form-row">
                        <label>端口</label>
                        <input class="mv-input" type="number"
                               v-model="addDeviceDialog.port"
                               placeholder="例如 10502"
                               @keyup.enter="submitAddDevice" />
                    </div>
                    <div v-if="addDeviceDialog.error" class="mv-form-error">{{ addDeviceDialog.error }}</div>
                </div>
                <div class="step-actions">
                    <button class="nav-btn" @click="closeAddDeviceDialog">取消</button>
                    <button class="nav-btn" style="background:#409eff; color:#fff;" @click="submitAddDevice">确定</button>
                </div>
            </div>
        </div>

        <!-- 增加地址范围弹窗 -->
        <div v-if="addRangeDialog.visible" class="save-overlay" @click.self="closeAddRangeDialog">
            <div class="save-dialog" style="width:380px;">
                <h6 :style="{color: addRangeDialog.type === 'read' ? '#409eff' : '#e6a23c', marginBottom: '16px'}">
                    ➕ 增加{{ addRangeDialog.type === 'read' ? '读取' : '写入' }}地址
                </h6>
                <div class="mv-edit-form">
                    <div class="mv-form-row">
                        <label>设备</label>
                        <span class="mv-form-val">{{ addRangeDialog.ip }}</span>
                    </div>
                    <div class="mv-form-row">
                        <label>开始地址</label>
                        <input class="mv-input" type="number"
                               v-model="addRangeDialog.start"
                               placeholder="例如 0" min="0"
                               @keyup.enter="submitAddRange"
                               ref="addRangeStartInput" />
                    </div>
                    <div class="mv-form-row">
                        <label>结束地址</label>
                        <input class="mv-input" type="number"
                               v-model="addRangeDialog.end"
                               placeholder="例如 5" min="0"
                               @keyup.enter="submitAddRange" />
                    </div>
                    <div v-if="addRangeDialog.error" class="mv-form-error">{{ addRangeDialog.error }}</div>
                </div>
                <div class="step-actions">
                    <button class="nav-btn" @click="closeAddRangeDialog">取消</button>
                    <button class="nav-btn"
                            :style="{background: addRangeDialog.type === 'read' ? '#409eff' : '#e6a23c', color: '#fff'}"
                            @click="submitAddRange">确定</button>
                </div>
            </div>
        </div>
    </div>
    `,
    data() {
        return {
            devices: {},
            selectedIp: '',
            selectedReadAddr: null,
            selectedWriteAddr: null,
            connected: false,
            editDialog: {
                visible: false,
                ip: '',
                address: null,
                oldValue: null,
                newValue: ''
            },
            addDeviceDialog: {
                visible: false,
                ip: '',
                port: '',
                error: ''
            },
            addRangeDialog: {
                visible: false,
                type: 'read',
                ip: '',
                start: '',
                end: '',
                error: ''
            },
            deleteDialog: {
                visible: false,
                type: 'read',
                ip: '',
                address: null
            }
        };
    },
    computed: {
        deviceList() {
            return Object.values(this.devices);
        },
        currentDevice() {
            return this.devices[this.selectedIp] || null;
        }
    },
    methods: {
        onModbusData(data) {
            if (!data || data.command !== 'modbus_data' || !Array.isArray(data.devices)) return;
            this.connected = true;
            const prevSelected = this.selectedIp;
            const prevSelectedExists = prevSelected && this.devices[prevSelected];
            for (const dev of data.devices) {
                const ip = dev.ip;
                if (!this.devices[ip]) {
                    this.devices[ip] = { ip, port: dev.port, read: [], write: [] };
                }
                this.devices[ip].port = dev.port;
                if (Array.isArray(dev.read)) {
                    this.devices[ip].read = dev.read;
                }
                if (Array.isArray(dev.write)) {
                    this.devices[ip].write = dev.write;
                }
            }
            if (!this.selectedIp && this.deviceList.length > 0) {
                this.selectedIp = this.deviceList[0].ip;
            }
        },
        onDeviceChange() {
            this.selectedReadAddr = null;
            this.selectedWriteAddr = null;
        },
        selectReadRow(address) {
            this.selectedReadAddr = this.selectedReadAddr === address ? null : address;
        },
        selectWriteRow(address) {
            this.selectedWriteAddr = this.selectedWriteAddr === address ? null : address;
        },

        // ── 修改值 ──
        openEditDialog() {
            if (this.selectedWriteAddr === null || !this.currentDevice) return;
            const item = this.currentDevice.write.find(w => w.address === this.selectedWriteAddr);
            if (!item) return;
            this.editDialog = {
                visible: true,
                ip: this.currentDevice.ip,
                address: item.address,
                oldValue: item.value,
                newValue: item.value === null ? '' : String(item.value)
            };
        },
        closeEditDialog() {
            this.editDialog.visible = false;
        },
        submitEdit() {
            const val = parseInt(this.editDialog.newValue, 10);
            if (isNaN(val) || val < 0 || val > 65535) {
                alert('请输入 0-65535 之间的整数');
                return;
            }
            mqttClient.publishModbusControl('write', {
                ip: this.editDialog.ip,
                address: this.editDialog.address,
                value: val
            });
            this.editDialog.visible = false;
        },

        // ── 增加设备 ──
        openAddDeviceDialog() {
            this.addDeviceDialog = { visible: true, ip: '', port: '10502', error: '' };
            this.$nextTick(() => {
                if (this.$refs.addDeviceIpInput) this.$refs.addDeviceIpInput.focus();
            });
        },
        closeAddDeviceDialog() {
            this.addDeviceDialog.visible = false;
        },
        submitAddDevice() {
            const ip = (this.addDeviceDialog.ip || '').trim();
            const port = parseInt(this.addDeviceDialog.port, 10);
            if (!ip) {
                this.addDeviceDialog.error = '请输入 IP 地址';
                return;
            }
            if (isNaN(port) || port < 1 || port > 65535) {
                this.addDeviceDialog.error = '请输入有效端口 (1-65535)';
                return;
            }
            for (const d of this.deviceList) {
                if (d.ip === ip && d.port === port) {
                    this.addDeviceDialog.error = '该设备已存在';
                    return;
                }
            }
            mqttClient.publishModbusControl('add_device', { ip, port });
            this.addDeviceDialog.visible = false;
            this.selectedIp = ip;
        },

        // ── 增加地址范围 ──
        openAddRangeDialog(type) {
            if (!this.currentDevice) return;
            this.addRangeDialog = {
                visible: true,
                type,
                ip: this.currentDevice.ip,
                start: '',
                end: '',
                error: ''
            };
            this.$nextTick(() => {
                if (this.$refs.addRangeStartInput) this.$refs.addRangeStartInput.focus();
            });
        },
        closeAddRangeDialog() {
            this.addRangeDialog.visible = false;
        },
        submitAddRange() {
            const start = parseInt(this.addRangeDialog.start, 10);
            const end = parseInt(this.addRangeDialog.end, 10);
            if (isNaN(start) || start < 0) {
                this.addRangeDialog.error = '请输入有效的开始地址 (≥0)';
                return;
            }
            if (isNaN(end) || end < start) {
                this.addRangeDialog.error = '结束地址必须 ≥ 开始地址';
                return;
            }
            if (end - start > 255) {
                this.addRangeDialog.error = '单次最多添加 256 个地址';
                return;
            }
            const cmd = this.addRangeDialog.type === 'read' ? 'add_read_addrs' : 'add_write_addrs';
            mqttClient.publishModbusControl(cmd, {
                ip: this.addRangeDialog.ip,
                start,
                end
            });
            this.addRangeDialog.visible = false;
        },

        // ── 删除地址 ──
        deleteReadAddr() {
            if (this.selectedReadAddr === null || !this.currentDevice) return;
            this.deleteDialog = {
                visible: true,
                type: 'read',
                ip: this.currentDevice.ip,
                address: this.selectedReadAddr
            };
        },
        deleteWriteAddr() {
            if (this.selectedWriteAddr === null || !this.currentDevice) return;
            this.deleteDialog = {
                visible: true,
                type: 'write',
                ip: this.currentDevice.ip,
                address: this.selectedWriteAddr
            };
        },
        closeDeleteDialog() {
            this.deleteDialog.visible = false;
        },
        confirmDelete() {
            const cmd = this.deleteDialog.type === 'read' ? 'del_read_addr' : 'del_write_addr';
            mqttClient.publishModbusControl(cmd, {
                ip: this.deleteDialog.ip,
                address: this.deleteDialog.address
            });
            if (this.deleteDialog.type === 'read') {
                this.selectedReadAddr = null;
            } else {
                this.selectedWriteAddr = null;
            }
            this.deleteDialog.visible = false;
        }
    },
    mounted() {
        mqttClient.addModbusDataCallback(this.onModbusData);
        mqttClient.publishModbusControl('read');
    },
    beforeUnmount() {
        mqttClient.removeModbusDataCallback(this.onModbusData);
    }
};
