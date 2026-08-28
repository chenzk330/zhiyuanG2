// data-collection.js
// 数据采集：多步骤向导
// 步骤1: 登陆 → 步骤2: 选择数据集 → 步骤3: 选择格式 → 步骤4: 采集页面 → 步骤5: 采集中

import { mqttClient } from './mqtt-client.js';

export default {
    name: 'DataCollection',
    template: `
    <div class="panel">
        <h5>数据采集</h5>

        <!-- 步骤指示器 -->
        <div class="step-indicator">
            <div v-for="(s, i) in stepNames" :key="i"
                 class="step-dot" :class="{ active: step === i+1, done: step > i+1 }">
                <span class="step-num">{{ i+1 }}</span>
                <span class="step-label">{{ s }}</span>
            </div>
        </div>

        <!-- 步骤1: 登陆 -->
        <div v-if="step === 1" class="step-content">
            <h6>登陆</h6>
            <div class="form-row">
                <label>账号</label>
                <input type="text" v-model="login.account" placeholder="请输入账号" />
            </div>
            <div class="form-row">
                <label>密码</label>
                <input type="password" v-model="login.password" placeholder="请输入密码" />
            </div>
            <div class="form-row">
                <label>云服务端</label>
                <input type="text" v-model="login.server" placeholder="https://cloud.example.com" />
            </div>
            <div class="step-actions">
                <button class="nav-btn" @click="doLogin">登陆</button>
            </div>
        </div>

        <!-- 步骤2: 选择数据集 -->
        <div v-if="step === 2" class="step-content">
            <h6>选择数据集</h6>
            <table class="dc-table">
                <thead>
                    <tr>
                        <th>任务名称</th>
                        <th>执行进度</th>
                        <th>下发时间</th>
                        <th>关联GPU</th>
                        <th>关联存储</th>
                        <th>状态</th>
                    </tr>
                </thead>
                <tbody>
                    <tr v-for="(task, i) in tasks" :key="i"
                        @click="selectedTask = i"
                        :class="{ selected: selectedTask === i }">
                        <td>{{ task.name }}</td>
                        <td>{{ task.local }}/{{ task.remote }}/{{ task.total }}</td>
                        <td>{{ task.time }}</td>
                        <td>{{ task.gpu }}</td>
                        <td>{{ task.storage }}</td>
                        <td>{{ task.status }}</td>
                    </tr>
                </tbody>
            </table>
            <div class="step-actions">
                <button class="nav-btn" @click="step = 1">上一步</button>
                <button class="nav-btn" @click="actionUpload" :disabled="selectedTask < 0">上传</button>
                <button class="nav-btn" @click="actionCollect" :disabled="selectedTask < 0">采集</button>
            </div>
        </div>

        <!-- 步骤3: 选择数据格式 -->
        <div v-if="step === 3" class="step-content">
            <h6>配置数据格式</h6>

            <div class="config-group">
                <label class="config-title">数据格式（多选）</label>
                <div class="checkbox-group">
                    <label v-for="fmt in formats" :key="fmt">
                        <input type="checkbox" :value="fmt" v-model="selectedFormats" /> {{ fmt }}
                    </label>
                </div>
            </div>

            <div class="config-group">
                <label class="config-title">关节（单选）</label>
                <div class="radio-group">
                    <label v-for="j in jointOptions" :key="j">
                        <input type="radio" :value="j" v-model="selectedJoint" /> {{ j }}
                    </label>
                </div>
            </div>

            <div class="config-group">
                <label class="config-title">相机（多选）</label>
                <div class="checkbox-group">
                    <label v-for="c in cameraOptions" :key="c">
                        <input type="checkbox" :value="c" v-model="selectedCameras" /> {{ c }}
                    </label>
                </div>
            </div>

            <div class="config-group">
                <label class="config-title">关联外围设备（多选）</label>
                <div class="checkbox-group">
                    <label v-for="d in deviceOptions" :key="d">
                        <input type="checkbox" :value="d" v-model="selectedDevices" /> {{ d }}
                    </label>
                </div>
            </div>

            <div class="step-actions">
                <button class="nav-btn" @click="step = 2">上一步</button>
                <button class="nav-btn" @click="enterCollectPage">进入采集</button>
            </div>
        </div>

        <!-- 步骤4: 数据采集页面 -->
        <div v-if="step === 4" class="step-content">
            <h6>数据采集</h6>
            <div class="collect-cameras">
                <div class="collect-cam" v-for="cam in collectCameras" :key="cam.key">
                    <div class="camera-title">{{ cam.name }}</div>
                    <img v-if="collectImages[cam.key]" :src="'data:image/jpeg;base64,' + collectImages[cam.key]" class="camera-img" />
                    <div v-else class="camera-placeholder">等待画面...</div>
                </div>
            </div>
            <div class="step-actions">
                <button class="nav-btn" @click="step = 3">上一步</button>
                <button class="nav-btn start-btn" @click="startCollect">开始</button>
            </div>
        </div>

        <!-- 步骤5: 采集中 -->
        <div v-if="step === 5" class="step-content">
            <h6>采集中...</h6>
            <div class="countdown-display">
                <div class="countdown-num">{{ countdown }}</div>
                <div class="countdown-text">正在采集数据</div>
            </div>
            <div class="step-actions">
                <button class="nav-btn" @click="finishCollect">完成</button>
            </div>
        </div>
    </div>
    `,
    data() {
        return {
            step: 1,
            stepNames: ['登陆', '选择数据集', '数据格式', '采集页面', '采集中'],
            // 步骤1
            login: { account: '', password: '', server: '' },
            // 步骤2
            tasks: [
                { name: '抓取放置任务A', local: 30, remote: 50, total: 100, time: '2026-07-14 10:00', gpu: 'GPU-01', storage: 'SSD-500G', status: '进行中' },
                { name: '拉车任务B',     local: 10, remote: 20, total: 50,  time: '2026-07-14 11:00', gpu: 'GPU-02', storage: 'SSD-1T',   status: '进行中' },
                { name: '装配任务C',     local: 0,  remote: 0,  total: 0,   time: '2026-07-14 12:00', gpu: 'GPU-03', storage: 'NAS-2T',   status: '待配置' },
            ],
            selectedTask: -1,
            // 步骤3
            formats: ['lerobot 2.1', 'lerobot 3.0', 'HDFS5', 'ROSbag2', 'Zarr'],
            selectedFormats: [],
            jointOptions: ['单左臂', '单右臂', '双臂', '全身'],
            selectedJoint: '',
            cameraOptions: ['头部彩色', '头部深度', '左手腕', '右手腕'],
            selectedCameras: [],
            deviceOptions: ['左手夹爪', '右手夹爪', '左手力反馈', '右手力反馈'],
            selectedDevices: [],
            // 步骤4
            collectCameras: [
                { key: 'head_color', name: '头部全局相机' },
                { key: 'left_wrist', name: '左手腕' },
                { key: 'right_wrist', name: '右手腕' },
            ],
            collectImages: {},
            // 步骤5
            countdown: 5,
            _collectTimer: null,
            _onCamera: null
        };
    },
    mounted() {
        this._onCamera = (data) => {
            if (!data) return;
            for (const cam of this.collectCameras) {
                if (data[cam.key]) {
                    this.collectImages[cam.key] = data[cam.key];
                }
            }
        };
        mqttClient.addCameraCallback(this._onCamera);
    },
    beforeUnmount() {
        if (this._onCamera) {
            mqttClient.removeCameraCallback(this._onCamera);
        }
        if (this._collectTimer) {
            clearInterval(this._collectTimer);
        }
    },
    methods: {
        doLogin() {
            if (!this.login.account || !this.login.password) {
                alert('请输入账号和密码');
                return;
            }
            // 模拟登陆成功
            this.step = 2;
        },
        actionUpload() {
            if (this.selectedTask < 0) return;
            const task = this.tasks[this.selectedTask];
            alert(`上传任务: ${task.name}\n本地待传: ${task.local} 条`);
        },
        actionCollect() {
            if (this.selectedTask < 0) return;
            const task = this.tasks[this.selectedTask];
            if (task.local === 0 && task.remote === 0 && task.total === 0) {
                // 首次需要配置
                this.step = 3;
            } else {
                // 已有任务，直接进入采集页面
                this.step = 4;
            }
        },
        enterCollectPage() {
            this.step = 4;
        },
        startCollect() {
            this.step = 5;
            this.countdown = 5;
            // 开启相机流
            mqttClient.publishCameraControl('start');
            this._collectTimer = setInterval(() => {
                this.countdown--;
                if (this.countdown <= 0) {
                    clearInterval(this._collectTimer);
                    this._collectTimer = null;
                    mqttClient.publishCameraControl('stop');
                    alert('采集完成');
                    this.step = 4;
                }
            }, 1000);
        },
        finishCollect() {
            if (this._collectTimer) {
                clearInterval(this._collectTimer);
                this._collectTimer = null;
                mqttClient.publishCameraControl('stop');
            }
            this.step = 4;
        }
    }
};
