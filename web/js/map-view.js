// map-view.js
// 地图视图：基于激光雷达点云数据的 2D 地图（canvas 渲染）
// 通过 MQTT 订阅 /G2_minth_cloud 接收实时点云

import { mqttClient } from './mqtt-client.js';

export default {
    name: 'MapView',
    template: `
    <div class="panel map-panel">
        <div class="map-canvas-wrap">
            <canvas ref="canvas" class="map-canvas-main"></canvas>
        </div>

        <!-- 右下角建图操作按钮 -->
        <div class="map-action-btns">
            <button :class="['map-btn', 'map-btn-start', isMapping ? 'map-btn-active' : '']" @click="confirmStartMapping" :disabled="isMapping">
                ▶ 开始扫图
            </button>
            <button :class="['map-btn', 'map-btn-stop']" @click="confirmStopMapping" :disabled="!isMapping">
                ■ 结束扫图
            </button>
            <button :class="['map-btn', 'map-btn-save']" @click="confirmSaveMap" :disabled="!isMapping">
                💾 保存地图
            </button>
            <span class="map-mapping-status" v-if="isMapping">🔴 建图中...</span>
        </div>

        <!-- 底部信息栏 -->
        <div class="map-infobar">
            <div class="map-info-left">
                <span class="map-info-item">总点数: <strong>{{ pointCount }}</strong></span>
                <span class="map-info-item">前: <strong>{{ frontCount }}</strong></span>
                <span class="map-info-item">后: <strong>{{ backCount }}</strong></span>
                <span class="map-info-item">缩放: <strong>{{ scale }}px/m</strong></span>
            </div>
            <div class="map-info-right">
                <span :class="['map-status-dot', connected ? 'map-connected' : 'map-disconnected']">
                    ● {{ connected ? '已连接' : '未连接' }}
                </span>
            </div>
        </div>

        <!-- 操作确认弹窗 -->
        <div v-if="confirmDialog.visible" class="save-overlay" @click.self="confirmDialog.visible = false">
            <div class="save-dialog" style="width:360px;">
                <h6 :style="{color: confirmDialog.color, marginBottom: '16px'}">
                    {{ confirmDialog.icon }} {{ confirmDialog.title }}
                </h6>
                <div style="font-size:14px; color:#606266; margin-bottom:20px; text-align:center; line-height:1.6;">
                    {{ confirmDialog.message }}
                </div>
                <div class="step-actions">
                    <button class="nav-btn" @click="confirmDialog.visible = false">取消</button>
                    <button class="nav-btn" :style="{background: confirmDialog.color, color:'#fff'}" @click="executeConfirm">确定</button>
                </div>
            </div>
        </div>
    </div>
    `,
    data() {
        return {
            pointCount: 0,
            frontCount: 0,
            backCount: 0,
            centerX: 0,
            centerY: 0,
            scale: 40,       // 1 米 = 40 像素
            connected: false,
            isMapping: false,
            points: [],       // [{x, y, z}]
            ctx: null,
            rafId: null,
            _onCloud: null,
            _onMapInfo: null,
            _pendingAction: null,
            confirmDialog: {
                visible: false,
                title: '',
                message: '',
                icon: '',
                color: '',
                action: null
            }
        };
    },
    mounted() {
        this.ctx = this.$refs.canvas.getContext('2d');
        this.resizeCanvas();
        window.addEventListener('resize', this.resizeCanvas);

        // 订阅点云数据
        this._onCloud = (data) => {
            this.connected = true;
            if (data && data.points) {
                this.points = data.points.map(p => ({ x: p[0], y: p[1], z: p[2] }));
                this.pointCount = this.points.length;
                this.frontCount = data.front_count || 0;
                this.backCount = data.back_count || 0;

                // 自动计算中心（取平均值）
                if (this.points.length > 0) {
                    let sx = 0, sy = 0;
                    for (const p of this.points) { sx += p.x; sy += p.y; }
                    this.centerX = sx / this.points.length;
                    this.centerY = sy / this.points.length;
                }
            }
        };

        // 订阅地图信息（SLAM状态）
        this._onMapInfo = (data) => {
            if (data.command === 'slam_state' && data.data) {
                this.isMapping = data.data.is_mapping || false;
            }
        };

        // mqttClient 已连接 /G2_minth_status，这里复用同一个连接
        // 通过 onMessageArrived 分发，需要额外注册回调
        mqttClient.addCloudCallback(this._onCloud);
        mqttClient.addMapInfoCallback(this._onMapInfo);

        // 通知后端开始发布点云，并请求地图/SLAM状态
        mqttClient.publishCloudControl('start_cloud');
        mqttClient.publishMapControl('read_maps');

        this.render();
    },
    beforeUnmount() {
        window.removeEventListener('resize', this.resizeCanvas);
        cancelAnimationFrame(this.rafId);
        if (this._onCloud) {
            mqttClient.removeCloudCallback(this._onCloud);
        }
        if (this._onMapInfo) {
            mqttClient.removeMapInfoCallback(this._onMapInfo);
        }
        // 通知后端停止发布点云
        mqttClient.publishCloudControl('stop_cloud');
    },
    methods: {
        _showConfirm(title, message, icon, color, action) {
            this.confirmDialog = {
                visible: true,
                title, message, icon, color,
                action
            };
        },
        executeConfirm() {
            const fn = this.confirmDialog.action;
            this.confirmDialog.visible = false;
            if (fn) fn.call(this);
        },
        confirmStartMapping() {
            this._showConfirm('开始扫图', '确定要开始激光扫图建图吗？请确保机器人已在需要建图的区域。', '▶', '#67c23a', this.startMapping);
        },
        confirmStopMapping() {
            this._showConfirm('结束扫图', '确定要结束扫图并停止建图吗？建图将在保存后停止。', '■', '#f56c6c', this.stopMapping);
        },
        confirmSaveMap() {
            this._showConfirm('保存地图', '确定要保存当前地图吗？地图将被写入机器人系统。', '💾', '#e6a23c', this.saveMap);
        },
        startMapping() {
            mqttClient.publishMapControl('start_mapping');
            console.log('[地图] 开始扫图');
        },
        stopMapping() {
            mqttClient.publishMapControl('stop_mapping');
            console.log('[地图] 结束扫图');
        },
        saveMap() {
            mqttClient.publishMapControl('save_map');
            console.log('[地图] 保存地图');
        },
        resizeCanvas() {
            const c = this.$refs.canvas;
            const rect = c.getBoundingClientRect();
            c.width = rect.width;
            c.height = rect.height;
        },
        render() {
            const ctx = this.ctx;
            const c = this.$refs.canvas;
            if (!ctx) return;

            // 清屏 - 白色背景
            ctx.fillStyle = '#f8fafc';
            ctx.fillRect(0, 0, c.width, c.height);

            const cx = c.width / 2;
            const cy = c.height / 2;
            const scale = this.scale;

            // 网格 - 浅蓝色网格线
            ctx.strokeStyle = '#e4ecf4';
            ctx.lineWidth = 1;
            for (let x = cx % 50; x < c.width; x += 50) {
                ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, c.height); ctx.stroke();
            }
            for (let y = cy % 50; y < c.height; y += 50) {
                ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(c.width, y); ctx.stroke();
            }

            // 坐标轴 - 蓝色
            ctx.strokeStyle = '#409eff';
            ctx.lineWidth = 1;
            ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(c.width, cy); ctx.stroke();
            ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, c.height); ctx.stroke();

            // 绘制点云 - 深蓝色
            ctx.fillStyle = '#409eff';
            for (const p of this.points) {
                const x = cx + (p.x - this.centerX) * scale;
                const y = cy - (p.y - this.centerY) * scale;
                if (x >= 0 && x < c.width && y >= 0 && y < c.height) {
                    ctx.fillRect(x - 0.5, y - 0.5, 1.5, 1.5);
                }
            }

            // 机器人位置（中心）- 绿色
            ctx.fillStyle = '#67c23a';
            ctx.beginPath();
            ctx.arc(cx, cy, 5, 0, Math.PI * 2);
            ctx.fill();

            // 机器人朝向指示 - 绿色
            ctx.strokeStyle = '#67c23a';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(cx, cy);
            ctx.lineTo(cx + 15, cy);
            ctx.stroke();

            this.rafId = requestAnimationFrame(this.render.bind(this));
        }
    }
};
