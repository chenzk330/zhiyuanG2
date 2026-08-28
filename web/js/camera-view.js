import { mqttClient } from './mqtt-client.js';

export default {
    name: 'CameraView',
    template: `
    <div class="cv-panel">
        <div class="cv-camera-area">
            <div class="camera-grid camera-grid-4">
                <div class="camera-cell" v-for="cam in cameras" :key="cam.key">
                    <div class="camera-title">{{ cam.name }}</div>
                    <img v-if="images[cam.key]" :src="'data:image/jpeg;base64,' + images[cam.key]" class="camera-img" />
                    <div v-else class="camera-placeholder">等待画面...</div>
                </div>
            </div>
        </div>
        <div class="cv-bar">
            <div class="cv-bar-left">
                <button class="cv-btn cv-btn-on" :class="{active: streaming}" @click="startStream" :disabled="streaming">开启</button>
                <button class="cv-btn cv-btn-off" :class="{active: !streaming}" @click="stopStream" :disabled="!streaming">关闭</button>
                <button class="cv-btn cv-btn-save" @click="saveImage">拍摄</button>
                <button v-if="!continuousCapturing" class="cv-btn cv-btn-continuous-start" @click="startContinuousCapture">持续拍照</button>
                <button v-else class="cv-btn cv-btn-continuous-stop" @click="stopContinuousCapture">停止连拍</button>
            </div>
            <div class="cv-bar-center">
                <span v-if="continuousCapturing" class="cv-status cv-status-capturing">● 连拍中 (0.5s/张)</span>
                <span v-else-if="streaming" class="cv-status cv-status-live">● 采集中</span>
                <span v-else class="cv-status">已停止</span>
            </div>
        </div>
    </div>
    `,
    data() {
        return {
            streaming: false,
            continuousCapturing: false,
            images: {},
            cameras: [
                { key: 'head_color', name: '头部RGB' },
                { key: 'head_depth', name: '头部深度' },
                { key: 'left_wrist', name: '左手腕' },
                { key: 'right_wrist', name: '右手腕' },
            ],
            _onCamera: null
        };
    },
    mounted() {
        this._onCamera = (data) => {
            if (!data) return;
            for (const cam of this.cameras) {
                if (data[cam.key]) {
                    this.images[cam.key] = data[cam.key];
                }
            }
        };
        mqttClient.addCameraCallback(this._onCamera);
    },
    beforeUnmount() {
        if (this._onCamera) {
            mqttClient.removeCameraCallback(this._onCamera);
        }
        if (this.streaming) {
            mqttClient.publishCameraControl('stop');
        }
        if (this.continuousCapturing) {
            mqttClient.publishCameraControl('stop_continuous_capture');
        }
    },
    methods: {
        startStream() {
            this.streaming = true;
            mqttClient.publishCameraControl('start');
        },
        stopStream() {
            this.streaming = false;
            mqttClient.publishCameraControl('stop');
        },
        saveImage() {
            mqttClient.publishCameraCommand('save_photo', { cameras: ['kHeadColor', 'kHeadDepth'] });
        },
        startContinuousCapture() {
            this.continuousCapturing = true;
            mqttClient.publishCameraControl('start_continuous_capture');
        },
        stopContinuousCapture() {
            this.continuousCapturing = false;
            mqttClient.publishCameraControl('stop_continuous_capture');
        }
    }
};
