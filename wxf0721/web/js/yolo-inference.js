import { mqttClient } from './mqtt-client.js';

export default {
    name: 'YoloInference',
    template: `
    <div class="yi-panel">
        <div class="yi-image-area">
            <div class="yi-image-block">
                <div class="yi-image-title">原始画面</div>
                <div class="yi-image-wrap">
                    <img :src="rawSrc + '?t=' + rawTs" class="yi-img" @error="onRawError" />
                    <div v-if="rawError" class="yi-placeholder">图片加载失败</div>
                </div>
            </div>
            <div class="yi-image-block">
                <div class="yi-image-title yi-title-result">计算结果</div>
                <div class="yi-image-wrap">
                    <img :src="resultSrc + '?t=' + resultTs" class="yi-img" @error="onResultError" />
                    <div v-if="resultError" class="yi-placeholder">等待计算结果...</div>
                </div>
            </div>
        </div>
        <div class="yi-bar">
            <div class="yi-bar-left">
                <div class="yi-select-group">
                    <label>模型</label>
                    <select class="cc-select yi-select" v-model="selectedModel">
                        <option value="7.14.pt">7.14.pt</option>
                        <option value="wxf.pt">wxf.pt</option>
                    </select>
                </div>
                <div class="yi-select-group">
                    <label>相机</label>
                    <select class="cc-select yi-select" v-model="selectedCamera">
                        <option value="head">头部</option>
                        <option value="left_wrist">左手腕</option>
                        <option value="right_wrist">右手腕</option>
                    </select>
                </div>
                <button class="cv-btn cv-btn-save" @click="takePhoto">拍照</button>
                <button class="cv-btn cv-btn-yolo" :disabled="inferring" @click="doInfer">
                    {{ inferring ? '计算中...' : '计算' }}
                </button>
            </div>
            <div class="yi-bar-right">
                <span v-if="inferring" class="cv-status cv-status-capturing">● 计算中...</span>
                <span v-else-if="lastInferTime" class="cv-status">上次计算: {{ lastInferTime }}</span>
            </div>
        </div>
    </div>
    `,
    data() {
        return {
            selectedModel: '7.14.pt',
            selectedCamera: 'head',
            rawSrc: 'yolo/rgb.jpg',
            resultSrc: 'yolo/server_result_rgb.jpg',
            rawTs: 0,
            resultTs: 0,
            rawError: false,
            resultError: true,
            inferring: false,
            lastInferTime: ''
        };
    },
    methods: {
        onRawError() {
            this.rawError = true;
        },
        onResultError() {
            this.resultError = true;
        },
        refreshRaw() {
            this.rawError = false;
            this.rawTs = Date.now();
        },
        refreshResult() {
            this.resultError = false;
            this.resultTs = Date.now();
        },
        takePhoto() {
            const camMap = {
                head: 'kHeadColor',
                left_wrist: 'kLeftWrist',
                right_wrist: 'kRightWrist'
            };
            mqttClient.publishCameraCommand('save_photo', {
                cameras: [camMap[this.selectedCamera] || 'kHeadColor'],
                model: this.selectedModel
            });
            setTimeout(() => this.refreshRaw(), 500);
        },
        doInfer() {
            if (this.inferring) return;
            this.inferring = true;
            this.resultError = true;
            mqttClient.publishCameraCommand('yolo_infer', {
                camera: this.selectedCamera,
                model: this.selectedModel
            });
            setTimeout(() => {
                this.refreshResult();
                this.inferring = false;
                const now = new Date();
                this.lastInferTime = now.toLocaleTimeString();
            }, 2000);
        }
    }
};
