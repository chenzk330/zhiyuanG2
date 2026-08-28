// mqtt-client.js
// MQTT 客户端封装，基于 paho-mqtt.min.js
// 连接本机 WebSocket 9001，订阅 /humanoid/ 主题
//
// 主题结构（与服务端 services/main.py 对齐）：
//   /humanoid/camera/data         服务端发布相机帧
//   /humanoid/camera/control      客户端控制相机
//   /humanoid/joints/data         服务端发布关节数据列表
//   /humanoid/joints/control      客户端控制关节运动
//   /humanoid/joints/save         客户端保存关节/位姿数据
//   /humanoid/status/data         服务端发布机器人状态
//   /humanoid/status/control      客户端控制状态/点云
//   /humanoid/status/cloud        服务端发布点云
//   /humanoid/commands/data       客户端发送动作命令
//   /humanoid/commands/done       服务端发布完成通知
//   /humanoid/map/points          服务端发布地图点位
//   /humanoid/map/control         客户端控制地图点位
//   /humanoid/programs/control    客户端控制程序调试
//   /humanoid/programs/step       服务端发布执行步骤
//   /humanoid/programs/codes      服务端发布代码内容
//   /humanoid/programs/files      服务端发布文件列表
//   /humanoid/programs/file_content  服务端发布指定文件内容
//   /humanoid/programs/upload_result  服务端发布上传结果
//   /humanoid/programs/delete_result  服务端发布删除结果

const MQTT_BROKER = location.hostname || '10.2.236.6';
const MQTT_PORT   = 9001;

// 服务端发布主题（客户端订阅）
const STATUS_TOPIC = '/humanoid/status/data';
const CLOUD_TOPIC  = '/humanoid/status/cloud';
const CAMERAS_TOPIC = '/humanoid/camera/data';
const JOINTS_DATA_TOPIC = '/humanoid/joints/data';
const DONE_TOPIC = '/humanoid/commands/done';
const MAP_POINTS_TOPIC = '/humanoid/map/points';
const MAP_INFO_TOPIC = '/humanoid/map/info';
const PROGRAMS_STEP_TOPIC = '/humanoid/programs/step';
const PROGRAMS_CODES_TOPIC = '/humanoid/programs/codes';
const PROGRAMS_FILES_TOPIC = '/humanoid/programs/files';
const PROGRAMS_FILE_CONTENT_TOPIC = '/humanoid/programs/file_content';
const PROGRAMS_UPLOAD_RESULT_TOPIC = '/humanoid/programs/upload_result';
const PROGRAMS_DELETE_RESULT_TOPIC = '/humanoid/programs/delete_result';
const MODBUS_DATA_TOPIC = '/humanoid/modbus/data';

// 客户端发布主题（客户端发送命令）
const CAMERA_CTRL_TOPIC = '/humanoid/camera/control';
const JOINTS_CTRL_TOPIC = '/humanoid/joints/control';
const JOINTS_SAVE_TOPIC = '/humanoid/joints/save';
const STATUS_CTRL_TOPIC = '/humanoid/status/control';
const COMMANDS_TOPIC = '/humanoid/commands/data';
const MAP_CTRL_TOPIC = '/humanoid/map/control';
const PROGRAMS_CTRL_TOPIC = '/humanoid/programs/control';
const MODBUS_CTRL_TOPIC = '/humanoid/modbus/control';

class MqttClient {
    constructor() {
        this.client = null;
        this.connected = false;
        this.statusCallback = null;
        this.cloudCallbacks = [];
        this.cameraCallbacks = [];
        this.cameraDetectCallbacks = [];
        this.doneCallbacks = [];
        this.runtimeStepCallbacks = [];
        this.runtimeCodesCallbacks = [];
        this.runtimeProgramFilesCallbacks = [];
        this.runtimeFileContentCallbacks = [];
        this.runtimeUploadResultCallbacks = [];
        this.runtimeDeleteResultCallbacks = [];
        this.dataRespCallbacks = [];
        this.mapPointsCallbacks = [];
        this.mapInfoCallbacks = [];
        this.modbusDataCallbacks = [];
    }

    connect() {
        const clientId = 'g2_web_' + Math.random().toString(16).substr(2, 8);
        this.client = new Paho.Client(MQTT_BROKER, MQTT_PORT, clientId);

        this.client.onConnectionLost = (responseObject) => {
            if (responseObject.errorCode !== 0) {
                console.error('[MQTT] 连接丢失:', responseObject.errorMessage);
            }
            this.connected = false;
            // 5 秒后重连
            setTimeout(() => this.connect(), 5000);
        };

        this.client.onMessageArrived = (message) => {
            try {
                const data = JSON.parse(message.payloadString);

                if (message.destinationName === STATUS_TOPIC && this.statusCallback) {
                    this.statusCallback(data);
                } else if (message.destinationName === CLOUD_TOPIC) {
                    this.cloudCallbacks.forEach(cb => cb(data));
                } else if (message.destinationName === CAMERAS_TOPIC) {
                    this.cameraCallbacks.forEach(cb => cb(data));
                    this.cameraDetectCallbacks.forEach(cb => cb(data));
                } else if (message.destinationName === DONE_TOPIC) {
                    this.doneCallbacks.forEach(cb => cb(data));
                } else if (message.destinationName === JOINTS_DATA_TOPIC) {
                    this.dataRespCallbacks.forEach(cb => cb(data));
                } else if (message.destinationName === PROGRAMS_STEP_TOPIC) {
                    this.runtimeStepCallbacks.forEach(cb => cb(data));
                } else if (message.destinationName === PROGRAMS_CODES_TOPIC) {
                    this.runtimeCodesCallbacks.forEach(cb => cb(data));
                } else if (message.destinationName === PROGRAMS_FILES_TOPIC) {
                    this.runtimeProgramFilesCallbacks.forEach(cb => cb(data));
                } else if (message.destinationName === PROGRAMS_FILE_CONTENT_TOPIC) {
                    this.runtimeFileContentCallbacks.forEach(cb => cb(data));
                } else if (message.destinationName === PROGRAMS_UPLOAD_RESULT_TOPIC) {
                    this.runtimeUploadResultCallbacks.forEach(cb => cb(data));
                } else if (message.destinationName === PROGRAMS_DELETE_RESULT_TOPIC) {
                    this.runtimeDeleteResultCallbacks.forEach(cb => cb(data));
                } else if (message.destinationName === MAP_POINTS_TOPIC) {
                    this.mapPointsCallbacks.forEach(cb => cb(data));
                } else if (message.destinationName === MAP_INFO_TOPIC) {
                    this.mapInfoCallbacks.forEach(cb => cb(data));
                } else if (message.destinationName === MODBUS_DATA_TOPIC) {
                    this.modbusDataCallbacks.forEach(cb => cb(data));
                }
            } catch (e) {
                console.error('[MQTT] JSON 解析失败:', e);
            }
        };

        this.client.connect({
            onSuccess: () => {
                console.log('[MQTT] 连接成功:', MQTT_BROKER + ':' + MQTT_PORT);
                this.connected = true;
                this.client.subscribe(STATUS_TOPIC, { qos: 0 });
                this.client.subscribe(CLOUD_TOPIC, { qos: 0 });
                this.client.subscribe(CAMERAS_TOPIC, { qos: 0 });
                this.client.subscribe(DONE_TOPIC, { qos: 0 });
                this.client.subscribe(JOINTS_DATA_TOPIC, { qos: 0 });
                this.client.subscribe(PROGRAMS_STEP_TOPIC, { qos: 0 });
                this.client.subscribe(PROGRAMS_CODES_TOPIC, { qos: 0 });
                this.client.subscribe(PROGRAMS_FILES_TOPIC, { qos: 0 });
                this.client.subscribe(PROGRAMS_FILE_CONTENT_TOPIC, { qos: 0 });
                this.client.subscribe(PROGRAMS_UPLOAD_RESULT_TOPIC, { qos: 0 });
                this.client.subscribe(PROGRAMS_DELETE_RESULT_TOPIC, { qos: 0 });
                this.client.subscribe(MAP_POINTS_TOPIC, { qos: 0 });
                this.client.subscribe(MAP_INFO_TOPIC, { qos: 0 });
                this.client.subscribe(MODBUS_DATA_TOPIC, { qos: 0 });
            },
            onFailure: (err) => {
                console.error('[MQTT] 连接失败:', err.errorMessage);
                setTimeout(() => this.connect(), 5000);
            },
            useSSL: false,
        });
    }

    onStatus(callback) {
        this.statusCallback = callback;
    }

    /**
     * 注册点云数据回调
     */
    addCloudCallback(callback) {
        if (!this.cloudCallbacks.includes(callback)) {
            this.cloudCallbacks.push(callback);
        }
    }

    removeCloudCallback(callback) {
        this.cloudCallbacks = this.cloudCallbacks.filter(cb => cb !== callback);
    }

    /**
     * 注册相机数据回调
     */
    addCameraCallback(callback) {
        if (!this.cameraCallbacks.includes(callback)) {
            this.cameraCallbacks.push(callback);
        }
    }

    removeCameraCallback(callback) {
        this.cameraCallbacks = this.cameraCallbacks.filter(cb => cb !== callback);
    }

    /**
     * 注册相机检测回调
     */
    addCameraDetectCallback(callback) {
        if (!this.cameraDetectCallbacks.includes(callback)) {
            this.cameraDetectCallbacks.push(callback);
        }
    }

    removeCameraDetectCallback(callback) {
        this.cameraDetectCallbacks = this.cameraDetectCallbacks.filter(cb => cb !== callback);
    }

    /**
     * 注册命令完成回调
     */
    addDoneCallback(callback) {
        if (!this.doneCallbacks.includes(callback)) {
            this.doneCallbacks.push(callback);
        }
    }

    removeDoneCallback(callback) {
        this.doneCallbacks = this.doneCallbacks.filter(cb => cb !== callback);
    }

    /**
     * 注册调试步骤回调
     */
    addRuntimeStepCallback(callback) {
        if (!this.runtimeStepCallbacks.includes(callback)) {
            this.runtimeStepCallbacks.push(callback);
        }
    }

    removeRuntimeStepCallback(callback) {
        this.runtimeStepCallbacks = this.runtimeStepCallbacks.filter(cb => cb !== callback);
    }

    /**
     * 注册代码内容回调
     */
    addRuntimeCodesCallback(callback) {
        if (!this.runtimeCodesCallbacks.includes(callback)) {
            this.runtimeCodesCallbacks.push(callback);
        }
    }

    removeRuntimeCodesCallback(callback) {
        this.runtimeCodesCallbacks = this.runtimeCodesCallbacks.filter(cb => cb !== callback);
    }

    /**
     * 注册程序文件列表回调
     */
    addRuntimeProgramFilesCallback(callback) {
        if (!this.runtimeProgramFilesCallbacks.includes(callback)) {
            this.runtimeProgramFilesCallbacks.push(callback);
        }
    }

    removeRuntimeProgramFilesCallback(callback) {
        this.runtimeProgramFilesCallbacks = this.runtimeProgramFilesCallbacks.filter(cb => cb !== callback);
    }

    /**
     * 注册指定文件内容回调
     */
    addRuntimeFileContentCallback(callback) {
        if (!this.runtimeFileContentCallbacks.includes(callback)) {
            this.runtimeFileContentCallbacks.push(callback);
        }
    }

    removeRuntimeFileContentCallback(callback) {
        this.runtimeFileContentCallbacks = this.runtimeFileContentCallbacks.filter(cb => cb !== callback);
    }

    /**
     * 注册程序上传结果回调
     */
    addRuntimeUploadResultCallback(callback) {
        if (!this.runtimeUploadResultCallbacks.includes(callback)) {
            this.runtimeUploadResultCallbacks.push(callback);
        }
    }

    removeRuntimeUploadResultCallback(callback) {
        this.runtimeUploadResultCallbacks = this.runtimeUploadResultCallbacks.filter(cb => cb !== callback);
    }

    /**
     * 注册程序删除结果回调
     */
    addRuntimeDeleteResultCallback(callback) {
        if (!this.runtimeDeleteResultCallbacks.includes(callback)) {
            this.runtimeDeleteResultCallbacks.push(callback);
        }
    }

    removeRuntimeDeleteResultCallback(callback) {
        this.runtimeDeleteResultCallbacks = this.runtimeDeleteResultCallbacks.filter(cb => cb !== callback);
    }

    /**
     * 注册数据响应回调
     */
    addDataRespCallback(callback) {
        if (!this.dataRespCallbacks.includes(callback)) {
            this.dataRespCallbacks.push(callback);
        }
    }

    removeDataRespCallback(callback) {
        this.dataRespCallbacks = this.dataRespCallbacks.filter(cb => cb !== callback);
    }

    /**
     * 注册地图点位回调
     */
    addMapPointsCallback(callback) {
        if (!this.mapPointsCallbacks.includes(callback)) {
            this.mapPointsCallbacks.push(callback);
        }
    }

    removeMapPointsCallback(callback) {
        this.mapPointsCallbacks = this.mapPointsCallbacks.filter(cb => cb !== callback);
    }

    /**
     * 注册地图信息回调（地图列表/SLAM状态）
     */
    addMapInfoCallback(callback) {
        if (!this.mapInfoCallbacks.includes(callback)) {
            this.mapInfoCallbacks.push(callback);
        }
    }

    removeMapInfoCallback(callback) {
        this.mapInfoCallbacks = this.mapInfoCallbacks.filter(cb => cb !== callback);
    }

    /**
     * 注册 Modbus 数据回调
     */
    addModbusDataCallback(callback) {
        if (!this.modbusDataCallbacks.includes(callback)) {
            this.modbusDataCallbacks.push(callback);
        }
    }

    removeModbusDataCallback(callback) {
        this.modbusDataCallbacks = this.modbusDataCallbacks.filter(cb => cb !== callback);
    }

    /**
     * 发布命令到指定 topic
     */
    publishToTopic(topic, payload) {
        if (!this.connected || !this.client) {
            console.warn('[MQTT] 未连接，无法发送:', topic);
            return;
        }
        const message = new Paho.Message(JSON.stringify(payload));
        message.destinationName = topic;
        message.qos = 0;
        this.client.send(message);
        console.log('[MQTT] 已发送到', topic, payload);
    }

    /**
     * 发布关节运动命令到 /humanoid/joints/control
     * @param {string} command - 命令名（WBC/arms/left/right/head/waist/joint）
     * @param {*} data - 命令数据
     */
    publishJointCommand(command, data) {
        this.publishToTopic(JOINTS_CTRL_TOPIC, { command, data });
    }

    /**
     * 发布动作命令到 /humanoid/commands/data
     * @param {string} command - 命令名（tts/offset_move/grab/go/go_rel/cam_head）
     * @param {*} data - 命令数据
     */
    publishCommand(command, data) {
        this.publishToTopic(COMMANDS_TOPIC, { command, data });
    }

    /**
     * 发送相机控制命令到 /humanoid/camera/control
     * @param {string} command - 命令名（start/stop/save_photo/detect）
     */
    publishCameraControl(command) {
        this.publishToTopic(CAMERA_CTRL_TOPIC, { command });
    }

    /**
     * 发送相机控制命令（带额外字段）到 /humanoid/camera/control
     * @param {string} command - 命令名
     * @param {object} extra - 额外字段，如 { cameras: [...] } 或 { yolo: 'wxf.pt' }
     */
    publishCameraCommand(command, extra = {}) {
        this.publishToTopic(CAMERA_CTRL_TOPIC, { command, ...extra });
    }

    /**
     * 发送数据保存命令到 /humanoid/joints/save
     * @param {object} payload - 完整命令消息
     */
    publishDataSave(payload) {
        this.publishToTopic(JOINTS_SAVE_TOPIC, payload);
    }

    /**
     * 发送数据读取请求到 /humanoid/joints/save
     * @param {string} command - 命令名（read/update/delete）
     * @param {object} extra - 额外字段
     */
    publishDataReq(command, extra = {}) {
        this.publishToTopic(JOINTS_SAVE_TOPIC, { command, ...extra });
    }

    /**
     * 发送云端控制指令（start_cloud / stop_cloud）到 /humanoid/status/control
     * @param {string} command - 命令名
     */
    publishCloudControl(command) {
        this.publishToTopic(STATUS_CTRL_TOPIC, { command });
    }

    /**
     * 发送地图点位控制命令到 /humanoid/map/control
     * @param {string} command - 命令名（read_points/save_point）
     * @param {*} data - 命令数据
     */
    publishMapControl(command, data = null) {
        const payload = data !== null ? { command, data } : { command };
        this.publishToTopic(MAP_CTRL_TOPIC, payload);
    }

    /**
     * 发送 Modbus 控制命令到 /humanoid/modbus/control
     * @param {string} command - 命令名（read/write）
     * @param {*} data - 命令数据
     */
    publishModbusControl(command, data = null) {
        const payload = data !== null ? { command, data } : { command };
        this.publishToTopic(MODBUS_CTRL_TOPIC, payload);
    }

    /**
     * 发送程序调试命令到 /humanoid/programs/control
     * @param {string} command - 命令名（run/debug/next/stop/copy/codes/read_files）
     * @param {*} data - 命令数据
     */
    publishProgramControl(command, data = null) {
        const payload = data !== null ? { command, data } : { command };
        this.publishToTopic(PROGRAMS_CTRL_TOPIC, payload);
    }

    /**
     * 兼容旧接口：发送 runtime 调试命令
     */
    publishRuntimeDebug(command, data) {
        this.publishProgramControl(command, data);
    }
}

export const mqttClient = new MqttClient();
