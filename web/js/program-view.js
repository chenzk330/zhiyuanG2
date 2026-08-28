// program-view.js
// 程序视图：显示 main.py 代码，支持读取代码、单步调试、高亮当前执行行

import { mqttClient } from './mqtt-client.js';

export default {
    name: 'ProgramView',
    inject: ['isLoggedIn'],
    template: `
    <div class="panel pv-panel">
        <div class="pv-code-wrap">
            <div class="code-block" ref="codeBlock">
                <div
                    v-for="(line, idx) in codeLines"
                    :key="idx"
                    :class="['code-line', { 'code-line-active': idx + 1 === currentLine }]"
                    :ref="idx + 1 === currentLine ? 'activeLine' : null"
                >
                    <span class="line-num">{{ idx + 1 }}</span>
                    <span class="line-content">{{ line || ' ' }}</span>
                </div>
            </div>
        </div>

        <div class="pv-actionbar">
            <div class="pv-stepinfo">
                <span v-if="currentLine > 0">
                    当前执行: 第 <span class="step-line-no">{{ currentLine }}</span> 行
                    <span class="step-code">{{ codeLines[currentLine - 1] }}</span>
                </span>
                <span v-else class="pv-idle">就绪</span>
            </div>
            <div class="program-actions">
                <button class="program-btn file-btn" @click="openFileList">文件</button>
                <button
                    class="program-btn run-btn"
                    :class="{ active: running }"
                    @click="runProgram">
                    {{ running ? '运行中...' : '运行' }}
                </button>
                <button
                    class="program-btn debug-btn"
                    :class="{ active: debugging }"
                    @click="startDebug">
                    {{ debugging ? '调试中...' : '单步调试' }}
                </button>
                <button
                    class="program-btn next-btn"
                    :disabled="!debugging"
                    @click="nextLine">
                    下一行
                </button>
                <button
                    class="program-btn stop-btn"
                    @click="stopProgram">
                    停止程序
                </button>
            </div>
        </div>

        <!-- 文件列表弹窗 -->
        <div v-if="showFileList" class="save-overlay" @click.self="closeFileList">
            <div class="save-dialog save-dialog-files">
                <h6 class="pv-dialog-title">程序文件</h6>

                <!-- 状态提示 -->
                <div v-if="fileMsg" :style="{color: fileMsgColor, fontSize:'13px', marginBottom:'8px', minHeight:'18px'}">{{ fileMsg }}</div>

                <!-- 隐藏的文件选择器 -->
                <input type="file" ref="fileInput" accept=".py" style="display:none;" @change="onFileSelected" />

                <!-- 文件列表 -->
                <div v-if="programFiles.length === 0" class="pv-empty-tip">
                    暂无文件
                </div>
                <ul v-else class="file-list" style="margin-bottom:12px; max-height:280px; overflow-y:auto;">
                    <li
                        v-for="f in programFiles"
                        :key="f"
                        :class="{'file-selected': selectedFile === f}"
                        @click="selectedFile = f"
                        style="cursor:pointer;"
                    >
                        {{ f }}
                    </li>
                </ul>

                <!-- 底部按钮栏 -->
                <div class="dialog-btn-bar dialog-btn-bar-noflexwrap">
                    <button class="nav-btn dialog-btn" @click="triggerFileSelect" :disabled="uploading" style="background:#2196F3; color:#fff;">
                        {{ uploading ? '上传中...' : '上传' }}
                    </button>
                    <button class="nav-btn dialog-btn" @click="openFile" :disabled="!selectedFile" style="background:#4CAF50; color:#fff;">
                        打开
                    </button>
                    <button class="nav-btn dialog-btn" @click="openEditFile" :disabled="!selectedFile || editingFile" style="background:#9C27B0; color:#fff;">
                        {{ editingFile ? '加载中...' : '修改' }}
                    </button>
                    <button class="nav-btn dialog-btn" @click="downloadFile" :disabled="!selectedFile || downloading" style="background:#FF9800; color:#fff;">
                        {{ downloading ? '下载中...' : '下载' }}
                    </button>
                    <button class="nav-btn dialog-btn" @click="deleteFile" :disabled="!selectedFile || selectedFile === 'main.py'" style="background:#f44336; color:#fff;">
                        删除
                    </button>
                    <button class="nav-btn dialog-btn" @click="closeFileList">关闭</button>
                </div>
            </div>
        </div>

        <!-- 代码编辑弹窗 -->
        <div v-if="showEditDialog" class="save-overlay" @click.self="cancelEdit">
            <div class="save-dialog save-dialog-edit">
                <h6 class="pv-dialog-title">修改文件 — {{ editFileName }}</h6>
                <textarea
                    ref="editTextarea"
                    v-model="editContent"
                    class="pv-edit-textarea"
                    spellcheck="false"
                    placeholder="代码加载中..."
                ></textarea>
                <div v-if="editMsg" :style="{color: editMsgColor, fontSize:'13px', marginTop:'8px', minHeight:'18px'}">{{ editMsg }}</div>
                <div class="dialog-btn-bar dialog-btn-bar-noflexwrap" style="justify-content:flex-end;">
                    <button class="nav-btn dialog-btn" @click="cancelEdit">取消</button>
                    <button class="nav-btn dialog-btn" @click="saveEdit" :disabled="editSaving" style="background:#4CAF50; color:#fff;">
                        {{ editSaving ? '保存中...' : '保存' }}
                    </button>
                </div>
            </div>
        </div>
        <!-- 确认弹窗 -->
        <div v-if="confirmDialog.visible" class="save-overlay" @click.self="cancelConfirm">
            <div class="save-dialog" style="width:360px;">
                <h6 :style="{marginBottom:'16px', color: confirmDialog.type === 'run' ? '#4CAF50' : '#FF9800'}">
                    {{ confirmDialog.title }}
                </h6>
                <div style="color:#ccc; margin-bottom:20px; line-height:1.6; white-space:pre-line;">{{ confirmDialog.message }}</div>
                <div class="step-actions">
                    <button class="nav-btn" @click="cancelConfirm">取消</button>
                    <button
                        class="nav-btn"
                        :style="{background: confirmDialog.type === 'run' ? '#4CAF50' : '#FF9800', color:'#fff'}"
                        @click="confirmAction">
                        确定
                    </button>
                </div>
            </div>
        </div>
    </div>
    `,
    data() {
        return {
            code: '// 点击「读取代码」加载程序',
            currentLine: 0,
            debugging: false,
            running: false,
            showFileList: false,
            programFiles: [],
            selectedFile: '',
            currentFileName: 'main.py',
            confirmDialog: { visible: false, type: '', title: '', message: '', action: null },
            uploading: false,
            downloading: false,
            fileMsg: '',
            fileMsgColor: '#4CAF50',
            showEditDialog: false,
            editFileName: '',
            editContent: '',
            editSaving: false,
            editingFile: false,
            editMsg: '',
            editMsgColor: '#4CAF50'
        };
    },
    computed: {
        codeLines() {
            return this.code.split('\n');
        }
    },
    watch: {
        currentLine(newVal) {
            if (newVal > 0) {
                this.$nextTick(() => {
                    this.scrollToActiveLine();
                });
            }
        }
    },
    methods: {
        readCode() {
            mqttClient.publishRuntimeDebug('codes');
            console.log('[程序] 已请求读取代码');
        },
        openFileList() {
            mqttClient.publishRuntimeDebug('read_files');
            this.showFileList = true;
            this.selectedFile = '';
            this.fileMsg = '';
            console.log('[程序] 已请求文件列表');
        },
        closeFileList() {
            this.showFileList = false;
            this.selectedFile = '';
            this.fileMsg = '';
        },
        showFileMsg(msg, isError = false) {
            this.fileMsg = msg;
            this.fileMsgColor = isError ? '#f44336' : '#4CAF50';
            if (msg) {
                setTimeout(() => { this.fileMsg = ''; }, 3000);
            }
        },
        triggerFileSelect() {
            this.$refs.fileInput.click();
        },
        onFileSelected(event) {
            const file = event.target.files[0];
            if (!file) return;
            if (!file.name.endsWith('.py')) {
                this.showFileMsg('只支持 .py 文件', true);
                event.target.value = '';
                return;
            }
            this.uploading = true;
            this.fileMsg = '';
            if (this._uploadTimer) clearTimeout(this._uploadTimer);
            this._uploadTimer = setTimeout(() => {
                if (this.uploading) {
                    this.uploading = false;
                    this.showFileMsg('上传超时，请检查服务是否正常', true);
                }
            }, 10000);
            const reader = new FileReader();
            reader.onload = (e) => {
                const content = e.target.result;
                mqttClient.publishProgramControl('upload', {
                    filename: file.name,
                    content: content
                });
                console.log('[程序] 已发送上传请求:', file.name, content.length, '字符');
            };
            reader.onerror = () => {
                this.uploading = false;
                if (this._uploadTimer) clearTimeout(this._uploadTimer);
                this.showFileMsg('文件读取失败', true);
            };
            reader.readAsText(file, 'utf-8');
            event.target.value = '';
        },
        onUploadResult(data) {
            this.uploading = false;
            this.editSaving = false;
            if (this._uploadTimer) clearTimeout(this._uploadTimer);
            if (this._editSaveTimer) clearTimeout(this._editSaveTimer);
            if (data && data.success) {
                if (this.showEditDialog && this.editFileName === data.filename) {
                    this.showEditMsg('保存成功: ' + data.filename);
                    this.showFileMsg('保存成功: ' + data.filename);
                    setTimeout(() => {
                        this.cancelEdit();
                        if (data.filename === this.currentFileName) {
                            this.readCode();
                        }
                        mqttClient.publishProgramControl('read_files');
                    }, 500);
                } else {
                    this.showFileMsg('成功上传: ' + data.filename);
                    this.selectedFile = data.filename;
                }
                console.log('[程序] 上传成功:', data.filename);
            } else {
                const errMsg = '上传失败: ' + (data && data.error ? data.error : '未知错误');
                if (this.showEditDialog) {
                    this.showEditMsg(errMsg, true);
                } else {
                    this.showFileMsg(errMsg, true);
                }
                console.error('[程序] 上传失败');
            }
        },
        openFile() {
            if (!this.selectedFile) return;
            mqttClient.publishRuntimeDebug('copy', this.selectedFile);
            this.currentFileName = this.selectedFile;
            this.showFileList = false;
            this.selectedFile = '';
            this.fileMsg = '';
            console.log('[程序] 已复制', this.currentFileName, '→ main.py');
            setTimeout(() => {
                this.readCode();
            }, 300);
        },
        downloadFile() {
            if (!this.selectedFile) return;
            this.downloading = true;
            this._pendingDownloadFile = this.selectedFile;
            this._pendingAction = 'download';
            mqttClient.publishProgramControl('read_file', this.selectedFile);
            console.log('[程序] 请求下载文件:', this.selectedFile);
        },
        openEditFile() {
            if (!this.selectedFile) return;
            this.editingFile = true;
            this.editFileName = this.selectedFile;
            this.editContent = '';
            this.editMsg = '';
            this._pendingAction = 'edit';
            mqttClient.publishProgramControl('read_file', this.selectedFile);
            console.log('[程序] 请求读取文件用于编辑:', this.selectedFile);
        },
        onFileContent(data) {
            this.downloading = false;
            this.editingFile = false;
            if (data && data.success && data.code !== undefined) {
                if (this._pendingAction === 'download') {
                    const blob = new Blob([data.code], { type: 'text/plain;charset=utf-8' });
                    const url = URL.createObjectURL(blob);
                    const a = document.createElement('a');
                    a.href = url;
                    a.download = data.filename;
                    document.body.appendChild(a);
                    a.click();
                    document.body.removeChild(a);
                    URL.revokeObjectURL(url);
                    this.showFileMsg('已下载: ' + data.filename);
                    console.log('[程序] 文件已下载:', data.filename);
                } else if (this._pendingAction === 'edit') {
                    this.editContent = data.code;
                    this.showEditDialog = true;
                    this.editMsg = '';
                    console.log('[程序] 文件已加载到编辑器:', data.filename, data.code.length, '字符');
                    this.$nextTick(() => {
                        if (this.$refs.editTextarea) {
                            this.$refs.editTextarea.focus();
                        }
                    });
                }
                this._pendingAction = null;
            } else {
                if (this._pendingAction === 'download') {
                    this.showFileMsg('下载失败: ' + (data && data.error ? data.error : '未知错误'), true);
                } else if (this._pendingAction === 'edit') {
                    this.showFileMsg('读取文件失败: ' + (data && data.error ? data.error : '未知错误'), true);
                }
                this._pendingAction = null;
                console.error('[程序] 文件读取失败');
            }
        },
        cancelEdit() {
            this.showEditDialog = false;
            this.editContent = '';
            this.editFileName = '';
            this.editMsg = '';
            this.editSaving = false;
        },
        showEditMsg(msg, isError = false) {
            this.editMsg = msg;
            this.editMsgColor = isError ? '#f44336' : '#4CAF50';
            if (msg) {
                setTimeout(() => { this.editMsg = ''; }, 3000);
            }
        },
        saveEdit() {
            if (!this.editFileName) return;
            this.editSaving = true;
            this.editMsg = '';
            if (this._editSaveTimer) clearTimeout(this._editSaveTimer);
            this._editSaveTimer = setTimeout(() => {
                if (this.editSaving) {
                    this.editSaving = false;
                    this.showEditMsg('保存超时，请检查服务是否正常', true);
                }
            }, 10000);
            mqttClient.publishProgramControl('upload', {
                filename: this.editFileName,
                content: this.editContent
            });
            console.log('[程序] 已发送保存请求:', this.editFileName, this.editContent.length, '字符');
        },
        deleteFile() {
            if (!this.selectedFile || this.selectedFile === 'main.py') return;
            this.confirmDialog = {
                visible: true,
                type: 'delete',
                title: '确认删除文件',
                message: `确定要删除文件「${this.selectedFile}」吗？\n此操作不可撤销。`,
                action: () => {
                    mqttClient.publishProgramControl('delete', this.selectedFile);
                    console.log('[程序] 已请求删除:', this.selectedFile);
                }
            };
        },
        onDeleteResult(data) {
            if (data && data.success) {
                this.showFileMsg('已删除: ' + data.filename);
                if (this.selectedFile === data.filename) {
                    this.selectedFile = '';
                }
                console.log('[程序] 删除成功:', data.filename);
            } else {
                this.showFileMsg('删除失败: ' + (data && data.error ? data.error : '未知错误'), true);
                console.error('[程序] 删除失败');
            }
        },
        runProgram() {
            if (this.running) return;
            this.confirmDialog = {
                visible: true,
                type: 'run',
                title: '确认运行程序',
                message: `即将运行程序「${this.currentFileName}」，机器人将开始执行动作。\n请确保周围环境安全，确认继续吗？`,
                action: () => this._doRun()
            };
        },
        _doRun() {
            this.running = true;
            this.currentLine = 0;
            mqttClient.publishRuntimeDebug('run');
            console.log('[程序] 已启动运行');
            this._runTimer = setTimeout(() => {
                this.running = false;
            }, 600000);
        },
        startDebug() {
            if (this.debugging) return;
            this.confirmDialog = {
                visible: true,
                type: 'debug',
                title: '确认单步调试',
                message: `即将开始单步调试「${this.currentFileName}」，每步需手动点击「下一行」。\n请确认开始调试吗？`,
                action: () => this._doDebug()
            };
        },
        _doDebug() {
            this.debugging = true;
            this.currentLine = 0;
            mqttClient.publishRuntimeDebug('debug');
            console.log('[程序] 已启动单步调试');
            this._debugTimer = setTimeout(() => {
                this.debugging = false;
            }, 60000);
        },
        cancelConfirm() {
            this.confirmDialog.visible = false;
        },
        confirmAction() {
            const act = this.confirmDialog.action;
            this.confirmDialog.visible = false;
            if (act) act();
        },
        nextLine() {
            if (!this.debugging) return;
            mqttClient.publishRuntimeDebug('next');
            console.log('[程序] 下一行');
        },
        stopProgram() {
            mqttClient.publishRuntimeDebug('stop');
            this.debugging = false;
            this.running = false;
            this.currentLine = 0;
            if (this._debugTimer) clearTimeout(this._debugTimer);
            if (this._runTimer) clearTimeout(this._runTimer);
            console.log('[程序] 已请求停止');
        },
        onStep(data) {
            if (data && data.lineno) {
                console.log('[程序] 步骤:', data.lineno, data.code);
                this.currentLine = data.lineno;
            }
        },
        onCodes(data) {
            if (data && data.code !== undefined) {
                this.code = data.code;
                console.log('[程序] 已加载代码', data.code.length, '字符');
            }
        },
        onProgramFiles(data) {
            if (data && data.files) {
                this.programFiles = data.files;
                console.log('[程序] 收到文件列表:', data.files);
            }
        },
        scrollToActiveLine() {
            const el = this.$refs.activeLine;
            if (el && el.length > 0) {
                el[0].scrollIntoView({ behavior: 'smooth', block: 'center' });
            }
        }
    },
    mounted() {
        mqttClient.addRuntimeStepCallback(this.onStep);
        mqttClient.addRuntimeCodesCallback(this.onCodes);
        mqttClient.addRuntimeProgramFilesCallback(this.onProgramFiles);
        mqttClient.addRuntimeFileContentCallback(this.onFileContent);
        mqttClient.addRuntimeUploadResultCallback(this.onUploadResult);
        mqttClient.addRuntimeDeleteResultCallback(this.onDeleteResult);
        this.readCode();
    },
    beforeUnmount() {
        mqttClient.removeRuntimeStepCallback(this.onStep);
        mqttClient.removeRuntimeCodesCallback(this.onCodes);
        mqttClient.removeRuntimeProgramFilesCallback(this.onProgramFiles);
        mqttClient.removeRuntimeFileContentCallback(this.onFileContent);
        mqttClient.removeRuntimeUploadResultCallback(this.onUploadResult);
        mqttClient.removeRuntimeDeleteResultCallback(this.onDeleteResult);
        if (this._debugTimer) clearTimeout(this._debugTimer);
        if (this._runTimer) clearTimeout(this._runTimer);
        if (this._uploadTimer) clearTimeout(this._uploadTimer);
        if (this._editSaveTimer) clearTimeout(this._editSaveTimer);
    }
};
