// vue_app.js
// 主 Vue 应用入口

import { createApp } from 'vue';
import { UrdfViewer } from './urdf-viewer.js';
import { mqttClient } from './mqtt-client.js';
import TeachJoints     from './teach-joints.js';
import TeachCoords     from './teach-coords.js';
import ProgramView     from './program-view.js';
import MapView         from './map-view.js';
import CameraView      from './camera-view.js';
import DataCollection  from './data-collection.js';
import ModelInference  from './model-inference.js';
import DataJointsCoords from './data-joints-coords.js';
import ModbusView       from './modbus-view.js';
import YoloInference   from './yolo-inference.js';
import ChassisControl  from './chassis-control.js';
import PlaceholderView from './placeholder-view.js';


const App = {
    data() {
        return {
            currentMenu: '',          // 当前选中的菜单项 id
            openDropdown: '',         // 当前展开的下拉父菜单 id
            isLoggedIn: false,        // 是否已登录管理模式
            loginDialog: {            // 登录弹窗
                visible: false,
                username: '',
                password: '',
                error: ''
            },
            menus: [
                {
                    id: 'teach', label: '示教', icon: '🎮', children: [
                        { id: 'teach_joints', label: '角度轴' },
                        { id: 'teach_coords', label: '末端坐标' },
                    ]
                },
                {
                    id: 'data', label: '数据', icon: '📊', children: [
                        { id: 'data_jc',      label: '关节/坐标' },
                        { id: 'data_modbus',  label: 'Modbus' },
                    ]
                },
                { id: 'program', label: '程序', icon: '📋' },
                {
                    id: 'map', label: '地图', icon: '🗺️', children: [
                        { id: 'map_scan',    label: '扫图建图' },
                        { id: 'map_chassis', label: '底盘控制' },
                    ]
                },
                {
                    id: 'camera', label: '相机', icon: '📷', children: [
                        { id: 'cam_capture', label: '采集' },
                        { id: 'cam_yolo',    label: 'YOLO计算' },
                    ]
                },
            ],
            urdfViewer: null,
            robotStatus: null   // 共享的机器人状态
        };
    },
    components: { TeachJoints, TeachCoords, ProgramView, MapView, CameraView, DataCollection, ModelInference, DataJointsCoords, ModbusView, YoloInference, ChassisControl, PlaceholderView },
    provide() {
        return {
            getUrdfViewer: () => this.urdfViewer,
            getRobotStatus: () => this.robotStatus,
            isLoggedIn: () => this.isLoggedIn
        };
    },
    computed: {
        // 根据登录状态过滤可见菜单（未登录时隐藏 地图、相机）
        visibleMenus() {
            if (this.isLoggedIn) return this.menus;
            return this.menus.filter(m => m.id !== 'map' && m.id !== 'camera');
        },
        // 当前菜单项的标题（用于占位组件）
        currentTitle() {
            for (const m of this.menus) {
                if (m.id === this.currentMenu) return m.label;
                if (m.children) {
                    const child = m.children.find(c => c.id === this.currentMenu);
                    if (child) return child.label;
                }
            }
            return '';
        },
        // 导航栏右侧显示的页面标题
        navPageTitle() {
            if (this.currentMenu === 'program') {
                const pv = this.$refs.programView;
                const name = pv ? pv.currentFileName : 'main.py';
                return `程序 - ${name}`;
            }
            const titles = {
                'data_jc': '关节 / 坐标数据',
                'data_modbus': 'Modbus 数据',
                'map_slam': '扫图建图',
                'map_points': '地图点管理',
                'cam_capture': '相机采集',
                'cam_yolo': 'YOLO 计算',
            };
            return titles[this.currentMenu] || '';
        },
        // 判断当前菜单是否为占位页面（非已实现的组件）
        isPlaceholder() {
            const implemented = [
                'teach_joints', 'teach_coords', 'program',
                'map_scan', 'map_chassis', 'cam_capture', 'cam_yolo', 'data_jc', 'data_modbus'
            ];
            return this.currentMenu && !implemented.includes(this.currentMenu);
        }
    },
    template: `
    <div @click="closeDropdown">
        <canvas id="bg-canvas" ref="bgCanvas"></canvas>

        <nav id="toolbar" @click.stop>
            <span class="brand" @click="toggleHome">
                底盘机器人
            </span>

            <template v-for="m in visibleMenus" :key="m.id">
                <!-- 无子菜单：直接按钮 -->
                <button v-if="!m.children"
                        class="menu-btn"
                        :class="{ active: currentMenu === m.id }"
                        @click="selectMenu(m.id)">
                    <span class="menu-icon">{{ m.icon }}</span>{{ m.label }}
                </button>

                <!-- 有子菜单：下拉 -->
                <div v-else class="menu-dropdown">
                    <button class="menu-btn"
                            :class="{ active: openDropdown === m.id || isChildActive(m) }"
                            @click="toggleDropdown(m.id)">
                        <span class="menu-icon">{{ m.icon }}</span>{{ m.label }}
                        <span class="dropdown-arrow">▾</span>
                    </button>
                    <div v-show="openDropdown === m.id" class="dropdown-panel">
                        <button v-for="c in m.children" :key="c.id"
                                class="dropdown-item"
                                :class="{ active: currentMenu === c.id }"
                                @click="selectMenu(c.id)">
                            {{ c.label }}
                        </button>
                    </div>
                </div>
            </template>

            <span class="nav-page-title" v-if="navPageTitle">{{ navPageTitle }}</span>

            <div class="toolbar-right">
                <button class="menu-btn login-btn"
                        :class="{ 'login-btn-out': isLoggedIn }"
                        @click="toggleLogin">
                    <span class="menu-icon">{{ isLoggedIn ? '🔒' : '🔓' }}</span>{{ isLoggedIn ? '退出' : '登录' }}
                </button>
                <span class="brand-logo"><img src="minth-logo.png" alt="Minth" /></span>
            </div>
        </nav>

        <!-- 登录弹窗 -->
        <div v-if="loginDialog.visible" class="save-overlay login-overlay" @click.self="closeLoginDialog">
            <div class="save-dialog login-dialog">
                <h6 class="login-title">🔐 管理模式登录</h6>
                <div class="login-form">
                    <div class="login-form-row">
                        <label>帐号</label>
                        <input type="text" v-model="loginDialog.username" class="login-input"
                               placeholder="请输入帐号" @keyup.enter="doLogin" ref="loginUserInput" />
                    </div>
                    <div class="login-form-row">
                        <label>密码</label>
                        <input type="password" v-model="loginDialog.password" class="login-input"
                               placeholder="请输入密码" @keyup.enter="doLogin" />
                    </div>
                    <div v-if="loginDialog.error" class="login-error">{{ loginDialog.error }}</div>
                    <div class="login-form-actions">
                        <button class="cv-btn cv-btn-save" @click="doLogin">登录</button>
                        <button class="cv-btn" @click="closeLoginDialog">取消</button>
                    </div>
                </div>
            </div>
        </div>

        <main id="content" :class="{ 'content-hidden': !currentMenu, 'content-fullscreen': currentMenu === 'map_chassis' || currentMenu === 'cam_capture' || currentMenu === 'cam_yolo' || currentMenu === 'data_jc' || currentMenu === 'data_modbus' || currentMenu === 'program' || currentMenu === 'teach_joints' || currentMenu === 'teach_coords' }">
            <teach-joints     v-if="currentMenu === 'teach_joints'"></teach-joints>
            <teach-coords     v-if="currentMenu === 'teach_coords'"></teach-coords>
            <program-view     ref="programView" v-if="currentMenu === 'program'"></program-view>
            <map-view         v-if="currentMenu === 'map_scan'"></map-view>

            <!-- 地图 > 底盘控制 -->
            <chassis-control  v-if="currentMenu === 'map_chassis'"></chassis-control>

            <!-- 相机 > 采集 -->
            <camera-view      v-if="currentMenu === 'cam_capture'"></camera-view>

            <!-- 相机 > YOLO计算 -->
            <yolo-inference   v-if="currentMenu === 'cam_yolo'"></yolo-inference>

            <!-- 数据 > 关节/坐标 -->
            <data-joints-coords v-if="currentMenu === 'data_jc'"></data-joints-coords>

            <!-- 数据 > Modbus -->
            <modbus-view      v-if="currentMenu === 'data_modbus'"></modbus-view>

            <!-- 占位页面 -->
            <placeholder-view :title="currentTitle" v-if="isPlaceholder"></placeholder-view>
        </main>
    </div>
    `,
    methods: {
        selectMenu(id) {
            this.currentMenu = id;
            this.openDropdown = '';
        },
        toggleLogin() {
            if (this.isLoggedIn) {
                // 退出登录
                this.isLoggedIn = false;
                // 如果当前处于隐藏菜单页面，回到首页
                if (this.currentMenu === 'map_scan' || this.currentMenu === 'map_chassis'
                    || this.currentMenu === 'cam_capture' || this.currentMenu === 'cam_yolo') {
                    this.currentMenu = '';
                }
                console.log('[登录] 已退出管理模式');
            } else {
                // 打开登录弹窗
                this.loginDialog.visible = true;
                this.loginDialog.username = '';
                this.loginDialog.password = '';
                this.loginDialog.error = '';
                this.$nextTick(() => {
                    if (this.$refs.loginUserInput) this.$refs.loginUserInput.focus();
                });
            }
        },
        closeLoginDialog() {
            this.loginDialog.visible = false;
        },
        doLogin() {
            const u = (this.loginDialog.username || '').trim();
            const p = (this.loginDialog.password || '').trim();
            if (!u || !p) {
                this.loginDialog.error = '请输入帐号和密码';
                return;
            }
            if (u === 'admin' && p === 'admin') {
                this.isLoggedIn = true;
                this.loginDialog.visible = false;
                console.log('[登录] 登录成功，进入管理模式');
            } else {
                this.loginDialog.error = '帐号或密码错误';
            }
        },
        toggleHome() {
            // 点击「底盘机器人」logo，收起所有功能页，显示 3D 模型
            this.currentMenu = '';
            this.openDropdown = '';
        },
        toggleDropdown(id) {
            this.openDropdown = this.openDropdown === id ? '' : id;
        },
        closeDropdown() {
            this.openDropdown = '';
        },
        isChildActive(menu) {
            if (!menu.children) return false;
            return menu.children.some(c => c.id === this.currentMenu);
        },
        toggleMenu(id) {
            this.currentMenu = this.currentMenu === id ? '' : id;
        },
        onStatus(data) {
            this.robotStatus = data;
            // 同步关节到 3D 模型
            if (data.joints && this.urdfViewer) {
                this.urdfViewer.setJointsFromStatus(data.joints);
            }
        }
    },
    mounted() {
        // 初始化背景 3D 模型
        this.urdfViewer = new UrdfViewer(this.$refs.bgCanvas);
        this.urdfViewer.loadUrdf('meshes/model.urdf');

        // 连接 MQTT，订阅机器人状态
        mqttClient.onStatus((data) => this.onStatus(data));
        mqttClient.connect();
    }
};

createApp(App).mount('#app');
