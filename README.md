# zhiyuanG2

智元 G2 人形机器人取放料控制系统。包含两个子项目：

| 目录 | 说明 | 默认部署路径 |
| ---- | ---- | ------------ |
| `wxf0721/` | 机器人控制服务（humanoid_server、相机、关节/夹爪、底盘、Web 控制台） | `/data/wxf/wxf0721` |
| `czk/` | 全自动取放料流水线（三步纠偏 + 取货 + 底盘调度 + 放货，辉羲 RPU 推理） | `/data/czk` |

> 本仓库是代码备份/分发用。实际运行时，`wxf0721/` 内容部署到 `/data/wxf/wxf0721`，`czk/` 内容部署到 `/data/czk`，两者通过 `czk/config/correct.yaml` 中的绝对路径关联。

---

## 一、运行环境要求

### 1. 硬件 / 系统

- 智元 G2 人形机器人
- Ubuntu + Python 3.10
- 辉羲 RPU 推理芯片（设备节点 `/dev/rpu`）

### 2. 智元 GDK SDK（专有，不可在线安装）

GDK 是智元提供的机器人控制 SDK，**不能通过 pip 下载**，需从原机器人整体拷贝 `/home/agi/app` 目录到目标机器同位置。

| 内容 | 路径 | 用途 |
| ---- | ---- | ---- |
| 环境脚本 | `/home/agi/app/env.sh` | 设置 `LD_LIBRARY_PATH` / `PYTHONPATH` |
| 动态库 | `/home/agi/app/lib` | `.so` 库（相机、DDS 等） |
| Python 模块 | `/home/agi/app/gdk/lib` | `agibot_gdk`（相机接口等） |
| 辉羲推理工具 | `/home/agi/app/bin/examples/batch_inference` | 调用 `.ref` 模型在 RPU 上推理 |

### 3. Python 依赖（pip 安装）

```bash
pip install numpy==1.26.4 opencv-python PyYAML paho-mqtt
```

| 包 | 版本 | 说明 |
| -- | ---- | ---- |
| `numpy` | `==1.26.4` | **必须固定**（2.x 与其它依赖不兼容） |
| `opencv-python` | 任意 | `cv2`，图像预处理 / 绘图 |
| `PyYAML` | 任意 | 读取 `config/correct.yaml` |
| `paho-mqtt` | 任意 | 夹爪 / 手臂 MQTT 控制 |

> `czk` 的检测推理走辉羲 RPU（`batch_inference` 调 `.ref` 模型），**不依赖 torch / ultralytics**，无需安装。

### 4. 系统服务

- **MQTT broker（mosquitto）**：必须监听 `localhost:1883`。
- **humanoid_server**：机器人控制总服务，负责相机 / 关节 / 状态的 MQTT 命令订阅。启动方式见下文。

### 5. 设备权限

辉羲 RPU 需要读写 `/dev/rpu`：

```bash
sudo chmod 666 /dev/rpu
```

### 6. 目录准备

相机图片保存目录需存在且可写（首次运行前创建一次即可）：

```bash
mkdir -p /data/wxf/wxf0721/images
```

---

## 二、启动步骤

按顺序执行：

```bash
# 1. 确认 MQTT broker 已监听 1883（通常由 mosquitto 系统服务管理）

# 2. 启动 humanoid_server（需在具备 GDK/DDS 访达能力的 shell 中运行）
cd /data/wxf/wxf0721/services && bash run.sh

# 3. 加载 GDK 环境变量（每个运行 czk 的 shell 都要 source 一次）
source /home/agi/app/env.sh /home/agi/app

# 4. 环境自检
cd /data/czk && python3 check_env.py

# 5. 正式运行（全自动流水线）
cd /data/czk && python3 full_pipeline.py
```

> `check_env.py` 会检查 GDK 环境、MQTT、humanoid_server、辉羲推理工具、模型文件、Python 模块、图片目录、手臂脚本与位姿 JSON，全部 `✓` 后再跑正式流程。

### 常用运行方式

```bash
python3 full_pipeline.py --dry-run           # 只打印流程，不实际执行
python3 full_pipeline.py --skip-init-pose    # 跳过 Phase0 待机姿态
python3 full_pipeline.py --skip-pick         # 跳过取货动作
python3 full_pipeline.py --skip-place        # 跳过放货动作
python3 full_pipeline.py --target-depth 1050 # 覆盖取货前后纠偏目标深度 mm
```

单场景纠偏入口：

```bash
python3 main.py --scene pick    # 取货纠偏
python3 main.py --scene place   # 放货纠偏
```

---

## 三、配置说明

`czk/config/correct.yaml` 是三步纠偏的主要配置，关键路径如下（除非目录规划变化，否则无需改动）：

```yaml
common:
  mqtt_broker: "localhost"
  mqtt_port: 1883
  image_save_dir: "/data/wxf/wxf0721/images"     # 相机 save_photo 输出目录
  gdk_services_dir: "/data/wxf/wxf0721/services" # agibot_gdk 模块路径
  minth_dir: "/data/wxf/wxf0721/runtime"         # minth 库路径
  batch_infer_bin: "/home/agi/app/bin/examples/batch_inference"
```

场景级参数（取货 / 放货分别配置）：模型文件（`.ref` / `.pt`）、目标深度 `target_depth`、LR 目标 `lr_target_x_override`、标定系数 `px_to_meter_override` 等。

---

## 四、注意事项

- **必须用 bash** 执行 `env.sh` 的 source（zsh 与 `BASH_SOURCE` 语法不兼容）。
- **numpy 固定 1.26.4**，不要升级到 2.x。
- **humanoid_server 需在不受限沙箱** 中运行（GDK 的 DDS `memfd` 操作会被受限沙箱拦截）。
- **RPU 设备** 每次重启后可能需要重新 `chmod 666 /dev/rpu`。
- **GDK 相机首次拍照** 可能出现瞬时噪声（`Frame is null`），后续拍照正常，不影响结果。
- 放货/取货若出现系统性左右偏差，优先调 `lr_target_x_override`（偏左调小、偏右调大），无需重新标定。