"""
YOLO 检测 (box) 模型训练。
数据: /home/wxf/图片/images/datasets/yolo_box/data.yaml
模型: yolo11n.pt (nano, 适合小数据集快速训练)
要求: 必须有可用的 CUDA GPU, 否则不训练 (不使用 CPU)。
"""
import sys
from pathlib import Path

import torch
from ultralytics import YOLO

DATA_YAML = "/home/wxf/图片/images/datasets/yolo_box/data.yaml"
MODEL = "yolo11n.pt"        # detect 模型 (非 seg)
PROJECT = "/home/wxf/图片/images/runs/detect"
NAME = "yolo11n_box_headcolor"
EPOCHS = 100
IMGSZ = 640
BATCH = 16
DEVICE = 0       # GPU 设备号, 0 表示第一块 GPU
AMP = False      # 关闭混合精度, 避免 AMP 检查下载 yolo26n.pt (沙箱网络受限)
WORKERS = 8


def check_cuda():
    """检测 CUDA GPU 是否可用, 不可用则直接退出 (不回退到 CPU)。"""
    if not torch.cuda.is_available():
        print("[ERROR] 未检测到可用的 CUDA GPU (torch.cuda.is_available()=False)。")
        print("        本脚本要求 GPU 训练, 不使用 CPU, 已终止。")
        print("        请确认: 1) 已安装 NVIDIA 驱动; 2) CUDA 可用; 3) torch 为 CUDA 版本。")
        sys.exit(1)
    n = torch.cuda.device_count()
    name = torch.cuda.get_device_name(0) if n > 0 else "N/A"
    cap = torch.cuda.get_device_capability(0) if n > 0 else (0, 0)
    print(f"[OK] 检测到 {n} 块 GPU: {name} (capability {cap})")
    print(f"[OK] torch={torch.__version__}, CUDA={torch.version.cuda}")


def main():
    check_cuda()
    model = YOLO(MODEL)
    results = model.train(
        data=DATA_YAML,
        epochs=EPOCHS,
        imgsz=IMGSZ,
        batch=BATCH,
        device=DEVICE,
        amp=AMP,
        workers=WORKERS,
        project=PROJECT,
        name=NAME,
        exist_ok=True,
        # 优化器与学习率
        optimizer="AdamW",
        lr0=1e-3,
        lrf=0.01,
        # 数据增强 (适中, 适合小数据集)
        hsv_h=0.015,
        hsv_s=0.5,
        hsv_v=0.4,
        degrees=10.0,
        translate=0.1,
        scale=0.3,
        flipud=0.0,
        fliplr=0.5,
        mosaic=0.5,
        # 训练控制
        patience=30,
        save=True,
        save_period=-1,
        val=True,
        plots=True,
        verbose=True,
    )
    print("==== 训练完成 ====")
    print("结果目录:", Path(PROJECT) / NAME)
    # 训练后验证 best.pt
    best = Path(PROJECT) / NAME / "weights" / "best.pt"
    if best.exists():
        metrics = model.val(model=str(best), data=DATA_YAML, imgsz=IMGSZ, batch=BATCH, device=DEVICE, plots=True)
        print("==== 验证指标 ====")
        print("box mAP50:", metrics.box.map50 if hasattr(metrics, "box") else None)
        print("box mAP50-95:", metrics.box.map if hasattr(metrics, "box") else None)
    else:
        print("未找到 best.pt:", best)


if __name__ == "__main__":
    main()
