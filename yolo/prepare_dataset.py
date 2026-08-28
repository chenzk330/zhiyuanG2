"""
将 X-AnyLabeling / LabelMe 标注 (JSON) 转换为 YOLO 检测 (box) 数据集。
- rectangle  -> 直接由两点得到 bbox
- polygon    -> 取所有顶点的外接矩形 (min/max) 作为 bbox
- 其它形状 (point/line/circle 等) 跳过
YOLO box 标签格式 (每行): class_id cx cy w h   (均归一化到 [0,1])
输出目录结构:
  datasets/yolo_box/images/{train,val}
  datasets/yolo_box/labels/{train,val}
  datasets/yolo_box/data.yaml
"""
import json
import random
import shutil
from pathlib import Path
from collections import Counter

# ===== 配置 =====
SRC_DIR = Path("/home/wxf/图片/images")
DST_DIR = SRC_DIR / "datasets" / "yolo_box"
VAL_RATIO = 0.2
SEED = 42
IMG_EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff")


def load_classes(classes_txt: Path):
    """读取 classes.txt 作为类别顺序 (index 即为 class id)"""
    if not classes_txt.exists():
        return []
    with open(classes_txt, "r", encoding="utf-8") as f:
        classes = [ln.strip() for ln in f if ln.strip()]
    return classes


def shape_to_bbox(shape):
    """把单个 shape 转为 (xmin, ymin, xmax, ymax)；无法转的返回 None。
    rectangle 的 points 可能是 2 点(对角)或 4 点(角点), 统一用所有点的 min/max 求外接矩形。"""
    st = shape.get("shape_type")
    pts = shape.get("points", [])
    if not pts:
        return None
    if st == "rectangle":
        if len(pts) < 2:
            return None
        xs = [float(p[0]) for p in pts]
        ys = [float(p[1]) for p in pts]
        return (min(xs), min(ys), max(xs), max(ys))
    if st == "polygon":
        xs = [float(p[0]) for p in pts]
        ys = [float(p[1]) for p in pts]
        if len(xs) < 3:
            return None
        return (min(xs), min(ys), max(xs), max(ys))
    # point / line / circle 等不适用于检测框 -> 跳过
    return None


def convert_one(json_path: Path, classes):
    """返回 YOLO box 格式的行列表 (归一化 cx cy w h)"""
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    W = data.get("imageWidth")
    H = data.get("imageHeight")
    if not W or not H:
        return []
    lines = []
    for shape in data.get("shapes", []):
        label = shape.get("label")
        if label not in classes:
            continue
        cls_id = classes.index(label)
        bbox = shape_to_bbox(shape)
        if not bbox:
            continue
        xmin, ymin, xmax, ymax = bbox
        # 归一化并裁剪到 [0,1]
        nxmin = max(0.0, min(1.0, xmin / W))
        nymin = max(0.0, min(1.0, ymin / H))
        nxmax = max(0.0, min(1.0, xmax / W))
        nymax = max(0.0, min(1.0, ymax / H))
        cx = (nxmin + nxmax) / 2.0
        cy = (nymin + nymax) / 2.0
        w = max(0.0, nxmax - nxmin)
        h = max(0.0, nymax - nymin)
        if w <= 0 or h <= 0:
            continue
        lines.append(f"{cls_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
    return lines


def find_image_for(json_path: Path):
    """根据 JSON 中的 imagePath 或同名文件寻找图片"""
    try:
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        ip = data.get("imagePath")
        if ip:
            cand = (json_path.parent / ip)
            if cand.exists():
                return cand
            cand = json_path.parent / Path(ip).name
            if cand.exists():
                return cand
    except Exception:
        pass
    stem = json_path.stem
    for ext in IMG_EXTS:
        cand = json_path.with_name(stem + ext)
        if cand.exists():
            return cand
    return None


def main():
    classes = load_classes(SRC_DIR / "classes.txt")
    if not classes:
        raise RuntimeError("classes.txt 为空或不存在")
    print("classes:", classes)

    # 清理输出
    if DST_DIR.exists():
        shutil.rmtree(DST_DIR)
    for sub in ["images/train", "images/val", "labels/train", "labels/val"]:
        (DST_DIR / sub).mkdir(parents=True, exist_ok=True)

    json_files = sorted(SRC_DIR.glob("*.json"))
    random.seed(SEED)
    random.shuffle(json_files)

    n_val = int(len(json_files) * VAL_RATIO)
    val_set = set(json_files[:n_val])

    stats = Counter()
    total_shapes = 0
    skipped_no_img = 0
    skipped_empty = 0

    for jp in json_files:
        img = find_image_for(jp)
        if img is None:
            skipped_no_img += 1
            continue
        lines = convert_one(jp, classes)
        if not lines:
            skipped_empty += 1
            continue
        split = "val" if jp in val_set else "train"
        # 复制图片
        dst_img = DST_DIR / "images" / split / img.name
        shutil.copy2(img, dst_img)
        # 写标签 (与图片同名 .txt)
        dst_lbl = DST_DIR / "labels" / split / (Path(img.name).stem + ".txt")
        with open(dst_lbl, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        stats[split] += 1
        total_shapes += len(lines)

    # 写 data.yaml
    yaml_path = DST_DIR / "data.yaml"
    yaml_content = (
        f"path: {DST_DIR.resolve()}\n"
        f"train: images/train\n"
        f"val: images/val\n"
        f"names:\n"
    )
    for i, c in enumerate(classes):
        yaml_content += f"  {i}: {c}\n"
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(yaml_content)

    print("---- 统计 ----")
    print("总 JSON:", len(json_files))
    print("train / val:", stats["train"], "/", stats["val"])
    print("总 box 标签:", total_shapes)
    print("跳过(无图片):", skipped_no_img)
    print("跳过(空标签/非检测框):", skipped_empty)
    print("data.yaml ->", yaml_path)


if __name__ == "__main__":
    main()
