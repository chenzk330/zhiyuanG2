import random
import time
from minth import Minth

G2 = Minth.G2()

while True:
    # 1. 腰部旋转：随机弧度值 [-0.05, 0.05]
    waist_val = random.uniform(-0.09, 0.09)
    G2.JOINT("idx05_body_joint5", value=waist_val)

    # 2. 随机位移量 a: [-0.2, 0.2]
    a = random.uniform(-0.15, 0.15)

    # 3. 随机方向选择 b: 1 或 2
    b = random.randint(1, 2)

    if b == 1:
        # x 方向移动 a，再移回 -a
        G2.REL({"x": a})
        G2.REL({"x": -a})
    else:
        # y 方向移动 a，再移回 -a
        G2.REL({"y": a})
        G2.REL({"y": -a})

    # 继续重复
