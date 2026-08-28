import time
from minth import Minth

G2 = Minth.G2(timeout=120)

# 依次导航到 0 → 1 → 2 → 0 号点
print("[wzd] 第 1/4 步：导航到 0 号点")
if not G2.GO(0):
    print("[wzd] ✗ 导航到 0 号点失败，退出程序")
    G2.close()
    raise SystemExit
time.sleep(2)

print("[wzd] 第 2/4 步：导航到 1 号点")
if not G2.GO(1):
    print("[wzd] ✗ 导航到 1 号点失败，退出程序")
    G2.close()
    raise SystemExit
time.sleep(2)

print("[wzd] 第 3/4 步：导航到 2 号点")
if not G2.GO(2):
    print("[wzd] ✗ 导航到 2 号点失败，退出程序")
    G2.close()
    raise SystemExit
time.sleep(2)

print("[wzd] 第 4/4 步：导航到 0 号点")
if not G2.GO(0):
    print("[wzd] ✗ 导航到 0 号点失败，退出程序")
    G2.close()
    raise SystemExit
time.sleep(2)

print("[wzd] ✓ 全部点位已按顺序走完")
G2.close()



