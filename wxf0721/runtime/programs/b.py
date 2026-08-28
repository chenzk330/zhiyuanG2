import time
from minth import Minth

G2 = Minth.G2()
G2.YOLO("wxf.pt")
G2.CHASSIS_CORRECT(px_to_meter=-130/50/1000)
G2.WAIST_CORRECT()
G2.ARMS("A2P1")
G2.ARMS("A2P2")
G2.ARMS("A2P3")
G2.ARMS("A2P4")
G2.ARMS("A2P5")
G2.GRIPPER({"left": -0.7, "right": -0.7})
time.sleep(1.5)
G2.ARMS("A2P6")