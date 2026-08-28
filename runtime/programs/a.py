import time
import agibot_gdk


def main():
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK初始化失败")
        return
    print("GDK初始化成功")

    try:
        robot = agibot_gdk.Robot()
        time.sleep(2)  # 等待 Robot 初始化完成

        # 构造左夹爪控制请求（omnipicker 类型，1 个关节）
        # position 取值范围 [-0.785, 0]：0 为闭合，-0.785 为完全张开
        joint_states = agibot_gdk.JointStates()
        joint_states.group = "left_tool"
        joint_states.target_type = "omnipicker"

        joint_state = agibot_gdk.JointState()
        joint_state.position = 0.0  # 闭合
        joint_states.states = [joint_state]
        joint_states.nums = len(joint_states.states)

        print("正在夹紧左夹爪...")
        result = robot.move_ee_pos(joint_states)
        print(f"左夹爪控制完成 (返回值: {result})")

        time.sleep(2)  # 等待动作完成
