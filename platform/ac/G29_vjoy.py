import time
from typing import Callable, Optional, Dict, Any

import pygame
import pyvjoy


# ─── 工具函数 ────────────────────────────────────────────
def to_vjoy_axis_norm(val: float) -> int:
    """[-1, 1] → [0, 0x8000]"""
    v = max(-1.0, min(1.0, val))
    return int((v + 1) / 2 * 0x8000)

def to_vjoy_axis_01(val: float) -> int:
    """[0, 1] → [0, 0x8000]"""
    return to_vjoy_axis_norm(val * 2 - 1)

def lowpass(prev: float, current: float, alpha: float = 0.3) -> float:
    """一阶低通滤波"""
    return prev + alpha * (current - prev)


def main(on_update: Optional[Callable[[Dict[str, Any]], None]] = None):
    # 初始化 vJoy（设备 1）
    vj = pyvjoy.VJoyDevice(1)

    # 初始化 pygame / 查找 G29
    pygame.init()
    pygame.joystick.init()

    g29_joystick = None
    for i in range(pygame.joystick.get_count()):
        joy = pygame.joystick.Joystick(i)
        joy.init()
        name = joy.get_name().lower()
        if "vjoy" in name:
            continue
        if "g29" in name or "logitech" in name or i == 1:
            g29_joystick = joy
            break

    if g29_joystick is None:
        print("未找到 G29 输入设备，请手动指定索引： g29_joystick = pygame.joystick.Joystick(n)")
        raise SystemExit(1)

    print(f"使用的 G29: {g29_joystick.get_name()} (ID {g29_joystick.get_id()})")
    print("开始桥接 G29 -> vJoy（无 UDP，Ctrl+C 退出）")

    # 滤波初始值
    filt_steer = filt_clutch = filt_throttle = filt_brake = 0.0

    try:
        while True:
            pygame.event.pump()

            # 轴与按键
            raw_steer    = g29_joystick.get_axis(0)
            raw_clutch   = g29_joystick.get_axis(3)
            raw_throttle = g29_joystick.get_axis(1)
            raw_brake    = g29_joystick.get_axis(2)
            left_paddle  = g29_joystick.get_button(5)
            right_paddle = g29_joystick.get_button(4)

            # 归一化到 [0,1]（某些方向盘为反向，这里做了 1 - x）
            clutch   = 1 - (raw_clutch   + 1.0) / 2.0
            throttle = 1 - (raw_throttle + 1.0) / 2.0
            brake    = 1 - (raw_brake    + 1.0) / 2.0

            # 低通滤波
            filt_steer    = lowpass(filt_steer,    raw_steer)
            filt_clutch   = lowpass(filt_clutch,   clutch)
            filt_throttle = lowpass(filt_throttle, throttle)
            filt_brake    = lowpass(filt_brake,    brake)

            # 写入 vJoy 轴/按钮
            vj.set_axis(pyvjoy.HID_USAGE_X,  to_vjoy_axis_norm(filt_steer))   # 方向
            vj.set_axis(pyvjoy.HID_USAGE_Y,  to_vjoy_axis_01(filt_clutch))    # 离合
            vj.set_axis(pyvjoy.HID_USAGE_Z,  to_vjoy_axis_01(filt_throttle))  # 油门
            vj.set_axis(pyvjoy.HID_USAGE_RX, to_vjoy_axis_01(filt_brake))     # 刹车
            vj.set_button(1, left_paddle)                                     # 左拨片
            vj.set_button(2, right_paddle)                                    # 右拨片

            # 可选：把最新状态回调给外部
            if on_update:
                state = {
                    "steer":       float(filt_steer),
                    "clutch":      float(filt_clutch),
                    "brake":       float(filt_brake),
                    "throttle":    float(filt_throttle),
                    "left_paddle": int(left_paddle),
                    "right_paddle":int(right_paddle),
                }
                on_update(state)

            time.sleep(0.003)  # 约 300+ Hz，按需调整
    except KeyboardInterrupt:
        print("\n退出。")


if __name__ == "__main__":
    # 示例：打印实时状态（可删除）
    def print_state(s: Dict[str, Any]):
        print(f"S:{s['steer']:+.3f}  C:{s['clutch']:.3f}  B:{s['brake']:.3f}  "
              f"T:{s['throttle']:.3f}  L:{s['left_paddle']}  R:{s['right_paddle']}")
    main(on_update=print_state)
