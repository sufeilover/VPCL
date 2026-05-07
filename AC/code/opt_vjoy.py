# opt_vjoy.py
# 优化器（或任何脚本）→ vJoy 的极简独立实现
# 轴位和 G29_vjoy.py 保持一致：X=steer, Y=clutch, Z=throttle, RX=brake

from __future__ import annotations
from typing import Optional
import pyvjoy

# ─── 工具函数 ────────────────────────────────────────────
def _to_vjoy_axis_norm(val: float) -> int:
    """[-1, 1] → [0, 0x8000]（vJoy 期望的 15bit 区间）"""
    v = max(-1.0, min(1.0, float(val)))
    return int((v + 1.0) * 0x8000 / 2.0)

def _to_vjoy_axis_01(val: float) -> int:
    """[0, 1] → [0, 0x8000]"""
    v = max(0.0, min(1.0, float(val)))
    return _to_vjoy_axis_norm(v * 2.0 - 1.0)

def _lowpass(prev: float, cur: float, a: float) -> float:
    if a <= 0.0:
        return cur
    if a >= 1.0:
        return cur  # 保护
    return prev + a * (cur - prev)

# ─── 面向优化器的 vJoy 输出类 ────────────────────────────
class VJoyOut:
    """
    用法：
        vj = VJoyOut(device_id=1, smooth_alpha=0.0)
        vj.apply(steer, gas, brake, clutch=1.0)
    说明：
        - steer ∈ [-1,1]；gas/brake/clutch ∈ [0,1]
        - 轴位与 G29_vjoy 保持一致，避免混淆：
            X  = steer
            Y  = clutch
            Z  = throttle (gas)
            RX = brake
    """
    def __init__(self, device_id: int = 1, smooth_alpha: float = 0.0):
        self.vj = pyvjoy.VJoyDevice(device_id)
        self.a  = float(smooth_alpha)

        # 滤波缓存
        self._steer   = 0.0
        self._gas     = 0.0
        self._brake   = 0.0
        self._clutch  = 1.0

    def apply(self, steer: float, gas: float, brake: float, clutch: Optional[float] = None):
        """把一帧控制量写入 vJoy。clutch 可选，默认沿用上一次"""
        if clutch is None:
            clutch = self._clutch

        # 一阶低通（可选）
        self._steer  = _lowpass(self._steer,  float(steer), self.a)
        self._gas    = _lowpass(self._gas,    float(gas),   self.a)
        self._brake  = _lowpass(self._brake,  float(brake), self.a)
        self._clutch = _lowpass(self._clutch, float(clutch),self.a)
        # print(self._steer,self._gas,self._brake,self._clutch)

        # 映射与写入（轴位与 G29_vjoy.py 对齐）
        self.vj.set_axis(pyvjoy.HID_USAGE_X,  _to_vjoy_axis_norm(self._steer))   # 方向
        self.vj.set_axis(pyvjoy.HID_USAGE_Y,  _to_vjoy_axis_01(self._clutch))    # 离合
        self.vj.set_axis(pyvjoy.HID_USAGE_Z,  _to_vjoy_axis_01(self._gas))       # 油门
        self.vj.set_axis(pyvjoy.HID_USAGE_RX, _to_vjoy_axis_01(self._brake))     # 刹车

    # 如需扩展按钮/拨片，可加 set_button 接口；此处保持极简
