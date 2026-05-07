# udp2_api.py
# -*- coding: utf-8 -*-
import socket
import struct
from typing import Dict, Tuple, Optional, Any

# 与发送端 acUpdate() 的 PackingString 完全一致（小端 + 固定字段顺序）
PACK_FORMAT = (
    "<"          # little-endian
    "l"          # packetId
    "l"          # LapTime
    "f"          # DriveTrainSpeed
    "f"          # TurboBoost
    "l"          # LapInvalidated
    "f"          # Caster
    "f"          # NormalizedSplinePosition
    "ffff"       # ToeInDeg[4]
    "ffff"       # CurrentTyresCoreTemp[4]
    "ffff"       # DynamicPressure[4]
    "f" "f" "f"  # Aero: CD, CL_Front, CL_Rear
    "ffff"       # SlipRatio[4]
    "ffff"       # SlipAngle[4]
    "ffff"       # NdSlip[4]
    "ffff"       # TyreSlip[4]
    "ffff"       # Mz[4]
    "ffff"       # TyreRadius[4]
    "ffff"       # TyreLoadedRadius[4]
)

FIELDS = [
    "packetId",
    "LapTime",
    "DriveTrainSpeed", "TurboBoost", "LapInvalidated",
    "Caster", "NormalizedSplinePosition",
    "ToeInDeg_FL", "ToeInDeg_FR", "ToeInDeg_RL", "ToeInDeg_RR",
    "TyreCoreTemp_FL", "TyreCoreTemp_FR", "TyreCoreTemp_RL", "TyreCoreTemp_RR",
    "DynamicPressure_FL", "DynamicPressure_FR", "DynamicPressure_RL", "DynamicPressure_RR",
    "Aero_CD", "Aero_CL_Front", "Aero_CL_Rear",
    "SlipRatio_FL", "SlipRatio_FR", "SlipRatio_RL", "SlipRatio_RR",
    "SlipAngle_FL", "SlipAngle_FR", "SlipAngle_RL", "SlipAngle_RR",
    "NdSlip_FL", "NdSlip_FR", "NdSlip_RL", "NdSlip_RR",
    "TyreSlip_FL", "TyreSlip_FR", "TyreSlip_RL", "TyreSlip_RR",
    "Mz_FL", "Mz_FR", "Mz_RL", "Mz_RR",
    "TyreRadius_FL", "TyreRadius_FR", "TyreRadius_RL", "TyreRadius_RR",
    "TyreLoadedRadius_FL", "TyreLoadedRadius_FR", "TyreLoadedRadius_RL", "TyreLoadedRadius_RR",
]

PACK_SIZE = struct.calcsize(PACK_FORMAT)

def open_udp2_receiver(ip: str = "0.0.0.0", port: int = 27152,
                       timeout: Optional[float] = None) -> socket.socket:
    """
    打开并绑定一个 UDP socket 用于持续接收。
    - ip: 监听地址（默认 0.0.0.0）
    - port: 监听端口（默认 27152）
    - timeout: 可选超时（秒）。None 表示阻塞等待。
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    if timeout is not None:
        sock.settimeout(timeout)
    return sock



