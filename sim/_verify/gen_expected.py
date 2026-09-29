#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成与 tb_madgwick.v 同场景的期望输出（四元数 + 欧拉角 + 位移，Q16.16）"""
from madgwick_model import FixedMadgwick, FixedDisplacement

fm = FixedMadgwick()
fd = FixedDisplacement()
with open("expected.txt", "w") as f:
    for i in range(4000):
        if i < 1000:
            ax, ay, az = 0, 0, 16384          # 静止平放
        else:
            ax, ay, az = 0, 8192, 14189       # 绕 X 倾斜 30°
        r, p, y, awx, awy, awz, motion = fm.update(0, 0, 0, ax, ay, az)
        px, py, pz, vx, vy, vz = fd.update(awx, awy, awz, motion)
        # 索引 + 四元数(Q16.16) + 欧拉角(Q16.16, 浮点精确) + 位移(Q16.16)
        f.write(f"{i} {fm.q0} {fm.q1} {fm.q2} {fm.q3} {r} {p} {y} {px} {py} {pz}\n")

print("written expected.txt")
# 关键帧预览（度）
fm2 = FixedMadgwick()
for i in range(4000):
    ax, ay, az = (0, 0, 16384) if i < 1000 else (0, 8192, 14189)
    fm2.update(0, 0, 0, ax, ay, az)
    if i in (499, 999, 1999, 3999):
        q0, q1, q2, q3 = fm2.q0, fm2.q1, fm2.q2, fm2.q3
        print(f"frame={i} q=({q0},{q1},{q2},{q3})  "
              f"roll={fm2.euler_q16()[0]:.3f} pitch={fm2.euler_q16()[1]:.3f} yaw={fm2.euler_q16()[2]:.3f}")
