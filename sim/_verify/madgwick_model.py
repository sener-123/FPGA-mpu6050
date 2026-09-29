#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Madgwick AHRS + displacement  —— bit-exact Python 金标准模型
============================================================
用途：验证 Verilog 定点实现（rtl/madgwick_ahrs.v / rtl/displacement_calc.v）
     与浮点参考一致，并生成 inv_sqrt 的 LUT 与各种缩放常数。

定点约定（与 Verilog 一一对应）：
  - 四元数 q0..q3 : Q16.16，1.0 = 0x10000，有符号 32 位
  - 归一化加速度 : Q16.16（|a|=1.0 = 0x10000）
  - 陀螺仪       : rad/s，Q16.16
  - 欧拉角       : 度，Q16.16
  - 世界线性加速度: m/s^2，Q16.16
  - 位置         : m，Q16.16
  - beta = 0.1 rad/s -> 0x199A (6554)
  - dt = 0.002 s -> DT = 131 (0.002*65536 = 131.07)
  - GYRO_SCALE : raw -> rad/s,  = raw*572224 >> 16   (8.73144/65536)
  - G_CONST    : 9.80665 * 65536 = 642688
"""
import math

M32 = (1 << 32) - 1
M64 = (1 << 64) - 1

def s32(x):
    x &= M32
    return x - (1 << 32) if x & (1 << 31) else x

def s64(x):
    x &= M64
    return x - (1 << 64) if x & (1 << 63) else x

# Q16.16 signed multiply -> Q16.16  (32x32 -> 64 -> >>16, 算术移位)
def mulq(a, b):
    return s32(s64(s32(a) * s32(b)) >> 16)

# Q16.16 signed multiply -> keep full 64-bit (Q32.32)
def mul64(a, b):
    return s64(s32(a) * s32(b))

# ============================================================
# 1. inv_sqrt LUT 生成 + 验证
#    目标：y = 1/sqrt(x)  Q16.16,  x 为 Q16.16 正整数
#    实现：归一化 -> LUT 初值 -> 2 次牛顿迭代
# ============================================================
def leading_zeros32(x):
    if x == 0:
        return 32
    return 31 - x.bit_length() + 1  # 32 - bit_length(x)

def build_lut():
    """16 项 LUT：g[m4] = round(65536 / sqrt(1 + (m4+0.5)/16))"""
    lut = []
    for m4 in range(16):
        M = 1.0 + (m4 + 0.5) / 16.0
        g = int(round(65536.0 / math.sqrt(M)))
        lut.append(g)
    return lut

LUT = build_lut()

def inv_sqrt_fixed(x):
    """x: Q16.16 正整数 -> y: Q16.16 = 1/sqrt(x_real)"""
    assert 0 < x
    x &= M32
    if x < 4:
        x = 4   # 与 Verilog 一致：钳位，避免 y^2>>16 溢出 32 位
    lz = leading_zeros32(x)
    s = 31 - lz                      # MSB 位置
    xh = (x << lz) & M32             # xh 最高位 = bit31
    m4 = (xh >> 27) & 0xF            # 4 位尾数（leading 1 之后 4 位）
    g = LUT[m4]
    # y = 2^(8 - s/2) * g
    # s/2 = a + odd*0.5  =>  8 - s/2 = (8-a) - odd*0.5
    a = s >> 1
    if s & 1:
        shift = 7 - a
        y = mulq(g, 92682)           # g * sqrt(2)  Q16.16
    else:
        shift = 8 - a
        y = g
    if shift >= 0:
        y = (y << shift) & M32
    else:
        y = (y >> (-shift)) & M32
    y &= M32
    # 牛顿迭代：y <- y*(3 - x*y^2)/2（64 位中间量，与 Verilog 一致，不截断到 32 位）
    for _ in range(2):
        y2_64  = (y * y) >> 16            # 64 位
        xy2_64 = (x * y2_64) >> 16
        y = ((y * (0x30000 - xy2_64)) >> 17) & M32
    return y & M32

def test_inv_sqrt():
    print("=== inv_sqrt LUT ===")
    print(LUT)
    worst = 0.0
    for x in [0x8000, 0x10000, 0x18000, 0x20000, 0x40000, 0x80000,
              0x100000, 0x200000, 0x400000, 0x800000, 0x1000000, 0xFFFF]:
        yf = inv_sqrt_fixed(x)
        yref = 1.0 / math.sqrt(x / 65536.0)
        y_float = yf / 65536.0
        err = abs(y_float - yref) / yref
        worst = max(worst, err)
        print(f"x=0x{x:06X} ({x/65536.0:.4f})  y=0x{yf:04X} ({y_float:.6f})  ref={yref:.6f}  relerr={err:.2e}")
    print(f"worst relerr = {worst:.2e}")
    assert worst < 1e-4, "inv_sqrt accuracy too low"
    print("inv_sqrt OK\n")

# ============================================================
# 2. 浮点 Madgwick 参考实现（对照 C 代码 MadgwickAHRSupdateIMU）
# ============================================================
class FloatMadgwick:
    def __init__(self, beta=0.1, freq=500.0):
        self.beta = beta
        self.freq = freq
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0

    def inv_sqrt(self, x):
        return 1.0 / math.sqrt(x) if x > 0 else 0.0

    def update(self, gx, gy, gz, ax, ay, az):
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        # gyro 已是 rad/s
        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        if not (ax == 0 and ay == 0 and az == 0):
            recip = self.inv_sqrt(ax*ax + ay*ay + az*az)
            ax *= recip; ay *= recip; az *= recip
            _2q0 = 2*q0; _2q1 = 2*q1; _2q2 = 2*q2; _2q3 = 2*q3
            _4q0 = 4*q0; _4q1 = 4*q1; _4q2 = 4*q2
            _8q1 = 8*q1; _8q2 = 8*q2
            q0q0 = q0*q0; q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3
            s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay
            s1 = _4q1*q3q3 - _2q3*ax + 4*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az
            s2 = 4*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q2*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az
            s3 = 4*q1q1*q3 - _2q1*ax + 4*q2q2*q3 - _2q2*ay
            recip = self.inv_sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3)
            s0 *= recip; s1 *= recip; s2 *= recip; s3 *= recip
            qDot1 -= self.beta * s0
            qDot2 -= self.beta * s1
            qDot3 -= self.beta * s2
            qDot4 -= self.beta * s3

        q0 += qDot1 / self.freq
        q1 += qDot2 / self.freq
        q2 += qDot3 / self.freq
        q3 += qDot4 / self.freq

        recip = self.inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        q0 *= recip; q1 *= recip; q2 *= recip; q3 *= recip
        self.q0, self.q1, self.q2, self.q3 = q0, q1, q2, q3

    def euler(self):
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        roll = math.atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2))
        pitch = math.asin(2*(q0*q2 - q1*q3))
        yaw = math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

# ============================================================
# 3. 定点 Madgwick（与 Verilog 一一对应）
# ============================================================
GYRO_SCALE = 572224
DT = 131            # 0.002 * 65536
BETA = 6554         # 0.1 * 65536
G_CONST = 642688    # 9.80665 * 65536

class FixedMadgwick:
    def __init__(self):
        self.q0 = 0x10000
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0

    def update(self, gx_raw, gy_raw, gz_raw, ax_raw, ay_raw, az_raw):
        """raw 为 16 位有符号；输出(roll,pitch,yaw 度 Q16.16, 世界线性加速度 m/s^2 Q16.16, motion)"""
        # --- 陀螺 raw -> rad/s Q16.16 ---
        gx = s32(s64(s32(gx_raw) * GYRO_SCALE) >> 16)
        gy = s32(s64(s32(gy_raw) * GYRO_SCALE) >> 16)
        gz = s32(s64(s32(gz_raw) * GYRO_SCALE) >> 16)
        # --- 加速度 raw -> g Q16.16 (raw<<2) ---
        ax_g = s32(s32(ax_raw) << 2)
        ay_g = s32(s32(ay_raw) << 2)
        az_g = s32(s32(az_raw) << 2)
        # --- 加速度归一化 ---
        n2 = (mul64(ax_g, ax_g) + mul64(ay_g, ay_g) + mul64(az_g, az_g)) & M64
        x_norm = s32(n2 >> 16) & M32     # Q16.16 |a|^2 (g^2)
        if x_norm == 0:
            x_norm = 1
        recip = inv_sqrt_fixed(x_norm)
        axn = mulq(ax_g, recip)
        ayn = mulq(ay_g, recip)
        azn = mulq(az_g, recip)

        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3

        # --- 辅助量 ---
        _2q0 = s32(2 * q0)   # 2*q0
        _2q1 = s32(2 * q1)
        _2q2 = s32(2 * q2)
        _2q3 = s32(2 * q3)
        _4q0 = s32(4 * q0)
        _4q1 = s32(4 * q1)
        _4q2 = s32(4 * q2)
        _8q1 = s32(8 * q1)
        _8q2 = s32(8 * q2)
        q0q0 = mulq(q0, q0)
        q1q1 = mulq(q1, q1)
        q2q2 = mulq(q2, q2)
        q3q3 = mulq(q3, q3)

        # --- 梯度 s0..s3 ---
        s0 = s32(mulq(_4q0, q2q2) + mulq(_2q2, axn) + mulq(_4q0, q1q1) - mulq(_2q1, ayn))
        s1 = s32(mulq(_4q1, q3q3) - mulq(_2q3, axn) + mulq(mulq(_4q0, q0), q1)
                 - mulq(_2q0, ayn) - _4q1 + mulq(_8q1, q1q1) + mulq(_8q1, q2q2) + mulq(_4q1, azn))
        s2 = s32(mulq(mulq(_4q0, q0), q2) + mulq(_2q0, axn) + mulq(_4q2, q3q3)
                 - mulq(_2q2, ayn) - _4q2 + mulq(_8q2, q1q1) + mulq(_8q2, q2q2) + mulq(_4q2, azn))
        s3 = s32(mulq(mulq(_4q1, q1), q3) - mulq(_2q1, axn) + mulq(mulq(_4q2, q2), q3) - mulq(_2q2, ayn))

        # --- s 归一化 ---
        s_n2 = (mul64(s0, s0) + mul64(s1, s1) + mul64(s2, s2) + mul64(s3, s3)) & M64
        s_x = s32(s_n2 >> 16) & M32
        if s_x == 0:
            s_x = 1
        s_rec = inv_sqrt_fixed(s_x)
        s0 = mulq(s0, s_rec); s1 = mulq(s1, s_rec); s2 = mulq(s2, s_rec); s3 = mulq(s3, s_rec)

        # --- qDot = 0.5*q⊗w - beta*s ---
        # 0.5*( -q1*gx - q2*gy - q3*gz ) = ( -q1*gx - q2*gy - q3*gz ) >> 1
        qd0 = s32(s32((-mulq(q1, gx) - mulq(q2, gy) - mulq(q3, gz)) >> 1) - mulq(BETA, s0))
        qd1 = s32(s32(( mulq(q0, gx) + mulq(q2, gz) - mulq(q3, gy)) >> 1) - mulq(BETA, s1))
        qd2 = s32(s32(( mulq(q0, gy) - mulq(q1, gz) + mulq(q3, gx)) >> 1) - mulq(BETA, s2))
        qd3 = s32(s32(( mulq(q0, gz) + mulq(q1, gy) - mulq(q2, gx)) >> 1) - mulq(BETA, s3))

        # --- 积分 q += qDot * dt ---
        q0 = s32(q0 + mulq(qd0, DT))
        q1 = s32(q1 + mulq(qd1, DT))
        q2 = s32(q2 + mulq(qd2, DT))
        q3 = s32(q3 + mulq(qd3, DT))

        # --- 四元数归一化：快速归一化 q *= (3-|q|^2)/2 ---
        qn2 = (mul64(q0, q0) + mul64(q1, q1) + mul64(q2, q2) + mul64(q3, q3)) & M64
        qnorm = s32(qn2 >> 16) & M32          # |q|^2  Q16.16 (~1.0)
        factor = s32(((3 << 16) - qnorm) >> 1)  # (3-|q|^2)/2
        q0 = mulq(q0, factor)
        q1 = mulq(q1, factor)
        q2 = mulq(q2, factor)
        q3 = mulq(q3, factor)

        self.q0, self.q1, self.q2, self.q3 = q0, q1, q2, q3

        # --- 欧拉角（度，Q16.16） ---
        wf = q0/65536.0; xf = q1/65536.0; yf = q2/65536.0; zf = q3/65536.0
        roll_deg  = math.degrees(math.atan2(2*(wf*xf + yf*zf), 1 - 2*(xf*xf + yf*yf)))
        pitch_deg = math.degrees(math.asin(2*(wf*yf - xf*zf)))
        yaw_deg   = math.degrees(math.atan2(2*(wf*zf + xf*yf), 1 - 2*(yf*yf + zf*zf)))
        roll  = int(round(roll_deg * 65536.0))
        pitch = int(round(pitch_deg * 65536.0))
        yaw   = int(round(yaw_deg * 65536.0))

        # --- 世界线性加速度（旋转到世界系，去重力） ---
        # R^T（body->earth，Convention B）:
        # [ 1-2(y^2+z^2)  2(xy-wz)       2(xz+wy)      ]
        # [ 2(xy+wz)      1-2(x^2+z^2)   2(yz-wx)      ]
        # [ 2(xz-wy)      2(yz+wx)       1-2(x^2+y^2)  ]
        w, x, y, z = q0, q1, q2, q3
        r00 = s32(0x10000 - 2*mulq(y, y) - 2*mulq(z, z))
        r01 = s32(2*mulq(x, y) - 2*mulq(w, z))
        r02 = s32(2*mulq(x, z) + 2*mulq(w, y))
        r10 = s32(2*mulq(x, y) + 2*mulq(w, z))
        r11 = s32(0x10000 - 2*mulq(x, x) - 2*mulq(z, z))
        r12 = s32(2*mulq(y, z) - 2*mulq(w, x))
        r20 = s32(2*mulq(x, z) - 2*mulq(w, y))
        r21 = s32(2*mulq(y, z) + 2*mulq(w, x))
        r22 = s32(0x10000 - 2*mulq(x, x) - 2*mulq(y, y))

        awx = s32(mulq(r00, ax_g) + mulq(r01, ay_g) + mulq(r02, az_g))          # g
        awy = s32(mulq(r10, ax_g) + mulq(r11, ay_g) + mulq(r12, az_g))          # g
        awz = s32(mulq(r20, ax_g) + mulq(r21, ay_g) + mulq(r22, az_g) - 0x10000)  # g，去重力

        # 转 m/s^2
        awx = mulq(awx, G_CONST)
        awy = mulq(awy, G_CONST)
        awz = mulq(awz, G_CONST)

        # --- motion 检测：|a_body|² 是否在 [0.95², 1.05²] g²（与 Verilog 一致） ---
        motion = 0 if (59146 <= x_norm <= 72253) else 1

        return roll, pitch, yaw, awx, awy, awz, motion

    def euler_q16(self):
        q0 = self.q0/65536.0; q1 = self.q1/65536.0
        q2 = self.q2/65536.0; q3 = self.q3/65536.0
        roll = math.degrees(math.atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)))
        pitch = math.degrees(math.asin(2*(q0*q2 - q1*q3)))
        yaw = math.degrees(math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)))
        return roll, pitch, yaw

# ============================================================
# 4. 位移（定点，与 Verilog 对应）
# ============================================================
class FixedDisplacement:
    def __init__(self):
        self.vx = self.vy = self.vz = 0
        self.px = self.py = self.pz = 0

    def update(self, awx, awy, awz, motion):
        # 静止（|a|~1g）时速度清零，位置保持
        if motion == 0:
            self.vx = self.vy = self.vz = 0
            return self.px, self.py, self.pz, self.vx, self.vy, self.vz
        # v += a*dt；p += v_old*dt（与 Verilog 非阻塞一致，位置用更新前速度）
        vx_old, vy_old, vz_old = self.vx, self.vy, self.vz
        self.vx = s32(self.vx + mulq(awx, DT))
        self.vy = s32(self.vy + mulq(awy, DT))
        self.vz = s32(self.vz + mulq(awz, DT))
        self.px = s32(self.px + mulq(vx_old, DT))
        self.py = s32(self.py + mulq(vy_old, DT))
        self.pz = s32(self.pz + mulq(vz_old, DT))
        return self.px, self.py, self.pz, self.vx, self.vy, self.vz

# ============================================================
# 5. 场景测试
# ============================================================
def scenario():
    print("=== 场景测试 ===")
    fm = FixedMadgwick()
    ff = FloatMadgwick(beta=0.1, freq=500.0)
    fd = FixedDisplacement()

    # 静止平放 2s：accel = (0,0,1g)，gyro = 0
    axr, ayr, azr = 0, 0, 16384
    gxr, gyr, gzr = 0, 0, 0
    for i in range(1000):
        fm.update(gxr, gyr, gzr, axr, ayr, azr)
        ff.update(0, 0, 0, 0, 0, 1.0)
    r, p, y = fm.euler_q16()
    rf, pf, yf = ff.euler()
    print(f"静止平放: roll={r:.3f} (ref {rf:.3f})  pitch={p:.3f} (ref {pf:.3f})  yaw={y:.3f} (ref {yf:.3f})")

    # 绕 X 轴倾斜 30°：重力分解  ay=sin30*g, az=cos30*g
    tilt = math.radians(30)
    axr, ayr, azr = 0, int(round(16384 * math.sin(tilt))), int(round(16384 * math.cos(tilt)))
    for i in range(3000):
        roll, pitch, yaw, awx, awy, awz, motion = fm.update(gxr, gyr, gzr, axr, ayr, azr)
        ff.update(0, 0, 0, 0, math.sin(tilt), math.cos(tilt))
    r, p, y = fm.euler_q16()
    rf, pf, yf = ff.euler()
    print(f"绕X倾斜30°: roll={r:.3f} (ref {rf:.3f})  pitch={p:.3f} (ref {pf:.3f})  yaw={y:.3f} (ref {yf:.3f})")
    assert abs(r - 30) < 1.0, f"roll 偏差过大: {r}"
    assert abs(p) < 1.0, f"pitch 应接近0: {p}"

    # 位移测试：静止时位置应保持 0
    for i in range(100):
        roll, pitch, yaw, awx, awy, awz, motion = fm.update(gxr, gyr, gzr, axr, ayr, azr)
        px, py, pz, vx, vy, vz = fd.update(awx, awy, awz, motion)
    print(f"静止时位移: px={px/65536.0:.6f} py={py/65536.0:.6f} pz={pz/65536.0:.6f} (应≈0)  motion={motion}")
    assert abs(px) < 0x100 and abs(py) < 0x100, "静止时位移漂移过大"

    print("场景测试通过\n")

if __name__ == "__main__":
    test_inv_sqrt()
    scenario()
    print("=== 缩放常数 ===")
    print(f"GYRO_SCALE={GYRO_SCALE}  DT={DT}  BETA={BETA}  G_CONST={G_CONST}")
