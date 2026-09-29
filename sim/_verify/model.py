import math

KP = 3753
GYRO_C = 572
ONE = 32768  # Q16.15

def s32(v):
    v &= 0xFFFFFFFF
    if v >= 0x80000000: v -= 0x100000000
    return v

def s64(v):
    v &= 0xFFFFFFFFFFFFFFFF
    if v >= 0x8000000000000000: v -= 0x10000000000000000
    return v

def arith_shift(v, n):
    # Python >> already arithmetic for negative ints
    return v >> n

class Quat:
    def __init__(self):
        self.q0 = ONE; self.q1 = 0; self.q2 = 0; self.q3 = 0
        self.r31 = 0; self.r32 = 0; self.r33 = ONE; self.r21 = 0; self.r11 = ONE

    def step(self, ax, ay, az, gx, gy, gz):
        # ax,ay,az: Q16.16 (1g=65536). gx..: raw LSB (131 per deg/s)
        # ---- A0: accel norm ----
        xs = ax*ax + ay*ay + az*az
        sh = s32(xs >> 16)
        acc_ok = (sh > 16384) and (sh < 147456)
        Y = 65536
        for _ in range(2):
            t_a1 = s32((sh*Y) >> 16)
            t_a2 = s32((t_a1*Y) >> 16)
            r_a = (196608 - t_a2) if (t_a2 < 196608) else 0
            if acc_ok:
                Y = s32((Y*r_a) >> 17)
        axn = s32((ax*Y) >> 16)
        ayn = s32((ay*Y) >> 16)
        azn = s32((az*Y) >> 16)
        # ---- A: gravity v ----
        q0,q1,q2,q3 = self.q0,self.q1,self.q2,self.q3
        t1 = s32((q1*q3)>>15); t2 = s32((q0*q2)>>15); vx = s32((t1-t2)<<1)
        t3 = s32((q2*q3)>>15); t4 = s32((q0*q1)>>15); vy = s32((t3+t4)<<1)
        t5 = s32((q1*q1)>>15); t6 = s32((q2*q2)>>15); vz = s32(ONE - ((t5+t6)<<1))
        # ---- B: error e = a x v ----
        if acc_ok:
            ex = s32((ayn*vz)>>15 - (azn*vy)>>15)
            ey = s32((azn*vx)>>15 - (axn*vz)>>15)
            ez = s32((axn*vy)>>15 - (ayn*vx)>>15)
        else:
            ex = ey = ez = 0
        # ---- C: wc = gyro + Kp e ----
        wxc = s32(gx + ((ex*KP)>>16))
        wyc = s32(gy + ((ey*KP)>>16))
        wzc = s32(gz + ((ez*KP)>>16))
        # ---- D: ws = wc * 572 (low 32 bits) ----
        wsx = s32((wxc*GYRO_C) & 0xFFFFFFFF)
        wsy = s32((wyc*GYRO_C) & 0xFFFFFFFF)
        wsz = s32((wzc*GYRO_C) & 0xFFFFFFFF)
        # ---- E: dq ----
        u1 = s32((q1*wsx)>>32); u2 = s32((q2*wsy)>>32); u3 = s32((q3*wsz)>>32)
        dq0 = s32(-(u1+u2+u3))
        u4 = s32((q0*wsx)>>32); u5 = s32((q3*wsy)>>32); u6 = s32((q2*wsz)>>32)
        dq1 = s32(u4-u5+u6)
        u7 = s32((q3*wsx)>>32); u8 = s32((q0*wsy)>>32); u9 = s32((q1*wsz)>>32)
        dq2 = s32(u7+u8-u9)
        u10= s32((q2*wsx)>>32);u11= s32((q1*wsy)>>32);u12= s32((q0*wsz)>>32)
        dq3 = s32(-u10+u11+u12)
        q0n = s32(q0+dq0); q1n = s32(q1+dq1); q2n = s32(q2+dq2); q3n = s32(q3+dq3)
        # ---- F: normalize ----
        qsq = q0n*q0n + q1n*q1n + q2n*q2n + q3n*q3n
        d = (1<<30) - qsq
        scale = s32((1<<30) + (d>>1))
        self.q0 = s32((q0n*scale)>>30)
        self.q1 = s32((q1n*scale)>>30)
        self.q2 = s32((q2n*scale)>>30)
        self.q3 = s32((q3n*scale)>>30)
        # ---- G: rotation matrix (uses OLD q for r31..r33, and old q for r21/r11) ----
        self.r31 = vx; self.r32 = vy; self.r33 = vz
        t28 = s32((q3*q3)>>15); t26 = s32((q1*q2)>>15); t27 = s32((q0*q3)>>15)
        self.r21 = s32((t26+t27)<<1)
        self.r11 = s32(ONE - ((t6+t28)<<1))
        return dict(acc_ok=acc_ok, sh=sh, Y=Y, axn=axn, ayn=ayn, azn=azn,
                    vx=vx, vy=vy, vz=vz, ex=ex, ey=ey, ez=ez,
                    wxc=wxc, wyc=wyc, wzc=wzc, wsx=wsx, wsy=wsy, wsz=wsz,
                    dq0=dq0,dq1=dq1,dq2=dq2,dq3=dq3)

def atan2_deg(y, x):
    return math.degrees(math.atan2(y, x))

def run_attitude_scenario():
    q = Quat()
    # tb_attitude: frames 0..599 static az=16384; frame 600..2599 roll30 (ay=8192,az=14190)
    # accel input to quat = (raw*8 sum >>1) => raw*4  (Q16.16 = raw*4 since raw is 16384 LSB/g)
    # 8-frame avg of constant raw => sum=8*raw; >>1 => 4*raw. So ax_q16 = raw*4.
    roll = pitch = yaw = 0.0
    results = []
    for f in range(0, 2600):
        if f < 600:
            ax_raw, ay_raw, az_raw = 0, 0, 16384
        else:
            ax_raw, ay_raw, az_raw = 0, 8192, 14190
        gx_raw = gy_raw = gz_raw = 0
        ax_q16 = ax_raw*4; ay_q16 = ay_raw*4; az_q16 = az_raw*4
        dbg = q.step(ax_q16, ay_q16, az_q16, gx_raw, gy_raw, gz_raw)
        r = atan2_deg(q.r32, q.r33)
        p = atan2_deg(-q.r31, q.r33)
        y = atan2_deg(q.r21, q.r11)
        if f in (599, 650, 700, 800, 1000, 1500, 2000, 2599):
            print(f"frame {f}: roll={r:7.2f} pitch={p:7.2f} yaw={y:7.2f} | R32={q.r32} R33={q.r33} R31={q.r31}")
            print(f"         q=({q.q0},{q.q1},{q.q2},{q.q3}) acc_ok={dbg['acc_ok']} sh={dbg['sh']} Y={dbg['Y']}")
    return q

if __name__ == "__main__":
    q = run_attitude_scenario()
    print("final roll Q16.16 =", round(math.atan2(q.r32, q.r33)*65536))
