import math
KP=3753; GYRO_C=572; ONE=32768
def s32(v):
    v &= 0xFFFFFFFF
    if v>=0x80000000: v-=0x100000000
    return v
class Quat:
    def __init__(self): self.q0=ONE;self.q1=0;self.q2=0;self.q3=0
    def step(self,ax,ay,az,gx,gy,gz):
        q0,q1,q2,q3=self.q0,self.q1,self.q2,self.q3
        xs=ax*ax+ay*ay+az*az; sh=s32(xs>>16)
        acc_ok=(sh>16384)and(sh<147456); Y=65536
        for _ in range(2):
            ta1=s32((sh*Y)>>16); ta2=s32((ta1*Y)>>16)
            ra=(196608-ta2) if ta2<196608 else 0
            if acc_ok: Y=s32((Y*ra)>>17)
        axn=s32((ax*Y)>>16); ayn=s32((ay*Y)>>16); azn=s32((az*Y)>>16)
        t1=s32((q1*q3)>>15);t2=s32((q0*q2)>>15);vx=s32((t1-t2)<<1)
        t3=s32((q2*q3)>>15);t4=s32((q0*q1)>>15);vy=s32((t3+t4)<<1)
        t5=s32((q1*q1)>>15);t6=s32((q2*q2)>>15);vz=s32(ONE-((t5+t6)<<1))
        if acc_ok:
            ex=s32(((ayn*vz)>>15)-((azn*vy)>>15)); ey=s32(((azn*vx)>>15)-((axn*vz)>>15)); ez=s32(((axn*vy)>>15)-((ayn*vx)>>15))
        else: ex=ey=ez=0
        wxc=s32(gx+((ex*KP)>>16)); wyc=s32(gy+((ey*KP)>>16)); wzc=s32(gz+((ez*KP)>>16))
        wsx=s32((wxc*GYRO_C)&0xFFFFFFFF); wsy=s32((wyc*GYRO_C)&0xFFFFFFFF); wsz=s32((wzc*GYRO_C)&0xFFFFFFFF)
        u1=s32((q1*wsx)>>32);u2=s32((q2*wsy)>>32);u3=s32((q3*wsz)>>32);dq0=s32(-(u1+u2+u3))
        u4=s32((q0*wsx)>>32);u5=s32((q3*wsy)>>32);u6=s32((q2*wsz)>>32);dq1=s32(u4-u5+u6)
        u7=s32((q3*wsx)>>32);u8=s32((q0*wsy)>>32);u9=s32((q1*wsz)>>32);dq2=s32(u7+u8-u9)
        u10=s32((q2*wsx)>>32);u11=s32((q1*wsy)>>32);u12=s32((q0*wsz)>>32);dq3=s32(-u10+u11+u12)
        q0n=s32(q0+dq0);q1n=s32(q1+dq1);q2n=s32(q2+dq2);q3n=s32(q3+dq3)
        qsq=q0n*q0n+q1n*q1n+q2n*q2n+q3n*q3n; d=(1<<30)-qsq; scale=s32((1<<30)+(d>>1))
        self.q0=s32((q0n*scale)>>30);self.q1=s32((q1n*scale)>>30);self.q2=s32((q2n*scale)>>30);self.q3=s32((q3n*scale)>>30)
        t28=s32((q3*q3)>>15);t26=s32((q1*q2)>>15);t27=s32((q0*q3)>>15)
        self.r21=s32((t26+t27)<<1); self.r11=s32(ONE-((t6+t28)<<1))

# T4: pure yaw 90 deg/s for 1 second (gz raw = 90*131 = 11790)
q = Quat()
for f in range(0, 500):  # 500 frames = 1s @500Hz
    q.step(0, 0, 65536, 0, 0, 11790)
yaw = math.degrees(math.atan2(q.r21, q.r11))
print(f"After 1s of 90deg/s yaw: yaw={yaw:.2f} deg (expect ~90), r21={q.r21} r11={q.r11}")
print(f"q=({q.q0},{q.q1},{q.q2},{q.q3}) norm={math.sqrt(q.q0**2+q.q1**2+q.q2**2+q.q3**2):.1f} (expect 32768)")

# Also test gyro-only roll for 1s at 90 deg/s (gx raw)
q2 = Quat()
for f in range(0, 500):
    q2.step(0, 0, 65536, 11790, 0, 0)
roll = math.degrees(math.atan2(q2.r32, q2.r33))
print(f"After 1s of 90deg/s roll: roll={roll:.2f} deg (expect ~90)")

# gyro scaling sanity: 131 LSB/(deg/s). Check GYRO_C correctness
print(f"\nGYRO_C check: 0.5*dt*pi/(180*131)*2^32 = {0.5*0.002*math.pi/(180*131)*2**32:.1f} (GYRO_C=572)")
