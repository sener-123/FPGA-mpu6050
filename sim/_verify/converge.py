import math
KP=7506; GYRO_C=572; ONE=32768
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
        self.r31=vx; self.r32=vy; self.r33=vz
        t28=s32((q3*q3)>>15);t26=s32((q1*q2)>>15);t27=s32((q0*q3)>>15)
        self.r21=s32((t26+t27)<<1); self.r11=s32(ONE-((t6+t28)<<1))

def roll(q): return math.degrees(math.atan2(q.r32,q.r33))
def pitch(q): return math.degrees(math.atan2(-q.r31,q.r33))

# roll=30 constant, KP=7506, check over 8s
q=Quat()
for f in range(0,4000):
    q.step(0, 32768, 56760, 0,0,0)
    if f in (500,1000,1500,2000,3000,3999):
        print(f"roll30 @{f/500:.1f}s: roll={roll(q):.2f}deg")

# pitch=160 constant
q2=Quat()
for f in range(0,8000):
    q2.step(-22411, 0, -61605, 0,0,0)
    if f in (500,1000,2000,3000,5000,7999):
        print(f"pitch160 @{f/500:.1f}s: pitch={pitch(q2):.2f}deg")
