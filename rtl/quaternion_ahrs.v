//------------------------------------------------------------------------------
// 文件名 : quaternion_ahrs.v
// 功能   : 四元数姿态引擎（Mahony 式互补滤波，逐帧定点迭代，单乘法器串行）
// 日期   : 2026-09-26（2026-09-26 对照 MahonyAHRS.c 成熟实现补全：加速度归一化、
//          加速度合理性门控、Kp 与标准实现定标对齐）
//------------------------------------------------------------------------------
/* @brief  四元数姿态引擎（与传感器无关，输入已标定的物理量）
 *         算法对照 x-io Technologies 的 MahonyAHRS.c（Madgwick 实现的
 *         Mahony DCM 滤波器，无人机/机器人界广泛使用的成熟实现）：
 *         每帧（500Hz，dt=2ms）执行：
 *         1) 加速度归一化：a /= |a|（牛顿迭代求 1/√|a|²，2 次）——
 *            标准实现第一步；消除 |a| 偏差（轴间灵敏度差异、校准残差）对
 *            修正幅度的影响
 *         2) 加速度合理性门控：|a| 超出 0.25g~2.25g 时跳过重力修正
 *            （e=0，只靠陀螺积分）——防止手持运动时的线性加速度把姿态拉歪
 *         3) 姿态误差：e = a × v（归一化加速度 a 与四元数估计的重力方向 v 叉积）
 *         4) 角速度修正：ωc = ω + Kp·e（比例项）
 *         5) 四元数积分：q += 0.5·dt·q⊗[0, ωc]（一阶积分）
 *         6) 模长修正：q ← q·(3-|q|²)/2（牛顿迭代一阶式，一步完成）
 *         7) 输出旋转矩阵元素（body→earth，供 atan2 换算欧拉角）
 *         四元数可连续跟踪任意 360° 姿态（无欧拉角奇点），
 *         修正项在任意姿态下均有效——即使板子翻转超过 ±90° 也不会折叠
 * @param  KP : 比例校正系数，3753（有符号）。
 *         与 MahonyAHRS.c 定标对齐：其 twoKp=1.0 且 halfe=0.5·sinθ，
 *         修正 = 0.5·sinθ rad/s；本模块 e = 65536·sinθ（Q16.16），
 *         Kp·e>>16 = Kp·sinθ raw = Kp/131·sinθ °/s，取 Kp=3753 时
 *         = 28.65°/s·sinθ = 0.5 rad/s·sinθ，与标准默认完全一致。
 *         早期版本 Kp=30024（为标准值的 8 倍）曾把加速度计噪声放大注入
 *         三个轴导致上板乱飘，Kp 不宜再调大
 * @param  accel_x/y/z[31:0] : 校准后加速度，Q16.16，1g=65536
 * @param  gyro_x/y/z[31:0]  : 校准后角速度原始值（131 LSB/(°/s)）
 * @param  frame_valid : 帧同步脉冲（每帧一次，处理约132个时钟）
 * @return q0~q3_out[31:0] : 四元数分量，Q16.15（1.0=32768），调试用
 * @return r31/r32/r33/r21/r11[31:0] : 旋转矩阵元素（body→earth），Q16.15
 * @return q_valid : 更新完成脉冲（与 r 元素对齐，滞后 frame_valid 约132个时钟）
 * @note   1) 四元数采用 Q16.15 定点（Q15.16 无法表示 1.0），1.0=32768，
 *            所有移位均已按此定标推导，修改时勿改动移位数
 *         2) 角速度增量比例常数 572 = 0.5·dt·π/(180·131)×2^32（Q31.32 定标，
 *            误差 0.04%）：w_s = raw×572 即每帧半角增量
 *         3) 归一化：q ← q·(3-|q|²)/2 为牛顿迭代的一阶式（y'=y(3-xy²)/2 取 y=1），
 *            每帧残差 O((|q|²-1)²) ≈ 1e-11 量级；极速旋转(250°/s)持续 1 小时
 *            累计误差约 0.04°，远小于陀螺零偏漂移，可忽略
 *         4) 加速度归一化的牛顿迭代：y' = y(3-|a|²y²)/2，初值 y=1.0(Q16.16)，
 *            收敛域 |a|²∈(0,3)；配合合理性门控（|a|∈0.25g~2.25g）保证收敛，
 *            门控失败时冻结 y、跳过修正
 *         5) 加速度计无绝对航向参考：yaw 仅由陀螺积分维持（缓慢漂移属正常）；
 *            姿态精度依赖加速度计噪声水平——调用方应对加速度做适当低通
 *            （本工程在 attitude_calc 内做 8 帧滑动平均）
 *         6) **重要**：所有幅度可能超过 2^31 的乘积必须经过 64 位寄存器 P64
 *            两拍完成。Verilog 赋值上下文的位宽规则会把 32 位赋值右端的乘法
 *            限制在 32 位计算——直接写 u <= (a*b)>>>n 会产生静默溢出，
 *            这是本模块曾出现过的 bug，请勿改回单拍写法
 *         7) **面积优化（PGL22G 教训）**：PGL22G 只有 30 个 APM（DSP18×18），
 *            并行摆放 25+ 个乘法器会把 DSP 耗尽、其余乘法全部展开成 LUT，
 *            曾导致 LUT 超限（19345/17536）。故本模块全部乘法串行
 *            复用同一个 P64 乘法器，代价是状态机多几十拍（132 拍仍远小于
 *            帧周期 10 万拍）。如需恢复并行乘法，先评估目标器件 DSP 数量
 *         8) frame_valid 期间若上一帧处理未完成，本帧被忽略（500Hz 帧周期
 *            2ms = 10万时钟，处理约132时钟，正常不会发生）
 *         9) **位宽教训**：step 与 LAST_STEP 必须用 8 位（状态机 132 拍 > 127）。
 *            曾用 7 位：LAST_STEP=7'd131 被截断为 3、case 标签 7'd128~131 被
 *            截断为 0~3 造成重复项，状态机死锁 → q_valid 永不产生 → 上板现象为
 *            角度锁死、LED2 心跳常亮。扩展状态机步数时务必同步检查位宽
 */
module quaternion_ahrs #(
    parameter signed [31:0] KP = 32'sd3753
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire signed [31:0] accel_x,   // Q16.16，1g = 65536
    input  wire signed [31:0] accel_y,
    input  wire signed [31:0] accel_z,
    input  wire signed [31:0] gyro_x,    // 校准后原始值
    input  wire signed [31:0] gyro_y,
    input  wire signed [31:0] gyro_z,
    input  wire        frame_valid,
    output wire signed [31:0] q0_out, q1_out, q2_out, q3_out,   // 调试端口
    output wire signed [31:0] r31, r32, r33, r21, r11,          // Q16.15，1.0=32768
    output wire        q_valid
);

    //====================== 定标常量 ======================
    // Q16.15：1.0 = 32768
    localparam signed [31:0] ONE    = 32'sd32768;
    // 每帧半角增量系数（Q31.32）：572 ≈ 0.5·dt·π/(180·131) × 2^32
    localparam signed [31:0] GYRO_C = 32'sd572;
    // 模长修正基准：|q|² 的标称值 2^30（Q32.30 下的 1.0²）
    localparam signed [63:0] QSQ    = 64'sd1073741824;

    //====================== 四元数状态（Q16.15） ======================
    reg signed [31:0] q0, q1, q2, q3;
    reg signed [31:0] r31_r, r32_r, r33_r, r21_r, r11_r;
    reg        q_valid_r;

    //====================== 迭代状态机 ======================
    localparam S_IDLE = 1'b0;
    localparam S_RUN  = 1'b1;
    // 注意：状态机共 132 拍（0~131），step 与 LAST_STEP 必须用 8 位——
    // 曾用 7 位导致 131 截断为 3，状态机卡死在第 3 步，q_valid 永不产生，
    // 现象为角度锁死、LED2 心跳停止（详见 @note 9）
    localparam [7:0] LAST_STEP = 8'd131;    // 完成步骤号
    reg  state;
    reg  [7:0] step;

    // 中间寄存器（t/u/dq/ws 等为 signed 32 位；P64 为共享 64 位乘法累加器）
    reg signed [63:0] P64;
    reg signed [31:0] t1, t2, t3, t4, t5, t6, t7, t8, t9, t10;
    reg signed [31:0] t11, t12, t13, t14, t15;
    reg signed [31:0] t26, t27, t28;
    reg signed [31:0] vx, vy, vz;
    reg signed [31:0] ex, ey, ez;
    reg signed [31:0] wxc, wyc, wzc;
    reg signed [31:0] wsx, wsy, wsz;
    reg signed [31:0] u1, u2, u3, u4, u5, u6, u7, u8, u9, u10, u11, u12;
    reg signed [31:0] dq0, dq1, dq2, dq3;
    reg signed [31:0] q0n, q1n, q2n, q3n, q0r, q1r, q2r, q3r;
    reg signed [31:0] scale;                       // (3-|q|²)/2，Q31.30
    reg signed [63:0] s0, s1, s2, s3, xs, d;       // |q|²/|a|² 累加（Q32.30/Q32.32）
    // 加速度归一化中间量
    reg signed [31:0] sh;                          // |a|²>>16，Q16.16（约65536）
    reg signed [31:0] Y;                           // 1/√|a|²，Q16.16（1g=1.0 时≈65536）
    reg signed [31:0] t_a1, t_a2, r_a;
    reg signed [31:0] axn, ayn, azn;               // 归一化加速度，Q16.16（模长65536）
    reg               acc_ok;                      // 加速度合理性门控

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            step      <= 7'd0;
            q0        <= ONE;         // 初始单位四元数（板子水平姿态）
            q1        <= 32'sd0;
            q2        <= 32'sd0;
            q3        <= 32'sd0;
            r31_r     <= 32'sd0;
            r32_r     <= 32'sd0;
            r33_r     <= ONE;
            r21_r     <= 32'sd0;
            r11_r     <= ONE;
            q_valid_r <= 1'b0;
            P64   <= 64'sd0;
            t1 <= 32'sd0;  t2 <= 32'sd0;  t3 <= 32'sd0;  t4 <= 32'sd0;  t5 <= 32'sd0;
            t6 <= 32'sd0;  t7 <= 32'sd0;  t8 <= 32'sd0;  t9 <= 32'sd0;  t10 <= 32'sd0;
            t11 <= 32'sd0; t12 <= 32'sd0; t13 <= 32'sd0; t14 <= 32'sd0; t15 <= 32'sd0;
            t26 <= 32'sd0; t27 <= 32'sd0; t28 <= 32'sd0;
            vx <= 32'sd0; vy <= 32'sd0; vz <= 32'sd0;
            ex <= 32'sd0; ey <= 32'sd0; ez <= 32'sd0;
            wxc <= 32'sd0; wyc <= 32'sd0; wzc <= 32'sd0;
            wsx <= 32'sd0; wsy <= 32'sd0; wsz <= 32'sd0;
            u1 <= 32'sd0; u2 <= 32'sd0; u3 <= 32'sd0; u4 <= 32'sd0; u5 <= 32'sd0; u6 <= 32'sd0;
            u7 <= 32'sd0; u8 <= 32'sd0; u9 <= 32'sd0; u10 <= 32'sd0; u11 <= 32'sd0; u12 <= 32'sd0;
            dq0 <= 32'sd0; dq1 <= 32'sd0; dq2 <= 32'sd0; dq3 <= 32'sd0;
            q0n <= 32'sd0; q1n <= 32'sd0; q2n <= 32'sd0; q3n <= 32'sd0;
            q0r <= 32'sd0; q1r <= 32'sd0; q2r <= 32'sd0; q3r <= 32'sd0;
            scale <= 32'sd0;
            s0 <= 64'sd0; s1 <= 64'sd0; s2 <= 64'sd0; s3 <= 64'sd0;
            xs <= 64'sd0; d  <= 64'sd0;
            sh  <= 32'sd0; Y  <= 32'sd65536;
            t_a1 <= 32'sd0; t_a2 <= 32'sd0; r_a <= 32'sd0;
            axn <= 32'sd0; ayn <= 32'sd0; azn <= 32'sd0;
            acc_ok <= 1'b0;
        end else begin
            q_valid_r <= 1'b0;          // 脉冲默认清零
            case (state)
                S_IDLE: begin
                    step <= 7'd0;
                    if (frame_valid) state <= S_RUN;
                end
                S_RUN: begin
                    case (step)
                        //---------- 阶段A0：加速度归一化（|a|² 与牛顿迭代 1/√x）----------
                        7'd0:   P64 <= accel_x*accel_x;
                        7'd1:   s0 <= P64;
                        7'd2:   P64 <= accel_y*accel_y;
                        7'd3:   s1 <= P64;
                        7'd4:   P64 <= accel_z*accel_z;
                        7'd5:   s2 <= P64;
                        7'd6:   xs <= s0 + s1 + s2;          // |a|²，Q32.32
                        7'd7:   sh <= xs>>>16;               // |a|² 的 Q16.16（1g=65536）
                        7'd8:   begin
                            // 合理性门控：|a|∈(0.25g, 2.25g)；初值 y=1.0
                            acc_ok <= (sh > 32'sd16384) && (sh < 32'sd147456);
                            Y      <= 32'sd65536;
                        end
                        7'd9:   P64 <= sh*Y;
                        7'd10:  t_a1 <= P64>>>16;            // |a|²y·2^16
                        7'd11:  P64 <= t_a1*Y;
                        7'd12:  t_a2 <= P64>>>16;            // |a|²y²·2^16
                        7'd13:  r_a <= (t_a2 < 32'sd196608) ? (32'sd196608 - t_a2)
                                                             : 32'sd0;
                        7'd14:  P64 <= Y*r_a;
                        7'd15:  if (acc_ok) Y <= P64>>>17;   // 牛顿迭代第1次
                        7'd16:  P64 <= sh*Y;
                        7'd17:  t_a1 <= P64>>>16;
                        7'd18:  P64 <= t_a1*Y;
                        7'd19:  t_a2 <= P64>>>16;
                        7'd20:  r_a <= (t_a2 < 32'sd196608) ? (32'sd196608 - t_a2)
                                                             : 32'sd0;
                        7'd21:  P64 <= Y*r_a;
                        7'd22:  if (acc_ok) Y <= P64>>>17;   // 牛顿迭代第2次
                        7'd23:  P64 <= accel_x*Y;
                        7'd24:  axn <= P64>>>16;             // a 归一化，Q16.16
                        7'd25:  P64 <= accel_y*Y;
                        7'd26:  ayn <= P64>>>16;
                        7'd27:  P64 <= accel_z*Y;
                        7'd28:  azn <= P64>>>16;
                        //---------- 阶段A：重力参考向量 v（6 个 q·q 乘积，串行走 P64）----------
                        7'd29:  P64 <= q1*q3;
                        7'd30:  t1 <= P64>>>15;
                        7'd31:  P64 <= q0*q2;
                        7'd32:  t2 <= P64>>>15;
                        7'd33:  P64 <= q2*q3;
                        7'd34:  t3 <= P64>>>15;
                        7'd35:  P64 <= q0*q1;
                        7'd36:  t4 <= P64>>>15;
                        7'd37:  P64 <= q1*q1;
                        7'd38:  t5 <= P64>>>15;
                        7'd39:  P64 <= q2*q2;
                        7'd40:  t6 <= P64>>>15;
                        7'd41:  vx <= (t1 - t2)<<<1;             // R31 = 2(q1q3-q0q2)
                        7'd42:  vy <= (t3 + t4)<<<1;             // R32 = 2(q2q3+q0q1)
                        7'd43:  vz <= ONE - ((t5 + t6)<<<1);     // R33 = 1-2(q1²+q2²)
                        //---------- 阶段B：误差 e = a×v（归一化 a，超界时 e=0）----------
                        7'd44:  P64 <= ayn*vz;
                        7'd45:  t7  <= P64>>>15;
                        7'd46:  P64 <= azn*vy;
                        7'd47:  t8  <= P64>>>15;
                        7'd48:  ex  <= acc_ok ? (t7 - t8) : 32'sd0;
                        7'd49:  P64 <= azn*vx;
                        7'd50:  t9  <= P64>>>15;
                        7'd51:  P64 <= axn*vz;
                        7'd52:  t10 <= P64>>>15;
                        7'd53:  ey  <= acc_ok ? (t9 - t10) : 32'sd0;
                        7'd54:  P64 <= axn*vy;
                        7'd55:  t11 <= P64>>>15;
                        7'd56:  P64 <= ayn*vx;
                        7'd57:  t12 <= P64>>>15;
                        7'd58:  ez  <= acc_ok ? (t11 - t12) : 32'sd0;
                        //---------- 阶段C：角速度修正 ωc = ω + Kp·e ----------
                        7'd59:  P64 <= ex*KP;
                        7'd60:  t13 <= P64>>>16;
                        7'd61:  wxc <= gyro_x + t13;
                        7'd62:  P64 <= ey*KP;
                        7'd63:  t14 <= P64>>>16;
                        7'd64:  wyc <= gyro_y + t14;
                        7'd65:  P64 <= ez*KP;
                        7'd66:  t15 <= P64>>>16;
                        7'd67:  wzc <= gyro_z + t15;
                        //---------- 阶段D：半角增量 ws = ωc·572（Q31.32，≤7.2e7）----------
                        7'd68:  P64 <= wxc*GYRO_C;
                        7'd69:  wsx <= P64[31:0];
                        7'd70:  P64 <= wyc*GYRO_C;
                        7'd71:  wsy <= P64[31:0];
                        7'd72:  P64 <= wzc*GYRO_C;
                        7'd73:  wsz <= P64[31:0];
                        //---------- 阶段E：四元数积分 dq = 0.5·dt·q⊗ω（q·ws 走 P64）----------
                        7'd74:  P64 <= q1*wsx;
                        7'd75:  u1  <= P64>>>32;
                        7'd76:  P64 <= q2*wsy;
                        7'd77:  u2  <= P64>>>32;
                        7'd78:  P64 <= q3*wsz;
                        7'd79:  u3  <= P64>>>32;
                        7'd80:  dq0 <= -(u1 + u2 + u3);
                        7'd81:  P64 <= q0*wsx;
                        7'd82:  u4  <= P64>>>32;
                        7'd83:  P64 <= q3*wsy;
                        7'd84:  u5  <= P64>>>32;
                        7'd85:  P64 <= q2*wsz;
                        7'd86:  u6  <= P64>>>32;
                        7'd87:  dq1 <= u4 - u5 + u6;
                        7'd88:  P64 <= q3*wsx;
                        7'd89:  u7  <= P64>>>32;
                        7'd90:  P64 <= q0*wsy;
                        7'd91:  u8  <= P64>>>32;
                        7'd92:  P64 <= q1*wsz;
                        7'd93:  u9  <= P64>>>32;
                        7'd94:  dq2 <= u7 + u8 - u9;
                        7'd95:  P64 <= q2*wsx;
                        7'd96:  u10 <= P64>>>32;
                        7'd97:  P64 <= q1*wsy;
                        7'd98:  u11 <= P64>>>32;
                        7'd99:  P64 <= q0*wsz;
                        7'd100: u12 <= P64>>>32;
                        7'd101: dq3 <= -u10 + u11 + u12;
                        7'd102: begin
                            q0n <= q0 + dq0;
                            q1n <= q1 + dq1;
                            q2n <= q2 + dq2;
                            q3n <= q3 + dq3;
                        end
                        //---------- 阶段F：模长修正 q' = q·(3-|q|²)/2（牛顿迭代一阶式）----------
                        7'd103: P64 <= q0n*q0n;
                        7'd104: s0 <= P64;
                        7'd105: P64 <= q1n*q1n;
                        7'd106: s1 <= P64;
                        7'd107: P64 <= q2n*q2n;
                        7'd108: s2 <= P64;
                        7'd109: P64 <= q3n*q3n;
                        7'd110: s3 <= P64;
                        7'd111: xs <= s0 + s1 + s2 + s3;              // |q|²，Q32.30
                        7'd112: d  <= QSQ - xs;                       // 2^30-|q|²，小量
                        7'd113: scale <= 32'sd1073741824 + (d>>>1);   // (3-|q|²)/2，Q31.30
                        7'd114: P64 <= q0n*scale;
                        7'd115: q0r <= P64>>>30;
                        7'd116: P64 <= q1n*scale;
                        7'd117: q1r <= P64>>>30;
                        7'd118: P64 <= q2n*scale;
                        7'd119: q2r <= P64>>>30;
                        7'd120: P64 <= q3n*scale;
                        7'd121: q3r <= P64>>>30;
                        //---------- 阶段G：旋转矩阵输出（r31/32/33 复用阶段A 结果）----------
                        7'd122: P64 <= q3*q3;
                        7'd123: t28 <= P64>>>15;
                        7'd124: P64 <= q1*q2;
                        7'd125: t26 <= P64>>>15;
                        7'd126: P64 <= q0*q3;
                        7'd127: t27 <= P64>>>15;
                        8'd128: begin
                            r31_r <= vx;         // 2(q1q3-q0q2)
                            r32_r <= vy;         // 2(q2q3+q0q1)
                            r33_r <= vz;         // 1-2(q1²+q2²)
                        end
                        8'd129: r21_r <= (t26 + t27)<<<1;             // 2(q1q2+q0q3)
                        8'd130: r11_r <= ONE - ((t6 + t28)<<<1);      // 1-2(q2²+q3²)
                        //---------- 完成：更新状态并输出有效脉冲 ----------
                        8'd131: begin
                            q0 <= q0r; q1 <= q1r; q2 <= q2r; q3 <= q3r;
                            q_valid_r <= 1'b1;
                            step  <= 7'd0;
                            state <= S_IDLE;
                        end
                        default: begin
                            step  <= 7'd0;
                            state <= S_IDLE;
                        end
                    endcase
                    if (step == 7'd0)        step <= 7'd1;
                    else if (step < LAST_STEP) step <= step + 1'b1;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    //====================== 输出 ======================
    assign q0_out = q0;
    assign q1_out = q1;
    assign q2_out = q2;
    assign q3_out = q3;
    assign r31    = r31_r;
    assign r32    = r32_r;
    assign r33    = r33_r;
    assign r21    = r21_r;
    assign r11    = r11_r;
    assign q_valid = q_valid_r;

endmodule
