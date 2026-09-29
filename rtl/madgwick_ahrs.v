//------------------------------------------------------------------------------
// 文件名 : madgwick_ahrs.v
// 功能   : Madgwick 姿态解算（四元数梯度下降），6 轴 IMU（时分复用，单乘法器）
// 日期   : 2026-09-29
//------------------------------------------------------------------------------
/* @brief  Madgwick AHRS（参考 Sebastian Madgwick 2011，x-io MadgwickAHRS.c）
 *         四元数梯度下降融合陀螺仪与加速度计，输出全范围 roll/pitch/yaw（无万向锁）
 *         + 世界系线性加速度（去重力，供位移积分）
 *         定点：四元数/归一化加速度 Q16.16，角速度 rad/s Q16.16，欧拉角 度 Q16.16
 *         资源：全部乘法通过单个共享乘法器 mul32x32 时分复用（微序列器，每帧约
 *         110 个时钟），适配 PGL22G（17.5K LUT）等小容量 FPGA。
 * @param  BETA : 反馈增益 0.1 rad/s，Q16.16 = 6554
 * @param  accel_x/y/z_raw[15:0] : 原始加速度（±2g，16384 LSB/g）
 * @param  gyro_x/y/z_raw[15:0]  : 原始角速度（±250°/s，131 LSB/(°/s)）
 * @param  data_valid_in : 帧同步脉冲（500Hz，dt=2ms）
 * @return roll/pitch/yaw[31:0] : 欧拉角 Q16.16，单位度
 * @return accel_world_x/y/z[31:0] : 世界系线性加速度 Q16.16，单位 m/s²（去重力）
 * @return motion : 1=检测到运动，0=近似静止
 * @return data_valid_out : 输出帧同步
 * @note   yaw 无磁力计参考，仅积分陀螺仪，会缓慢漂移
 */
module madgwick_ahrs #(
    parameter signed [31:0] BETA = 32'sd6554   // 0.1 rad/s
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] accel_x_raw,
    input  wire [15:0] accel_y_raw,
    input  wire [15:0] accel_z_raw,
    input  wire [15:0] gyro_x_raw,
    input  wire [15:0] gyro_y_raw,
    input  wire [15:0] gyro_z_raw,
    input  wire        data_valid_in,
    output wire signed [31:0] roll,
    output wire signed [31:0] pitch,
    output wire signed [31:0] yaw,
    output wire signed [31:0] accel_world_x,
    output wire signed [31:0] accel_world_y,
    output wire signed [31:0] accel_world_z,
    output wire        motion,
    output wire        data_valid_out,
    output wire signed [31:0] q0_dbg,
    output wire signed [31:0] q1_dbg,
    output wire signed [31:0] q2_dbg,
    output wire signed [31:0] q3_dbg
);

    //====================== 常数 ======================
    localparam signed [31:0] GYRO_SCALE = 32'sd572224;
    localparam signed [31:0] DT         = 32'sd131;
    localparam signed [31:0] G_CONST    = 32'sd642688;
    localparam [31:0] ONE = 32'h00010000;
    localparam [31:0] MOTION_LO = 32'd59146;
    localparam [31:0] MOTION_HI = 32'd72253;

    //====================== 共享乘法器 ======================
    reg signed [31:0] ma, mb;
    wire signed [63:0] mp = $signed({ {32{ma[31]}}, ma }) * $signed({ {32{mb[31]}}, mb });
    wire signed [31:0] mq = mp[47:16];    // (a*b)>>16

    reg signed [31:0] acc;
    reg signed [63:0] acc64;
    wire [63:0] nsum = acc64 + mp;   // 模方和（组合）

    //====================== 采样 / 物理量 ======================
    reg signed [15:0] ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
    reg signed [31:0] ax_g, ay_g, az_g;
    reg signed [31:0] gx, gy, gz;
    reg signed [31:0] axn, ayn, azn;

    //====================== 四元数 ======================
    reg signed [31:0] q0, q1, q2, q3;
    reg signed [31:0] q0q0, q1q1, q2q2, q3q3;
    reg signed [31:0] _4q0q0, _4q1q1, _4q2q2;

    wire signed [31:0] _2q0 = q0 << 1; wire signed [31:0] _2q1 = q1 << 1;
    wire signed [31:0] _2q2 = q2 << 1; wire signed [31:0] _2q3 = q3 << 1;
    wire signed [31:0] _4q0 = q0 << 2; wire signed [31:0] _4q1 = q1 << 2;
    wire signed [31:0] _4q2 = q2 << 2;
    wire signed [31:0] _8q1 = q1 << 3; wire signed [31:0] _8q2 = q2 << 3;

    //====================== 梯度 / 导数 ======================
    reg signed [31:0] s0, s1, s2, s3;
    reg signed [31:0] p_q1gx, p_q2gy, p_q3gz, p_q0gx, p_q2gz, p_q3gy;
    reg signed [31:0] p_q0gy, p_q1gz, p_q3gx, p_q0gz, p_q1gy, p_q2gx;
    reg signed [31:0] p_bs0, p_bs1, p_bs2, p_bs3;

    // qDot（组合，由上面乘积寄存器直接合成）
    wire signed [31:0] qd0_w = ((-p_q1gx - p_q2gy - p_q3gz) >>> 1) - p_bs0;
    wire signed [31:0] qd1_w = (( p_q0gx + p_q2gz - p_q3gy) >>> 1) - p_bs1;
    wire signed [31:0] qd2_w = (( p_q0gy - p_q1gz + p_q3gx) >>> 1) - p_bs2;
    wire signed [31:0] qd3_w = (( p_q0gz + p_q1gy - p_q2gx) >>> 1) - p_bs3;

    //====================== 归一化 ======================
    reg [31:0] x_acc, x_s;
    reg [31:0] recip_acc, recip_s, recip_pitch;

    //====================== 交叉项 / 世界加速度 ======================
    reg signed [31:0] m01, m02, m03, m12, m13, m23;
    reg signed [31:0] vp, w_pitch, sqrt_w;
    reg signed [31:0] awx, awy, awz;
    // 旋转矩阵 R^T（组合，body->earth）
    wire signed [31:0] r00 = ONE - (q2q2 << 1) - (q3q3 << 1);
    wire signed [31:0] r01 = (m12 << 1) - (m03 << 1);
    wire signed [31:0] r02 = (m13 << 1) + (m02 << 1);
    wire signed [31:0] r10 = (m12 << 1) + (m03 << 1);
    wire signed [31:0] r11 = ONE - (q1q1 << 1) - (q3q3 << 1);
    wire signed [31:0] r12 = (m23 << 1) - (m01 << 1);
    wire signed [31:0] r20 = (m13 << 1) - (m02 << 1);
    wire signed [31:0] r21 = (m23 << 1) + (m01 << 1);
    wire signed [31:0] r22 = ONE - (q1q1 << 1) - (q2q2 << 1);

    //====================== 输出 ======================
    wire signed [31:0] roll_deg, pitch_deg, yaw_deg;
    reg signed [31:0] roll_o, pitch_o, yaw_o;
    reg signed [31:0] awx_o, awy_o, awz_o;
    reg motion_o, valid_o;

    wire signed [31:0] x_roll = ONE - (q1q1 << 1) - (q2q2 << 1);
    wire signed [31:0] y_roll = (m01 + m23) << 1;
    wire signed [31:0] x_yaw  = ONE - (q2q2 << 1) - (q3q3 << 1);
    wire signed [31:0] y_yaw  = (m03 + m12) << 1;

    //====================== inv_sqrt（共享单实例） ======================
    reg  isqrt_start;
    reg  [31:0] isqrt_in;
    wire [31:0] isqrt_y;
    wire        isqrt_done;
    inv_sqrt u_isqrt (
        .clk(clk), .rst_n(rst_n),
        .x(isqrt_in), .valid_in(isqrt_start), .y(isqrt_y), .valid_out(isqrt_done)
    );

    //====================== CORDIC（atan2 -> 度） ======================
    reg cordic_start;
    wire cordic_done;
    cordic #(.STAGES(16)) u_cordic_roll (
        .clk(clk), .rst_n(rst_n), .x_in(x_roll), .y_in(y_roll), .valid_in(cordic_start),
        .angle(roll_deg), .mag(), .valid_out(cordic_done)
    );
    cordic #(.STAGES(16)) u_cordic_pitch (
        .clk(clk), .rst_n(rst_n), .x_in(sqrt_w), .y_in(vp), .valid_in(cordic_start),
        .angle(pitch_deg), .mag(), .valid_out()
    );
    cordic #(.STAGES(16)) u_cordic_yaw (
        .clk(clk), .rst_n(rst_n), .x_in(x_yaw), .y_in(y_yaw), .valid_in(cordic_start),
        .angle(yaw_deg), .mag(), .valid_out()
    );

    //====================== 微序列器 ======================
    localparam S_IDLE    = 8'd0;
    localparam S_GYRO    = 8'd1;    // 3
    localparam S_ANORM   = 8'd4;    // 3
    localparam S_AWAIT   = 8'd7;
    localparam S_NAX     = 8'd8;    // 3
    localparam S_AUX     = 8'd11;   // 7
    localparam S_S0      = 8'd18;   // 4
    localparam S_S1      = 8'd22;   // 7
    localparam S_S2      = 8'd29;   // 7
    localparam S_S3      = 8'd36;   // 4
    localparam S_SNORM   = 8'd40;   // 4
    localparam S_SWAIT   = 8'd44;
    localparam S_NS      = 8'd45;   // 4
    localparam S_QDG     = 8'd49;   // 12
    localparam S_QDB     = 8'd61;   // 4
    localparam S_INTEG   = 8'd65;   // 4
    localparam S_QNORM   = 8'd69;   // 4
    localparam S_QNF     = 8'd73;   // 4
    localparam S_QSQ     = 8'd77;   // 3
    localparam S_EULER   = 8'd80;   // 6
    localparam S_PITCH   = 8'd86;   // 1
    localparam S_PWAIT   = 8'd87;
    localparam S_SQRTW   = 8'd88;   // 1
    localparam S_ROT     = 8'd89;   // 9
    localparam S_GRAV    = 8'd98;   // 3
    localparam S_CWAIT   = 8'd101;
    localparam S_OUT     = 8'd102;

    reg [7:0] st;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE;
            ax_r <= 16'sd0; ay_r <= 16'sd0; az_r <= 16'sd0;
            gx_r <= 16'sd0; gy_r <= 16'sd0; gz_r <= 16'sd0;
            ax_g <= 32'sd0; ay_g <= 32'sd0; az_g <= 32'sd0;
            gx <= 32'sd0; gy <= 32'sd0; gz <= 32'sd0;
            axn <= 32'sd0; ayn <= 32'sd0; azn <= 32'sd0;
            q0 <= 32'sd65536; q1 <= 32'sd0; q2 <= 32'sd0; q3 <= 32'sd0;
            q0q0 <= 32'sd0; q1q1 <= 32'sd0; q2q2 <= 32'sd0; q3q3 <= 32'sd0;
            _4q0q0 <= 32'sd0; _4q1q1 <= 32'sd0; _4q2q2 <= 32'sd0;
            s0 <= 32'sd0; s1 <= 32'sd0; s2 <= 32'sd0; s3 <= 32'sd0;
            p_q1gx <= 32'sd0; p_q2gy <= 32'sd0; p_q3gz <= 32'sd0;
            p_q0gx <= 32'sd0; p_q2gz <= 32'sd0; p_q3gy <= 32'sd0;
            p_q0gy <= 32'sd0; p_q1gz <= 32'sd0; p_q3gx <= 32'sd0;
            p_q0gz <= 32'sd0; p_q1gy <= 32'sd0; p_q2gx <= 32'sd0;
            p_bs0 <= 32'sd0; p_bs1 <= 32'sd0; p_bs2 <= 32'sd0; p_bs3 <= 32'sd0;
            x_acc <= 32'd0; x_s <= 32'd0;
            recip_acc <= 32'd0; recip_s <= 32'd0; recip_pitch <= 32'd0;
            m01 <= 32'sd0; m02 <= 32'sd0; m03 <= 32'sd0;
            m12 <= 32'sd0; m13 <= 32'sd0; m23 <= 32'sd0;
            vp <= 32'sd0; w_pitch <= 32'sd0; sqrt_w <= 32'sd0;
            awx <= 32'sd0; awy <= 32'sd0; awz <= 32'sd0;
            roll_o <= 32'sd0; pitch_o <= 32'sd0; yaw_o <= 32'sd0;
            awx_o <= 32'sd0; awy_o <= 32'sd0; awz_o <= 32'sd0;
            motion_o <= 1'b0; valid_o <= 1'b0;
            ma <= 32'sd0; mb <= 32'sd0; acc <= 32'sd0; acc64 <= 64'sd0;
            isqrt_start <= 1'b0; isqrt_in <= 32'd0; cordic_start <= 1'b0;
        end else begin
            isqrt_start <= 1'b0;
            cordic_start <= 1'b0;
            valid_o     <= 1'b0;

            case (st)
                S_IDLE: begin
                    if (data_valid_in) begin
                        ax_r <= $signed(accel_x_raw); ay_r <= $signed(accel_y_raw); az_r <= $signed(accel_z_raw);
                        gx_r <= $signed(gyro_x_raw);  gy_r <= $signed(gyro_y_raw);  gz_r <= $signed(gyro_z_raw);
                        ax_g <= $signed({accel_x_raw, 2'b00}); ay_g <= $signed({accel_y_raw, 2'b00}); az_g <= $signed({accel_z_raw, 2'b00});
                        ma <= $signed({{16{gyro_x_raw[15]}}, gyro_x_raw}); mb <= GYRO_SCALE;
                        st <= S_GYRO;
                    end
                end
                S_GYRO:   begin gx <= mq; ma <= $signed({{16{gyro_y_raw[15]}}, gyro_y_raw}); mb <= GYRO_SCALE; st <= S_GYRO+1; end
                S_GYRO+1: begin gy <= mq; ma <= $signed({{16{gyro_z_raw[15]}}, gyro_z_raw}); mb <= GYRO_SCALE; st <= S_GYRO+2; end
                S_GYRO+2: begin gz <= mq; ma <= ax_g; mb <= ax_g; st <= S_ANORM; end

                S_ANORM:   begin acc64 <= mp; ma <= ay_g; mb <= ay_g; st <= S_ANORM+1; end
                S_ANORM+1: begin acc64 <= acc64 + mp; ma <= az_g; mb <= az_g; st <= S_ANORM+2; end
                S_ANORM+2: begin
                    x_acc <= nsum[47:16];
                    isqrt_in <= nsum[47:16] == 32'd0 ? 32'd1 : nsum[47:16];
                    isqrt_start <= 1'b1;
                    st <= S_AWAIT;
                end
                S_AWAIT: if (isqrt_done) begin recip_acc <= isqrt_y; ma <= ax_g; mb <= isqrt_y; st <= S_NAX; end

                S_NAX:   begin axn <= mq; ma <= ay_g; mb <= recip_acc; st <= S_NAX+1; end
                S_NAX+1: begin ayn <= mq; ma <= az_g; mb <= recip_acc; st <= S_NAX+2; end
                S_NAX+2: begin azn <= mq; ma <= q0; mb <= q0; st <= S_AUX; end

                S_AUX:   begin q0q0 <= mq; ma <= q1; mb <= q1; st <= S_AUX+1; end
                S_AUX+1: begin q1q1 <= mq; ma <= q2; mb <= q2; st <= S_AUX+2; end
                S_AUX+2: begin q2q2 <= mq; ma <= q3; mb <= q3; st <= S_AUX+3; end
                S_AUX+3: begin q3q3 <= mq; ma <= _4q0; mb <= q0; st <= S_AUX+4; end
                S_AUX+4: begin _4q0q0 <= mq; ma <= _4q1; mb <= q1; st <= S_AUX+5; end
                S_AUX+5: begin _4q1q1 <= mq; ma <= _4q2; mb <= q2; st <= S_AUX+6; end
                S_AUX+6: begin _4q2q2 <= mq; ma <= _4q0; mb <= q2q2; st <= S_S0; end

                S_S0:   begin acc <= mq; ma <= _2q2; mb <= axn; st <= S_S0+1; end
                S_S0+1: begin acc <= acc + mq; ma <= _4q0; mb <= q1q1; st <= S_S0+2; end
                S_S0+2: begin acc <= acc + mq; ma <= _2q1; mb <= ayn; st <= S_S0+3; end
                S_S0+3: begin s0 <= acc - mq; acc <= -_4q1; ma <= _4q1; mb <= q3q3; st <= S_S1; end

                S_S1:   begin acc <= acc + mq; ma <= _2q3; mb <= axn; st <= S_S1+1; end
                S_S1+1: begin acc <= acc - mq; ma <= _4q0q0; mb <= q1; st <= S_S1+2; end
                S_S1+2: begin acc <= acc + mq; ma <= _2q0; mb <= ayn; st <= S_S1+3; end
                S_S1+3: begin acc <= acc - mq; ma <= _8q1; mb <= q1q1; st <= S_S1+4; end
                S_S1+4: begin acc <= acc + mq; ma <= _8q1; mb <= q2q2; st <= S_S1+5; end
                S_S1+5: begin acc <= acc + mq; ma <= _4q1; mb <= azn; st <= S_S1+6; end
                S_S1+6: begin s1 <= acc + mq; acc <= -_4q2; ma <= _4q0q0; mb <= q2; st <= S_S2; end

                S_S2:   begin acc <= acc + mq; ma <= _2q0; mb <= axn; st <= S_S2+1; end
                S_S2+1: begin acc <= acc + mq; ma <= _4q2; mb <= q3q3; st <= S_S2+2; end
                S_S2+2: begin acc <= acc + mq; ma <= _2q2; mb <= ayn; st <= S_S2+3; end
                S_S2+3: begin acc <= acc - mq; ma <= _8q2; mb <= q1q1; st <= S_S2+4; end
                S_S2+4: begin acc <= acc + mq; ma <= _8q2; mb <= q2q2; st <= S_S2+5; end
                S_S2+5: begin acc <= acc + mq; ma <= _4q2; mb <= azn; st <= S_S2+6; end
                S_S2+6: begin s2 <= acc + mq; acc <= 32'sd0; ma <= _4q1q1; mb <= q3; st <= S_S3; end

                S_S3:   begin acc <= mq; ma <= _2q1; mb <= axn; st <= S_S3+1; end
                S_S3+1: begin acc <= acc - mq; ma <= _4q2q2; mb <= q3; st <= S_S3+2; end
                S_S3+2: begin acc <= acc + mq; ma <= _2q2; mb <= ayn; st <= S_S3+3; end
                S_S3+3: begin s3 <= acc - mq; ma <= s0; mb <= s0; acc64 <= 64'sd0; st <= S_SNORM; end

                S_SNORM:   begin acc64 <= mp; ma <= s1; mb <= s1; st <= S_SNORM+1; end
                S_SNORM+1: begin acc64 <= acc64 + mp; ma <= s2; mb <= s2; st <= S_SNORM+2; end
                S_SNORM+2: begin acc64 <= acc64 + mp; ma <= s3; mb <= s3; st <= S_SNORM+3; end
                S_SNORM+3: begin
                    x_s <= nsum[47:16];
                    isqrt_in <= nsum[47:16] == 32'd0 ? 32'd1 : nsum[47:16];
                    isqrt_start <= 1'b1;
                    st <= S_SWAIT;
                end
                S_SWAIT: if (isqrt_done) begin recip_s <= isqrt_y; ma <= s0; mb <= isqrt_y; st <= S_NS; end

                S_NS:   begin s0 <= mq; ma <= s1; mb <= recip_s; st <= S_NS+1; end
                S_NS+1: begin s1 <= mq; ma <= s2; mb <= recip_s; st <= S_NS+2; end
                S_NS+2: begin s2 <= mq; ma <= s3; mb <= recip_s; st <= S_NS+3; end
                S_NS+3: begin s3 <= mq; ma <= q1; mb <= gx; st <= S_QDG; end

                S_QDG:   begin p_q1gx <= mq; ma <= q2; mb <= gy; st <= S_QDG+1; end
                S_QDG+1: begin p_q2gy <= mq; ma <= q3; mb <= gz; st <= S_QDG+2; end
                S_QDG+2: begin p_q3gz <= mq; ma <= q0; mb <= gx; st <= S_QDG+3; end
                S_QDG+3: begin p_q0gx <= mq; ma <= q2; mb <= gz; st <= S_QDG+4; end
                S_QDG+4: begin p_q2gz <= mq; ma <= q3; mb <= gy; st <= S_QDG+5; end
                S_QDG+5: begin p_q3gy <= mq; ma <= q0; mb <= gy; st <= S_QDG+6; end
                S_QDG+6: begin p_q0gy <= mq; ma <= q1; mb <= gz; st <= S_QDG+7; end
                S_QDG+7: begin p_q1gz <= mq; ma <= q3; mb <= gx; st <= S_QDG+8; end
                S_QDG+8: begin p_q3gx <= mq; ma <= q0; mb <= gz; st <= S_QDG+9; end
                S_QDG+9: begin p_q0gz <= mq; ma <= q1; mb <= gy; st <= S_QDG+10; end
                S_QDG+10: begin p_q1gy <= mq; ma <= q2; mb <= gx; st <= S_QDG+11; end
                S_QDG+11: begin p_q2gx <= mq; ma <= BETA; mb <= s0; st <= S_QDB; end

                S_QDB:   begin p_bs0 <= mq; ma <= BETA; mb <= s1; st <= S_QDB+1; end
                S_QDB+1: begin p_bs1 <= mq; ma <= BETA; mb <= s2; st <= S_QDB+2; end
                S_QDB+2: begin p_bs2 <= mq; ma <= BETA; mb <= s3; st <= S_QDB+3; end
                S_QDB+3: begin p_bs3 <= mq; ma <= qd0_w; mb <= DT; st <= S_INTEG; end

                S_INTEG:   begin q0 <= q0 + mq; ma <= qd1_w; mb <= DT; st <= S_INTEG+1; end
                S_INTEG+1: begin q1 <= q1 + mq; ma <= qd2_w; mb <= DT; st <= S_INTEG+2; end
                S_INTEG+2: begin q2 <= q2 + mq; ma <= qd3_w; mb <= DT; st <= S_INTEG+3; end
                S_INTEG+3: begin q3 <= q3 + mq; ma <= q0; mb <= q0; acc64 <= 64'sd0; st <= S_QNORM; end

                S_QNORM:   begin acc64 <= mp; ma <= q1; mb <= q1; st <= S_QNORM+1; end
                S_QNORM+1: begin acc64 <= acc64 + mp; ma <= q2; mb <= q2; st <= S_QNORM+2; end
                S_QNORM+2: begin acc64 <= acc64 + mp; ma <= q3; mb <= q3; st <= S_QNORM+3; end
                S_QNORM+3: begin
                    acc64 <= acc64 + mp;
                    ma <= q0; mb <= ((3 * ONE) - ((acc64 + mp) >>> 16)) >>> 1;
                    st <= S_QNF;
                end

                S_QNF:   begin q0 <= mq; ma <= q1; mb <= ((3 * ONE) - (acc64 >>> 16)) >>> 1; st <= S_QNF+1; end
                S_QNF+1: begin q1 <= mq; ma <= q2; mb <= ((3 * ONE) - (acc64 >>> 16)) >>> 1; st <= S_QNF+2; end
                S_QNF+2: begin q2 <= mq; ma <= q3; mb <= ((3 * ONE) - (acc64 >>> 16)) >>> 1; st <= S_QNF+3; end
                S_QNF+3: begin q3 <= mq; ma <= q1; mb <= q1; st <= S_QSQ; end

                // 重算 q1q1/q2q2/q3q3（供欧拉角与旋转矩阵，用归一化后的新 q）
                S_QSQ:   begin q1q1 <= mq; ma <= q2; mb <= q2; st <= S_QSQ+1; end
                S_QSQ+1: begin q2q2 <= mq; ma <= q3; mb <= q3; st <= S_QSQ+2; end
                S_QSQ+2: begin q3q3 <= mq; ma <= q0; mb <= q1; st <= S_EULER; end

                S_EULER:   begin m01 <= mq; ma <= q0; mb <= q2; st <= S_EULER+1; end
                S_EULER+1: begin m02 <= mq; ma <= q0; mb <= q3; st <= S_EULER+2; end
                S_EULER+2: begin m03 <= mq; ma <= q1; mb <= q2; st <= S_EULER+3; end
                S_EULER+3: begin m12 <= mq; ma <= q1; mb <= q3; st <= S_EULER+4; end
                S_EULER+4: begin m13 <= mq; ma <= q2; mb <= q3; st <= S_EULER+5; end
                S_EULER+5: begin
                    m23 <= mq;
                    vp <= (m02 - m13) << 1;
                    ma <= (m02 - m13) << 1; mb <= (m02 - m13) << 1;
                    st <= S_PITCH;
                end

                S_PITCH: begin
                    w_pitch <= ONE - mq;
                    isqrt_in <= (ONE - mq) == 32'd0 ? 32'd1 : (ONE - mq);
                    isqrt_start <= 1'b1;
                    st <= S_PWAIT;
                end
                S_PWAIT: if (isqrt_done) begin recip_pitch <= isqrt_y; ma <= w_pitch; mb <= isqrt_y; st <= S_SQRTW; end

                S_SQRTW: begin sqrt_w <= mq; ma <= r00; mb <= ax_g; st <= S_ROT; end

                S_ROT:   begin awx <= mq; ma <= r01; mb <= ay_g; st <= S_ROT+1; end
                S_ROT+1: begin awx <= awx + mq; ma <= r02; mb <= az_g; st <= S_ROT+2; end
                S_ROT+2: begin awx <= awx + mq; ma <= r10; mb <= ax_g; st <= S_ROT+3; end
                S_ROT+3: begin awy <= mq; ma <= r11; mb <= ay_g; st <= S_ROT+4; end
                S_ROT+4: begin awy <= awy + mq; ma <= r12; mb <= az_g; st <= S_ROT+5; end
                S_ROT+5: begin awy <= awy + mq; ma <= r20; mb <= ax_g; st <= S_ROT+6; end
                S_ROT+6: begin awz <= mq; ma <= r21; mb <= ay_g; st <= S_ROT+7; end
                S_ROT+7: begin awz <= awz + mq; ma <= r22; mb <= az_g; st <= S_ROT+8; end
                S_ROT+8: begin awz <= awz + mq - ONE; ma <= awx; mb <= G_CONST; st <= S_GRAV; end

                S_GRAV:   begin awx <= mq; ma <= awy; mb <= G_CONST; st <= S_GRAV+1; end
                S_GRAV+1: begin awy <= mq; ma <= awz; mb <= G_CONST; st <= S_GRAV+2; end
                S_GRAV+2: begin awz <= mq; cordic_start <= 1'b1; st <= S_CWAIT; end

                S_CWAIT: if (cordic_done) begin
                    roll_o <= roll_deg; pitch_o <= pitch_deg; yaw_o <= yaw_deg;
                    awx_o <= awx; awy_o <= awy; awz_o <= awz;
                    motion_o <= ((x_acc < MOTION_LO) || (x_acc > MOTION_HI)) ? 1'b1 : 1'b0;
                    valid_o <= 1'b1;
                    st <= S_IDLE;
                end

                default: st <= S_IDLE;
            endcase
        end
    end

    assign roll           = roll_o;
    assign pitch          = pitch_o;
    assign yaw            = yaw_o;
    assign accel_world_x  = awx_o;
    assign accel_world_y  = awy_o;
    assign accel_world_z  = awz_o;
    assign motion         = motion_o;
    assign data_valid_out = valid_o;
    assign q0_dbg = q0; assign q1_dbg = q1; assign q2_dbg = q2; assign q3_dbg = q3;

endmodule
