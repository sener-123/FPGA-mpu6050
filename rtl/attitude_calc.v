//------------------------------------------------------------------------------
// 文件名 : attitude_calc.v
// 功能   : MPU6050 姿态解算（原始数据 → 加速度/角速度/欧拉角）
//          含上电零偏校准，姿态由四元数引擎（quaternion_ahrs）解算
// 日期   : 2026-09-24（2026-09-26 增加零偏校准；改用四元数姿态解算）
//------------------------------------------------------------------------------
/* @brief  MPU6050 姿态解算（全部定点运算）
 *         输入原始传感器数据（500Hz 帧同步），输出：
 *         1) 三轴加速度，Q16.16，单位 g（1g = 0x10000）
 *         2) 三轴角速度，Q16.16，单位 °/s（1°/s = 0x10000）
 *         3) 欧拉角 roll/pitch/yaw，Q16.16，单位 °
 *         算法流程：
 *         - 上电零偏校准：复位后前 512 帧（约1秒，需保持板子水平静止）累加
 *           各轴原始值取平均，之后每帧减去零偏（az 以 1g=16384 归一）
 *         - 加速度 8 帧滑动平均低通（16ms 窗口）：抑制传感器噪声被四元数
 *           修正项放大注入三个轴（含 yaw），避免静止时角度抖动
 *         - 四元数姿态引擎（Mahony 式互补滤波）：
 *           修正项 e = a×v（加速度计测量与四元数估计重力方向叉积）在任意姿态
 *           下均有效，四元数可连续跟踪 360° 姿态，无欧拉角奇点/折叠
 *         - 欧拉角（由旋转矩阵元素经 CORDIC atan2 换算）：
 *           roll  = atan2(R32, R33)   范围 (-180°,180°]
 *           pitch = atan2(-R31, R33)  范围 (-180°,180°]，全范围倾斜角，
 *                                     超过 ±90° 不再折叠（与四元数状态一致）
 *           yaw   = atan2(R21, R11)   范围 (-180°,180°]
 * @param  LATENCY : 总流水延迟约150个时钟（四元数状态机约132 + CORDIC 17 + 寄存1）
 * @param  accel_x/y/z_raw[15:0] : 原始加速度（来自 mpu6050_driver）
 * @param  gyro_x/y/z_raw[15:0]  : 原始角速度（来自 mpu6050_driver）
 * @param  data_valid_in : 帧同步脉冲（500Hz）
 * @return accel_x/y/z_g[31:0]  : 校准后加速度 Q16.16，单位 g
 * @return gyro_x/y/z_dps[31:0] : 校准后角速度 Q16.16，单位 °/s（raw/131 → raw×500）
 * @return roll/pitch/yaw[31:0] : 姿态角 Q16.16，单位 °
 * @return data_valid_out : 输出帧同步（校准完成后才有效）
 * @return calib_done : 零偏校准完成标志（高有效）
 * @note   1) 校准期间（复位后约1秒）必须保持板子**水平静止**，否则零偏不准；
 *            校准期间输出保持 0，校准完成后约 1 秒滤波收敛
 *         2) pitch 采用全范围定义（绕 Y 轴倾斜角），在极端组合姿态（roll 接近
 *            ±90°）时与标准欧拉角定义存在差异，属欧拉角表示固有特性
 *         3) yaw 无绝对参考（MPU6050 无磁力计），仅由陀螺积分维持，
 *            零偏校准后漂移显著减小，上电时从 0 开始
 *         4) 角速度显示 ×500（65536/131=500.275 取整，误差 0.055%），
 *            与四元数引擎内部的精确定标（×572@Q31.32，误差 0.04%）相互独立
 */
module attitude_calc #(
    parameter LATENCY = 96
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
    output wire signed [31:0] accel_x_g,
    output wire signed [31:0] accel_y_g,
    output wire signed [31:0] accel_z_g,
    output wire signed [31:0] gyro_x_dps,
    output wire signed [31:0] gyro_y_dps,
    output wire signed [31:0] gyro_z_dps,
    output wire signed [31:0] roll,
    output wire signed [31:0] pitch,
    output wire signed [31:0] yaw,
    output wire        data_valid_out,
    output reg         calib_done
);

    //====================== 采样寄存器（帧同步锁存） ======================
    reg signed [15:0] ax_r, ay_r, az_r;   // 加速度原始值
    reg signed [15:0] gx_r, gy_r, gz_r;   // 角速度原始值

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ax_r <= 16'sd0; ay_r <= 16'sd0; az_r <= 16'sd0;
            gx_r <= 16'sd0; gy_r <= 16'sd0; gz_r <= 16'sd0;
        end else if (data_valid_in) begin
            ax_r <= accel_x_raw;
            ay_r <= accel_y_raw;
            az_r <= accel_z_raw;
            gx_r <= gyro_x_raw;
            gy_r <= gyro_y_raw;
            gz_r <= gyro_z_raw;
        end
    end

    //====================== 上电零偏校准 ======================
    // 复位后前 CALIB_FRAMES 帧（约1秒，板子须保持水平静止）累加各轴原始值，
    // 之后每帧减去零偏。az 零偏以 1g=16384 为基准归一
    localparam CALIB_FRAMES = 10'd512;    // 512帧 ≈ 1.024s @500Hz

    reg signed [24:0] gx_acc, gy_acc, gz_acc;
    reg signed [24:0] ax_acc, ay_acc, az_acc;
    reg  [9:0]  calib_cnt;
    reg  [15:0] gx_off, gy_off, gz_off;   // 陀螺仪零偏（raw 单位）
    reg  [15:0] ax_off, ay_off, az_off;   // 加速度计零偏（raw 单位）

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            calib_cnt  <= 10'd0;
            calib_done <= 1'b0;
            gx_acc <= 25'sd0; gy_acc <= 25'sd0; gz_acc <= 25'sd0;
            ax_acc <= 25'sd0; ay_acc <= 25'sd0; az_acc <= 25'sd0;
            gx_off <= 16'sd0; gy_off <= 16'sd0; gz_off <= 16'sd0;
            ax_off <= 16'sd0; ay_off <= 16'sd0; az_off <= 16'sd0;
        end else if (data_valid_in && !calib_done) begin
            gx_acc <= gx_acc + gx_r;
            gy_acc <= gy_acc + gy_r;
            gz_acc <= gz_acc + gz_r;
            ax_acc <= ax_acc + ax_r;
            ay_acc <= ay_acc + ay_r;
            az_acc <= az_acc + az_r;
            if (calib_cnt == CALIB_FRAMES - 1) begin
                calib_done <= 1'b1;
                gx_off <= gx_acc[24:9];            // 累加和 >> 9 = 平均值
                gy_off <= gy_acc[24:9];
                gz_off <= gz_acc[24:9];
                ax_off <= ax_acc[24:9];
                ay_off <= ay_acc[24:9];
                az_off <= az_acc[24:9] - 16'sd16384;   // az 以 1g 为基准
            end else begin
                calib_cnt <= calib_cnt + 1'b1;
            end
        end
    end

    //====================== 零偏校准后的值 ======================
    // 角速度：减去零偏后符号扩展为32位
    wire signed [31:0] gx_cal = $signed({{16{gx_r[15]}}, gx_r}) - $signed({{16{gx_off[15]}}, gx_off});
    wire signed [31:0] gy_cal = $signed({{16{gy_r[15]}}, gy_r}) - $signed({{16{gy_off[15]}}, gy_off});
    wire signed [31:0] gz_cal = $signed({{16{gz_r[15]}}, gz_r}) - $signed({{16{gz_off[15]}}, gz_off});

    // 加速度：减去零偏（两操作数显式符号扩展后再相减——ax_r/ax_off 是补码值，
    // 直接写 ax_r-ax_off 会按无符号运算，负差值被当成 +65k 大数，姿态与显示全错）
    wire signed [17:0] ax_d = $signed({ax_r[15], ax_r}) - $signed({ax_off[15], ax_off});
    wire signed [17:0] ay_d = $signed({ay_r[15], ay_r}) - $signed({ay_off[15], ay_off});
    wire signed [17:0] az_d = $signed({az_r[15], az_r}) - $signed({az_off[15], az_off});

    //====================== 加速度 8 帧滑动平均（低通，16ms 窗口） ======================
    // MPU6050 ±2g 加速度计噪声（DLPF=1，184Hz 带宽）RMS 约 5mg：若直接注入四元数
    // 修正项，会被 Kp 放大成约 1°/s 级的角速度抖动，三个轴（含 yaw）静止时乱飘。
    // 8 帧平均把噪声带宽压到约 31Hz（噪声降约 2.8 倍），相位延迟仅 16ms，
    // 对姿态响应无影响。平均输出 = sum/8（raw 单位），再 ×4 即 Q16.16
    reg signed [17:0] ax_h [0:7], ay_h [0:7], az_h [0:7];   // 8 级移位链
    reg signed [21:0] ax_sum, ay_sum, az_sum;               // 滑动窗口和（8×±65535）
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (k = 0; k < 8; k = k + 1) begin
                ax_h[k] <= 18'sd0; ay_h[k] <= 18'sd0; az_h[k] <= 18'sd0;
            end
            ax_sum <= 22'sd0; ay_sum <= 22'sd0; az_sum <= 22'sd0;
        end else if (data_valid_in) begin
            ax_h[0] <= ax_d; ay_h[0] <= ay_d; az_h[0] <= az_d;
            for (k = 1; k < 8; k = k + 1) begin
                ax_h[k] <= ax_h[k-1]; ay_h[k] <= ay_h[k-1]; az_h[k] <= az_h[k-1];
            end
            ax_sum <= ax_sum + ax_d - ax_h[7];
            ay_sum <= ay_sum + ay_d - ay_h[7];
            az_sum <= az_sum + az_d - az_h[7];
        end
    end

    // 平均后换算 Q16.16：avg_raw×4 = sum/8×4 = sum/2 = sum<<<1
    wire signed [31:0] ax_q16 = $signed(ax_sum) <<< 1;
    wire signed [31:0] ay_q16 = $signed(ay_sum) <<< 1;
    wire signed [31:0] az_q16 = $signed(az_sum) <<< 1;

    // 角速度显示：°/s = raw/131，Q16.16 定点 = raw×65536/131 ≈ raw×500（误差 0.055%）
    wire signed [31:0] gx_q16 = gx_cal * 32'sd500;
    wire signed [31:0] gy_q16 = gy_cal * 32'sd500;
    wire signed [31:0] gz_q16 = gz_cal * 32'sd500;

    //====================== 四元数姿态引擎 ======================
    wire signed [31:0] r31, r32, r33, r21, r11;
    wire q_valid;

    quaternion_ahrs u_quaternion_ahrs (
        .clk         (clk),
        .rst_n       (rst_n),
        .accel_x     (ax_q16),
        .accel_y     (ay_q16),
        .accel_z     (az_q16),
        .gyro_x      (gx_cal),
        .gyro_y      (gy_cal),
        .gyro_z      (gz_cal),
        .frame_valid (data_valid_in && calib_done),
        .q0_out      (),
        .q1_out      (),
        .q2_out      (),
        .q3_out      (),
        .r31         (r31),
        .r32         (r32),
        .r33         (r33),
        .r21         (r21),
        .r11         (r11),
        .q_valid     (q_valid)
    );

    //====================== 欧拉角换算（CORDIC atan2，三路并行） ======================
    // 旋转矩阵元素为 Q16.15（1.0=32768），两输入同比例，atan2 结果不受定标影响
    wire signed [31:0] roll_deg, pitch_deg, yaw_deg;
    wire roll_valid, pitch_valid, yaw_valid;

    cordic #(.STAGES(16)) u_cordic_roll (
        .clk(clk), .rst_n(rst_n),
        .x_in(r33), .y_in(r32),
        .valid_in(q_valid),
        .angle(roll_deg), .mag(),
        .valid_out(roll_valid)
    );
    cordic #(.STAGES(16)) u_cordic_pitch (
        .clk(clk), .rst_n(rst_n),
        .x_in(r33), .y_in(-r31),         // 全范围：atan2(-R31, R33) ∈ ±180°
        .valid_in(q_valid),
        .angle(pitch_deg), .mag(),
        .valid_out(pitch_valid)
    );
    cordic #(.STAGES(16)) u_cordic_yaw (
        .clk(clk), .rst_n(rst_n),
        .x_in(r11), .y_in(r21),
        .valid_in(q_valid),
        .angle(yaw_deg), .mag(),
        .valid_out(yaw_valid)
    );

    //====================== 输出寄存（与 data_valid_out 对齐） ======================
    // 校准期间输出保持 0，data_valid_out 亦不产生脉冲
    reg signed [31:0] accel_x_o, accel_y_o, accel_z_o;
    reg signed [31:0] gyro_x_o,  gyro_y_o,  gyro_z_o;
    reg signed [31:0] roll_o, pitch_o, yaw_o;
    reg valid_o;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            accel_x_o <= 32'sd0; accel_y_o <= 32'sd0; accel_z_o <= 32'sd0;
            gyro_x_o  <= 32'sd0; gyro_y_o  <= 32'sd0; gyro_z_o  <= 32'sd0;
            roll_o    <= 32'sd0; pitch_o   <= 32'sd0; yaw_o     <= 32'sd0;
            valid_o   <= 1'b0;
        end else begin
            accel_x_o <= calib_done ? ax_q16 : 32'sd0;
            accel_y_o <= calib_done ? ay_q16 : 32'sd0;
            accel_z_o <= calib_done ? az_q16 : 32'sd0;
            gyro_x_o  <= calib_done ? gx_q16 : 32'sd0;
            gyro_y_o  <= calib_done ? gy_q16 : 32'sd0;
            gyro_z_o  <= calib_done ? gz_q16 : 32'sd0;
            roll_o    <= roll_deg;
            pitch_o   <= pitch_deg;
            yaw_o     <= yaw_deg;
            valid_o   <= roll_valid;
        end
    end

    assign accel_x_g     = accel_x_o;
    assign accel_y_g     = accel_y_o;
    assign accel_z_g     = accel_z_o;
    assign gyro_x_dps    = gyro_x_o;
    assign gyro_y_dps    = gyro_y_o;
    assign gyro_z_dps    = gyro_z_o;
    assign roll          = roll_o;
    assign pitch         = pitch_o;
    assign yaw           = yaw_o;
    assign data_valid_out = valid_o;

endmodule
