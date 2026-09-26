//------------------------------------------------------------------------------
// 文件名 : attitude_calc.v
// 功能   : MPU6050 姿态解算（原始数据 → 加速度/角速度/欧拉角）
// 日期   : 2026-09-24
//------------------------------------------------------------------------------
/* @brief  MPU6050 姿态解算（全部 Q16.16 定点运算，1符号+15整数+16小数）
 *         输入原始传感器数据（500Hz 帧同步），输出：
 *         1) 三轴加速度，Q16.16，单位 g（1g = 0x10000）
 *         2) 三轴角速度，Q16.16，单位 °/s（1°/s = 0x10000）
 *         3) 欧拉角 roll/pitch/yaw，Q16.16，单位 °
 *         算法：
 *         - 加速度计静态角（CORDIC 向量模式实现）：
 *           roll  = atan2(ay, az)
 *           pitch = atan2(-ax, √(ay²+az²))
 *         - 互补滤波：angle = 0.98×(angle + gyro×dt) + 0.02×accel_angle
 *           抑制陀螺仪温漂，同时滤除加速度计的振动/运动干扰
 *         - yaw = 纯陀螺仪 z 轴积分（无磁力计，见 @note）
 * @param  LATENCY : 流水线总延迟，固定为35个时钟（仅文档用，不影响逻辑）
 * @param  accel_x/y/z_raw[15:0] : 原始加速度（来自 mpu6050_driver）
 * @param  gyro_x/y/z_raw[15:0]  : 原始角速度（来自 mpu6050_driver）
 * @param  data_valid_in : 帧同步脉冲（500Hz）
 * @return accel_x/y/z_g[31:0]  : 加速度 Q16.16，单位 g（g = raw/16384 → raw<<2）
 * @return gyro_x/y/z_dps[31:0] : 角速度 Q16.16，单位 °/s（°/s = raw/131 → raw×500）
 * @return roll/pitch/yaw[31:0] : 姿态角 Q16.16，单位 °，roll/yaw ∈ (-180,180]
 * @return data_valid_out : 输出帧同步（滞后输入35个时钟）
 * @note   1) 每帧角度增量推导：°/s = raw/131，dt = 2ms，
 *            增量(Q16.16) = raw/131 × 0.002 × 65536 = raw × 1.00055 ≈ raw，
 *            故直接用 raw 累加（误差 0.055%）；角速度输出 ×500 同源误差
 *         2) yaw 无绝对参考（MPU6050 无磁力计），仅积分角速度，
 *            会随时间漂移，上电时从 0 开始
 *         3) pitch 定义在 (-90°, 90°)（atan2 公式固有范围），
 *            板子翻转超过 ±90° 时请改用四元数方案
 *         4) 互补滤波 K=0.98 适用于一般应用；上电后角度约 0.2s 收敛
 *         5) 滤波初值 0 与真实姿态的收敛时间常数 ≈ 1/(1-K) 帧 ≈ 100ms
 */
module attitude_calc #(
    parameter LATENCY = 35
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
    output wire        data_valid_out
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

    //====================== 原始值 → 物理量（Q16.16） ======================
    // 加速度：g = raw/16384（±2g 量程），Q16.16 定点 = raw×4（精确移位）
    wire signed [31:0] ax_q16 = $signed({ax_r, 2'b00});
    wire signed [31:0] ay_q16 = $signed({ay_r, 2'b00});
    wire signed [31:0] az_q16 = $signed({az_r, 2'b00});
    // 角速度：°/s = raw/131（±250°/s 量程），Q16.16 定点 = raw×65536/131
    //        ≈ raw×500（65536/131 = 500.275，取500，误差 0.055%）
    wire signed [31:0] gx_q16 = gx_r * 32'sd500;
    wire signed [31:0] gy_q16 = gy_r * 32'sd500;
    wire signed [31:0] gz_q16 = gz_r * 32'sd500;

    //====================== CORDIC 实例1 ======================
    // roll = atan2(ay, az)，同时得到 √(ay²+az²) 供 pitch 使用
    wire signed [31:0] roll_deg, ayaz_mag;
    wire cordic1_valid;
    cordic #(.STAGES(16)) u_cordic_roll (
        .clk       (clk),
        .rst_n     (rst_n),
        .x_in      (az_q16),
        .y_in      (ay_q16),
        .valid_in  (data_valid_in),
        .angle     (roll_deg),
        .mag       (ayaz_mag),
        .valid_out (cordic1_valid)
    );

    //====================== CORDIC 实例2 ======================
    // pitch = atan2(-ax, √(ay²+az²))，x=模长≥0，无需象限修正
    wire signed [31:0] pitch_deg;
    wire cordic2_valid;
    cordic #(.STAGES(16)) u_cordic_pitch (
        .clk       (clk),
        .rst_n     (rst_n),
        .x_in      (ayaz_mag),
        .y_in      (-ax_q16),
        .valid_in  (cordic1_valid),
        .angle     (pitch_deg),
        .mag       (),               // 模长未使用
        .valid_out (cordic2_valid)
    );

    //====================== 互补滤波 + 积分 ======================
    // 滤波公式：angle = 0.98×(angle + gyro×dt) + 0.02×accel_angle
    // 每帧陀螺增量 ≈ raw（推导见模块头 @note 1）
    localparam signed [31:0] K_C   = 32'sd64225;   // K   = 0.98 的 Q16.16
    localparam signed [31:0] K_1MC = 32'sd1311;    // 1-K = 0.02 的 Q16.16

    // 原始值符号扩展到32位（每帧角度增量直接累加）
    wire signed [31:0] gx_ext = $signed({{16{gx_r[15]}}, gx_r});
    wire signed [31:0] gy_ext = $signed({{16{gy_r[15]}}, gy_r});
    wire signed [31:0] gz_ext = $signed({{16{gz_r[15]}}, gz_r});

    // 滤波状态寄存器（须在使用它的连续赋值之前声明）
    reg signed [31:0] roll_f, pitch_f, yaw_f;

    // 陀螺预测角（上一帧角度 + 本帧角速度积分）
    wire signed [31:0] roll_pred  = roll_f  + gx_ext;
    wire signed [31:0] pitch_pred = pitch_f + gy_ext;

    // 64位中间量避免溢出（最大 ±180°×64225 ≈ ±7.6e11 < 2^63）
    wire signed [63:0] roll_prod_k   = roll_pred  * K_C;
    wire signed [63:0] pitch_prod_k  = pitch_pred * K_C;
    wire signed [63:0] roll_prod_1k  = roll_deg  * K_1MC;
    wire signed [63:0] pitch_prod_1k = pitch_deg * K_1MC;

    // 互补滤波输出（Q16.16 度）
    wire signed [31:0] roll_next  = (roll_prod_k  >>> 16) + (roll_prod_1k  >>> 16);
    wire signed [31:0] pitch_next = (pitch_prod_k >>> 16) + (pitch_prod_1k >>> 16);
    // yaw：纯陀螺积分（无加速度计参考，见 @note 2）
    wire signed [31:0] yaw_next   = yaw_f + gz_ext;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            roll_f  <= 32'sd0;
            pitch_f <= 32'sd0;
            yaw_f   <= 32'sd0;
        end else if (cordic2_valid) begin
            roll_f  <= roll_next;
            pitch_f <= pitch_next;
            yaw_f   <= yaw_next;
        end
    end

    //====================== 输出寄存（与 data_valid_out 对齐） ======================
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
            accel_x_o <= ax_q16;
            accel_y_o <= ay_q16;
            accel_z_o <= az_q16;
            gyro_x_o  <= gx_q16;
            gyro_y_o  <= gy_q16;
            gyro_z_o  <= gz_q16;
            roll_o    <= roll_next;
            pitch_o   <= pitch_next;
            yaw_o     <= yaw_next;
            valid_o   <= cordic2_valid;
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
