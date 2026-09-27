//------------------------------------------------------------------------------
// 文件名 : cordic.v
// 功能   : 定点 CORDIC（向量模式）流水线，实现 atan2 与模长（开方）
// 日期   : 2026-09-24
//------------------------------------------------------------------------------
/* @brief  定点 CORDIC（向量模式，16级流水线）
 *         输入向量 (x_in, y_in)，输出：
 *         angle = atan2(y_in, x_in)，Q16.16，单位：度，范围 (-180°, 180°]
 *         mag   = √(x_in² + y_in²)，Q16.16
 *         迭代公式（向量模式，把 y 向 0 逼近）：
 *            y>=0 时：x' = x + (y>>>i);  y' = y - (x>>>i);  a' = a + atan(2^-i)
 *            y<0  时：x' = x - (y>>>i);  y' = y + (x>>>i);  a' = a - atan(2^-i)
 * @param  STAGES : 迭代级数，默认16（角度精度约 ±0.003°）
 * @param  x_in/y_in[31:0] : 输入向量，Q16.16 有符号
 * @param  valid_in : 输入有效脉冲
 * @return angle[31:0] : atan2(y,x)，Q16.16，单位度，范围 (-180°, 180°]
 * @return mag[31:0]   : √(x²+y²)，Q16.16
 * @return valid_out   : 输出有效脉冲（滞后输入 STAGES+1=17 个时钟）
 * @note   1) 输入幅度限制：√(x²+y²) < 2^31/1.6468 ≈ 1.3e9；本工程输入为 ±2g
 *            量程加速度（≤1.9e5），远小于上限
 *         2) 角度换算：弧度×180/π，Q16.16 系数 3754936 ≈ 57.2957795131×65536
 *         3) 模长增益补偿：16级迭代增益 1/K = 1.646760258，Q16.16 系数 107923
 *         4) x<0 时先对 (x,y) 同时取反（向量方向不变），CORDIC 算出镜像角度后
 *            再做 ±180° 象限修正，使 atan2 覆盖全部象限
 */
module cordic #(
    parameter STAGES = 16
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire signed [31:0] x_in,
    input  wire signed [31:0] y_in,
    input  wire        valid_in,
    output wire signed [31:0] angle,
    output wire signed [31:0] mag,
    output wire        valid_out
);

    //====================== atan(2^-i) 查找表 ======================
    // 单位：弧度，Q16.16 定点（值 = atan(2^-i) × 65536，四舍五入）
    function [31:0] atan_table;
        input integer i;
        begin
            case (i)
                 0: atan_table = 32'd51472;    // atan(1)      = 0.785398 rad = 45°
                 1: atan_table = 32'd30386;    // atan(0.5)    = 0.463648 rad
                 2: atan_table = 32'd16055;    // atan(0.25)   = 0.244979 rad
                 3: atan_table = 32'd8150;     // atan(0.125)  = 0.124355 rad
                 4: atan_table = 32'd4091;     // atan(0.0625) = 0.062419 rad
                 5: atan_table = 32'd2047;     // atan(0.03125)= 0.031240 rad
                 6: atan_table = 32'd1024;     // atan(0.015625)=0.015624 rad
                 7: atan_table = 32'd512;      // atan(0.0078125)
                 8: atan_table = 32'd256;      // atan(0.00390625)
                 9: atan_table = 32'd128;
                10: atan_table = 32'd64;
                11: atan_table = 32'd32;
                12: atan_table = 32'd16;
                13: atan_table = 32'd8;
                14: atan_table = 32'd4;
                15: atan_table = 32'd2;
                default: atan_table = 32'd0;
            endcase
        end
    endfunction

    //====================== 流水线各级寄存器 ======================
    reg signed [31:0] x_r [0:STAGES];   // x 分量
    reg signed [31:0] y_r [0:STAGES];   // y 分量
    reg signed [31:0] a_r [0:STAGES];   // 累计旋转角（弧度，Q16.16）
    reg [STAGES+1:0]  valid_pipe;       // 有效信号流水
    // 输入符号（象限修正用）：必须与 x_r/y_r/a_r 同步打拍——早期版本只用单级
    // 寄存器，与 a_r[STAGES] 错位 STAGES 拍，输入向量更新瞬间会输出 ±180° 跳变
    reg               x_neg_pipe [0:STAGES];
    reg               y_sign_pipe [0:STAGES];

    //---------------------- 输入级：x<0 时取反，保证 CORDIC 收敛域 ------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_r[0]        <= 32'd0;
            y_r[0]        <= 32'd0;
            a_r[0]        <= 32'd0;
            x_neg_pipe[0] <= 1'b0;
            y_sign_pipe[0] <= 1'b0;
            valid_pipe[0] <= 1'b0;
        end else begin
            valid_pipe[0]  <= valid_in;
            x_neg_pipe[0]  <= x_in[31];
            y_sign_pipe[0] <= y_in[31];
            if (x_in[31]) begin
                x_r[0] <= -x_in;   // 取反后 x>0，|atan2| ≤ 90°，保证收敛
                y_r[0] <= -y_in;
            end else begin
                x_r[0] <= x_in;
                y_r[0] <= y_in;
            end
            a_r[0] <= 32'd0;
        end
    end

    //---------------------- 迭代级：每级一次移位+两次加减 ----------------
    genvar i;
    generate
        for (i = 0; i < STAGES; i = i + 1) begin : cordic_stage
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    x_r[i+1] <= 32'd0;
                    y_r[i+1] <= 32'd0;
                    a_r[i+1] <= 32'd0;
                    x_neg_pipe[i+1]  <= 1'b0;
                    y_sign_pipe[i+1] <= 1'b0;
                end else begin
                    valid_pipe[i+1]  <= valid_pipe[i];
                    x_neg_pipe[i+1]  <= x_neg_pipe[i];
                    y_sign_pipe[i+1] <= y_sign_pipe[i];
                    if (y_r[i][31]) begin
                        // y<0：顺时针旋转，把 y 向 0 逼近
                        x_r[i+1] <= x_r[i] - (y_r[i] >>> i);
                        y_r[i+1] <= y_r[i] + (x_r[i] >>> i);
                        a_r[i+1] <= a_r[i] - atan_table(i);
                    end else begin
                        // y>=0：逆时针旋转
                        x_r[i+1] <= x_r[i] + (y_r[i] >>> i);
                        y_r[i+1] <= y_r[i] - (x_r[i] >>> i);
                        a_r[i+1] <= a_r[i] + atan_table(i);
                    end
                end
            end
        end
    endgenerate

    //====================== 末级换算 ======================
    // 弧度 → 度：×180/π（64位中间量，取 [47:16] 即 >>16）
    wire signed [63:0] deg_prod = a_r[STAGES] * 32'sd3754936;
    wire signed [31:0] deg_raw  = deg_prod[47:16];

    // 象限修正：x<0 时取反后算出的是镜像角 θ'，
    // 原 y<0 → 真实角 = θ'-180°；原 y>0 → 真实角 = θ'+180°（180° = 0xB40000）
    // 注意符号标志与 a_r[STAGES] 同一流水级，确保修正与角度数据对齐
    wire signed [31:0] deg_fix = x_neg_pipe[STAGES]
                                 ? (y_sign_pipe[STAGES] ? deg_raw - 32'sd11796480
                                                        : deg_raw + 32'sd11796480)
                                 : deg_raw;

    // 模长增益补偿：16级迭代增益 1/K = 1.646760258（64位中间量）
    wire signed [63:0] mag_prod = x_r[STAGES] * 32'sd107923;
    wire signed [31:0] mag_raw  = mag_prod[47:16];

    //---------------------- 输出寄存（与数据对齐，再延迟1级）--------------
    reg signed [31:0] angle_reg, mag_reg;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            angle_reg          <= 32'd0;
            mag_reg            <= 32'd0;
            valid_pipe[STAGES+1] <= 1'b0;
        end else begin
            angle_reg            <= deg_fix;
            mag_reg              <= mag_raw;
            valid_pipe[STAGES+1] <= valid_pipe[STAGES];
        end
    end

    assign angle     = angle_reg;
    assign mag       = mag_reg;
    assign valid_out = valid_pipe[STAGES+1];

endmodule
