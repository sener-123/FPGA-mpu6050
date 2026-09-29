//------------------------------------------------------------------------------
// 文件名 : displacement_calc.v
// 功能   : 世界系线性加速度双重积分 -> 速度 + 位移（Q16.16 定点）
// 日期   : 2026-09-29
//------------------------------------------------------------------------------
/* @brief  位移估计（惯性导航航位推算，无绝对参考，仅供演示）
 *         输入世界系线性加速度（去重力，m/s²），积分得速度（m/s）与位置（m）。
 *         采用零速修正（ZUPT）：当 madgwick 判定 |a|≈1g（近似静止）时速度清零、
 *         位置保持，抑制双重积分的二次发散。
 * @param  accel_x/y/z[31:0] : 世界系线性加速度 Q16.16，单位 m/s²（去重力）
 * @param  motion : 1=运动，0=静止（|a_body|≈1g，触发 ZUPT）
 * @param  data_valid_in : 帧同步脉冲（500Hz）
 * @return pos_x/y/z[31:0] : 位置 Q16.16，单位 m
 * @return vel_x/y/z[31:0] : 速度 Q16.16，单位 m/s
 * @return data_valid_out  : 输出帧同步
 * @note   1) 积分 dt=2ms：dv = a*0.002 = a*131>>16；dp = v*131>>16
 *         2) 位置用更新前速度（欧拉前向），与 Python 模型一致
 *         3) ⚠️ 纯惯导双重积分误差随时间二次增长，无外部参考时会漂移，
 *            本模块为演示级精度；运动检测的 1g 带对水平匀速运动不敏感
 */
module displacement_calc (
    input  wire        clk,
    input  wire        rst_n,
    input  wire signed [31:0] accel_x,
    input  wire signed [31:0] accel_y,
    input  wire signed [31:0] accel_z,
    input  wire        motion,
    input  wire        data_valid_in,
    output wire signed [31:0] pos_x,
    output wire signed [31:0] pos_y,
    output wire signed [31:0] pos_z,
    output wire signed [31:0] vel_x,
    output wire signed [31:0] vel_y,
    output wire signed [31:0] vel_z,
    output wire        data_valid_out
);

    localparam signed [31:0] DT = 32'sd131;   // 0.002 s * 65536

    // Q16.16 x Q16.16 -> Q16.16（64 位中间量）
    function signed [63:0] smul64;
        input signed [31:0] a;
        input signed [31:0] b;
        begin
            smul64 = $signed({ {32{a[31]}}, a }) * $signed({ {32{b[31]}}, b });
        end
    endfunction
    function signed [31:0] mulq;
        input signed [31:0] a;
        input signed [31:0] b;
        begin
            mulq = smul64(a, b) >>> 16;
        end
    endfunction

    reg signed [31:0] vx, vy, vz;
    reg signed [31:0] px, py, pz;
    reg valid_o;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vx <= 32'sd0; vy <= 32'sd0; vz <= 32'sd0;
            px <= 32'sd0; py <= 32'sd0; pz <= 32'sd0;
            valid_o <= 1'b0;
        end else begin
            valid_o <= 1'b0;
            if (data_valid_in) begin
                if (motion) begin
                    vx <= vx + mulq(accel_x, DT);
                    vy <= vy + mulq(accel_y, DT);
                    vz <= vz + mulq(accel_z, DT);
                    // 位置用更新前速度（vx 仍为旧值，非阻塞）
                    px <= px + mulq(vx, DT);
                    py <= py + mulq(vy, DT);
                    pz <= pz + mulq(vz, DT);
                end else begin
                    // ZUPT：静止 -> 速度清零，位置保持
                    vx <= 32'sd0; vy <= 32'sd0; vz <= 32'sd0;
                end
                valid_o <= 1'b1;
            end
        end
    end

    assign pos_x = px; assign pos_y = py; assign pos_z = pz;
    assign vel_x = vx; assign vel_y = vy; assign vel_z = vz;
    assign data_valid_out = valid_o;

endmodule
