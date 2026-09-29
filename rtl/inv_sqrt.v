//------------------------------------------------------------------------------
// 文件名 : inv_sqrt.v
// 功能   : 定点倒数平方根 1/sqrt(x)，Q16.16 输入输出（时分复用，单乘法器）
// 日期   : 2026-09-29
//------------------------------------------------------------------------------
/* @brief  定点倒数平方根（用于 Madgwick 归一化），等价 y = 2^24/sqrt(x)
 *         算法：归一化 -> 16 项 LUT 初值 -> 2 次牛顿迭代（除法自由）
 *           y <- y * (3 - x*y^2) / 2
 *         资源：单共享乘法器 mul32x32 时分复用（约 14 个时钟），几乎不占 LUT
 * @param  x[31:0]   : 输入，Q16.16 正整数（调用方保证 >= 2）
 * @param  valid_in  : 输入有效脉冲
 * @return y[31:0]   : 输出，Q16.16，y ≈ 1/sqrt(x/2^16)
 * @return valid_out : 输出有效脉冲（滞后 valid_in 约 14 个时钟）
 */
module inv_sqrt (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [31:0] x,
    input  wire        valid_in,
    output reg  [31:0] y,
    output reg         valid_out
);

    //====================== LUT：g[m4] = 65536/sqrt(1+(m4+0.5)/16) ============
    function [15:0] lut;
        input [3:0] m4;
        begin
            case (m4)
                4'd0 : lut = 16'd64535;
                4'd1 : lut = 16'd62664;
                4'd2 : lut = 16'd60947;
                4'd3 : lut = 16'd59364;
                4'd4 : lut = 16'd57898;
                4'd5 : lut = 16'd56535;
                4'd6 : lut = 16'd55265;
                4'd7 : lut = 16'd54076;
                4'd8 : lut = 16'd52961;
                4'd9 : lut = 16'd51912;
                4'd10: lut = 16'd50923;
                4'd11: lut = 16'd49989;
                4'd12: lut = 16'd49104;
                4'd13: lut = 16'd48265;
                4'd14: lut = 16'd47467;
                default: lut = 16'd46707;
            endcase
        end
    endfunction

    function [5:0] clz32;
        input [31:0] v;
        reg [1:0] b;
        reg [7:0] bs;
        reg [5:0] r;
        begin
            if      (v[31:24] != 8'd0) b = 2'd0;
            else if (v[23:16] != 8'd0) b = 2'd1;
            else if (v[15: 8] != 8'd0) b = 2'd2;
            else                        b = 2'd3;
            case (b)
                2'd0: bs = v[31:24];
                2'd1: bs = v[23:16];
                2'd2: bs = v[15: 8];
                default: bs = v[7:0];
            endcase
            if      (bs[7]) r = 6'd0;
            else if (bs[6]) r = 6'd1;
            else if (bs[5]) r = 6'd2;
            else if (bs[4]) r = 6'd3;
            else if (bs[3]) r = 6'd4;
            else if (bs[2]) r = 6'd5;
            else if (bs[1]) r = 6'd6;
            else            r = 6'd7;
            clz32 = {b, 3'd0} + r;
        end
    endfunction

    //====================== 初值（组合，基于钳位后的输入） ======================
    wire [31:0] x_clamp = (x < 32'd4) ? 32'd4 : x;
    wire [5:0]  s_w   = 6'd31 - clz32(x_clamp);
    wire [31:0] xh_w  = x_clamp << clz32(x_clamp);
    wire [3:0]  m4_w  = xh_w[30:27];
    wire [15:0] g_w   = lut(m4_w);
    wire [3:0]  a_w   = s_w[5:1];
    wire        odd_w = s_w[0];
    wire [31:0] g_s2  = ({32'd0, g_w} * 64'd92682) >> 16;   // g*sqrt(2)
    wire [31:0] g_sel = odd_w ? g_s2 : {16'd0, g_w};
    wire [31:0] y0_w =
        odd_w ? ( (a_w <= 4'd7) ? (g_s2 << (7 - a_w)) : (g_s2 >> (a_w - 4'd7)) )
              : ( (a_w <= 4'd8) ? (g_sel << (8 - a_w)) : (g_sel >> (a_w - 4'd8)) );

    //====================== 共享乘法器 ======================
    reg [31:0] x0, y0, y1;
    reg signed [31:0] ma, mb;
    wire signed [63:0] mp = $signed({ {32{ma[31]}}, ma }) * $signed({ {32{mb[31]}}, mb });
    reg [63:0] t;

    //====================== 状态机 ======================
    localparam S_IDLE = 4'd0;
    localparam S_N1A  = 4'd1;   // 牛顿1：y0*y0
    localparam S_N1B  = 4'd2;   // x*y2
    localparam S_N1C  = 4'd3;   // y0*num
    localparam S_N2A  = 4'd4;   // 牛顿2：y1*y1
    localparam S_N2B  = 4'd5;   // x*y2b
    localparam S_N2C  = 4'd6;   // y1*num
    reg [3:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; y <= 32'd0; valid_out <= 1'b0;
            x0 <= 32'd0; y0 <= 32'd0; y1 <= 32'd0;
            ma <= 32'd0; mb <= 32'd0; t <= 64'd0;
        end else begin
            valid_out <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (valid_in) begin
                        x0 <= x_clamp;   // 钳位，避免 y^2>>16 溢出 32 位
                        y0 <= y0_w;      // 初值（由 x_clamp 组合求出）
                        ma  <= $signed(y0_w);
                        mb  <= $signed(y0_w);
                        state <= S_N1A;
                    end
                end
                // 牛顿 1
                S_N1A: begin t <= mp; ma <= $signed(x0); mb <= mp[47:16]; state <= S_N1B; end
                S_N1B: begin t <= mp; ma <= $signed(y0); mb <= 32'h30000 - mp[47:16]; state <= S_N1C; end
                S_N1C: begin y1 <= mp[48:17]; ma <= mp[48:17]; mb <= mp[48:17]; state <= S_N2A; end
                // 牛顿 2
                S_N2A: begin t <= mp; ma <= $signed(x0); mb <= mp[47:16]; state <= S_N2B; end
                S_N2B: begin t <= mp; ma <= y1; mb <= 32'h30000 - mp[47:16]; state <= S_N2C; end
                S_N2C: begin y <= mp[48:17]; valid_out <= 1'b1; state <= S_IDLE; end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
