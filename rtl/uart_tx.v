//------------------------------------------------------------------------------
// 文件名 : uart_tx.v
// 功能   : 通用 UART 串口发送器（8数据位、无校验、1停止位）
// 日期   : 2026-09-24（2026-09-26 修复位时序 bug）
//------------------------------------------------------------------------------
/* @brief  UART 发送器
 *         握手方式：tx_start 发 1 个时钟脉冲触发 1 字节发送，
 *         tx_busy=1 期间忽略新的触发；空闲时输出线保持高电平
 * @param  CLK_FREQ : 主时钟频率(Hz)，默认 50_000_000
 * @param  BAUD     : 波特率，默认 115200
 * @param  tx_data[7:0] : 待发送字节（触发时采样）
 * @param  tx_start : 发送触发（1个时钟脉冲，busy=1 时忽略）
 * @return tx_busy  : 发送中标志
 * @return uart_tx  : 串行数据输出（可直接接 USB-TTL 模块 RX）
 * @note   1) 波特率分频 = CLK_FREQ/BAUD = 434（50MHz/115200 = 434.03，
 *            取 434，误差 0.007%，远小于 UART 允许的 ±2%）
 *         2) 发送时序：起始位(0) → 8 数据位(低位先发) → 停止位(1)
 *         3) 位周期严格等于 BAUD_CNT 个时钟：bit_idx=0 为起始位周期，
 *            0→1 切换时输出 data[0]……8→9 切换时输出停止位，
 *            停止位周期满后才释放 busy（保证停止位 ≥1 个位周期）
 *         4) 2026-09-26 修复：旧版本起始位占 2 个位周期且停止位几乎为 0，
 *            接收端整体错位 1 比特，导致所有字符帧错误（乱码）
 */
module uart_tx #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 115200
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  tx_data,
    input  wire        tx_start,
    output reg         tx_busy,
    output reg         uart_tx
);

    // 每比特时钟数（50MHz/115200 = 434）
    localparam BAUD_CNT = CLK_FREQ / BAUD;

    reg [11:0] bit_cnt;    // 波特率分频计数
    reg [3:0]  bit_idx;    // 当前位周期：0=起始位 1~8=数据位 9=停止位
    reg [7:0]  data_reg;   // 发送数据锁存

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_busy  <= 1'b0;
            uart_tx  <= 1'b1;      // 空闲高电平
            bit_cnt  <= 12'd0;
            bit_idx  <= 4'd0;
            data_reg <= 8'd0;
        end else begin
            if (tx_start && !tx_busy) begin
                // 触发：进入起始位周期（bit_idx=0 本身就是起始位，持续 1 个位周期）
                tx_busy  <= 1'b1;
                data_reg <= tx_data;
                bit_cnt  <= 12'd0;
                bit_idx  <= 4'd0;
                uart_tx  <= 1'b0;
            end else if (tx_busy) begin
                if (bit_cnt == BAUD_CNT - 1) begin
                    // 一个位周期结束，进入下一位
                    bit_cnt <= 12'd0;
                    if (bit_idx == 4'd9) begin
                        // 停止位周期结束，释放总线（uart_tx 保持 1）
                        tx_busy <= 1'b0;
                    end else begin
                        bit_idx <= bit_idx + 1'b1;
                        case (bit_idx)
                            4'd0: uart_tx <= data_reg[0];     // 起始位结束 → 数据位0
                            4'd1: uart_tx <= data_reg[1];
                            4'd2: uart_tx <= data_reg[2];
                            4'd3: uart_tx <= data_reg[3];
                            4'd4: uart_tx <= data_reg[4];
                            4'd5: uart_tx <= data_reg[5];
                            4'd6: uart_tx <= data_reg[6];
                            4'd7: uart_tx <= data_reg[7];
                            4'd8: uart_tx <= 1'b1;            // 停止位
                            default: uart_tx <= 1'b1;
                        endcase
                    end
                end else begin
                    bit_cnt <= bit_cnt + 1'b1;
                end
            end
        end
    end

endmodule
