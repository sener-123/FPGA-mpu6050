//------------------------------------------------------------------------------
// 文件名 : i2c_master.v
// 功能   : 通用 I2C 主机控制器（可复用，与具体芯片解耦，不含任何 MPU6050 相关内容）
// 日期   : 2026-09-24
//------------------------------------------------------------------------------
/* @brief  I2C 主机控制器
 *         支持两种事务：
 *         1) 写事务：START → 设备地址+W → 寄存器地址 → 写数据 → STOP
 *         2) 读事务：START → 设备地址+W → 寄存器地址 → 重复START → 设备地址+R
 *                    → 连续读 rd_len 字节（最后一字节回NACK）→ STOP
 *         支持从机时钟拉伸（SCL 高相期间被从机拉低时自动挂起等待）
 * @param  CLK_FREQ : 主时钟频率(Hz)，默认 50_000_000
 * @param  I2C_FREQ : I2C 总线频率(Hz)，默认 400_000
 * @param  dev_addr[6:0] : 7位从机地址（MPU6050 为 0x68，AD0=0）
 * @param  start    : 事务触发（1个时钟高脉冲，busy=0 时有效）
 * @param  rw       : 0=写事务，1=读事务（事务期间必须保持稳定）
 * @param  reg_addr[7:0] : 寄存器首地址
 * @param  wr_data[7:0]  : 写事务数据字节
 * @param  rd_len[7:0]   : 读事务读取字节数
 * @return busy     : 事务进行中标志（含事务后总线空闲保护期）
 * @return done     : 事务完成脉冲（1个时钟）
 * @return ack_err  : 从机NACK标志（与 done 同时有效，1=发生NACK）
 * @return rd_data[7:0]  : 读回数据字节
 * @return rd_valid : 读数据有效脉冲（与 rd_data 对齐，每收到1字节1个脉冲）
 * @return scl/sda  : I2C 总线（开漏模拟，FPGA 引脚需外接 4.7kΩ 左右上拉电阻）
 * @note   1) 不支持10位地址与多主机仲裁
 *         2) 事务期间 dev_addr/rw/reg_addr/wr_data/rd_len 必须保持稳定
 *         3) SCL 实际频率 = CLK_FREQ/(4*SCL_QUARTER)，50MHz/400kHz 下约 379kHz，
 *            满足手册快模式时序（tLOW≈1.32µs ≥ 1.3µs，tHIGH≈1.32µs ≥ 0.6µs，
 *            tSU.STO/tHD.STA≈1.32µs ≥ 0.6µs，tBUF≈1.32µs ≥ 1.3µs）
 *         4) 写方向在 SCL 下降沿后 1 个时钟更新 SDA（20ns ≥ tHD.DAT=0），
 *            采样点在 SCL 高相中点（建立时间 ≈ 1.3µs，远大于 tSU.DAT=100ns）
 */
module i2c_master #(
    parameter CLK_FREQ = 50_000_000,          // 主时钟频率 Hz
    parameter I2C_FREQ = 400_000              // I2C 总线频率 Hz
)(
    input  wire        clk,
    input  wire        rst_n,
    // ---- 主机侧控制接口 ----
    input  wire [6:0]  dev_addr,
    input  wire        start,
    input  wire        rw,
    input  wire [7:0]  reg_addr,
    input  wire [7:0]  wr_data,
    input  wire [7:0]  rd_len,
    output reg         busy,
    output reg         done,
    output reg         ack_err,
    output reg  [7:0]  rd_data,
    output reg         rd_valid,
    // ---- I2C 总线（开漏，需外部上拉）----
    inout  wire        scl,
    inout  wire        sda
);

    //====================== 参数 ======================
    // SCL 四分之一周期计数：CLK_FREQ/I2C_FREQ = 125 个时钟/周期，
    // 四分频理论值 31.25，加 2 取 33 → SCL 周期 132 时钟 ≈ 379kHz，
    // 保证 tLOW = tHIGH = 66 时钟 = 1.32µs ≥ 1.3µs（快模式最小要求）
    localparam SCL_QUARTER = (CLK_FREQ / (4 * I2C_FREQ)) + 2;   // 33 @50MHz/400kHz
    localparam CNT_MAX     = 4 * SCL_QUARTER - 1;               // 131
    localparam T_HALF      = 2 * SCL_QUARTER;                   // 66 ≈ 1.32µs

    // 状态机
    localparam S_IDLE   = 3'd0;   // 空闲，等待事务触发
    localparam S_START  = 3'd1;   // 起始条件：SCL高期间SDA拉低
    localparam S_BYTE   = 3'd2;   // 发送/接收 8 位数据
    localparam S_ACK    = 3'd3;   // 第9位：写方向收ACK，读方向发ACK/NACK
    localparam S_RSTART = 3'd4;   // 重复起始（读事务）
    localparam S_STOP   = 3'd5;   // 停止条件：SCL高期间SDA释放上升
    localparam S_WAIT   = 3'd6;   // 事务后总线保护（tBUF ≥ 1.3µs）

    // 事务步骤
    localparam STEP_ADDR_W = 3'd0;   // 发送 设备地址+W
    localparam STEP_REG    = 3'd1;   // 发送 寄存器地址
    localparam STEP_DATA_W = 3'd2;   // 发送 写数据（仅写事务）
    localparam STEP_ADDR_R = 3'd3;   // 发送 设备地址+R（仅读事务，重复START后）
    localparam STEP_DATA_R = 3'd4;   // 接收 数据字节（仅读事务，共 rd_len 字节）

    //====================== 内部信号 ======================
    reg  [2:0] state;
    reg  [7:0] cnt;          // SCL 相位计数器
    reg  [3:0] bit_cnt;      // 当前字节位计数（0~7=数据位，8=ACK位）
    reg  [2:0] step;         // 事务步骤
    reg  [7:0] rd_len_reg;   // 读字节数锁存
    reg  [7:0] rd_cnt;       // 已读字节计数
    reg  [7:0] tx_buf;       // 发送移位寄存器（高位先出）
    reg  [7:0] rd_buf;       // 接收移位寄存器（高位先入）
    reg        scl_low;      // 1=拉低SCL（开漏：只拉低或释放）
    reg        sda_low;      // 1=拉低SDA

    // 开漏输出模拟：只输出低电平或高阻，高电平由外部上拉电阻提供
    assign scl = scl_low ? 1'b0 : 1'bz;
    assign sda = sda_low ? 1'b0 : 1'bz;
    wire scl_in = scl;                  // 回读SCL总线电平（时钟拉伸检测用）
    wire sda_in = sda;                  // 回读SDA总线电平（采样ACK/数据用）

    // 当前字节是否为主机接收（只有读事务的数据字节阶段为真）
    wire reading = (step == STEP_DATA_R);

    //====================== 主状态机 ======================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            busy       <= 1'b0;
            done       <= 1'b0;
            ack_err    <= 1'b0;
            rd_valid   <= 1'b0;
            rd_data    <= 8'd0;
            scl_low    <= 1'b0;
            sda_low    <= 1'b0;
            cnt        <= 8'd0;
            bit_cnt    <= 4'd0;
            step       <= STEP_ADDR_W;
            rd_cnt     <= 8'd0;
            rd_buf     <= 8'd0;
            tx_buf     <= 8'd0;
            rd_len_reg <= 8'd0;
        end else begin
            done     <= 1'b0;      // 脉冲信号默认清零
            rd_valid <= 1'b0;

            case (state)
                //----------------------------------------------------------
                // 空闲：释放总线，等待事务触发
                //----------------------------------------------------------
                S_IDLE: begin
                    busy    <= 1'b0;
                    scl_low <= 1'b0;
                    sda_low <= 1'b0;
                    if (start && !busy) begin
                        busy       <= 1'b1;
                        ack_err    <= 1'b0;
                        rd_cnt     <= 8'd0;
                        rd_buf     <= 8'd0;
                        rd_len_reg <= rd_len;
                        step       <= STEP_ADDR_W;
                        cnt        <= 8'd0;
                        state      <= S_START;
                    end
                end

                //----------------------------------------------------------
                // 起始条件：SCL 保持高，SDA 拉低
                //----------------------------------------------------------
                S_START: begin
                    scl_low <= 1'b0;   // 保持 SCL 高（释放）
                    sda_low <= 1'b1;   // SDA 拉低 → START
                    if (!scl_in) begin
                        cnt <= 8'd0;   // 总线被占用（SCL为低），等待
                    end else if (cnt == T_HALF) begin
                        // 满足 tHD.STA ≥ 0.6µs 后拉低 SCL，进入第1个数据位
                        scl_low <= 1'b1;
                        cnt      <= 8'd0;
                        bit_cnt  <= 4'd0;
                        tx_buf   <= {dev_addr, 1'b0};   // 设备地址 + 写位
                        state    <= S_BYTE;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                //----------------------------------------------------------
                // 8 位数据：SCL 低相 cnt=0~T_HALF-1，高相 cnt=T_HALF~CNT_MAX
                //----------------------------------------------------------
                S_BYTE: begin
                    scl_low <= (cnt < T_HALF) ? 1'b1 : 1'b0;

                    if (reading) begin
                        // 接收：释放 SDA，在 SCL 高相中点采样
                        sda_low <= 1'b0;
                        if (cnt == 3 * SCL_QUARTER) begin
                            rd_buf <= {rd_buf[6:0], sda_in};
                            if (bit_cnt == 4'd7) begin
                                rd_data  <= {rd_buf[6:0], sda_in};
                                rd_valid <= 1'b1;
                                rd_cnt   <= rd_cnt + 1'b1;
                            end
                        end
                    end else if (cnt == 8'd1) begin
                        // 发送：SCL 下降沿后 1 个时钟更新 SDA（满足 tHD.DAT）
                        sda_low <= ~tx_buf[7];
                        tx_buf  <= {tx_buf[6:0], 1'b0};
                    end

                    // 时钟拉伸检测 + SCL 相位计数
                    if (cnt >= T_HALF && !scl_in) begin
                        cnt <= cnt;       // 从机拉低 SCL，挂起等待
                    end else if (cnt == CNT_MAX) begin
                        cnt <= 8'd0;
                        if (bit_cnt == 4'd7) begin
                            bit_cnt <= 4'd8;   // 8位完毕，进入ACK位
                            state   <= S_ACK;
                        end else begin
                            bit_cnt <= bit_cnt + 1'b1;
                        end
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                //----------------------------------------------------------
                // 第9位：写方向采样从机ACK，读方向发送ACK/NACK
                //----------------------------------------------------------
                S_ACK: begin
                    scl_low <= (cnt < T_HALF) ? 1'b1 : 1'b0;

                    if (cnt == 8'd0) begin
                        if (reading) begin
                            // 主机回ACK(拉低)/NACK(释放)：最后一字节回NACK
                            sda_low <= (rd_cnt < rd_len_reg - 1'b1) ? 1'b1 : 1'b0;
                        end else begin
                            sda_low <= 1'b0;   // 释放SDA，等待从机ACK
                        end
                    end

                    // 写方向：SCL 高相中点采样从机应答
                    if (!reading && cnt == 3 * SCL_QUARTER && sda_in == 1'b1) begin
                        ack_err <= 1'b1;       // 从机未应答（NACK）
                    end

                    if (cnt >= T_HALF && !scl_in) begin
                        cnt <= cnt;
                    end else if (cnt == CNT_MAX) begin
                        cnt <= 8'd0;
                        case (step)
                            STEP_ADDR_W: begin
                                if (ack_err) begin
                                    state <= S_STOP;   // 从机无应答，中止
                                end else begin
                                    step    <= STEP_REG;
                                    tx_buf  <= reg_addr;
                                    bit_cnt <= 4'd0;
                                    state   <= S_BYTE;
                                end
                            end
                            STEP_REG: begin
                                if (ack_err) begin
                                    state <= S_STOP;
                                end else if (rw) begin
                                    step  <= STEP_ADDR_R;   // 读事务 → 重复START
                                    state <= S_RSTART;
                                end else begin
                                    step    <= STEP_DATA_W;
                                    tx_buf  <= wr_data;
                                    bit_cnt <= 4'd0;
                                    state   <= S_BYTE;
                                end
                            end
                            STEP_DATA_W: begin
                                state <= S_STOP;           // 写事务完成
                            end
                            STEP_ADDR_R: begin
                                if (ack_err) begin
                                    state <= S_STOP;
                                end else begin
                                    step    <= STEP_DATA_R;
                                    bit_cnt <= 4'd0;
                                    state   <= S_BYTE;
                                end
                            end
                            STEP_DATA_R: begin
                                if (rd_cnt == rd_len_reg) begin
                                    state <= S_STOP;       // 全部字节读完
                                end else begin
                                    bit_cnt <= 4'd0;
                                    state   <= S_BYTE;
                                end
                            end
                            default: state <= S_STOP;
                        endcase
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                //----------------------------------------------------------
                // 重复起始：先结束ACK位（SCL拉低），SDA上升后按 START 时序执行
                // 顺序：SCL低期间释放SDA → 释放SCL(上升) → SDA拉低(START) → SCL拉低
                //----------------------------------------------------------
                S_RSTART: begin
                    if (cnt == 8'd0) begin
                        scl_low <= 1'b1;   // 拉低SCL，结束ACK位
                        sda_low <= 1'b0;   // 释放SDA（从机已释放ACK，SDA上升）
                    end
                    if (cnt == T_HALF) begin
                        scl_low <= 1'b0;   // 释放SCL → SCL上升
                    end
                    if (cnt == 2 * T_HALF) begin
                        sda_low <= 1'b1;   // SCL高期间SDA拉低 → START（tSU.STA≈1.32µs）
                    end
                    if (cnt == 3 * T_HALF) begin
                        scl_low <= 1'b1;   // 满足 tHD.STA 后拉低SCL
                        cnt      <= 8'd0;
                        bit_cnt  <= 4'd0;
                        tx_buf   <= {dev_addr, 1'b1};   // 设备地址 + 读位
                        state    <= S_BYTE;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                //----------------------------------------------------------
                // 停止条件：SCL高期间SDA释放上升
                // 顺序：SCL拉低(结束ACK位) → 释放SCL(上升) → 释放SDA(上升=STOP)
                //----------------------------------------------------------
                S_STOP: begin
                    if (cnt == 8'd0) begin
                        scl_low <= 1'b1;   // 拉低SCL，结束ACK位
                        sda_low <= 1'b1;   // 确保SDA为低
                    end
                    if (cnt == T_HALF) begin
                        scl_low <= 1'b0;   // 释放SCL → SCL上升
                    end
                    if (cnt == 2 * T_HALF) begin
                        sda_low <= 1'b0;   // SCL高期间释放SDA → SDA上升 = STOP
                    end
                    if (cnt == 3 * T_HALF) begin
                        done  <= 1'b1;     // 事务完成
                        state <= S_WAIT;
                        cnt   <= 8'd0;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                //----------------------------------------------------------
                // 事务后总线保护：tBUF ≥ 1.3µs（背靠背事务间隔）
                //----------------------------------------------------------
                S_WAIT: begin
                    if (cnt == T_HALF) begin
                        busy  <= 1'b0;
                        state <= S_IDLE;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
