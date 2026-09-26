//------------------------------------------------------------------------------
// 文件名 : mpu6050_driver.v
// 功能   : MPU6050 初始化配置 + 周期数据采集（通过通用 i2c_master 实现）
// 日期   : 2026-09-24
//------------------------------------------------------------------------------
/* @brief  MPU6050 驱动
 *         上电流程：
 *         1) 延时 100ms（手册要求上电/复位后等待，官方复位序列为 100ms）
 *         2) 读 WHO_AM_I(0x75) 校验是否为 0x68（失败重试3次，仍失败报错停机）
 *         3) 写 PWR_MGMT_1(0x6B) = 0x00 唤醒（复位值 0x40，SLEEP=1 默认睡眠）
 *         4) 延时 50ms（陀螺仪上电稳定时间约 30ms，留余量）
 *         5) 配置寄存器：
 *            SMPLRT_DIV (0x19) = 0x01 → 采样率 = 1kHz/(1+1) = 500Hz
 *            CONFIG      (0x1A) = 0x01 → DLPF_CFG=1（加速度带宽184Hz/陀螺188Hz）
 *            GYRO_CONFIG (0x1B) = 0x00 → FS_SEL=0，±250°/s，131 LSB/(°/s)
 *            ACCEL_CONFIG(0x1C) = 0x00 → AFS_SEL=0，±2g，16384 LSB/g
 *         6) 每 2ms 从 0x3B 突发读 14 字节（加速度6+温度2+陀螺仪6），
 *            大端拼接后输出，data_valid 帧同步
 * @param  CLK_FREQ : 主时钟频率(Hz)，默认 50_000_000
 * @param  I2C_FREQ : I2C 总线频率(Hz)，默认 400_000
 * @param  DEV_ADDR : MPU6050 从机地址，默认 7'h68（AD0=0）
 * @param  clk/rst_n : 时钟与异步复位（低有效）
 * @return i2c_scl  : I2C 时钟线（接芯片 SCL，需 4.7kΩ 上拉）
 * @return i2c_sda  : I2C 数据线（接芯片 SDA，需 4.7kΩ 上拉）
 * @return accel_x/y/z[15:0] : 三轴加速度原始值（16位有符号补码）
 * @return gyro_x/y/z[15:0]  : 三轴陀螺仪原始值（16位有符号补码）
 * @return temp_raw[15:0]    : 温度原始值（换算：°C = raw/340 + 36.53）
 * @return data_valid : 完整帧数据有效脉冲（约 500Hz）
 * @return error[1:0] : 00=正常 01=WHO_AM_I校验失败 10=事务NACK错误
 * @return state_dbg[7:0] : 当前状态机状态（调试用）
 * @note   1) 寄存器地址依据官方 RM-MPU-6000A-00 Rev4.2 寄存器映射
 *         2) 采样节拍由独立自由运行定时器产生（与状态机无关），
 *            帧间隔严格等于 2ms（突发读约 0.4ms，远小于 2ms 不会漏帧），
 *            该精确帧间隔是姿态解算积分 dt=2ms 的前提，请勿随意改动
 *         3) WHO_AM_I 校验失败后停在错误状态，需复位重启
 */
module mpu6050_driver #(
    parameter CLK_FREQ = 50_000_000,
    parameter I2C_FREQ = 400_000,
    parameter DEV_ADDR = 7'h68
)(
    input  wire        clk,
    input  wire        rst_n,
    // ---- I2C 总线 ----
    inout  wire        i2c_scl,
    inout  wire        i2c_sda,
    // ---- 传感器原始数据输出 ----
    output reg  [15:0] accel_x,
    output reg  [15:0] accel_y,
    output reg  [15:0] accel_z,
    output reg  [15:0] gyro_x,
    output reg  [15:0] gyro_y,
    output reg  [15:0] gyro_z,
    output reg  [15:0] temp_raw,
    output reg         data_valid,
    // ---- 状态输出 ----
    output reg  [1:0]  error,
    output reg  [7:0]  state_dbg
);

    //====================== MPU6050 寄存器地址 ======================
    // 来源：官方寄存器映射文档 RM-MPU-6000A-00 Rev4.2
    localparam REG_WHO_AM_I    = 8'h75;   // 芯片ID，复位值 0x68
    localparam REG_PWR_MGMT_1  = 8'h6B;   // 电源管理1，复位值 0x40（SLEEP=1）
    localparam REG_SMPLRT_DIV  = 8'h19;   // 采样率分频：F = 1kHz/(1+div)
    localparam REG_CONFIG       = 8'h1A;   // 数字低通滤波 DLPF_CFG
    localparam REG_GYRO_CONFIG  = 8'h1B;   // 陀螺仪量程 FS_SEL
    localparam REG_ACCEL_CONFIG = 8'h1C;   // 加速度计量程 AFS_SEL
    localparam REG_ACCEL_XOUT_H = 8'h3B;   // 数据首地址（连读14字节自动递增）

    //====================== 计数参数（50MHz下） ======================
    localparam T100MS = 25'd5_000_000;     // 上电等待 100ms
    localparam T50MS  = 25'd2_500_000;     // 唤醒后稳定 50ms
    localparam T2MS   = 17'd100_000;       // 采样帧周期 2ms（500Hz）

    //====================== 状态机 ======================
    localparam S_PWR_DELAY = 5'd0;    // 上电延时100ms
    localparam S_WHO_TRIG  = 5'd1;    // 触发读 WHO_AM_I
    localparam S_WHO_WAIT  = 5'd2;    // 等待完成并校验
    localparam S_WAKE_TRIG = 5'd3;    // 触发唤醒写
    localparam S_WAKE_WAIT = 5'd4;    // 等待唤醒写完成
    localparam S_STB_DELAY = 5'd5;    // 稳定延时50ms
    localparam S_CFG1_TRIG = 5'd6;    // 触发写 SMPLRT_DIV
    localparam S_CFG1_WAIT = 5'd7;
    localparam S_CFG2_TRIG = 5'd8;    // 触发写 CONFIG
    localparam S_CFG2_WAIT = 5'd9;
    localparam S_CFG3_TRIG = 5'd10;   // 触发写 GYRO_CONFIG
    localparam S_CFG3_WAIT = 5'd11;
    localparam S_CFG4_TRIG = 5'd12;   // 触发写 ACCEL_CONFIG
    localparam S_CFG4_WAIT = 5'd13;
    localparam S_RD_IDLE   = 5'd14;   // 采样等待（2ms 定帧节拍）
    localparam S_RD_BURST  = 5'd15;   // 突发读14字节
    localparam S_RD_ASSEMB = 5'd16;   // 组装输出
    localparam S_ERR       = 5'd17;   // 错误停机

    //====================== 内部信号 ======================
    reg  [4:0]   state;
    reg  [24:0]  delay_cnt;       // 上电/稳定延时计数
    reg  [16:0]  tick_cnt;        // 自由运行 2ms 采样节拍计数
    wire         tick_2ms;        // 采样节拍脉冲（每2ms一个时钟）
    reg  [2:0]   who_retry;       // WHO_AM_I 重试计数
    reg  [3:0]   rd_cnt;          // 已收字节计数
    reg  [111:0] rdata_buf;       // 14字节接收缓冲（先收的字节在高位）
    reg          i2c_start;       // I2C 触发脉冲
    reg          i2c_rw;          // 0=写 1=读
    reg  [7:0]   i2c_reg_addr;
    reg  [7:0]   i2c_wr_data;
    reg  [7:0]   i2c_rd_len;
    wire         i2c_busy, i2c_done, i2c_ack_err, i2c_rd_valid;
    wire [7:0]   i2c_rd_data;

    //====================== 例化通用 I2C 主机 ======================
    i2c_master #(
        .CLK_FREQ (CLK_FREQ),
        .I2C_FREQ (I2C_FREQ)
    ) u_i2c_master (
        .clk      (clk),
        .rst_n    (rst_n),
        .dev_addr (DEV_ADDR),
        .start    (i2c_start),
        .rw       (i2c_rw),
        .reg_addr (i2c_reg_addr),
        .wr_data  (i2c_wr_data),
        .rd_len   (i2c_rd_len),
        .busy     (i2c_busy),
        .done     (i2c_done),
        .ack_err  (i2c_ack_err),
        .rd_data  (i2c_rd_data),
        .rd_valid (i2c_rd_valid),
        .scl      (i2c_scl),
        .sda      (i2c_sda)
    );

    //====================== 自由运行 2ms 采样节拍 ======================
    // 与状态机完全独立，保证相邻两次采样触发的间隔严格等于 2ms
    // （姿态解算的积分 dt=2ms 依赖该精度，见 @note 2）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cnt <= 17'd0;
        end else if (tick_cnt == T2MS) begin
            tick_cnt <= 17'd0;
        end else begin
            tick_cnt <= tick_cnt + 1'b1;
        end
    end
    assign tick_2ms = (tick_cnt == T2MS);

    //====================== 主状态机 ======================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_PWR_DELAY;
            delay_cnt    <= 25'd0;
            who_retry    <= 3'd0;
            rd_cnt       <= 4'd0;
            rdata_buf    <= 112'd0;
            i2c_start    <= 1'b0;
            i2c_rw       <= 1'b0;
            i2c_reg_addr <= 8'd0;
            i2c_wr_data  <= 8'd0;
            i2c_rd_len   <= 8'd1;
            accel_x      <= 16'd0;
            accel_y      <= 16'd0;
            accel_z      <= 16'd0;
            gyro_x       <= 16'd0;
            gyro_y       <= 16'd0;
            gyro_z       <= 16'd0;
            temp_raw     <= 16'd0;
            data_valid   <= 1'b0;
            error        <= 2'd0;
            state_dbg    <= 8'd0;
        end else begin
            i2c_start  <= 1'b0;    // 脉冲信号默认清零
            data_valid <= 1'b0;
            state_dbg  <= {3'd0, state};

            case (state)
                //----------------------------------------------------------
                // 上电延时 100ms（手册上电/复位要求）
                //----------------------------------------------------------
                S_PWR_DELAY: begin
                    if (delay_cnt == T100MS) begin
                        state     <= S_WHO_TRIG;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                //----------------------------------------------------------
                // 读 WHO_AM_I（1字节），校验芯片ID
                //----------------------------------------------------------
                S_WHO_TRIG: begin
                    i2c_rw       <= 1'b1;
                    i2c_reg_addr <= REG_WHO_AM_I;
                    i2c_rd_len   <= 8'd1;
                    if (!i2c_busy) begin
                        i2c_start <= 1'b1;
                        state     <= S_WHO_WAIT;
                    end
                end
                S_WHO_WAIT: begin
                    if (i2c_done) begin
                        if (!i2c_ack_err && i2c_rd_data == 8'h68) begin
                            state <= S_WAKE_TRIG;      // ID正确，继续
                        end else if (who_retry < 3'd3) begin
                            who_retry <= who_retry + 1'b1;
                            state     <= S_WHO_TRIG;   // 重试
                        end else begin
                            error <= 2'b01;            // 校验失败
                            state <= S_ERR;
                        end
                    end
                end

                //----------------------------------------------------------
                // 唤醒：PWR_MGMT_1 = 0x00（复位值0x40，SLEEP=1 默认睡眠）
                //----------------------------------------------------------
                S_WAKE_TRIG: begin
                    i2c_rw       <= 1'b0;
                    i2c_reg_addr <= REG_PWR_MGMT_1;
                    i2c_wr_data  <= 8'h00;
                    if (!i2c_busy) begin
                        i2c_start <= 1'b1;
                        state     <= S_WAKE_WAIT;
                    end
                end
                S_WAKE_WAIT: begin
                    if (i2c_done) begin
                        if (i2c_ack_err) begin
                            error <= 2'b10;
                            state <= S_ERR;
                        end else begin
                            delay_cnt <= 25'd0;
                            state     <= S_STB_DELAY;
                        end
                    end
                end

                //----------------------------------------------------------
                // 唤醒后稳定延时 50ms（陀螺仪稳定约30ms，留余量）
                //----------------------------------------------------------
                S_STB_DELAY: begin
                    if (delay_cnt == T50MS) begin
                        state <= S_CFG1_TRIG;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                //----------------------------------------------------------
                // 配置1：SMPLRT_DIV = 1 → 采样率 1kHz/(1+1) = 500Hz
                //----------------------------------------------------------
                S_CFG1_TRIG: begin
                    i2c_rw       <= 1'b0;
                    i2c_reg_addr <= REG_SMPLRT_DIV;
                    i2c_wr_data  <= 8'h01;
                    if (!i2c_busy) begin
                        i2c_start <= 1'b1;
                        state     <= S_CFG1_WAIT;
                    end
                end
                S_CFG1_WAIT: begin
                    if (i2c_done) begin
                        if (i2c_ack_err) begin error <= 2'b10; state <= S_ERR; end
                        else             state <= S_CFG2_TRIG;
                    end
                end

                //----------------------------------------------------------
                // 配置2：CONFIG = 1 → DLPF_CFG=1（加速度带宽184Hz/陀螺188Hz）
                //----------------------------------------------------------
                S_CFG2_TRIG: begin
                    i2c_rw       <= 1'b0;
                    i2c_reg_addr <= REG_CONFIG;
                    i2c_wr_data  <= 8'h01;
                    if (!i2c_busy) begin
                        i2c_start <= 1'b1;
                        state     <= S_CFG2_WAIT;
                    end
                end
                S_CFG2_WAIT: begin
                    if (i2c_done) begin
                        if (i2c_ack_err) begin error <= 2'b10; state <= S_ERR; end
                        else             state <= S_CFG3_TRIG;
                    end
                end

                //----------------------------------------------------------
                // 配置3：GYRO_CONFIG = 0 → ±250°/s，131 LSB/(°/s)
                //----------------------------------------------------------
                S_CFG3_TRIG: begin
                    i2c_rw       <= 1'b0;
                    i2c_reg_addr <= REG_GYRO_CONFIG;
                    i2c_wr_data  <= 8'h00;
                    if (!i2c_busy) begin
                        i2c_start <= 1'b1;
                        state     <= S_CFG3_WAIT;
                    end
                end
                S_CFG3_WAIT: begin
                    if (i2c_done) begin
                        if (i2c_ack_err) begin error <= 2'b10; state <= S_ERR; end
                        else             state <= S_CFG4_TRIG;
                    end
                end

                //----------------------------------------------------------
                // 配置4：ACCEL_CONFIG = 0 → ±2g，16384 LSB/g
                //----------------------------------------------------------
                S_CFG4_TRIG: begin
                    i2c_rw       <= 1'b0;
                    i2c_reg_addr <= REG_ACCEL_CONFIG;
                    i2c_wr_data  <= 8'h00;
                    if (!i2c_busy) begin
                        i2c_start <= 1'b1;
                        state     <= S_CFG4_WAIT;
                    end
                end
                S_CFG4_WAIT: begin
                    if (i2c_done) begin
                        if (i2c_ack_err) begin error <= 2'b10; state <= S_ERR; end
                        else             state <= S_RD_IDLE;
                    end
                end

                //----------------------------------------------------------
                // 采样等待：每个 2ms 节拍触发一次突发读（busy 时放弃本帧）
                //----------------------------------------------------------
                S_RD_IDLE: begin
                    i2c_rw       <= 1'b1;
                    i2c_reg_addr <= REG_ACCEL_XOUT_H;   // 0x3B 起连读
                    i2c_rd_len   <= 8'd14;              // 加速度6+温度2+陀螺仪6
                    rd_cnt       <= 4'd0;
                    if (tick_2ms && !i2c_busy) begin
                        i2c_start <= 1'b1;
                        state     <= S_RD_BURST;
                    end
                end

                //----------------------------------------------------------
                // 突发读：接收14字节移入缓冲（先收的字节在高位）
                //----------------------------------------------------------
                S_RD_BURST: begin
                    if (i2c_rd_valid) begin
                        rdata_buf <= {rdata_buf[103:0], i2c_rd_data};
                        rd_cnt    <= rd_cnt + 1'b1;
                    end
                    if (i2c_done) begin
                        if (i2c_ack_err) begin
                            error <= 2'b10;    // NACK错误：放弃本帧，下帧自愈
                            state <= S_RD_IDLE;
                        end else begin
                            state <= S_RD_ASSEMB;
                        end
                    end
                end

                //----------------------------------------------------------
                // 组装：16位大端拼接（高字节在前）+ 帧同步脉冲
                //----------------------------------------------------------
                S_RD_ASSEMB: begin
                    accel_x <= {rdata_buf[111:104], rdata_buf[103:96]};   // 0x3B,0x3C
                    accel_y <= {rdata_buf[ 95: 88], rdata_buf[ 87:80]};   // 0x3D,0x3E
                    accel_z <= {rdata_buf[ 79: 72], rdata_buf[ 71:64]};   // 0x3F,0x40
                    temp_raw<= {rdata_buf[ 63: 56], rdata_buf[ 55:48]};   // 0x41,0x42
                    gyro_x  <= {rdata_buf[ 47: 40], rdata_buf[ 39:32]};   // 0x43,0x44
                    gyro_y  <= {rdata_buf[ 31: 24], rdata_buf[ 23:16]};   // 0x45,0x46
                    gyro_z  <= {rdata_buf[ 15:  8], rdata_buf[  7: 0]};   // 0x47,0x48
                    data_valid <= 1'b1;
                    error   <= 2'b00;              // 正常收帧，清除瞬时NACK标志
                    state   <= S_RD_IDLE;
                end

                //----------------------------------------------------------
                // 错误停机（复位后重新初始化）
                //----------------------------------------------------------
                S_ERR: begin
                    state <= S_ERR;
                end

                default: state <= S_ERR;
            endcase
        end
    end

endmodule
