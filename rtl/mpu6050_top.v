//------------------------------------------------------------------------------
// 文件名 : mpu6050_top.v
// 功能   : 顶层模块：例化驱动/解算/串口，并把结果格式化成 ASCII 文本帧打印
// 日期   : 2026-09-24（2026-09-26 精简端口适配 PGL22G 开发板）
//------------------------------------------------------------------------------
/* @brief  MPU6050 姿态解算顶层
 *         模块层次：
 *           mpu6050_top
 *             ├── mpu6050_driver   （I2C 初始化 + 500Hz 采集，内含 i2c_master）
 *             ├── attitude_calc    （CORDIC + 互补滤波，内含 2 个 cordic）
 *             └── uart_tx          （115200 串口打印）
 *         串口输出：每 100ms 一帧 90 字节 ASCII 文本，格式示例：
 *           R: -12.3 P: -45.6 Y: 180.0 | AX: -0.01 AY:  0.02 AZ:  1.00 | GX:   0.1 GY:  -0.2 GZ:   0.3
 * @param  CLK_FREQ : 主时钟频率(Hz)，默认 50_000_000（PGL22G 板载晶振）
 * @param  BAUD     : 串口波特率，默认 115200
 * @param  clk/rst_n : 时钟与异步复位（低有效）
 * @return i2c_scl  : I2C 时钟线（接 MPU6050 SCL，需 4.7kΩ 上拉，GY-521 模块自带）
 * @return i2c_sda  : I2C 数据线（接 MPU6050 SDA）
 * @return uart_tx  : 串口发送（板载 CP2102 USB 转串口）
 * @return led[3:0] : 状态指示灯：
 *                    [0]=错误指示（error≠0 点亮）
 *                    [1]=数据心跳（约2Hz，数据正常流动时闪烁）
 *                    [2]=初始化完成（进入采样状态后常亮）
 *                    [3]=首帧数据成功（sticky，常亮表示系统正常）
 * @note   1) 默认端口仅 9 个（占用 9 个 IO），适配 PGL22G-6CMBG324（240 IO）
 *         2) 编译时定义宏 DEBUG_PORTS（PDS 工程设置里加宏，或 +define+DEBUG_PORTS）
 *            可额外引出 9 个宽调试端口（加速度/角速度/角度 Q16.16 各 32 位 +
 *            data_valid + error + state_dbg），共 304 个 IO——仅适合大封装器件，
 *            且必须自行在 .adc 中为这些端口添加引脚约束
 *         3) 板载 LED 为低电平点亮（led[3:0] 已按此极性适配，
 *            "点亮=错误/初始化完成/首帧成功"的语义见 @return 说明）
 *         4) 本模块无任何厂商原语，Vivado/Quartus/PDS 均可综合
 */
module mpu6050_top #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 115200
)(
    input  wire        clk,          // PGL22G: B5 板载50MHz晶振
    input  wire        rst_n,        // PGL22G: F10 复位按键（低有效）
    // ---- I2C（接 MPU6050，GY-521 模块自带4.7k上拉）----
    inout  wire        i2c_scl,      // PGL22G: N14（J8排针第36脚）
    inout  wire        i2c_sda,      // PGL22G: R18（J8排针第35脚）
    // ---- UART ----
    output wire        uart_tx,      // PGL22G: C10（板载CP2102）
    // ---- 状态指示 LED ----
    output wire [3:0]  led           // PGL22G: U10/V10/U11/V11（LED1~LED4）
`ifdef DEBUG_PORTS
    // ---- 调试端口（默认关闭，见模块头 @note 2）----
    ,output wire signed [31:0] accel_x_g
    ,output wire signed [31:0] accel_y_g
    ,output wire signed [31:0] accel_z_g
    ,output wire signed [31:0] gyro_x_dps
    ,output wire signed [31:0] gyro_y_dps
    ,output wire signed [31:0] gyro_z_dps
    ,output wire signed [31:0] roll_deg
    ,output wire signed [31:0] pitch_deg
    ,output wire signed [31:0] yaw_deg
    ,output wire        data_valid
    ,output wire [1:0]  error
    ,output wire [7:0]  state_dbg
`endif
);

    //====================== 内部连线 ======================
    wire [15:0] accel_x_raw, accel_y_raw, accel_z_raw;
    wire [15:0] gyro_x_raw,  gyro_y_raw,  gyro_z_raw;
    wire [15:0] temp_raw;
    wire        frame_valid;

`ifdef DEBUG_PORTS
    // 调试端口已声明为模块输出，直接由子模块驱动
`else
    // 未引出调试端口时，数据信号仍作为内部连线使用
    // （姿态结果供 ASCII 格式化使用，error/state_dbg 供 LED 指示使用）
    wire signed [31:0] accel_x_g,  accel_y_g,  accel_z_g;
    wire signed [31:0] gyro_x_dps, gyro_y_dps, gyro_z_dps;
    wire signed [31:0] roll_deg,   pitch_deg,  yaw_deg;
    wire        data_valid;
    wire [1:0]  error;
    wire [7:0]  state_dbg;
`endif

    //====================== 例化 MPU6050 驱动 ======================
    mpu6050_driver u_mpu6050_driver (
        .clk        (clk),
        .rst_n      (rst_n),
        .i2c_scl    (i2c_scl),
        .i2c_sda    (i2c_sda),
        .accel_x    (accel_x_raw),
        .accel_y    (accel_y_raw),
        .accel_z    (accel_z_raw),
        .gyro_x     (gyro_x_raw),
        .gyro_y     (gyro_y_raw),
        .gyro_z     (gyro_z_raw),
        .temp_raw   (temp_raw),       // 温度未显示，端口预留
        .data_valid (frame_valid),
        .error      (error),
        .state_dbg  (state_dbg)
    );

    //====================== 例化姿态解算 ======================
    attitude_calc u_attitude_calc (
        .clk            (clk),
        .rst_n          (rst_n),
        .accel_x_raw    (accel_x_raw),
        .accel_y_raw    (accel_y_raw),
        .accel_z_raw    (accel_z_raw),
        .gyro_x_raw     (gyro_x_raw),
        .gyro_y_raw     (gyro_y_raw),
        .gyro_z_raw     (gyro_z_raw),
        .data_valid_in  (frame_valid),
        .accel_x_g      (accel_x_g),
        .accel_y_g      (accel_y_g),
        .accel_z_g      (accel_z_g),
        .gyro_x_dps     (gyro_x_dps),
        .gyro_y_dps     (gyro_y_dps),
        .gyro_z_dps     (gyro_z_dps),
        .roll           (roll_deg),
        .pitch          (pitch_deg),
        .yaw            (yaw_deg),
        .data_valid_out (data_valid)
    );

    //====================== 状态指示 LED ======================
    // 注意：板载 LED 为低电平点亮（阳极接 3.3V、阴极接 FPGA 引脚），
    // 故除心跳（闪烁与极性无关）外均反相输出，"点亮"对应信号为 0
    // led[0]：错误指示（WHO_AM_I 校验失败或 I2C NACK 时点亮）
    assign led[0] = (error == 2'b00);

    // led[1]：数据心跳，约2Hz（500Hz 帧同步 256 分频），数据流动时闪烁
    reg [7:0] hb_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hb_cnt <= 8'd0;
        end else if (data_valid) begin
            hb_cnt <= hb_cnt + 1'b1;
        end
    end
    assign led[1] = hb_cnt[7];

    // led[2]：初始化完成（驱动状态机进入采样状态 S_RD_IDLE=14 之后点亮）
    assign led[2] = ~((state_dbg[4:0] >= 5'd14) && (state_dbg[4:0] != 5'd17));

    // led[3]：首帧数据成功（sticky，点亮表示系统正常）
    reg first_frame;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            first_frame <= 1'b0;
        end else if (data_valid) begin
            first_frame <= 1'b1;
        end
    end
    assign led[3] = ~first_frame;

    //====================== ASCII 格式化 + 串口发送 ======================
    // 帧长度 90 字节，每 100ms 发送一帧
    localparam F_MSG_LEN  = 90;
    localparam T100MS     = 25'd5_000_000;

    reg  [24:0] tick_cnt;     // 100ms 定时
    reg  [6:0]  char_idx;     // 当前字符位置 0~89
    reg  [7:0]  char_byte;    // 当前字符 ASCII 码（组合逻辑生成）
    reg         sending;      // 一帧发送中
    reg         char_sent;    // 当前字符已交给 uart 标志
    reg         uart_start;   // uart 触发脉冲
    wire        tx_busy;
    // 帧开始时锁存的数值（保证一帧内数据一致）
    reg signed [31:0] roll_lat,  pitch_lat, yaw_lat;
    reg signed [31:0] ax_lat, ay_lat, az_lat, gx_lat, gy_lat, gz_lat;

    //---------------------- Q16.16 → ASCII 字符 ----------------------
    // pos: 0=符号位 1=百位 2=十位 3=个位 4=小数点 5=小数第1位 6=小数第2位
    // 高位整数为0时输出空格对齐；除以常数100/10由综合器优化为乘加
    function [7:0] q16_char;
        input [31:0] val;
        input [2:0]  pos;
        reg [31:0] abs_v;
        reg [15:0] int_v;
        reg [15:0] hun, ten, one;
        reg [15:0] f2;
        reg [15:0] f1, f0;
        begin
            abs_v = val[31] ? (~val + 1'b1) : val;
            int_v = abs_v[31:16];
            hun = int_v / 16'd100;
            ten = (int_v % 16'd100) / 16'd10;
            one = int_v % 16'd10;
            f2  = (abs_v[15:0] * 16'd100) >> 16;      // 两位小数组合值 0~99
            f1  = f2 / 16'd10;                        // 小数第1位
            f0  = f2 % 16'd10;                        // 小数第2位
            case (pos)
                3'd0: q16_char = val[31] ? "-" : " ";
                3'd1: q16_char = (hun == 16'd0) ? " " : (8'd48 + hun[7:0]);
                3'd2: q16_char = ((hun == 16'd0) && (ten == 16'd0)) ? " " : (8'd48 + ten[7:0]);
                3'd3: q16_char = 8'd48 + one[7:0];
                3'd4: q16_char = ".";
                3'd5: q16_char = 8'd48 + f1[3:0];
                3'd6: q16_char = 8'd48 + f0[3:0];
                default: q16_char = " ";
            endcase
        end
    endfunction

    //---------------------- 帧内容（按字符位置查表） ----------------------
    // 角度 6列：符号+3位整数(空格对齐)+小数点+1位小数
    // 加速度 5列：符号+1位整数+小数点+2位小数
    // 角速度 6列：同角度
    always @(*) begin
        case (char_idx)
            // ---- Roll ----
            7'd0:  char_byte = "R";
            7'd1:  char_byte = ":";
            7'd2:  char_byte = q16_char(roll_lat, 3'd0);
            7'd3:  char_byte = q16_char(roll_lat, 3'd1);
            7'd4:  char_byte = q16_char(roll_lat, 3'd2);
            7'd5:  char_byte = q16_char(roll_lat, 3'd3);
            7'd6:  char_byte = q16_char(roll_lat, 3'd4);
            7'd7:  char_byte = q16_char(roll_lat, 3'd5);
            7'd8:  char_byte = " ";
            // ---- Pitch ----
            7'd9:  char_byte = "P";
            7'd10: char_byte = ":";
            7'd11: char_byte = q16_char(pitch_lat, 3'd0);
            7'd12: char_byte = q16_char(pitch_lat, 3'd1);
            7'd13: char_byte = q16_char(pitch_lat, 3'd2);
            7'd14: char_byte = q16_char(pitch_lat, 3'd3);
            7'd15: char_byte = q16_char(pitch_lat, 3'd4);
            7'd16: char_byte = q16_char(pitch_lat, 3'd5);
            7'd17: char_byte = " ";
            // ---- Yaw ----
            7'd18: char_byte = "Y";
            7'd19: char_byte = ":";
            7'd20: char_byte = q16_char(yaw_lat, 3'd0);
            7'd21: char_byte = q16_char(yaw_lat, 3'd1);
            7'd22: char_byte = q16_char(yaw_lat, 3'd2);
            7'd23: char_byte = q16_char(yaw_lat, 3'd3);
            7'd24: char_byte = q16_char(yaw_lat, 3'd4);
            7'd25: char_byte = q16_char(yaw_lat, 3'd5);
            7'd26: char_byte = " ";
            7'd27: char_byte = "|";
            7'd28: char_byte = " ";
            // ---- 加速度 X ----
            7'd29: char_byte = "A";
            7'd30: char_byte = "X";
            7'd31: char_byte = ":";
            7'd32: char_byte = q16_char(ax_lat, 3'd0);
            7'd33: char_byte = q16_char(ax_lat, 3'd3);
            7'd34: char_byte = q16_char(ax_lat, 3'd4);
            7'd35: char_byte = q16_char(ax_lat, 3'd5);
            7'd36: char_byte = q16_char(ax_lat, 3'd6);
            7'd37: char_byte = " ";
            // ---- 加速度 Y ----
            7'd38: char_byte = "A";
            7'd39: char_byte = "Y";
            7'd40: char_byte = ":";
            7'd41: char_byte = q16_char(ay_lat, 3'd0);
            7'd42: char_byte = q16_char(ay_lat, 3'd3);
            7'd43: char_byte = q16_char(ay_lat, 3'd4);
            7'd44: char_byte = q16_char(ay_lat, 3'd5);
            7'd45: char_byte = q16_char(ay_lat, 3'd6);
            7'd46: char_byte = " ";
            // ---- 加速度 Z ----
            7'd47: char_byte = "A";
            7'd48: char_byte = "Z";
            7'd49: char_byte = ":";
            7'd50: char_byte = q16_char(az_lat, 3'd0);
            7'd51: char_byte = q16_char(az_lat, 3'd3);
            7'd52: char_byte = q16_char(az_lat, 3'd4);
            7'd53: char_byte = q16_char(az_lat, 3'd5);
            7'd54: char_byte = q16_char(az_lat, 3'd6);
            7'd55: char_byte = " ";
            7'd56: char_byte = "|";
            7'd57: char_byte = " ";
            // ---- 角速度 X ----
            7'd58: char_byte = "G";
            7'd59: char_byte = "X";
            7'd60: char_byte = ":";
            7'd61: char_byte = q16_char(gx_lat, 3'd0);
            7'd62: char_byte = q16_char(gx_lat, 3'd1);
            7'd63: char_byte = q16_char(gx_lat, 3'd2);
            7'd64: char_byte = q16_char(gx_lat, 3'd3);
            7'd65: char_byte = q16_char(gx_lat, 3'd4);
            7'd66: char_byte = q16_char(gx_lat, 3'd5);
            7'd67: char_byte = " ";
            // ---- 角速度 Y ----
            7'd68: char_byte = "G";
            7'd69: char_byte = "Y";
            7'd70: char_byte = ":";
            7'd71: char_byte = q16_char(gy_lat, 3'd0);
            7'd72: char_byte = q16_char(gy_lat, 3'd1);
            7'd73: char_byte = q16_char(gy_lat, 3'd2);
            7'd74: char_byte = q16_char(gy_lat, 3'd3);
            7'd75: char_byte = q16_char(gy_lat, 3'd4);
            7'd76: char_byte = q16_char(gy_lat, 3'd5);
            7'd77: char_byte = " ";
            // ---- 角速度 Z ----
            7'd78: char_byte = "G";
            7'd79: char_byte = "Z";
            7'd80: char_byte = ":";
            7'd81: char_byte = q16_char(gz_lat, 3'd0);
            7'd82: char_byte = q16_char(gz_lat, 3'd1);
            7'd83: char_byte = q16_char(gz_lat, 3'd2);
            7'd84: char_byte = q16_char(gz_lat, 3'd3);
            7'd85: char_byte = q16_char(gz_lat, 3'd4);
            7'd86: char_byte = q16_char(gz_lat, 3'd5);
            7'd87: char_byte = " ";
            // ---- 换行 ----
            7'd88: char_byte = "\r";     // 8'h0D
            7'd89: char_byte = "\n";     // 8'h0A
            default: char_byte = " ";
        endcase
    end

    //---------------------- 发送控制状态机 ----------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cnt  <= 25'd0;
            char_idx  <= 7'd0;
            sending   <= 1'b0;
            char_sent <= 1'b0;
            uart_start<= 1'b0;
            roll_lat  <= 32'sd0;
            pitch_lat <= 32'sd0;
            yaw_lat   <= 32'sd0;
            ax_lat    <= 32'sd0;
            ay_lat    <= 32'sd0;
            az_lat    <= 32'sd0;
            gx_lat    <= 32'sd0;
            gy_lat    <= 32'sd0;
            gz_lat    <= 32'sd0;
        end else begin
            uart_start <= 1'b0;    // 脉冲信号默认清零

            if (!sending) begin
                // 每 100ms 锁存最新数据并开始发送一帧
                if (tick_cnt == T100MS) begin
                    tick_cnt  <= 25'd0;
                    roll_lat  <= roll_deg;
                    pitch_lat <= pitch_deg;
                    yaw_lat   <= yaw_deg;
                    ax_lat    <= accel_x_g;
                    ay_lat    <= accel_y_g;
                    az_lat    <= accel_z_g;
                    gx_lat    <= gyro_x_dps;
                    gy_lat    <= gyro_y_dps;
                    gz_lat    <= gyro_z_dps;
                    char_idx  <= 7'd0;
                    char_sent <= 1'b0;
                    sending   <= 1'b1;
                end else begin
                    tick_cnt <= tick_cnt + 1'b1;
                end
            end else if (!tx_busy && !char_sent) begin
                // 发送器空闲：把当前字符交给 uart
                uart_start <= 1'b1;
                char_sent  <= 1'b1;
            end else if (tx_busy && char_sent) begin
                // 当前字符已被 uart 锁存：指向下一个字符
                char_sent <= 1'b0;
                if (char_idx == F_MSG_LEN - 1) begin
                    char_idx <= 7'd0;
                    sending  <= 1'b0;      // 一帧发送完毕
                end else begin
                    char_idx <= char_idx + 1'b1;
                end
            end
        end
    end

    //---------------------- 例化 UART 发送器 ----------------------
    // tx_data 直接取组合逻辑生成的 char_byte，
    // uart 在 tx_start 同一时钟沿采样，此时 char_idx 尚未更新，数据稳定
    uart_tx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD     (BAUD)
    ) u_uart_tx (
        .clk      (clk),
        .rst_n    (rst_n),
        .tx_data  (char_byte),
        .tx_start (uart_start),
        .tx_busy  (tx_busy),
        .uart_tx  (uart_tx)
    );

endmodule
