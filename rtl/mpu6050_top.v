//------------------------------------------------------------------------------
// 文件名 : mpu6050_top.v
// 功能   : 顶层模块：例化驱动/Madgwick 姿态解算/位移积分/串口，VOFA+ Firewater 输出
// 日期   : 2026-09-24（2026-09-29 改为 Madgwick + 位移 + Firewater 协议）
//------------------------------------------------------------------------------
/* @brief  MPU6050 姿态 + 位移解算顶层
 *         模块层次：
 *           mpu6050_top
 *             ├── mpu6050_driver   （I2C 初始化 + 500Hz 采集，内含 i2c_master）
 *             ├── madgwick_ahrs    （Madgwick 四元数姿态解算 + 世界系加速度）
 *             │     ├── inv_sqrt ×3（定点倒数平方根）
 *             │     └── cordic  ×3（atan2 -> 欧拉角）
 *             ├── displacement_calc（世界加速度双重积分 -> 位移）
 *             └── uart_tx          （115200 串口）
 *         串口输出：VOFA+ Firewater 协议，100Hz，每帧 54 字节，6 通道逗号分隔 + 换行：
 *           roll,pitch,yaw,x,y,z\n
 *           每通道 8 字符（符号 + 3 位整数 + 小数点 + 3 位小数），例如：
 *           +029.998,+000.000,+000.000,+000.123,-000.045,+000.000\n
 * @param  CLK_FREQ : 主时钟频率(Hz)，默认 50_000_000（PGL22G 板载晶振）
 * @param  BAUD     : 串口波特率，默认 115200
 * @note   1) 默认端口 9 个，适配 PGL22G-6CMBG324（240 IO）
 *         2) VOFA+ 协议引擎选"FireWater"，波特率 115200，6 个通道即对应 6 条曲线
 *         3) 板载 LED 为低电平点亮
 */
module mpu6050_top #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 115200
)(
    input  wire        clk,          // PGL22G: B5 板载50MHz晶振
    input  wire        rst_n,        // PGL22G: F10 复位按键（低有效）
    inout  wire        i2c_scl,      // PGL22G: N14（J8排针第36脚）
    inout  wire        i2c_sda,      // PGL22G: R18（J8排针第35脚）
    output wire        uart_tx,      // PGL22G: C10（板载CP2102）
    output wire [3:0]  led           // PGL22G: U10/V10/U11/V11（LED1~LED4）
);

    //====================== 内部连线 ======================
    wire [15:0] accel_x_raw, accel_y_raw, accel_z_raw;
    wire [15:0] gyro_x_raw,  gyro_y_raw,  gyro_z_raw;
    wire [15:0] temp_raw;
    wire        frame_valid;
    wire [1:0]  error;
    wire [7:0]  state_dbg;

    wire signed [31:0] roll, pitch, yaw;           // 度 Q16.16
    wire signed [31:0] awx, awy, awz;              // m/s² Q16.16（世界系，去重力）
    wire        motion;
    wire        madgwick_valid;

    wire signed [31:0] pos_x, pos_y, pos_z;        // m Q16.16
    wire signed [31:0] vel_x, vel_y, vel_z;        // m/s Q16.16
    wire        disp_valid;

    //====================== MPU6050 驱动 ======================
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
        .temp_raw   (temp_raw),
        .data_valid (frame_valid),
        .error      (error),
        .state_dbg  (state_dbg)
    );

    //====================== Madgwick 姿态解算 ======================
    madgwick_ahrs u_madgwick (
        .clk            (clk),
        .rst_n          (rst_n),
        .accel_x_raw    (accel_x_raw),
        .accel_y_raw    (accel_y_raw),
        .accel_z_raw    (accel_z_raw),
        .gyro_x_raw     (gyro_x_raw),
        .gyro_y_raw     (gyro_y_raw),
        .gyro_z_raw     (gyro_z_raw),
        .data_valid_in  (frame_valid),
        .roll           (roll),
        .pitch          (pitch),
        .yaw            (yaw),
        .accel_world_x  (awx),
        .accel_world_y  (awy),
        .accel_world_z  (awz),
        .motion         (motion),
        .data_valid_out (madgwick_valid)
    );

    //====================== 位移积分 ======================
    displacement_calc u_disp (
        .clk            (clk),
        .rst_n          (rst_n),
        .accel_x        (awx),
        .accel_y        (awy),
        .accel_z        (awz),
        .motion         (motion),
        .data_valid_in  (madgwick_valid),
        .pos_x          (pos_x),
        .pos_y          (pos_y),
        .pos_z          (pos_z),
        .vel_x          (vel_x),
        .vel_y          (vel_y),
        .vel_z          (vel_z),
        .data_valid_out (disp_valid)
    );

    //====================== 状态指示 LED ======================
    // 板载 LED 低电平点亮（阳极接 3.3V、阴极接 FPGA 引脚）
    assign led[0] = (error == 2'b00);              // 错误指示

    reg [7:0] hb_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) hb_cnt <= 8'd0;
        else if (madgwick_valid) hb_cnt <= hb_cnt + 1'b1;
    end
    assign led[1] = hb_cnt[7];                     // 数据心跳 ~2Hz

    assign led[2] = ~((state_dbg[4:0] >= 5'd14) && (state_dbg[4:0] != 5'd17));  // 初始化完成

    reg first_frame;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) first_frame <= 1'b0;
        else if (madgwick_valid) first_frame <= 1'b1;
    end
    assign led[3] = ~first_frame;                  // 首帧成功

    //====================== Firewater 格式化 + 串口发送 ======================
    // 6 通道 × 8 字符 + 5 逗号 + 1 换行 = 54 字节，100Hz 发送
    localparam F_MSG_LEN = 54;
    localparam T10MS     = 25'd500_000;   // 50MHz / 100Hz

    reg  [24:0] tick_cnt;
    reg  [5:0]  char_idx;
    reg  [7:0]  char_byte;
    reg         sending;
    reg         char_sent;
    reg         uart_start;
    wire        tx_busy;
    reg signed [31:0] roll_lat, pitch_lat, yaw_lat;
    reg signed [31:0] x_lat, y_lat, z_lat;

    //---------------------- Q16.16 -> 8 字符字段 ----------------------
    // pos 0=符号(+/-) 1=百位 2=十位 3=个位 4=小数点 5~7=小数3位
    function [7:0] fire_char;
        input [31:0] val;
        input [2:0]  pos;
        reg [31:0] abs_v;
        reg [15:0] int_v;
        reg [15:0] hun, ten, one;
        reg [15:0] frac3, fd2, fd1, fd0;
        begin
            abs_v = val[31] ? (~val + 1'b1) : val;
            int_v = abs_v[31:16];
            hun   = int_v / 16'd100;
            ten   = (int_v % 16'd100) / 16'd10;
            one   = int_v % 16'd10;
            frac3 = (abs_v[15:0] * 32'd1000) >> 16;   // 0~999
            fd2   = frac3 / 8'd100;
            fd1   = (frac3 % 8'd100) / 8'd10;
            fd0   = frac3 % 8'd10;
            case (pos)
                3'd0: fire_char = val[31] ? "-" : "+";
                3'd1: fire_char = 8'd48 + hun[7:0];
                3'd2: fire_char = 8'd48 + ten[7:0];
                3'd3: fire_char = 8'd48 + one[7:0];
                3'd4: fire_char = ".";
                3'd5: fire_char = 8'd48 + fd2[3:0];
                3'd6: fire_char = 8'd48 + fd1[3:0];
                3'd7: fire_char = 8'd48 + fd0[3:0];
                default: fire_char = " ";
            endcase
        end
    endfunction

    //---------------------- 帧内容（按字节查表） ----------------------
    // 字段 k 占字节 [9k, 9k+7]，[9k+8] 为逗号；末字段后换行
    function [7:0] frame_char;
        input [5:0] idx;
        reg [2:0] f;
        reg [3:0] p;
        begin
            f = idx / 9;
            p = idx % 9;
            if (p == 4'd8) begin
                frame_char = (f == 3'd5) ? "\n" : ",";
            end else begin
                case (f)
                    3'd0: frame_char = fire_char(roll_lat,  p);
                    3'd1: frame_char = fire_char(pitch_lat, p);
                    3'd2: frame_char = fire_char(yaw_lat,   p);
                    3'd3: frame_char = fire_char(x_lat,     p);
                    3'd4: frame_char = fire_char(y_lat,     p);
                    default: frame_char = fire_char(z_lat,  p);
                endcase
            end
        end
    endfunction

    always @(*) begin
        char_byte = frame_char(char_idx);
    end

    //---------------------- 发送控制状态机 ----------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cnt  <= 25'd0;
            char_idx  <= 6'd0;
            sending   <= 1'b0;
            char_sent <= 1'b0;
            uart_start<= 1'b0;
            roll_lat  <= 32'sd0;
            pitch_lat <= 32'sd0;
            yaw_lat   <= 32'sd0;
            x_lat     <= 32'sd0;
            y_lat     <= 32'sd0;
            z_lat     <= 32'sd0;
        end else begin
            uart_start <= 1'b0;

            if (!sending) begin
                if (tick_cnt == T10MS) begin
                    tick_cnt  <= 25'd0;
                    roll_lat  <= roll;
                    pitch_lat <= pitch;
                    yaw_lat   <= yaw;
                    x_lat     <= pos_x;
                    y_lat     <= pos_y;
                    z_lat     <= pos_z;
                    char_idx  <= 6'd0;
                    char_sent <= 1'b0;
                    sending   <= 1'b1;
                end else begin
                    tick_cnt <= tick_cnt + 1'b1;
                end
            end else if (!tx_busy && !char_sent) begin
                uart_start <= 1'b1;
                char_sent  <= 1'b1;
            end else if (tx_busy && char_sent) begin
                char_sent <= 1'b0;
                if (char_idx == F_MSG_LEN - 1) begin
                    char_idx <= 6'd0;
                    sending  <= 1'b0;
                end else begin
                    char_idx <= char_idx + 1'b1;
                end
            end
        end
    end

    //---------------------- 例化 UART 发送器 ----------------------
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
