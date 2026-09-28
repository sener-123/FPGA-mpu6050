//------------------------------------------------------------------------------
// 文件名 : mpu6050_top.v
// 功能   : 顶层模块：例化驱动/解算/串口，并把结果格式化成 ASCII 文本帧打印
// 日期   : 2026-09-24（2026-09-26 精简端口适配 PGL22G 开发板；
//          2026-09-28 修复小数位恒为 0 的乘法位宽 bug）
//------------------------------------------------------------------------------
/* @brief  MPU6050 姿态解算顶层
 *         模块层次：
 *           mpu6050_top
 *             ├── mpu6050_driver   （I2C 初始化 + 500Hz 采集，内含 i2c_master）
 *             ├── attitude_calc    （零偏校准 + 四元数姿态解算 + 3 路 CORDIC）
 *             └── uart_tx          （115200 串口打印）
 *         串口输出：每 100ms 一帧 FireWater 协议（VOFA+）CSV 数值流，
 *         9 通道逗号分隔、\r\n 结尾（帧长可变，前导零/正号省略），示例：
 *           0.00,-0.35,0.00,0.00,0.00,1.00,0.00,0.00,0.00
 *         通道顺序：roll,pitch,yaw,ax,ay,az,gx,gy,gz
 *         （角度单位 °，加速度单位 g，角速度单位 °/s，均两位小数）
 * @param  CLK_FREQ : 主时钟频率(Hz)，默认 50_000_000（PGL22G 板载晶振）
 * @param  BAUD     : 串口波特率，默认 115200
 * @param  clk/rst_n : 时钟与异步复位（低有效）
 * @return i2c_scl  : I2C 时钟线（接 MPU6050 SCL，需 4.7kΩ 上拉，GY-521 模块自带）
 * @return i2c_sda  : I2C 数据线（接 MPU6050 SDA）
 * @return uart_tx  : 串口发送（板载 CP2102 或外接 USB-TTL）
 * @return led[3:0] : 状态指示灯（低电平点亮）：
 *                    [0]=错误指示（error≠0 点亮）
 *                    [1]=数据心跳（约2Hz，数据正常流动时闪烁）
 *                    [2]=初始化完成（进入采样状态后点亮）
 *                    [3]=零偏校准完成（点亮表示系统就绪；校准期间请保持板子水平静止约1秒）
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
    wire        calib_done;      // 零偏校准完成（复位后约1秒）

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
        .data_valid_out (data_valid),
        .calib_done     (calib_done)
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

    // led[3]：零偏校准完成（点亮表示系统就绪，校准期间保持板子水平静止约1秒）
    assign led[3] = ~calib_done;

    //====================== ASCII 格式化 + 串口发送 ======================
    // FireWater 协议（VOFA+）：CSV 风格数值流，9 通道逗号分隔、\r\n 结尾，
    // 帧长度可变（前导零与正号省略），如：
    //   0.00,-0.35,0.00,0.00,0.00,1.00,0.00,0.00,0.00\r\n
    // 每 100ms 发送一帧
    localparam T100MS     = 25'd5_000_000;

    reg  [24:0] tick_cnt;     // 100ms 定时
    reg  [7:0]  char_byte;    // 当前字符 ASCII 码（组合逻辑生成）
    reg         char_valid;   // 当前字符有效（0=跳过：正号/前导零省略）
    reg         sending;      // 一帧发送中
    reg         char_sent;    // 当前字符已交给 uart 标志
    reg         uart_start;   // uart 触发脉冲
    reg  [3:0]  field;        // 字段：0=roll 1=pitch 2=yaw 3=ax 4=ay 5=az 6=gx 7=gy 8=gz
    reg  [3:0]  sub;          // 字符位：0=负号 1=百位 2=十位 3=个位 4=小数点 5=小数1
                              //         6=小数2 7=逗号/回车 8=换行
    wire        tx_busy;
    // 帧开始时锁存的数值（保证一帧内数据一致）
    reg signed [31:0] roll_lat,  pitch_lat, yaw_lat;
    reg signed [31:0] ax_lat, ay_lat, az_lat, gx_lat, gy_lat, gz_lat;

    //---------------------- 当前字段的十进制分解（组合逻辑） ----------------------
    // 负数取绝对值后分解：整数部分取 Q16.16 高16位，小数部分 ×100 取整（两位小数）
    reg signed [31:0] cur_val;
    reg [31:0] abs_v;
    reg [15:0] hun, ten, one, f2, f1, f0;
    reg        neg;
    always @(*) begin
        case (field)
            4'd0: cur_val = roll_lat;
            4'd1: cur_val = pitch_lat;
            4'd2: cur_val = yaw_lat;
            4'd3: cur_val = ax_lat;
            4'd4: cur_val = ay_lat;
            4'd5: cur_val = az_lat;
            4'd6: cur_val = gx_lat;
            4'd7: cur_val = gy_lat;
            default: cur_val = gz_lat;
        endcase
        neg  = cur_val[31];
        abs_v = neg ? (~cur_val + 1'b1) : cur_val;
        hun = abs_v[31:16] / 16'd100;
        ten = (abs_v[31:16] % 16'd100) / 16'd10;
        one = abs_v[31:16] % 16'd10;
        // 两位小数组合值 0~99：乘法必须用 32 位——16×16 的 Verilog 结果位宽
        // 只有 16 位，乘积被截断后 >>16 恒为 0，小数位永远打印 .00
        // （2026-09-28 修复）
        f2  = (abs_v[15:0] * 32'd100) >> 16;
        f1  = f2 / 16'd10;
        f0  = f2 % 16'd10;
    end

    //---------------------- 字符生成（按 field/sub 查表） ----------------------
    always @(*) begin
        char_valid = 1'b1;
        case (sub)
            4'd0: begin                       // 负号：正数省略
                char_byte  = "-";
                char_valid = neg;
            end
            4'd1: begin                       // 百位：为0省略
                char_byte  = 8'd48 + hun[3:0];
                char_valid = (hun != 16'd0);
            end
            4'd2: begin                       // 十位：百位十位都为0省略
                char_byte  = 8'd48 + ten[3:0];
                char_valid = (hun != 16'd0) || (ten != 16'd0);
            end
            4'd3: char_byte = 8'd48 + one[3:0];
            4'd4: char_byte = ".";
            4'd5: char_byte = 8'd48 + f1[3:0];
            4'd6: char_byte = 8'd48 + f0[3:0];
            4'd7: char_byte = (field < 4'd8) ? "," : "\r";   // 字段间逗号，帧尾回车
            4'd8: char_byte = "\n";                          // 帧尾换行（FireWater 必须）
            default: begin
                char_byte  = " ";
                char_valid = 1'b0;
            end
        endcase
    end

    //---------------------- 发送控制状态机 ----------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cnt  <= 25'd0;
            field     <= 4'd0;
            sub       <= 4'd0;
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
                    field     <= 4'd0;
                    sub       <= 4'd0;
                    char_sent <= 1'b0;
                    sending   <= 1'b1;
                end else begin
                    tick_cnt <= tick_cnt + 1'b1;
                end
            end else if (!tx_busy && !char_sent) begin
                // 发送器空闲：有效字符交给 uart，无效字符（省略位）直接跳过
                if (char_valid) begin
                    uart_start <= 1'b1;
                    char_sent  <= 1'b1;
                end else if (sub < 4'd7) begin
                    sub <= sub + 1'b1;
                end else if (sub == 4'd7) begin
                    if (field < 4'd8) begin
                        field <= field + 1'b1;
                        sub   <= 4'd0;
                    end else begin
                        sub <= 4'd8;
                    end
                end else begin
                    // sub==8：一帧发送完毕
                    field   <= 4'd0;
                    sub     <= 4'd0;
                    sending <= 1'b0;
                end
            end else if (tx_busy && char_sent) begin
                // 当前字符已被 uart 锁存：指向下一个字符
                char_sent <= 1'b0;
                if (sub < 4'd7) begin
                    sub <= sub + 1'b1;
                end else if (sub == 4'd7) begin
                    if (field < 4'd8) begin
                        field <= field + 1'b1;
                        sub   <= 4'd0;
                    end else begin
                        sub <= 4'd8;
                    end
                end else begin
                    // sub==8：一帧发送完毕
                    field   <= 4'd0;
                    sub     <= 4'd0;
                    sending <= 1'b0;
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
