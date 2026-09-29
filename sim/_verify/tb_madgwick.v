`timescale 1ns/1ns
// Madgwick + 位移 验证：与 Python 金标准 gen_expected.py 逐帧比对
// 输出 actual.txt：索引 + 四元数(Q16.16) + 欧拉角(Q16.16) + 位移(Q16.16)
module tb_madgwick;
    reg clk, rst_n;
    reg signed [15:0] ax, ay, az, gx, gy, gz;
    reg frame_valid;
    wire signed [31:0] roll, pitch, yaw, awx, awy, awz;
    wire motion, mvalid;
    wire signed [31:0] pos_x, pos_y, pos_z, vel_x, vel_y, vel_z;
    wire dvalid;
    wire signed [31:0] q0, q1, q2, q3;

    madgwick_ahrs uut (
        .clk(clk), .rst_n(rst_n),
        .accel_x_raw(ax), .accel_y_raw(ay), .accel_z_raw(az),
        .gyro_x_raw(gx), .gyro_y_raw(gy), .gyro_z_raw(gz),
        .data_valid_in(frame_valid),
        .roll(roll), .pitch(pitch), .yaw(yaw),
        .accel_world_x(awx), .accel_world_y(awy), .accel_world_z(awz),
        .motion(motion), .data_valid_out(mvalid),
        .q0_dbg(q0), .q1_dbg(q1), .q2_dbg(q2), .q3_dbg(q3)
    );

    displacement_calc ud (
        .clk(clk), .rst_n(rst_n),
        .accel_x(awx), .accel_y(awy), .accel_z(awz),
        .motion(motion), .data_valid_in(mvalid),
        .pos_x(pos_x), .pos_y(pos_y), .pos_z(pos_z),
        .vel_x(vel_x), .vel_y(vel_y), .vel_z(vel_z),
        .data_valid_out(dvalid)
    );

    always #10 clk = ~clk;

    integer frame_cnt;
    reg [15:0] period_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin period_cnt<=0; frame_valid<=0; frame_cnt<=0; end
        else begin
            frame_valid <= 1'b0;
            if (period_cnt == 16'd1999) begin
                period_cnt<=0; frame_valid<=1'b1; frame_cnt<=frame_cnt+1;
            end else period_cnt<=period_cnt+1;
        end
    end

    // 输入：前 1000 帧静止平放，之后绕 X 倾斜 30°
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin ax<=0; ay<=0; az<=0; gx<=0; gy<=0; gz<=0; end
        else begin
            if (frame_cnt < 1000) begin ax<=0; ay<=0; az<=16'sd16384; end
            else                 begin ax<=0; ay<=16'sd8192; az<=16'sd14189; end
            gx<=0; gy<=0; gz<=0;
        end
    end

    integer out_idx;
    integer fh;
    initial begin
        fh = $fopen("actual.txt", "w");
        out_idx = 0;
    end

    always @(posedge clk) begin
        if (mvalid) begin
            $fdisplay(fh, "%0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d",
                out_idx, q0, q1, q2, q3, roll, pitch, yaw, pos_x, pos_y, pos_z);
            out_idx <= out_idx + 1;
            if (out_idx == 3999) $stop;
        end
    end

    initial begin
        clk=0; rst_n=0;
        #100 rst_n=1;
    end
endmodule
