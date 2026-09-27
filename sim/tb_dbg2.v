`timescale 1ns/1ns
module tb_dbg2;
    reg clk, rst_n;
    reg signed [31:0] ax, ay, az, gx, gy, gz;
    reg frame_valid;
    wire signed [31:0] q0,q1,q2,q3;
    wire signed [31:0] r31,r32,r33,r21,r11;
    wire q_valid;
    quaternion_ahrs uut (
        .clk(clk),.rst_n(rst_n),
        .accel_x(ax),.accel_y(ay),.accel_z(az),
        .gyro_x(gx),.gyro_y(gy),.gyro_z(gz),
        .frame_valid(frame_valid),
        .q0_out(q0),.q1_out(q1),.q2_out(q2),.q3_out(q3),
        .r31(r31),.r32(r32),.r33(r33),.r21(r21),.r11(r11),
        .q_valid(q_valid)
    );
    always #10 clk = ~clk;
    integer frame_cnt;
    reg [15:0] period_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            period_cnt <= 0; frame_valid <= 0; frame_cnt <= 0;
        end else begin
            frame_valid <= 1'b0;
            if (period_cnt == 16'd999) begin
                period_cnt <= 0; frame_valid <= 1'b1; frame_cnt <= frame_cnt + 1;
            end else period_cnt <= period_cnt + 1;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ax <= 0; ay <= 0; az <= 32'sd65536; gx <= 0; gy <= 0; gz <= 0;
        end else begin
            if (frame_valid && frame_cnt == 50) begin
                ax <= 0; ay <= 32'sd32768; az <= 32'sd56755;
            end
        end
    end
    always @(posedge clk) begin
        if (q_valid && frame_cnt >= 51 && frame_cnt <= 56) begin
            $display("f=%0d q1=%0d | vx=%0d vy=%0d vz=%0d | ex=%0d ey=%0d ez=%0d | t13=%0d wxc=%0d wsx=%0d | u4=%0d dq1=%0d",
                frame_cnt, q1,
                uut.vx, uut.vy, uut.vz, uut.ex, uut.ey, uut.ez,
                uut.t13, uut.wxc, uut.wsx, uut.u4, uut.dq1);
        end
        if (frame_cnt == 60) $stop;
    end
    initial begin clk = 0; rst_n = 0; #100 rst_n = 1; end
endmodule
