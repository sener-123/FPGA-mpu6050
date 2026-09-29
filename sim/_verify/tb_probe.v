`timescale 1ns/1ns
module tb_probe;
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
        if (!rst_n) begin period_cnt<=0; frame_valid<=0; frame_cnt<=0; end
        else begin
            frame_valid <= 1'b0;
            if (period_cnt == 16'd1999) begin
                period_cnt<=0; frame_valid<=1'b1; frame_cnt<=frame_cnt+1;
            end else period_cnt<=period_cnt+1;
        end
    end
    // constant roll=30 input (ay=0.5g, az=0.866g), gyro=0
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin ax<=0; ay<=0; az<=32'sd65536; gx<=0; gy<=0; gz<=0; end
        else begin
            ay <= 32'sd32768; az <= 32'sd56760;   // constant, ignore rst transition
        end
    end
    always @(posedge clk) begin
        if (q_valid && (frame_cnt==100 || frame_cnt==300 || frame_cnt==600 || frame_cnt==1000 || frame_cnt==1500 || frame_cnt==2000)) begin
            $display("frame=%0d q=(%0d,%0d,%0d,%0d) r31=%0d r32=%0d r33=%0d | ex=%0d wxc=%0d wsx=%0d dq1=%0d",
                frame_cnt, q0,q1,q2,q3, r31,r32,r33, uut.ex, uut.wxc, uut.wsx, uut.dq1);
        end
        if (frame_cnt == 2001) $stop;
    end
    initial begin clk=0; rst_n=0; #100 rst_n=1; end
endmodule
