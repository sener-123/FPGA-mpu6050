`timescale 1ns/1ns
module tb_fv;
    reg clk, rst_n;
    reg [15:0] ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    reg frame_valid;
    wire [31:0] roll; wire data_valid_out, calib_done;
    attitude_calc uut (
        .clk(clk), .rst_n(rst_n),
        .accel_x_raw(ax_raw), .accel_y_raw(ay_raw), .accel_z_raw(az_raw),
        .gyro_x_raw(gx_raw), .gyro_y_raw(gy_raw), .gyro_z_raw(gz_raw),
        .data_valid_in(frame_valid), .accel_x_g(), .accel_y_g(), .accel_z_g(),
        .gyro_x_dps(), .gyro_y_dps(), .gyro_z_dps(), .roll(roll), .pitch(), .yaw(),
        .data_valid_out(data_valid_out), .calib_done(calib_done)
    );
    always #10 clk = ~clk;
    integer frame_cnt;
    reg [15:0] period_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin period_cnt<=0; frame_valid<=0; frame_cnt<=0; end
        else begin
            frame_valid <= 1'b0;
            if (period_cnt == 16'd1999) begin period_cnt<=0; frame_valid<=1'b1; frame_cnt<=frame_cnt+1; end
            else period_cnt<=period_cnt+1;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin ax_raw<=0; ay_raw<=0; az_raw<=16'sd16384; gx_raw<=0; gy_raw<=0; gz_raw<=0; end
    end
    // Dump quaternion frame_valid + state at each testbench frame_valid pulse
    integer fv_cnt, qfv_cnt;
    initial begin fv_cnt=0; qfv_cnt=0; end
    always @(posedge clk) begin
        if (frame_valid) begin
            fv_cnt <= fv_cnt + 1;
            if (frame_cnt >= 510 && frame_cnt <= 520)
                $display("frame=%0d calib=%b quat.frame_valid=%b quat.state=%b quat.step=%0d",
                    frame_cnt, calib_done, uut.u_quaternion_ahrs.frame_valid, uut.u_quaternion_ahrs.state, uut.u_quaternion_ahrs.step);
        end
        if (uut.u_quaternion_ahrs.frame_valid) qfv_cnt <= qfv_cnt + 1;
        if (frame_cnt == 2600) begin
            $display("testbench frame_valid=%0d  quat.frame_valid=%0d", fv_cnt, qfv_cnt);
            $stop;
        end
    end
    initial begin clk=0; rst_n=0; #100 rst_n=1; end
endmodule
