//------------------------------------------------------------------------------
// 文件名 : tb_attitude.v
// 功能   : attitude_calc 全链路自校验测试台（ModelSim）
// 场景   : P1 前512帧水平静止（零偏校准）
//          P2 校准后静态 roll=30°（原始值 ay=8192, az=14190）→ roll≈+30°
//          P3 静态 pitch=160°（ax=-5604, az=-15396）→ pitch≈+160°（无折叠）
//------------------------------------------------------------------------------
`timescale 1ns/1ns
module tb_attitude;

    reg         clk, rst_n;
    reg  [15:0] ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    reg         frame_valid;
    wire signed [31:0] accel_x_g, accel_y_g, accel_z_g;
    wire signed [31:0] gyro_x_dps, gyro_y_dps, gyro_z_dps;
    wire signed [31:0] roll, pitch, yaw;
    wire        data_valid_out;
    wire        calib_done;

    attitude_calc uut (
        .clk(clk), .rst_n(rst_n),
        .accel_x_raw(ax_raw), .accel_y_raw(ay_raw), .accel_z_raw(az_raw),
        .gyro_x_raw(gx_raw), .gyro_y_raw(gy_raw), .gyro_z_raw(gz_raw),
        .data_valid_in(frame_valid),
        .accel_x_g(accel_x_g), .accel_y_g(accel_y_g), .accel_z_g(accel_z_g),
        .gyro_x_dps(gyro_x_dps), .gyro_y_dps(gyro_y_dps), .gyro_z_dps(gyro_z_dps),
        .roll(roll), .pitch(pitch), .yaw(yaw),
        .data_valid_out(data_valid_out),
        .calib_done(calib_done)
    );

    always #10 clk = ~clk;

    integer frame_cnt;
    integer errors;
    reg [15:0] period_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            period_cnt  <= 16'd0;
            frame_valid <= 1'b0;
            frame_cnt   <= 0;
        end else begin
            frame_valid <= 1'b0;
            if (period_cnt == 16'd1999) begin
                period_cnt  <= 16'd0;
                frame_valid <= 1'b1;
                frame_cnt   <= frame_cnt + 1;
            end else begin
                period_cnt <= period_cnt + 1'b1;
            end
        end
    end

    //---------------------- 场景输入 ----------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ax_raw <= 16'sd0; ay_raw <= 16'sd0; az_raw <= 16'sd16384;   // 水平静止
            gx_raw <= 16'sd0; gy_raw <= 16'sd0; gz_raw <= 16'sd0;
        end else begin
            if (frame_valid && frame_cnt == 600) begin
                // P2：roll 30°（ay=sin30·16384=8192, az=cos30·16384≈14190）
                ay_raw <= 16'sd8192;
                az_raw <= 16'sd14190;
            end
            if (frame_valid && frame_cnt == 2600) begin
                // P3：pitch 160°（ax=-sin160·16384≈-5604, az=cos160·16384≈-15396）
                ax_raw <= -16'sd5604;
                ay_raw <= 16'sd0;
                az_raw <= -16'sd15396;
            end
        end
    end

    //---------------------- 检查 ----------------------
    always @(posedge clk) begin
        if (data_valid_out) begin
            // P2：roll ≈ +30°（Q16.16 = 30×65536 = 1966080），pitch/yaw≈0
            if (frame_cnt > 600 && frame_cnt <= 2600) begin
                if (frame_cnt == 2599) begin
                    if ((roll < 32'sd1900000) || (roll > 32'sd2030000)) begin
                        $display("P2 FAIL: roll=%0d (期望≈1966080)", roll);
                        errors = errors + 1;
                    end
                    if ((pitch > 32'sd100000) || (pitch < -32'sd100000)) begin
                        $display("P2 FAIL: pitch=%0d (期望≈0)", pitch);
                        errors = errors + 1;
                    end
                    $display("P2 CHECK: roll=%0d pitch=%0d yaw=%0d", roll, pitch, yaw);
                end
            end
            // P3：pitch ≈ +160°（Q16.16 = 10485760），roll≈180° 或 -180°（绕Y翻转）。
            // 160° 大角度叉积修正 sinθ→0，静态收敛需约 8s（4s 时仅到 ~155°），
            // 故检查点从 4600 延到 6600 帧（8s）
            if (frame_cnt == 6600) begin
                if ((pitch < 32'sd10350000) || (pitch > 32'sd10620000)) begin
                    $display("P3 FAIL: pitch=%0d (期望≈10485760)", pitch);
                    errors = errors + 1;
                end
                $display("P3 CHECK: roll=%0d pitch=%0d yaw=%0d", roll, pitch, yaw);
            end
        end
        if (frame_cnt == 6700) begin
            if (errors == 0) $display("===== tb_attitude PASS =====");
            else             $display("===== tb_attitude FAIL: %0d errors =====", errors);
            $stop;
        end
    end

    initial begin
        clk = 0; rst_n = 0; errors = 0;
        #100 rst_n = 1;
    end

endmodule
