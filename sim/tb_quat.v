//------------------------------------------------------------------------------
// 文件名 : tb_quat.v
// 功能   : quaternion_ahrs 四元数引擎自校验测试台（ModelSim）
// 场景   : T1 水平静止（恒等四元数保持）
//          T2 静态 roll=30° 收敛（a=[0,0.5g,0.866g]）
//          T3 静态 pitch=160° 收敛（a=[-0.342g,0,-0.94g]，检验无折叠）
//          T4 纯 yaw 旋转 90°/s×1s（检验积分与归一化）
//------------------------------------------------------------------------------
`timescale 1ns/1ns
module tb_quat;

    reg         clk;
    reg         rst_n;
    reg signed [31:0] ax, ay, az, gx, gy, gz;
    reg         frame_valid;
    wire signed [31:0] q0, q1, q2, q3;
    wire signed [31:0] r31, r32, r33, r21, r11;
    wire        q_valid;

    quaternion_ahrs uut (
        .clk(clk), .rst_n(rst_n),
        .accel_x(ax), .accel_y(ay), .accel_z(az),
        .gyro_x(gx), .gyro_y(gy), .gyro_z(gz),
        .frame_valid(frame_valid),
        .q0_out(q0), .q1_out(q1), .q2_out(q2), .q3_out(q3),
        .r31(r31), .r32(r32), .r33(r33), .r21(r21), .r11(r11),
        .q_valid(q_valid)
    );

    always #10 clk = ~clk;      // 50MHz，周期20ns

    integer frame_cnt;
    integer errors;
    reg [15:0] period_cnt;

    //---------------------- 帧产生（每2000时钟一帧，等效500Hz） ----------------------
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

    //---------------------- 场景输入（随帧数切换，帧沿更新） ----------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ax <= 0; ay <= 0; az <= 32'sd65536;   // 水平静止 1g
            gx <= 0; gy <= 0; gz <= 0;
        end else begin
            if (frame_valid && frame_cnt == 2000) begin
                // T2：roll 30°（a_body = g·[0, sin30, cos30]）
                ax <= 0; ay <= 32'sd32768; az <= 32'sd56755;
            end
            if (frame_valid && frame_cnt == 4000) begin
                // T3：pitch 160°（a_body = g·[-sin160, 0, cos160]）
                ax <= -32'sd22411; ay <= 0; az <= -32'sd61605;
            end
            if (frame_valid && frame_cnt == 6000) begin
                // 回正静置：加速度水平、陀螺归零，让姿态从160°回落（约1.5s）
                ax <= 0; ay <= 0; az <= 32'sd65536;
                gz <= 0;
            end
            if (frame_valid && frame_cnt == 7510) begin
                // T4：纯 yaw 90°/s（raw=11790），加速度保持水平
                ax <= 0; ay <= 0; az <= 32'sd65536;
                gz <= 32'sd11790;
            end
        end
    end

    //---------------------- T4 前置复位：四元数回恒等态 ----------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rst_n <= 1'b0;
        end else if (frame_cnt == 7500) begin
            rst_n <= 1'b0;          // 复位四元数
        end else if (frame_cnt == 7503) begin
            rst_n <= 1'b1;
        end
    end

    //---------------------- 期望值检查（在 q_valid 脉冲沿取样） ----------------------
    always @(posedge clk) begin
        if (q_valid) begin
            // T1：水平静止（帧0~1999），恒等四元数
            if (frame_cnt <= 2000) begin
                if ((r33 < 32'sd32268) || (r33 > 32'sd33268)) begin
                    $display("T1 FAIL @frame %0d: r33=%0d (期望≈32768)", frame_cnt, r33);
                    errors = errors + 1;
                end
                if ((r31 > 32'sd500) || (r31 < -32'sd500) || (r32 > 32'sd500) || (r32 < -32'sd500)) begin
                    $display("T1 FAIL @frame %0d: r31=%0d r32=%0d (期望≈0)", frame_cnt, r31, r32);
                    errors = errors + 1;
                end
            end
            // T2：roll 30°（帧2000~3999，收敛后 R32=sin30·32768, R33=cos30·32768）
            if (frame_cnt == 3999) begin
                if ((r32 < 32'sd15900) || (r32 > 32'sd16900)) begin
                    $display("T2 FAIL: r32=%0d (期望≈16384)", r32);
                    errors = errors + 1;
                end
                if ((r33 < 32'sd27900) || (r33 > 32'sd28900)) begin
                    $display("T2 FAIL: r33=%0d (期望≈28378)", r33);
                    errors = errors + 1;
                end
                if ((r31 > 32'sd600) || (r31 < -32'sd600)) begin
                    $display("T2 FAIL: r31=%0d (期望≈0)", r31);
                    errors = errors + 1;
                end
                $display("T2 CHECK: r31=%0d r32=%0d r33=%0d", r31, r32, r33);
            end
            // T3：pitch 160°（帧4000~5999），无折叠：R31=-sin160·32768, R33=cos160·32768<0
            if (frame_cnt == 5999) begin
                if ((r31 < -32'sd11709) || (r31 > -32'sd10709)) begin
                    $display("T3 FAIL: r31=%0d (期望≈-11209)", r31);
                    errors = errors + 1;
                end
                if ((r33 < -32'sd31300) || (r33 > -32'sd30300)) begin
                    $display("T3 FAIL: r33=%0d (期望≈-30795)", r33);
                    errors = errors + 1;
                end
                $display("T3 CHECK: r31=%0d r32=%0d r33=%0d", r31, r32, r33);
            end
            // T4：yaw 90°（帧7510~8009，1秒后 R21=sin90·32768, R11=cos90·32768≈0）
            if (frame_cnt == 8009) begin
                if ((r21 < 32'sd32100) || (r21 > 32'sd33400)) begin
                    $display("T4 FAIL: r21=%0d (期望≈32768)", r21);
                    errors = errors + 1;
                end
                if ((r11 > 32'sd1500) || (r11 < -32'sd1500)) begin
                    $display("T4 FAIL: r11=%0d (期望≈0)", r11);
                    errors = errors + 1;
                end
                // 四元数范数保持：|q|² ≈ 32768²（Q16.15）
                $display("T4 CHECK: r21=%0d r11=%0d r33=%0d", r21, r11, r33);
                $display("T4 NORM: q0=%0d q1=%0d q2=%0d q3=%0d", q0, q1, q2, q3);
                if (((q0*q0)+(q1*q1)+(q2*q2)+(q3*q3)) < 64'sd1073741824 - 64'sd130000000 ||
                    ((q0*q0)+(q1*q1)+(q2*q2)+(q3*q3)) > 64'sd1073741824 + 64'sd130000000) begin
                    $display("T4 FAIL: |q|² 偏差过大");
                    errors = errors + 1;
                end
            end
        end
        // 仿真结束
        if (frame_cnt == 8120) begin
            if (errors == 0) $display("===== tb_quat PASS =====");
            else             $display("===== tb_quat FAIL: %0d errors =====", errors);
            $stop;
        end
    end

    initial begin
        clk = 0; rst_n = 0; errors = 0;
        #100 rst_n = 1;
    end

endmodule
