`timescale 1ns/1ns
// 验证 Firewater 格式化函数（与 mpu6050_top.v 内一致）
module tb_firewater;
    reg signed [31:0] roll_lat, pitch_lat, yaw_lat, x_lat, y_lat, z_lat;

    // ---- 拷贝 mpu6050_top.v 的 fire_char / frame_char ----
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
            frac3 = (abs_v[15:0] * 32'd1000) >> 16;
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

    integer i;
    reg [7:0] c;
    integer nerr;

    initial begin
        nerr = 0;
        roll_lat  = 32'sd819200;      // +12.500
        pitch_lat = -32'sd2981888;    // -45.500
        yaw_lat   = 32'sd11796480;    // +180.000
        x_lat     = 32'sd8192;        // +0.125
        y_lat     = -32'sd32768;      // -0.500
        z_lat     = 32'sd32768;       // +0.500

        $write("frame = ");
        for (i = 0; i < 54; i = i + 1) begin
            c = frame_char(i[5:0]);
            $write("%c", c);
        end
        $write("\n");

        // 期望 "+012.500,-045.500,+180.000,+000.125,-000.500,+000.500\n"
        if (frame_char(0)  != "+") nerr = nerr + 1;
        if (frame_char(1)  != "0") nerr = nerr + 1;
        if (frame_char(3)  != "2") nerr = nerr + 1;
        if (frame_char(5)  != "5") nerr = nerr + 1;
        if (frame_char(8)  != ",") nerr = nerr + 1;
        if (frame_char(9)  != "-") nerr = nerr + 1;
        if (frame_char(11) != "4") nerr = nerr + 1;
        if (frame_char(17) != ",") nerr = nerr + 1;
        if (frame_char(20) != "8") nerr = nerr + 1;
        if (frame_char(26) != ",") nerr = nerr + 1;
        if (frame_char(27) != "+") nerr = nerr + 1;
        if (frame_char(32) != "1") nerr = nerr + 1;   // .125 的 '1'
        if (frame_char(35) != ",") nerr = nerr + 1;
        if (frame_char(36) != "-") nerr = nerr + 1;
        if (frame_char(44) != ",") nerr = nerr + 1;
        if (frame_char(45) != "+") nerr = nerr + 1;
        if (frame_char(53) != 8'd10) nerr = nerr + 1;  // '\n'

        if (nerr == 0) $display("Firewater formatter PASS");
        else $display("Firewater formatter FAIL: %0d errors", nerr);
        $stop;
    end
endmodule
