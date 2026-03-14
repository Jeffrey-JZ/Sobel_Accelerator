`timescale 1ns / 1ps

module tb_sobel_datapath #(
    parameter WIDTH         = 12,
    parameter CLIPPED_W     = 8,
    parameter ADDR_W        = 14,
    parameter CLK_PERIOD    = 10
) ();

    reg                 clk;
    reg                 rst_n;
    reg                 valid_in;
    reg  [ADDR_W-1:0]   addr_in;
    reg  [7:0]          p00, p01, p02;
    reg  [7:0]          p10, p11, p12;
    reg  [7:0]          p20, p21, p22;
    reg  [CLIPPED_W-1:0] threshold;

    wire                valid_out;
    wire [ADDR_W-1:0]   addr_out;
    wire [CLIPPED_W-1:0] edge_out;

    sobel_datapath #(
        .WIDTH(WIDTH),
        .clipped_w(CLIPPED_W),
        .ADDR_W(ADDR_W)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(valid_in),
        .addr_in(addr_in),
        .p00(p00), .p01(p01), .p02(p02),
        .p10(p10), .p11(p11), .p12(p12),
        .p20(p20), .p21(p21), .p22(p22),
        .threshold(threshold),
        .valid_out(valid_out),
        .addr_out(addr_out),
        .edge_out(edge_out)
    );

    always #(CLK_PERIOD/2) clk = ~clk;

    initial begin
        clk = 0;
        rst_n = 0;
        valid_in = 0;
        addr_in = 0;
        threshold = 8'd60;
        p00=0; p01=0; p02=0; p10=0; p11=0; p12=0; p20=0; p21=0; p22=0;

        #(CLK_PERIOD * 2);
        rst_n = 1;
        #(CLK_PERIOD);

        // --- Scenario 1: Detecting vertical edges (Gx is large) ---
        // Simulating left black (0), right white (255)
        // Expected Gx = (255 + 2*255 + 255) - (0 + 0 + 0) = 1020
        // Expected Gy = (0 + 0 + 0) - (0 + 0 + 0) = 0
        // Expected Mag = 1020 -> exceeds 255, output should be clipped to 255
        drive_pixels(14'h001, 0, 0, 255, 0, 0, 255, 0, 0, 255);
        
        // --- Scene 2: Detecting horizontal edges (Gy greater) ---
        // Expected Gx = 0, Gy = 1020, Mag = 1020 -> Output 255
        drive_pixels(14'h002, 0, 0, 0, 0, 0, 0, 255, 255, 255);

        // --- Scene 3: Smoothed Region (below threshold) ---
        // Expected Mag = 0 -> Output 0
        drive_pixels(14'h003, 50, 50, 50, 50, 50, 50, 50, 50, 50);

        // --- Scene 4: Weak Edge (Near Threshold) ---
        // p02=20, p12=20, p22=20 -> Gx = 80, Gy = 0, Mag = 80
        // 80 > 60 (threshold), output should be 80
        drive_pixels(14'h004, 0, 0, 20, 0, 0, 20, 0, 0, 20);

        @(posedge clk);

        // Stop input
        valid_in = 0;
        
        #(CLK_PERIOD * 10);
        $display("===== Sobel Datapath Testing completed =====");
        $finish;
    end

    // Auxiliary Task: Drive Pixel Input
    task drive_pixels(
        input [ADDR_W-1:0] addr,
        input [7:0] _p00, input [7:0] _p01, input [7:0] _p02,
        input [7:0] _p10, input [7:0] _p11, input [7:0] _p12,
        input [7:0] _p20, input [7:0] _p21, input [7:0] _p22
    );
    begin
        @(posedge clk);
        valid_in <= 1;
        addr_in  <= addr;
        p00 <= _p00; p01 <= _p01; p02 <= _p02;
        p10 <= _p10; p11 <= _p11; p12 <= _p12;
        p20 <= _p20; p21 <= _p21; p22 <= _p22;
    end
    endtask

    // Monitor output (taking into account a two-cycle delay)
    always @(posedge clk) begin
        if (valid_out) begin
            $display("[RESULT] Time: %t | Addr: %h | Edge: %d", $time, addr_out, edge_out);
        end
    end
endmodule