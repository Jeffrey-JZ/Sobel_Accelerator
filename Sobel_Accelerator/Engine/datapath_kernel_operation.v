`timescale 1ns / 1ps

// Datapath of the engine
// The kernel operations of Engine
// Author: Junze Jiang
// 19/2/2026

// 1. Data Path: Kernel operation
/*
origianl image: 00 01 02    Sobel Kernel:   x:  -1 0 1  y:  -1 -2 -1
                10 11 12                        -2 0 2       0  0  0
                20 21 22                        -1 0 1       1  2  1

Gx�?=(p02+2p12+p22)�?(p00+2p10+p20)
Gy�?=(p20+2p21+p22)�?(p00+2p01+p02)
mag≈∣Gx​∣+∣Gy​∣

Using 3 stages pipeline to increase the throughput
    Stage 1: Gradient calculation
    Stage 2: Absolute + Magnitude
    Stage 3: Compare + Clip
Latency: 2 cycle
*/
module sobel_datapath #(
    parameter   WIDTH       = 12,
    parameter   clipped_w   = 8,
    parameter   ADDR_W      = 14
) (
    input   clk     ,
    input   rst_n   ,

    input   wire                    valid_in,
    input   wire    [ADDR_W-1:0]    addr_in,

    input   wire    [7:0]   p00,    input   wire    [7:0]   p01,    input   wire    [7:0]   p02,
    input   wire    [7:0]   p10,    input   wire    [7:0]   p11,    input   wire    [7:0]   p12,
    input   wire    [7:0]   p20,    input   wire    [7:0]   p21,    input   wire    [7:0]   p22,

    input   wire    [clipped_w  - 1 :0] threshold,

    output  reg                         valid_out,
    output  reg     [ADDR_W-1       :0] addr_out,

    output  reg     [clipped_w  - 1 :0] edge_out
);

    // -------- Stage S1: gradient calculation --------
    // Extend the 8-bit pixels to 12-bit signed integers
    wire    [WIDTH - 1:0]   s00 = $signed({4'd0, p00});
    wire    [WIDTH - 1:0]   s01 = $signed({4'd0, p01});
    wire    [WIDTH - 1:0]   s02 = $signed({4'd0, p02});
    wire    [WIDTH - 1:0]   s10 = $signed({4'd0, p10});
    wire    [WIDTH - 1:0]   s11 = $signed({4'd0, p11});
    wire    [WIDTH - 1:0]   s12 = $signed({4'd0, p12});
    wire    [WIDTH - 1:0]   s20 = $signed({4'd0, p20});
    wire    [WIDTH - 1:0]   s21 = $signed({4'd0, p21});
    wire    [WIDTH - 1:0]   s22 = $signed({4'd0, p22});

    // Shifter to get 2s12, 2s10, 2s21, 2s01
    wire    [WIDTH - 1:0]   lef_shifter_2s12,    lef_shifter_2s10,    lef_shifter_2s21,    lef_shifter_2s01;
    left_shift_ip #(.WIDTH(12)) u_shif_2s12 (.a(s12), .shifted(lef_shifter_2s12));
    left_shift_ip #(.WIDTH(12)) u_shif_2s10 (.a(s10), .shifted(lef_shifter_2s10));
    left_shift_ip #(.WIDTH(12)) u_shif_2s21 (.a(s21), .shifted(lef_shifter_2s21));
    left_shift_ip #(.WIDTH(12)) u_shif_2s01 (.a(s10), .shifted(lef_shifter_2s01));

    //Signed Subtractor
    wire    [WIDTH - 1:0]   sub_0200,   sub_212210, sub_2220,
                            sub_2000,   sub_221201, sub_2202;
    signed_sub_ip #(.WIDTH(12)) u_sub_s02s00    (.a(s02),               .b(s00),                .sub(sub_0200)  );
    signed_sub_ip #(.WIDTH(12)) u_sub_2s122s10  (.a(lef_shifter_2s12),  .b(lef_shifter_2s10),   .sub(sub_212210));
    signed_sub_ip #(.WIDTH(12)) u_sub_s22s20    (.a(s22),               .b(s20),                .sub(sub_2220)  );
    signed_sub_ip #(.WIDTH(12)) u_sub_s20s00    (.a(s20),               .b(s00),                .sub(sub_2000)  );
    signed_sub_ip #(.WIDTH(12)) u_sub_2s212s01  (.a(lef_shifter_2s21),  .b(lef_shifter_2s01),   .sub(sub_221201));
    signed_sub_ip #(.WIDTH(12)) u_sub_s22s02    (.a(s22),               .b(s02),                .sub(sub_2202)  );

    // Signed Adder
    wire    [WIDTH - 1:0]   sum_x_12,   sum_x,
                            sum_y_12,   sum_y;
    signed_add_ip #(.WIDTH(12)) u_add_x_inter   (.a(sub_0200), .b(sub_212210), .sum(sum_x_12));
    signed_add_ip #(.WIDTH(12)) u_add_x         (.a(sum_x_12), .b(sub_2220)  , .sum(sum_x)   );
    signed_add_ip #(.WIDTH(12)) u_add_y_inter   (.a(sub_2000), .b(sub_221201), .sum(sum_y_12));
    signed_add_ip #(.WIDTH(12)) u_add_y         (.a(sum_y_12), .b(sub_2202)  , .sum(sum_y)   );

    // Stage 1 register
    reg signed  [WIDTH-1    :0]     sumx_s1, sumy_s1;
    reg                             v_s1;
    reg         [ADDR_W-1   :0]     addr_s1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sumx_s1 <=   'd0;
            sumy_s1 <=   'd0;
            v_s1    <=  1'd0;
            addr_s1 <=   'd0;
        end else begin
            sumx_s1 <=  sum_x;
            sumy_s1 <=  sum_y;
            v_s1    <=  valid_in;
            addr_s1 <=  addr_in;
        end
    end


    // -------- Stage S2: absolute + magnitude --------
    // mag≈∣Gx​∣+∣Gy​∣
    wire    [WIDTH - 1:0]   gx, gy, mag;
    abs_ip #(.WIDTH(12)) u_abs_gx    (.a(sumx_s1), .abs(gx));
    abs_ip #(.WIDTH(12)) u_abs_gy    (.a(sumy_s1), .abs(gy));
    
    signed_add_ip #(.WIDTH(12)) u_add_magnitude (.a(gx), .b(gy), .sum(mag));

    // Stage 2 register
    reg signed  [WIDTH-1    :0]     mag_s2;
    reg                             v_s2;
    reg         [ADDR_W-1   :0]     addr_s2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mag_s2  <=   'd0;
            v_s2    <=  1'd0;
            addr_s2 <=   'd0;
        end else begin
            mag_s2  <=  mag;
            v_s2    <=  v_s1;
            addr_s2 <=  addr_s1;
        end
    end


    // -------- Stage S3: compare + clip --------
    // Clip the operation results to meet the threshold
    wire                        com_threshold      ;
    wire    [clipped_w - 1:0]   clipped_result   ;
    // Determine if the mag meets the threshold requiremnet. In the completed software poart, threshold = 60.
    com_ip  #(.WIDTH(12)) u_com_threshold   (.a(mag_s2), .b({4'd0, threshold}), .com(com_threshold));
    // Get the last 8 bits
    clip_ip #(.WIDTH(12)) u_clip_8bits      (.a(mag_s2), .clip(clipped_result)                     );

    // Stage 3
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            edge_out    <=   'd0;
            valid_out   <=  1'd0;
            addr_out    <=   'd0;
        end else begin
            edge_out    <=  com_threshold ? clipped_result : 8'd0;
            valid_out   <=  v_s2;
            addr_out    <=  addr_s2;
        end
    end

endmodule
