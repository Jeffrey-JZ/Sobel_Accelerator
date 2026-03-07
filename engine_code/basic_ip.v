`timescale 1ns / 1ps

/* 
Basic ips of the Sobel accelerator engine
Author: Junze Jiang
18/2/2026

origianl image: 00 01 02    Sobel Kernel:   x:  -1 0 1  y:  -1 -2 -1
                10 11 12                        -2 0 2       0  0  0
                20 21 22                        -1 0 1       1  2  1

Gx​=(p02+2p12+p22)−(p00+2p10+p20)
Gy​=(p20+2p21+p22)−(p00+2p01+p02)
mag≈∣Gx​∣+∣Gy​∣

Including Signed Adder, Signed Subtractor, Shifter, Absolute value, Camparator, Output clipping to 8 bits/0~255 (saturation)
Total: 6 ips
*/

// 1. Signed Adder
module signed_add_ip #(
    parameter WIDTH = 12
) (
    input   wire    signed  [WIDTH - 1:0]   a,
    input   wire    signed  [WIDTH - 1:0]   b,
    output  wire    signed  [WIDTH - 1:0]   sum
);
    assign sum = a + b;

endmodule

// 2. Signed Subtractor
module signed_sub_ip #(
    parameter WIDTH = 12
) (
    input   wire    signed  [WIDTH - 1:0]   a,
    input   wire    signed  [WIDTH - 1:0]   b,
    output  wire    signed  [WIDTH - 1:0]   sub
);
    assign sub = a - b;
endmodule

// 3. Left Shifter (×2)
module left_shift_ip #(
    parameter WIDTH = 12
) (
    input   wire    signed  [WIDTH - 1:0]   a,
    output  wire    signed  [WIDTH - 1:0]   shifted
);
    assign shifted = a <<< 1;
endmodule

// 4. Absolute value
module abs_ip #(
    parameter WIDTH = 12
) (
    input   wire    signed  [WIDTH - 1:0]   a,
    output  wire            [WIDTH - 1:0]   abs
);
    assign abs = a[WIDTH - 1] ? (~a + {{(WIDTH - 1){1'b0}}, 1'b1}) : a;
endmodule

// 5. Camparator
module com_ip #(
    parameter WIDTH = 12
) (
    input   wire            [WIDTH - 1:0]   a,
    input   wire            [WIDTH - 1:0]   b,
    output  wire                            com
);
    assign com = (a > b);
endmodule

// 6. Output clipping to 8 bits
module clip_ip #(
    parameter WIDTH = 12
) ( 
    input   wire            [WIDTH - 1:0]   a,
    output  wire            [7        :0]   clip
);
    wire com_255;

    com_ip #(.WIDTH(12)) clip_com (
        .a(a),
        .b(12'd255),
        .com(com_255)
    );

    assign clip = com_255 ? 8'd255 : a[7:0];
endmodule