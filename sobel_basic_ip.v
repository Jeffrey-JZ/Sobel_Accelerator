`timescale 1ns / 1ps

// Basic arithmetic
// logic primitive IP blocks for Sobel Engine composition.


// 1. Signed Adder
module ip_add_signed #(
    parameter integer W = 12
)(
    input  wire signed [W-1:0] a,
    input  wire signed [W-1:0] b,
    output wire signed [W-1:0] y
);
    assign y = a + b;
endmodule

// 2. Signed Subtractor
module ip_sub_signed #(
    parameter integer W = 12
)(
    input  wire signed [W-1:0] a,
    input  wire signed [W-1:0] b,
    output wire signed [W-1:0] y
);
    assign y = a - b;
endmodule

// 3. Left shifter (×2)
module ip_shl1_signed #(
    parameter integer W = 12
)(
    input  wire signed [W-1:0] a,
    output wire signed [W-1:0] y
);
    assign y = a <<< 1;
endmodule

// 4. Absolute value
module ip_abs_signed #(
    parameter integer W = 12
)(
    input  wire signed [W-1:0] a,
    output wire        [W-1:0] y
);
    assign y = a[W-1] ? (~a + {{(W-1){1'b0}},1'b1}) : a;
endmodule

// 5. Unsigned comparator
module ip_gt_unsigned #(
    parameter integer W = 12
)(
    input  wire [W-1:0] a,
    input  wire [W-1:0] b,
    output wire         gt
);
    assign gt = (a > b);
endmodule

// 6. 12-bit -> 8-bit Saturator
module ip_saturate_u12_to_u8 (
    input  wire [11:0] in,
    output wire [7:0]  out
);
    wire gt_255;
    ip_gt_unsigned #(.W(12)) u_cmp_sat (
        .a(in),
        .b(12'd255),
        .gt(gt_255)
    );
    assign out = gt_255 ? 8'd255 : in[7:0];
endmodule
