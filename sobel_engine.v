`timescale 1ns / 1ps

// Sobel Engine for 128x128 gray image (parameterizable)
// - Reads pixels from private frame buffer through rd_req/rd_data_valid handshake
// - Writes edge result back through wr_en interface
// - Boundary pixels are forced to 0

module sobel_kernel_3x3 (
    input  [7:0] p00, input [7:0] p01, input [7:0] p02,
    input  [7:0] p10, input [7:0] p11, input [7:0] p12,
    input  [7:0] p20, input [7:0] p21, input [7:0] p22,
    output [11:0] magnitude
);
    // Extend to signed 12-bit
    wire signed [11:0] s00 = $signed({4'd0, p00});
    wire signed [11:0] s01 = $signed({4'd0, p01});
    wire signed [11:0] s02 = $signed({4'd0, p02});
    wire signed [11:0] s10 = $signed({4'd0, p10});
    wire signed [11:0] s12 = $signed({4'd0, p12});
    wire signed [11:0] s20 = $signed({4'd0, p20});
    wire signed [11:0] s21 = $signed({4'd0, p21});
    wire signed [11:0] s22 = $signed({4'd0, p22});

    // x2 terms by shift-left primitive
    wire signed [11:0] s10_x2, s12_x2, s01_x2, s21_x2;
    ip_shl1_signed #(.W(12)) u_shl_s10 (.a(s10), .y(s10_x2));
    ip_shl1_signed #(.W(12)) u_shl_s12 (.a(s12), .y(s12_x2));
    ip_shl1_signed #(.W(12)) u_shl_s01 (.a(s01), .y(s01_x2));
    ip_shl1_signed #(.W(12)) u_shl_s21 (.a(s21), .y(s21_x2));

    // Gx = (p00 - p02) + (2*p10 - 2*p12) + (p20 - p22)
    wire signed [11:0] gx_t0, gx_t1, gx_t2, gx_t3;
    ip_sub_signed #(.W(12)) u_gx_sub0 (.a(s00),   .b(s02),   .y(gx_t0));
    ip_sub_signed #(.W(12)) u_gx_sub1 (.a(s10_x2),.b(s12_x2),.y(gx_t1));
    ip_add_signed #(.W(12)) u_gx_add0 (.a(gx_t0), .b(gx_t1), .y(gx_t2));
    ip_sub_signed #(.W(12)) u_gx_sub2 (.a(s20),   .b(s22),   .y(gx_t3));
    wire signed [11:0] gx;
    ip_add_signed #(.W(12)) u_gx_add1 (.a(gx_t2), .b(gx_t3), .y(gx));

    // Gy = (p00 + 2*p01 + p02) - (p20 + 2*p21 + p22)
    wire signed [11:0] gy_top0, gy_top1, gy_top;
    wire signed [11:0] gy_bot0, gy_bot1, gy_bot;
    ip_add_signed #(.W(12)) u_gy_add0 (.a(s00),   .b(s01_x2), .y(gy_top0));
    ip_add_signed #(.W(12)) u_gy_add1 (.a(gy_top0),.b(s02),    .y(gy_top));
    ip_add_signed #(.W(12)) u_gy_add2 (.a(s20),   .b(s21_x2), .y(gy_bot0));
    ip_add_signed #(.W(12)) u_gy_add3 (.a(gy_bot0),.b(s22),    .y(gy_bot));
    wire signed [11:0] gy;
    ip_sub_signed #(.W(12)) u_gy_sub0 (.a(gy_top), .b(gy_bot), .y(gy));

    // magnitude = |gx| + |gy|
    wire [11:0] abs_gx, abs_gy;
    ip_abs_signed #(.W(12)) u_abs_gx (.a(gx), .y(abs_gx));
    ip_abs_signed #(.W(12)) u_abs_gy (.a(gy), .y(abs_gy));

    wire signed [11:0] mag_signed;
    ip_add_signed #(.W(12)) u_mag_add (
        .a($signed(abs_gx)),
        .b($signed(abs_gy)),
        .y(mag_signed)
    );

    assign magnitude = mag_signed[11:0];
endmodule

module sobel_threshold_clip (
    input  [11:0] magnitude,
    input  [7:0]  threshold,
    output [7:0]  edge_pixel
);
    wire mag_gt_th;
    wire [7:0] mag_sat;

    ip_gt_unsigned #(.W(12)) u_cmp_th (
        .a(magnitude),
        .b({4'd0, threshold}),
        .gt(mag_gt_th)
    );

    ip_saturate_u12_to_u8 u_sat (
        .in(magnitude),
        .out(mag_sat)
    );

    assign edge_pixel = mag_gt_th ? mag_sat : 8'd0;
endmodule

module sobel_engine #(
    parameter integer IMG_W  = 128,
    parameter integer IMG_H  = 128,
    parameter integer ADDR_W = 14
)(
    input  wire                 clk,
    input  wire                 rst_n,

    input  wire                 start,
    input  wire [7:0]           threshold,

    // Read port to private frame buffer
    output reg                  rd_req,
    output reg  [ADDR_W-1:0]    rd_addr,
    input  wire [7:0]           rd_data,
    input  wire                 rd_data_valid,

    // Write port to private frame buffer (edge result area)
    output reg                  wr_en,
    output reg  [ADDR_W-1:0]    wr_addr,
    output reg  [7:0]           wr_data,

    output reg                  busy,
    output reg                  done
);
    localparam [2:0] S_IDLE        = 3'd0;
    localparam [2:0] S_PIXEL_START = 3'd1;
    localparam [2:0] S_REQ         = 3'd2;
    localparam [2:0] S_WAIT        = 3'd3;
    localparam [2:0] S_WRITE       = 3'd4;
    localparam [2:0] S_DONE        = 3'd5;

    reg [2:0] state;

    reg [7:0] x;
    reg [7:0] y;
    reg [3:0] k_idx;
    reg       border_pixel;

    reg [7:0] p00, p01, p02;
    reg [7:0] p10, p11, p12;
    reg [7:0] p20, p21, p22;

    wire [11:0] sobel_mag;
    wire [7:0]  sobel_out;

    reg [ADDR_W-1:0] rd_addr_calc;

    wire last_pixel;
    assign last_pixel = (x == (IMG_W-1)) && (y == (IMG_H-1));

    sobel_kernel_3x3 u_kernel (
        .p00(p00), .p01(p01), .p02(p02),
        .p10(p10), .p11(p11), .p12(p12),
        .p20(p20), .p21(p21), .p22(p22),
        .magnitude(sobel_mag)
    );

    sobel_threshold_clip u_post (
        .magnitude(sobel_mag),
        .threshold(threshold),
        .edge_pixel(sobel_out)
    );

    always @(*) begin
        case (k_idx)
            4'd0: rd_addr_calc = (y-1) * IMG_W + (x-1);
            4'd1: rd_addr_calc = (y-1) * IMG_W + (x  );
            4'd2: rd_addr_calc = (y-1) * IMG_W + (x+1);
            4'd3: rd_addr_calc = (y  ) * IMG_W + (x-1);
            4'd4: rd_addr_calc = (y  ) * IMG_W + (x  );
            4'd5: rd_addr_calc = (y  ) * IMG_W + (x+1);
            4'd6: rd_addr_calc = (y+1) * IMG_W + (x-1);
            4'd7: rd_addr_calc = (y+1) * IMG_W + (x  );
            4'd8: rd_addr_calc = (y+1) * IMG_W + (x+1);
            default: rd_addr_calc = {ADDR_W{1'b0}};
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            rd_req <= 1'b0;
            rd_addr <= {ADDR_W{1'b0}};
            wr_en <= 1'b0;
            wr_addr <= {ADDR_W{1'b0}};
            wr_data <= 8'd0;
            x <= 8'd0;
            y <= 8'd0;
            k_idx <= 4'd0;
            border_pixel <= 1'b0;
            p00 <= 8'd0; p01 <= 8'd0; p02 <= 8'd0;
            p10 <= 8'd0; p11 <= 8'd0; p12 <= 8'd0;
            p20 <= 8'd0; p21 <= 8'd0; p22 <= 8'd0;
        end else begin
            rd_req <= 1'b0;
            wr_en <= 1'b0;
            done <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;
                        x <= 8'd0;
                        y <= 8'd0;
                        state <= S_PIXEL_START;
                    end
                end

                S_PIXEL_START: begin
                    border_pixel <= (x == 0) || (y == 0) || (x == (IMG_W-1)) || (y == (IMG_H-1));
                    if ((x == 0) || (y == 0) || (x == (IMG_W-1)) || (y == (IMG_H-1))) begin
                        state <= S_WRITE;
                    end else begin
                        k_idx <= 4'd0;
                        state <= S_REQ;
                    end
                end

                S_REQ: begin
                    rd_req <= 1'b1;
                    rd_addr <= rd_addr_calc;
                    state <= S_WAIT;
                end

                S_WAIT: begin
                    if (rd_data_valid) begin
                        case (k_idx)
                            4'd0: p00 <= rd_data;
                            4'd1: p01 <= rd_data;
                            4'd2: p02 <= rd_data;
                            4'd3: p10 <= rd_data;
                            4'd4: p11 <= rd_data;
                            4'd5: p12 <= rd_data;
                            4'd6: p20 <= rd_data;
                            4'd7: p21 <= rd_data;
                            4'd8: p22 <= rd_data;
                            default: ;
                        endcase

                        if (k_idx == 4'd8) begin
                            state <= S_WRITE;
                        end else begin
                            k_idx <= k_idx + 4'd1;
                            state <= S_REQ;
                        end
                    end
                end

                S_WRITE: begin
                    wr_en <= 1'b1;
                    wr_addr <= y * IMG_W + x;
                    wr_data <= border_pixel ? 8'd0 : sobel_out;

                    if (last_pixel) begin
                        state <= S_DONE;
                    end else begin
                        if (x == (IMG_W-1)) begin
                            x <= 8'd0;
                            y <= y + 8'd1;
                        end else begin
                            x <= x + 8'd1;
                        end
                        state <= S_PIXEL_START;
                    end
                end

                S_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    if (!start) begin
                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
