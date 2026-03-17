`timescale 1ns / 1ps

// Wishbone CSR block for Sobel DMA control/state registers
// Register map (word offset):
// 0x00 CTRL   [0]=start (W1P)
// 0x04 SRC_BASE_ADDR
// 0x08 DST_BASE_ADDR
// 0x0C PIXEL_COUNT
// 0x10 STATUS [0]=busy [1]=done [2]=error
module csr_wb #(
    parameter   WB_ADDR_W       = 8,
    parameter   WB_DATA_W       = 32,
    parameter   AXI_ADDR_W      = 32,
    parameter   PIXELS_MAX_W    = 16
) (
    input   wire                            wb_clk_i,
    input   wire                            wb_rst_i,
    input   wire [WB_ADDR_W       - 1:0]    wb_adr_i,
    input   wire [WB_DATA_W       - 1:0]    wb_dat_i,
    output  reg  [WB_DATA_W       - 1:0]    wb_dat_o,
    input   wire [(WB_DATA_W / 8) - 1:0]    wb_sel_i,
    input   wire                            wb_we_i,
    input   wire                            wb_stb_i,
    input   wire                            wb_cyc_i,
    output  reg                             wb_ack_o,

    // to DMA core
    output  reg                             dma_start_pulse,
    output  reg  [AXI_ADDR_W   - 1:0]       dma_src_base_addr,
    output  reg  [AXI_ADDR_W   - 1:0]       dma_dst_base_addr,
    output  reg  [PIXELS_MAX_W - 1:0]       dma_pixel_count,
    input   wire                            dma_busy,
    input   wire                            dma_done,
    input   wire                            dma_error
);
    localparam [3:0] REG_CTRL   = 4'h0;
    localparam [3:0] REG_SRC    = 4'h1;
    localparam [3:0] REG_DST    = 4'h2;
    localparam [3:0] REG_COUNT  = 4'h3;
    localparam [3:0] REG_STATUS = 4'h4;

    wire wb_req;
    wire [3:0] reg_idx;
    assign wb_req  = wb_cyc_i & wb_stb_i;
    assign reg_idx = wb_adr_i[5:2];

    reg done_latched;
    reg err_latched;

    always @(posedge wb_clk_i or posedge wb_rst_i) begin
        if (wb_rst_i) begin
            wb_ack_o          <= 1'b0;
            wb_dat_o          <= {WB_DATA_W{1'b0}};
            dma_start_pulse   <= 1'b0;
            dma_src_base_addr <= {AXI_ADDR_W{1'b0}};
            dma_dst_base_addr <= {AXI_ADDR_W{1'b0}};
            dma_pixel_count   <= {PIXELS_MAX_W{1'b0}};
            done_latched      <= 1'b0;
            err_latched       <= 1'b0;
        end else begin
            wb_ack_o        <= 1'b0;
            dma_start_pulse <= 1'b0;

            if (dma_done)
                done_latched <= 1'b1;
            if (dma_error)
                err_latched <= 1'b1;

            if (wb_req && !wb_ack_o) begin
                wb_ack_o <= 1'b1;

                if (wb_we_i) begin
                    case (reg_idx)
                        REG_CTRL: begin
                            if (wb_sel_i[0] && wb_dat_i[0])
                                dma_start_pulse <= 1'b1;
                            // clear latched status by writing 1
                            if (wb_sel_i[0] && wb_dat_i[1])
                                done_latched <= 1'b0;
                            if (wb_sel_i[0] && wb_dat_i[2])
                                err_latched <= 1'b0;
                        end
                        REG_SRC: begin
                            dma_src_base_addr <= wb_dat_i[AXI_ADDR_W-1:0];
                        end
                        REG_DST: begin
                            dma_dst_base_addr <= wb_dat_i[AXI_ADDR_W-1:0];
                        end
                        REG_COUNT: begin
                            dma_pixel_count <= wb_dat_i[PIXELS_MAX_W-1:0];
                        end
                        default: ;
                    endcase
                end else begin
                    case (reg_idx)
                        REG_CTRL: begin
                            wb_dat_o <= {WB_DATA_W{1'b0}};
                        end
                        REG_SRC: begin
                            wb_dat_o <= dma_src_base_addr;
                        end
                        REG_DST: begin
                            wb_dat_o <= dma_dst_base_addr;
                        end
                        REG_COUNT: begin
                            wb_dat_o <= {{(WB_DATA_W-PIXELS_MAX_W){1'b0}}, dma_pixel_count};
                        end
                        REG_STATUS: begin
                            wb_dat_o <= {29'd0, (dma_error | err_latched), (dma_done | done_latched), dma_busy};
                        end
                        default: begin
                            wb_dat_o <= {WB_DATA_W{1'b0}};
                        end
                    endcase
                end
            end
        end
    end
endmodule
