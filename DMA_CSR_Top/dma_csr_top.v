`timescale 1ns / 1ps

// DMA subsystem = Wishbone CSR + DMA core
// CPU writes CSR via Wishbone, CSR drives sobel_dma_top.
module dma_csr_top #(
    parameter WB_ADDR_W     = 8,
    parameter WB_DATA_W     = 32,
    parameter AXI_ADDR_W    = 32,
    parameter AXI_DATA_W    = 32,
    parameter PFB_ADDR_W    = 14,
    parameter PIXELS_MAX_W  = 16
) (
    input  wire                         clk,
    input  wire                         rst_n,

    // Wishbone slave interface (from CPU/interconnect)
    input  wire     [WB_ADDR_W     - 1:0]   wb_adr_i,
    input  wire     [WB_DATA_W     - 1:0]   wb_dat_i,
    output wire     [WB_DATA_W     - 1:0]   wb_dat_o,
    input  wire     [(WB_DATA_W/8) - 1:0]   wb_sel_i,
    input  wire                             wb_we_i,
    input  wire                             wb_stb_i,
    input  wire                             wb_cyc_i,
    output wire                             wb_ack_o,

    // to Engine
    output wire                             engine_start,
    input  wire                             engine_done,

    // to PFB DMA-side interface
    output wire                             pfb_dma_rd_req,
    output wire     [PFB_ADDR_W - 1:0]      pfb_dma_rd_addr,
    input  wire     [7:0]                   pfb_dma_rd_data,
    input  wire                             pfb_dma_rd_valid,
    input  wire                             pfb_dma_rd_ready,

    output wire                             pfb_dma_wr_req,
    output wire     [PFB_ADDR_W - 1:0]      pfb_dma_wr_addr,
    output wire     [7:0]                   pfb_dma_wr_data,
    input  wire                             pfb_dma_wr_ready,

    // AXI4 master interface
    output wire     [AXI_ADDR_W - 1:0]      m_axi_araddr,
    output wire     [7:0]                   m_axi_arlen,
    output wire     [2:0]                   m_axi_arsize,
    output wire     [1:0]                   m_axi_arburst,
    output wire                             m_axi_arvalid,
    input  wire                             m_axi_arready,

    input  wire     [AXI_DATA_W - 1:0]      m_axi_rdata,
    input  wire     [1:0]                   m_axi_rresp,
    input  wire                             m_axi_rlast,
    input  wire                             m_axi_rvalid,
    output wire                             m_axi_rready,

    output wire     [AXI_ADDR_W - 1:0]      m_axi_awaddr,
    output wire     [7:0]                   m_axi_awlen,
    output wire     [2:0]                   m_axi_awsize,
    output wire     [1:0]                   m_axi_awburst,
    output wire                             m_axi_awvalid,
    input  wire                             m_axi_awready,

    output wire     [AXI_DATA_W     - 1:0]  m_axi_wdata,
    output wire     [(AXI_DATA_W/8) - 1:0]  m_axi_wstrb,
    output wire                             m_axi_wlast,
    output wire                             m_axi_wvalid,
    input  wire                             m_axi_wready,

    input  wire     [1:0]                   m_axi_bresp,
    input  wire                             m_axi_bvalid,
    output wire                             m_axi_bready,

    // Engine configuration (from CSR to engine)
    output wire     [7:0]                   threshold
);
    wire                            dma_start;
    wire    [AXI_ADDR_W - 1:0]      dma_src_addr;
    wire    [AXI_ADDR_W - 1:0]      dma_dst_addr;
    wire    [PIXELS_MAX_W - 1:0]    dma_pixel_count;
    wire                            dma_busy;
    wire                            dma_done;
    wire                            dma_error;

    csr_wb #(
        .WB_ADDR_W          (WB_ADDR_W),
        .WB_DATA_W          (WB_DATA_W),
        .AXI_ADDR_W         (AXI_ADDR_W),
        .PIXELS_MAX_W       (PIXELS_MAX_W)
    ) u_csr (
        .wb_clk_i           (clk),
        .wb_rst_i           (~rst_n),
        .wb_adr_i           (wb_adr_i),
        .wb_dat_i           (wb_dat_i),
        .wb_dat_o           (wb_dat_o),
        .wb_sel_i           (wb_sel_i),
        .wb_we_i            (wb_we_i),
        .wb_stb_i           (wb_stb_i),
        .wb_cyc_i           (wb_cyc_i),
        .wb_ack_o           (wb_ack_o),
        .dma_start_pulse    (dma_start),
        .dma_src_base_addr  (dma_src_addr),
        .dma_dst_base_addr  (dma_dst_addr),
        .dma_pixel_count    (dma_pixel_count),
        .dma_busy           (dma_busy),
        .dma_done           (dma_done),
        .dma_error          (dma_error),
        .threshold          (threshold)
    );

    dma_module #(
        .AXI_ADDR_W         (AXI_ADDR_W),
        .AXI_DATA_W         (AXI_DATA_W),
        .PFB_ADDR_W         (PFB_ADDR_W),
        .PIXELS_MAX_W       (PIXELS_MAX_W)
    ) u_dma_core (
        .clk                (clk),
        .rst_n              (rst_n),
        .start              (dma_start),
        .src_base_addr      (dma_src_addr),
        .dst_base_addr      (dma_dst_addr),
        .pixel_count        (dma_pixel_count),
        .busy               (dma_busy),
        .done               (dma_done),
        .error              (dma_error),
        .engine_start       (engine_start),
        .engine_done        (engine_done),
        .pfb_dma_rd_req     (pfb_dma_rd_req),
        .pfb_dma_rd_addr    (pfb_dma_rd_addr),
        .pfb_dma_rd_data    (pfb_dma_rd_data),
        .pfb_dma_rd_valid   (pfb_dma_rd_valid),
        .pfb_dma_rd_ready   (pfb_dma_rd_ready),
        .pfb_dma_wr_req     (pfb_dma_wr_req),
        .pfb_dma_wr_addr    (pfb_dma_wr_addr),
        .pfb_dma_wr_data    (pfb_dma_wr_data),
        .pfb_dma_wr_ready   (pfb_dma_wr_ready),
        .m_axi_araddr       (m_axi_araddr),
        .m_axi_arlen        (m_axi_arlen),
        .m_axi_arsize       (m_axi_arsize),
        .m_axi_arburst      (m_axi_arburst),
        .m_axi_arvalid      (m_axi_arvalid),
        .m_axi_arready      (m_axi_arready),
        .m_axi_rdata        (m_axi_rdata),
        .m_axi_rresp        (m_axi_rresp),
        .m_axi_rlast        (m_axi_rlast),
        .m_axi_rvalid       (m_axi_rvalid),
        .m_axi_rready       (m_axi_rready),
        .m_axi_awaddr       (m_axi_awaddr),
        .m_axi_awlen        (m_axi_awlen),
        .m_axi_awsize       (m_axi_awsize),
        .m_axi_awburst      (m_axi_awburst),
        .m_axi_awvalid      (m_axi_awvalid),
        .m_axi_awready      (m_axi_awready),
        .m_axi_wdata        (m_axi_wdata),
        .m_axi_wstrb        (m_axi_wstrb),
        .m_axi_wlast        (m_axi_wlast),
        .m_axi_wvalid       (m_axi_wvalid),
        .m_axi_wready       (m_axi_wready),
        .m_axi_bresp        (m_axi_bresp),
        .m_axi_bvalid       (m_axi_bvalid),
        .m_axi_bready       (m_axi_bready)
    );
endmodule
