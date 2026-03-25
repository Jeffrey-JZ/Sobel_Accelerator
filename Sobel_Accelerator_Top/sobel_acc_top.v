`timescale 1ns / 1ps

// Sobel Accelerator Top-Level
// Author: Junze Jiang
// 17/3/2026

// This is the full system top integrating two major subsystems:
//   1. dma_csr_top   — Wishbone CSR + DMA engine  (CPU control + AXI data mover)
//   2. enigne_pfb_top — Sobel compute engine + Private Frame Buffer

// Internal connections (automatically wired):
//   - DMA ↔ PFB read/write data ports
//   - DMA engine_start → Engine start
//   - Engine done → DMA engine_done
//
// External ports exposed:
//   - Wishbone slave   (CPU / interconnect side)
//   - AXI4 master      (external memory side)
//   - Status / debug   (busy, done, wr_conflict, eng_wr_ready, eng_rd_ready)

// Typical operation flow (software perspective):
//   1) CPU writes SRC_BASE_ADDR, DST_BASE_ADDR, PIXEL_COUNT, THRESHOLD via Wishbone
//   2) CPU writes CTRL.start=1 via Wishbone
//   3) DMA loads image from AXI → PFB
//   4) DMA kicks engine_start → Engine processes Sobel in-place
//   5) Engine asserts done → DMA stores result PFB → AXI
//   6) DMA asserts done, STATUS register readable by CPU

module sobel_acc_top #(
    // Wishbone
    parameter WB_ADDR_W     = 8,
    parameter WB_DATA_W     = 32,
    // AXI
    parameter AXI_ADDR_W    = 32,
    parameter AXI_DATA_W    = 32,
    // PFB / Engine
    parameter PFB_ADDR_W    = 14,
    parameter PFB_DATA_W    = 8,
    parameter PFB_BANKS     = 4,
    parameter IMG_W         = 128,
    parameter IMG_H         = 128,
    parameter MAG_W         = 12,
    parameter PIXELS_MAX_W  = 16
) (
    input  wire                         clk,
    input  wire                         rst_n,

    // Wishbone Slave (CPU side)
    input  wire [WB_ADDR_W-1:0]         wb_adr_i,
    input  wire [WB_DATA_W-1:0]         wb_dat_i,
    output wire [WB_DATA_W-1:0]         wb_dat_o,
    input  wire [WB_DATA_W/8-1:0]       wb_sel_i,
    input  wire                         wb_we_i,
    input  wire                         wb_stb_i,
    input  wire                         wb_cyc_i,
    output wire                         wb_ack_o,

    // AXI4 Master (Memory side)
    // AR channel
    output wire [AXI_ADDR_W-1:0]        m_axi_araddr,
    output wire [7:0]                   m_axi_arlen,
    output wire [2:0]                   m_axi_arsize,
    output wire [1:0]                   m_axi_arburst,
    output wire                         m_axi_arvalid,
    input  wire                         m_axi_arready,
    // R channel
    input  wire [AXI_DATA_W-1:0]        m_axi_rdata,
    input  wire [1:0]                   m_axi_rresp,
    input  wire                         m_axi_rlast,
    input  wire                         m_axi_rvalid,
    output wire                         m_axi_rready,
    // AW channel
    output wire [AXI_ADDR_W-1:0]        m_axi_awaddr,
    output wire [7:0]                   m_axi_awlen,
    output wire [2:0]                   m_axi_awsize,
    output wire [1:0]                   m_axi_awburst,
    output wire                         m_axi_awvalid,
    input  wire                         m_axi_awready,
    // W channel
    output wire [AXI_DATA_W-1:0]        m_axi_wdata,
    output wire [AXI_DATA_W/8-1:0]      m_axi_wstrb,
    output wire                         m_axi_wlast,
    output wire                         m_axi_wvalid,
    input  wire                         m_axi_wready,
    // B channel
    input  wire [1:0]                   m_axi_bresp,
    input  wire                         m_axi_bvalid,
    output wire                         m_axi_bready,

    // Status / Debug
    output wire                         engine_busy,
    output wire                         engine_done,
    output wire                         eng_wr_ready,   // PFB engine write port ready
    output wire                         eng_rd_ready,   // PFB engine read port ready
    output wire                         wr_conflict     // PFB write-port collision flag
);

    //  Internal wires: DMA ↔ PFB data path

    // DMA → PFB read port (DMA reads processed image out of PFB)
    wire                        pfb_dma_rd_req;
    wire [PFB_ADDR_W-1:0]       pfb_dma_rd_addr;
    wire [PFB_DATA_W-1:0]       pfb_dma_rd_data;
    wire                        pfb_dma_rd_valid;
    wire                        pfb_dma_rd_ready;

    // DMA → PFB write port (DMA writes raw image into PFB)
    wire                        pfb_dma_wr_req;
    wire [PFB_ADDR_W-1:0]       pfb_dma_wr_addr;
    wire [PFB_DATA_W-1:0]       pfb_dma_wr_data;
    wire                        pfb_dma_wr_ready;

    //  Internal wires: DMA ↔ Engine control handshake

    wire                        engine_start_w;     // DMA tells engine to begin
    wire                        engine_done_w;      // Engine tells DMA it finished

    //  Internal wire: threshold from CSR
    wire [7:0]                  threshold_w;

    //  Subsystem 1: DMA + CSR  (Wishbone slave / AXI master)

    dma_csr_top #(
        .WB_ADDR_W          (WB_ADDR_W),
        .WB_DATA_W          (WB_DATA_W),
        .AXI_ADDR_W         (AXI_ADDR_W),
        .AXI_DATA_W         (AXI_DATA_W),
        .PFB_ADDR_W         (PFB_ADDR_W),
        .PIXELS_MAX_W       (PIXELS_MAX_W)
    ) u_dma_csr (
        .clk                (clk),
        .rst_n              (rst_n),

        // Wishbone — directly to top ports
        .wb_adr_i           (wb_adr_i),
        .wb_dat_i           (wb_dat_i),
        .wb_dat_o           (wb_dat_o),
        .wb_sel_i           (wb_sel_i),
        .wb_we_i            (wb_we_i),
        .wb_stb_i           (wb_stb_i),
        .wb_cyc_i           (wb_cyc_i),
        .wb_ack_o           (wb_ack_o),

        // Engine handshake — internal
        .engine_start       (engine_start_w),
        .engine_done        (engine_done_w),

        // PFB DMA read — internal
        .pfb_dma_rd_req     (pfb_dma_rd_req),
        .pfb_dma_rd_addr    (pfb_dma_rd_addr),
        .pfb_dma_rd_data    (pfb_dma_rd_data),
        .pfb_dma_rd_valid   (pfb_dma_rd_valid),
        .pfb_dma_rd_ready   (pfb_dma_rd_ready),

        // PFB DMA write — internal
        .pfb_dma_wr_req     (pfb_dma_wr_req),
        .pfb_dma_wr_addr    (pfb_dma_wr_addr),
        .pfb_dma_wr_data    (pfb_dma_wr_data),
        .pfb_dma_wr_ready   (pfb_dma_wr_ready),

        // AXI — directly to top ports
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
        .m_axi_bready       (m_axi_bready),

        .threshold          (threshold_w)
    );

    //  Subsystem 2: Sobel Engine + Private Frame Buffer

    enigne_pfb_top #(
        .DATA_W         (PFB_DATA_W),
        .ADDR_W         (PFB_ADDR_W),
        .BANKS          (PFB_BANKS),
        .IMG_W          (IMG_W),
        .IMG_H          (IMG_H),
        .MAG_W          (MAG_W)
    ) u_engine_pfb (
        .clk            (clk),
        .rst_n          (rst_n),

        // Control — engine_start from DMA
        .start          (engine_start_w),
        .threshold      (threshold_w),
        .busy           (engine_busy),
        .done           (engine_done_w),

        // DMA read port — internal (DMA reads processed pixels)
        .dma_rd_req     (pfb_dma_rd_req),
        .dma_rd_addr    (pfb_dma_rd_addr),
        .dma_rd_data    (pfb_dma_rd_data),
        .dma_rd_valid   (pfb_dma_rd_valid),
        .dma_rd_ready   (pfb_dma_rd_ready),

        // DMA write port — internal (DMA writes raw pixels)
        .dma_wr_req     (pfb_dma_wr_req),
        .dma_wr_addr    (pfb_dma_wr_addr),
        .dma_wr_data    (pfb_dma_wr_data),
        .dma_wr_ready   (pfb_dma_wr_ready),

        // Debug / status
        .eng_wr_ready   (eng_wr_ready),
        .eng_rd_ready   (eng_rd_ready),
        .wr_conflict    (wr_conflict)
    );

    // Engine done fed back to DMA, also exposed at top level
    assign engine_done = engine_done_w;

endmodule
