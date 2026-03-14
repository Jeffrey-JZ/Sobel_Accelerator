`timescale 1ns / 1ps

// DMA Top Module
// Author: Junze Jiang
// 14/3/2026
//
// DMA 顶层模块：集成 load engine 和 store engine，并提供简单的控制器 FSM
//
// 外部接口:
//   1) 控制接口 (连接 CSR 模块 / Wishbone 从机)
//      - load_start / store_start: 分别触发 LOAD / STORE 操作
//      - src_addr: LOAD 时的 Main Memory 源地址
//      - dst_addr: STORE 时的 Main Memory 目标地址
//      - load_done / store_done: 操作完成脉冲
//      - dma_busy: DMA 正在工作
//
//   2) AXI4 Master 接口 (连接 AXI4 Interconnect → Main Memory)
//      - AR/R 通道: load engine 使用
//      - AW/W/B 通道: store engine 使用
//      - 由于 LOAD 和 STORE 不会同时进行，AR/R 和 AW/W/B 自然分离，无需仲裁
//
//   3) PFB DMA 接口 (连接 engine_pfb_top 的 dma_rd_* / dma_wr_* 端口)
//      - dma_wr_*: load engine 使用（往 PFB 写入输入图像）
//      - dma_rd_*: store engine 使用（从 PFB 读出处理结果）
//
// 整体工作流 (由 CSR 模块编排):
//   1) CSR 发起 load_start → DMA LOAD: Main Memory → PFB
//   2) CSR 发起 engine start → Engine 处理 Sobel 运算
//   3) CSR 发起 store_start → DMA STORE: PFB → Main Memory
//
// 模块层次:
//   dma_top
//   ├── dma_load_engine    (AXI4 burst read + pixel unpack + PFB write)
//   └── dma_store_engine   (PFB read + pixel pack + AXI4 burst write)

module dma_top #(
    parameter AXI_ADDR_W    = 32,
    parameter AXI_DATA_W    = 32,
    parameter AXI_ID_W      = 4,
    parameter PIXEL_W       = 8,
    parameter PFB_ADDR_W    = 14,
    parameter BURST_LEN     = 256,
    parameter TOTAL_PIXELS  = 16384
) (
    input  wire                     clk,
    input  wire                     rst_n,

    // ==== 控制接口 (来自 CSR) ====
    input  wire                     load_start,         // 脉冲: 开始 LOAD (Main Memory → PFB)
    input  wire                     store_start,        // 脉冲: 开始 STORE (PFB → Main Memory)
    input  wire [AXI_ADDR_W-1:0]    src_addr,           // LOAD 的源地址
    input  wire [AXI_ADDR_W-1:0]    dst_addr,           // STORE 的目标地址
    output wire                     load_done,          // LOAD 完成脉冲
    output wire                     store_done,         // STORE 完成脉冲
    output wire                     dma_busy,           // DMA 正忙

    // ==== AXI4 Master - Read Channels (AR + R) ====
    // 由 load engine 驱动
    output wire [AXI_ID_W-1:0]     m_axi_arid,
    output wire [AXI_ADDR_W-1:0]   m_axi_araddr,
    output wire [7:0]              m_axi_arlen,
    output wire [2:0]              m_axi_arsize,
    output wire [1:0]              m_axi_arburst,
    output wire                    m_axi_arvalid,
    input  wire                    m_axi_arready,

    input  wire [AXI_ID_W-1:0]     m_axi_rid,
    input  wire [AXI_DATA_W-1:0]   m_axi_rdata,
    input  wire [1:0]              m_axi_rresp,
    input  wire                    m_axi_rlast,
    input  wire                    m_axi_rvalid,
    output wire                    m_axi_rready,

    // ==== AXI4 Master - Write Channels (AW + W + B) ====
    // 由 store engine 驱动
    output wire [AXI_ID_W-1:0]     m_axi_awid,
    output wire [AXI_ADDR_W-1:0]   m_axi_awaddr,
    output wire [7:0]              m_axi_awlen,
    output wire [2:0]              m_axi_awsize,
    output wire [1:0]              m_axi_awburst,
    output wire                    m_axi_awvalid,
    input  wire                    m_axi_awready,

    output wire [AXI_DATA_W-1:0]   m_axi_wdata,
    output wire [AXI_DATA_W/8-1:0] m_axi_wstrb,
    output wire                    m_axi_wlast,
    output wire                    m_axi_wvalid,
    input  wire                    m_axi_wready,

    input  wire [AXI_ID_W-1:0]     m_axi_bid,
    input  wire [1:0]              m_axi_bresp,
    input  wire                    m_axi_bvalid,
    output wire                    m_axi_bready,

    // ==== PFB DMA Write Port (load engine → PFB) ====
    output wire                     dma_wr_req,
    output wire [PFB_ADDR_W-1:0]   dma_wr_addr,
    output wire [PIXEL_W-1:0]      dma_wr_data,
    input  wire                     dma_wr_ready,

    // ==== PFB DMA Read Port (PFB → store engine) ====
    output wire                     dma_rd_req,
    output wire [PFB_ADDR_W-1:0]   dma_rd_addr,
    input  wire [PIXEL_W-1:0]      dma_rd_data,
    input  wire                     dma_rd_valid,
    input  wire                     dma_rd_ready
);

    // 内部信号
    wire load_busy, store_busy;

    // DMA 整体 busy = load_busy | store_busy
    assign dma_busy = load_busy | store_busy;

    // ================================================================
    // Load Engine: Main Memory → PFB
    //   AXI4 AR/R → 32-bit word 拆包 → 逐像素写 PFB
    // ================================================================
    dma_load_engine #(
        .AXI_ADDR_W     (AXI_ADDR_W),
        .AXI_DATA_W     (AXI_DATA_W),
        .AXI_ID_W       (AXI_ID_W),
        .PIXEL_W        (PIXEL_W),
        .PFB_ADDR_W     (PFB_ADDR_W),
        .BURST_LEN      (BURST_LEN),
        .TOTAL_PIXELS   (TOTAL_PIXELS)
    ) u_load_engine (
        .clk            (clk),
        .rst_n          (rst_n),

        // 控制
        .start          (load_start),
        .src_addr       (src_addr),
        .done           (load_done),
        .busy           (load_busy),

        // AXI4 Read Channel
        .m_axi_arid     (m_axi_arid),
        .m_axi_araddr   (m_axi_araddr),
        .m_axi_arlen    (m_axi_arlen),
        .m_axi_arsize   (m_axi_arsize),
        .m_axi_arburst  (m_axi_arburst),
        .m_axi_arvalid  (m_axi_arvalid),
        .m_axi_arready  (m_axi_arready),

        .m_axi_rid      (m_axi_rid),
        .m_axi_rdata    (m_axi_rdata),
        .m_axi_rresp    (m_axi_rresp),
        .m_axi_rlast    (m_axi_rlast),
        .m_axi_rvalid   (m_axi_rvalid),
        .m_axi_rready   (m_axi_rready),

        // PFB Write
        .dma_wr_req     (dma_wr_req),
        .dma_wr_addr    (dma_wr_addr),
        .dma_wr_data    (dma_wr_data),
        .dma_wr_ready   (dma_wr_ready)
    );

    // ================================================================
    // Store Engine: PFB → Main Memory
    //   逐像素读 PFB → 4 像素打包 → AXI4 AW/W/B burst write
    // ================================================================
    dma_store_engine #(
        .AXI_ADDR_W     (AXI_ADDR_W),
        .AXI_DATA_W     (AXI_DATA_W),
        .AXI_ID_W       (AXI_ID_W),
        .PIXEL_W        (PIXEL_W),
        .PFB_ADDR_W     (PFB_ADDR_W),
        .BURST_LEN      (BURST_LEN),
        .TOTAL_PIXELS   (TOTAL_PIXELS)
    ) u_store_engine (
        .clk            (clk),
        .rst_n          (rst_n),

        // 控制
        .start          (store_start),
        .dst_addr       (dst_addr),
        .done           (store_done),
        .busy           (store_busy),

        // AXI4 Write Channel
        .m_axi_awid     (m_axi_awid),
        .m_axi_awaddr   (m_axi_awaddr),
        .m_axi_awlen    (m_axi_awlen),
        .m_axi_awsize   (m_axi_awsize),
        .m_axi_awburst  (m_axi_awburst),
        .m_axi_awvalid  (m_axi_awvalid),
        .m_axi_awready  (m_axi_awready),

        .m_axi_wdata    (m_axi_wdata),
        .m_axi_wstrb    (m_axi_wstrb),
        .m_axi_wlast    (m_axi_wlast),
        .m_axi_wvalid   (m_axi_wvalid),
        .m_axi_wready   (m_axi_wready),

        .m_axi_bid      (m_axi_bid),
        .m_axi_bresp    (m_axi_bresp),
        .m_axi_bvalid   (m_axi_bvalid),
        .m_axi_bready   (m_axi_bready),

        // PFB Read
        .dma_rd_req     (dma_rd_req),
        .dma_rd_addr    (dma_rd_addr),
        .dma_rd_data    (dma_rd_data),
        .dma_rd_valid   (dma_rd_valid),
        .dma_rd_ready   (dma_rd_ready)
    );

endmodule
