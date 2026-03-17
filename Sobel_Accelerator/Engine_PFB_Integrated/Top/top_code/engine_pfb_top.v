`timescale 1ns / 1ps

// Engine + Private Frame Buffer Top file
// Author: Junze Jiang
// 13/3/2026

// Integration module that wires the sobel_engine's read/write ports
// directly into the private_frame_buffer's engine-side ports.
// The DMA ports and control signals are exposed to the outside world
// so that an external DMA controller can load/read images and
// a host can trigger processing.

// Typical usage flow:
//   1) DMA writes input image into PFB through dma_wr_* ports
//   2) Host asserts `start` for one cycle
//   3) Engine reads input pixels, computes Sobel, writes results back (in-place)
//   4) Engine raises `done`
//   5) DMA reads processed image out through dma_rd_* ports

module enigne_pfb_top #(
    parameter DATA_W    = 8,
    parameter ADDR_W    = 14,
    parameter BANKS     = 4,
    parameter IMG_W     = 128,
    parameter IMG_H     = 128,
    parameter MAG_W     = 12
) (
    input  wire                 clk,
    input  wire                 rst_n,

    // Control
    input  wire                 start,
    input  wire [7:0]           threshold,
    output wire                 busy,
    output wire                 done,

    // DMA read port (PFB -> external)
    input  wire                 dma_rd_req,
    input  wire [ADDR_W-1:0]    dma_rd_addr,
    output wire [DATA_W-1:0]    dma_rd_data,
    output wire                 dma_rd_valid,
    output wire                 dma_rd_ready,

    // DMA write port (external -> PFB)
    input  wire                 dma_wr_req,
    input  wire [ADDR_W-1:0]    dma_wr_addr,
    input  wire [DATA_W-1:0]    dma_wr_data,
    output wire                 dma_wr_ready,

    // Status
    output wire                 eng_wr_ready,
    output wire                 eng_rd_ready,
    output wire                 wr_conflict
);
    // Internal wires: Engine <-> PFB
    wire                eng_rd_req;
    wire [ADDR_W-1:0]   eng_rd_addr;
    wire [DATA_W-1:0]   eng_rd_data;
    wire                eng_rd_valid;
    wire                eng_rd_ready_w;

    wire                eng_wr_en;
    wire [ADDR_W-1:0]   eng_wr_addr;
    wire [DATA_W-1:0]   eng_wr_data;
    wire                eng_wr_ready_w;

    assign eng_rd_ready = eng_rd_ready_w;   // From pfb output
    assign eng_wr_ready = eng_wr_ready_w;   // From pfb output

    // Engine
    sobel_engine #(
        .IMG_W          (IMG_W),
        .IMG_H          (IMG_H),
        .ADDR_W         (ADDR_W),
        .MAG_W          (MAG_W),
        .clipped_w      (DATA_W)
    ) u_engine (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start),
        .threshold      (threshold),

        // Engine read port  -> PFB engine read port
        .rd_req         (eng_rd_req),       // Output
        .rd_addr        (eng_rd_addr),      // Output
        .rd_data        (eng_rd_data),      // From pfb output
        .rd_data_valid  (eng_rd_valid),     // From pfb output

        // Engine write port -> PFB engine write port
        .wr_en          (eng_wr_en),        // Output
        .wr_addr        (eng_wr_addr),      // Output
        .wr_data        (eng_wr_data),      // Output

        .busy           (busy),             // Output
        .done           (done)              // Output
    );

    // Private Frame Buffer
    private_frame_buffer #(
        .DATA_W         (DATA_W),
        .ADDR_W         (ADDR_W),
        .BANKS          (BANKS),
        .IMG_W          (IMG_W),
        .IMG_H          (IMG_H)
    ) u_pfb (
        .clk            (clk),
        .rst_n          (rst_n),

        // DMA ports – directly exposed
        .dma_rd_req     (dma_rd_req),
        .dma_rd_addr    (dma_rd_addr),
        .dma_rd_data    (dma_rd_data),      // Output
        .dma_rd_valid   (dma_rd_valid),     // Output
        .dma_rd_ready   (dma_rd_ready),     // Output

        .dma_wr_req     (dma_wr_req),
        .dma_wr_addr    (dma_wr_addr),
        .dma_wr_data    (dma_wr_data),
        .dma_wr_ready   (dma_wr_ready),     // Output

        // Engine ports – connected internally
        .eng_rd_req     (eng_rd_req),       // From engine output
        .eng_rd_addr    (eng_rd_addr),      // From engine output
        .eng_rd_data    (eng_rd_data),      // Output
        .eng_rd_valid   (eng_rd_valid),     // Output
        .eng_rd_ready   (eng_rd_ready_w),   // Output

        .eng_wr_en      (eng_wr_en),        // From engine output
        .eng_wr_addr    (eng_wr_addr),      // From engine output
        .eng_wr_data    (eng_wr_data),      // From engine output
        .eng_wr_ready   (eng_wr_ready_w),   // Output

        .wr_conflict    (wr_conflict)       // Output
    );

endmodule
