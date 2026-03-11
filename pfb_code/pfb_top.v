`timescale 1ns / 1ps

// Sobel accelerator private frame buffer(pfb)
// Author: Junze Jiang
// 4/3/2026
// Both ports can work in parallel when addressing any bank
// If both ports write the same bank+address in the same cycle, dma side wins and eng write is blocked for deterministic behavior

// - 128x128, 8-bit pixels, 4-bank interleaving
// - Single storage (in-place writeback), no split input/output PFB
// - Exposes independent engine read/write ports and DMA read/write ports
// - Internally arbitrates 4 logical requesters onto 2 physical ports per bank

module private_frame_buffer #(
    parameter DATA_W  = 8,
    parameter ADDR_W  = 14,
    parameter BANKS   = 4,
    parameter IMG_W   = 128,
    parameter IMG_H   = 128
) (
    input  wire                 clk,
    input  wire                 rst_n,

    // DMA read port
    input  wire                 dma_rd_req,
    input  wire [ADDR_W-1:0]    dma_rd_addr,
    output reg  [DATA_W-1:0]    dma_rd_data,
    output reg                  dma_rd_valid,
    output reg                  dma_rd_ready,

    // DMA write port
    input  wire                 dma_wr_req,
    input  wire [ADDR_W-1:0]    dma_wr_addr,
    input  wire [DATA_W-1:0]    dma_wr_data,
    output reg                  dma_wr_ready,

    // Engine read port
    input  wire                 eng_rd_req,
    input  wire [ADDR_W-1:0]    eng_rd_addr,
    output reg  [DATA_W-1:0]    eng_rd_data,
    output reg                  eng_rd_valid,
    output reg                  eng_rd_ready,

    // Engine writeback port
    input  wire                 eng_wr_en,
    input  wire [ADDR_W-1:0]    eng_wr_addr,
    input  wire [DATA_W-1:0]    eng_wr_data,
    output reg                  eng_wr_ready,

    // Any write requester blocked in this cycle
    output reg                  wr_conflict
);
    localparam integer PIXELS       = IMG_W * IMG_H;
    localparam integer BANK_BITS    = $clog2(BANKS);
    localparam integer LOCAL_ADDR_W = ADDR_W - BANK_BITS;

    localparam [1:0] SRC_NONE   = 2'd0;
    localparam [1:0] SRC_DMA_RD = 2'd1;
    localparam [1:0] SRC_ENG_RD = 2'd2;

    initial begin
        if (PIXELS > (1 << ADDR_W)) begin
            $error("private_frame_buffer: IMG_W*IMG_H exceeds ADDR_W address space.");
        end
    end

    wire [BANK_BITS    - 1  :0] dma_rd_bank_sel, dma_wr_bank_sel, eng_rd_bank_sel, eng_wr_bank_sel;
    wire [LOCAL_ADDR_W - 1  :0] dma_rd_local_addr, dma_wr_local_addr, eng_rd_local_addr, eng_wr_local_addr;

    pfb_addr_map #(
        .ADDR_W         (ADDR_W), 
        .BANK_BITS      (BANK_BITS)
    ) u_dma_rd_map (
        .global_addr    (dma_rd_addr), 
        .bank_sel       (dma_rd_bank_sel), 
        .local_addr     (dma_rd_local_addr)
    );

    pfb_addr_map #(
        .ADDR_W         (ADDR_W), 
        .BANK_BITS      (BANK_BITS)
    ) u_dma_wr_map (
        .global_addr    (dma_wr_addr), 
        .bank_sel       (dma_wr_bank_sel), 
        .local_addr     (dma_wr_local_addr)
    );

    pfb_addr_map #(
        .ADDR_W         (ADDR_W), 
        .BANK_BITS      (BANK_BITS)
    ) u_eng_rd_map (
        .global_addr    (eng_rd_addr), 
        .bank_sel       (eng_rd_bank_sel), 
        .local_addr     (eng_rd_local_addr)
    );

    pfb_addr_map #(
        .ADDR_W         (ADDR_W), 
        .BANK_BITS      (BANK_BITS)
    ) u_eng_wr_map (
        .global_addr    (eng_wr_addr), 
        .bank_sel       (eng_wr_bank_sel), 
        .local_addr     (eng_wr_local_addr)
    );

    reg  [BANKS         - 1:0]  a_en_vec, a_we_vec, b_en_vec, b_we_vec;
    reg  [LOCAL_ADDR_W  - 1:0]  a_addr_vec [0:BANKS - 1];
    reg  [LOCAL_ADDR_W  - 1:0]  b_addr_vec [0:BANKS - 1];
    reg  [DATA_W        - 1:0]  a_wdata_vec[0:BANKS - 1];
    reg  [DATA_W        - 1:0]  b_wdata_vec[0:BANKS - 1];
    reg  [1:0]                  a_rd_src_n [0:BANKS - 1];
    reg  [1:0]                  b_rd_src_n [0:BANKS - 1];

    reg  [1:0]                  a_rd_src_q [0:BANKS - 1];
    reg  [1:0]                  b_rd_src_q [0:BANKS - 1];

    wire [BANKS*DATA_W - 1:0]   a_rdata_vec, b_rdata_vec;
    wire [BANKS        - 1:0]   a_rvalid_vec, b_rvalid_vec;

    integer i;
    always @(*) begin
        dma_rd_ready = 1'b0;
        dma_wr_ready = 1'b0;
        eng_rd_ready = 1'b0;
        eng_wr_ready = 1'b0;

        for (i = 0; i < BANKS; i = i + 1) begin
            a_en_vec[i]     = 1'b0;
            a_we_vec[i]     = 1'b0;
            b_en_vec[i]     = 1'b0;
            b_we_vec[i]     = 1'b0;
            a_addr_vec[i]   = {LOCAL_ADDR_W{1'b0}};
            b_addr_vec[i]   = {LOCAL_ADDR_W{1'b0}};
            a_wdata_vec[i]  = {DATA_W{1'b0}};
            b_wdata_vec[i]  = {DATA_W{1'b0}};
            a_rd_src_n[i]   = SRC_NONE;
            b_rd_src_n[i]   = SRC_NONE;

            // Port A priority: engine write > DMA write > DMA read > engine read
            if (eng_wr_en && (eng_wr_bank_sel == i[BANK_BITS-1:0])) begin
                a_en_vec[i]     = 1'b1;
                a_we_vec[i]     = 1'b1;
                a_addr_vec[i]   = eng_wr_local_addr;
                a_wdata_vec[i]  = eng_wr_data;
                eng_wr_ready    = 1'b1;
            end else if (dma_wr_req && (dma_wr_bank_sel == i[BANK_BITS-1:0])) begin
                a_en_vec[i]     = 1'b1;
                a_we_vec[i]     = 1'b1;
                a_addr_vec[i]   = dma_wr_local_addr;
                a_wdata_vec[i]  = dma_wr_data;
                dma_wr_ready    = 1'b1;
            end else if (dma_rd_req && (dma_rd_bank_sel == i[BANK_BITS-1:0])) begin
                a_en_vec[i]     = 1'b1;
                a_we_vec[i]     = 1'b0;
                a_addr_vec[i]   = dma_rd_local_addr;
                a_rd_src_n[i]   = SRC_DMA_RD;
                dma_rd_ready    = 1'b1;
            end else if (eng_rd_req && (eng_rd_bank_sel == i[BANK_BITS-1:0])) begin
                a_en_vec[i]     = 1'b1;
                a_we_vec[i]     = 1'b0;
                a_addr_vec[i]   = eng_rd_local_addr;
                a_rd_src_n[i]   = SRC_ENG_RD;
                eng_rd_ready    = 1'b1;
            end

            // Port B priority: engine read > DMA write > DMA read > engine write
            // Only requesters not already granted on port A can be granted here.
            if (eng_rd_req && !eng_rd_ready && (eng_rd_bank_sel == i[BANK_BITS-1:0])) begin
                b_en_vec[i]     = 1'b1;
                b_we_vec[i]     = 1'b0;
                b_addr_vec[i]   = eng_rd_local_addr;
                b_rd_src_n[i]   = SRC_ENG_RD;
                eng_rd_ready    = 1'b1;
            end else if (dma_wr_req && !dma_wr_ready && (dma_wr_bank_sel == i[BANK_BITS-1:0])) begin
                b_en_vec[i]     = 1'b1;
                b_we_vec[i]     = 1'b1;
                b_addr_vec[i]   = dma_wr_local_addr;
                b_wdata_vec[i]  = dma_wr_data;
                dma_wr_ready    = 1'b1;
            end else if (dma_rd_req && !dma_rd_ready && (dma_rd_bank_sel == i[BANK_BITS-1:0])) begin
                b_en_vec[i]     = 1'b1;
                b_we_vec[i]     = 1'b0;
                b_addr_vec[i]   = dma_rd_local_addr;
                b_rd_src_n[i]   = SRC_DMA_RD;
                dma_rd_ready    = 1'b1;
            end else if (eng_wr_en && !eng_wr_ready && (eng_wr_bank_sel == i[BANK_BITS-1:0])) begin
                b_en_vec[i]     = 1'b1;
                b_we_vec[i]     = 1'b1;
                b_addr_vec[i]   = eng_wr_local_addr;
                b_wdata_vec[i]  = eng_wr_data;
                eng_wr_ready    = 1'b1;
            end
        end
    end

    genvar b;
    generate
        for (b = 0; b < BANKS; b = b + 1) begin : g_banks
            pfb_bank_ram #(
                .DATA_W(DATA_W),
                .ADDR_W(LOCAL_ADDR_W)
            ) u_bank (
                .clk     (clk),
                .rst_n   (rst_n),
                .a_en    (a_en_vec[b]),
                .a_we    (a_we_vec[b]),
                .a_addr  (a_addr_vec[b]),
                .a_wdata (a_wdata_vec[b]),
                .a_rdata (a_rdata_vec[b*DATA_W +: DATA_W]),
                .a_rvalid(a_rvalid_vec[b]),
                .b_en    (b_en_vec[b]),
                .b_we    (b_we_vec[b]),
                .b_addr  (b_addr_vec[b]),
                .b_wdata (b_wdata_vec[b]),
                .b_rdata (b_rdata_vec[b*DATA_W +: DATA_W]),
                .b_rvalid(b_rvalid_vec[b])
            );
        end
    endgenerate

    integer j;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dma_rd_data  <= {DATA_W{1'b0}};
            dma_rd_valid <= 1'b0;
            eng_rd_data  <= {DATA_W{1'b0}};
            eng_rd_valid <= 1'b0;
            wr_conflict  <= 1'b0;
            for (j = 0; j < BANKS; j = j + 1) begin
                a_rd_src_q[j] <= SRC_NONE;
                b_rd_src_q[j] <= SRC_NONE;
            end
        end else begin
            dma_rd_valid <= 1'b0;
            eng_rd_valid <= 1'b0;

            // Track who issued read on each physical port (1-cycle RAM read latency)
            for (j = 0; j < BANKS; j = j + 1) begin
                a_rd_src_q[j] <= (a_en_vec[j] && !a_we_vec[j]) ? a_rd_src_n[j] : SRC_NONE;
                b_rd_src_q[j] <= (b_en_vec[j] && !b_we_vec[j]) ? b_rd_src_n[j] : SRC_NONE;
            end

            // Return read data to each logical requester
            for (j = 0; j < BANKS; j = j + 1) begin
                if (a_rvalid_vec[j]) begin
                    case (a_rd_src_q[j])
                        SRC_DMA_RD: begin
                            dma_rd_data  <= a_rdata_vec[j*DATA_W +: DATA_W];
                            dma_rd_valid <= 1'b1;
                        end
                        SRC_ENG_RD: begin
                            eng_rd_data  <= a_rdata_vec[j*DATA_W +: DATA_W];
                            eng_rd_valid <= 1'b1;
                        end
                    endcase
                end
                if (b_rvalid_vec[j]) begin
                    case (b_rd_src_q[j])
                        SRC_DMA_RD: begin
                            dma_rd_data  <= b_rdata_vec[j*DATA_W +: DATA_W];
                            dma_rd_valid <= 1'b1;
                        end
                        SRC_ENG_RD: begin
                            eng_rd_data  <= b_rdata_vec[j*DATA_W +: DATA_W];
                            eng_rd_valid <= 1'b1;
                        end
                    endcase
                end
            end

            wr_conflict <= (dma_wr_req && !dma_wr_ready) || (eng_wr_en && !eng_wr_ready);
        end
    end
endmodule