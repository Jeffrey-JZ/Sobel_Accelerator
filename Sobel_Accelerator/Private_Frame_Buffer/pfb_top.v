`timescale 1ns / 1ps

// Sobel accelerator private frame buffer(pfb)
// Author: Junze Jiang
// 4/3/2026
// Both ports can work in parallel when addressing any bank
// If both ports write the same bank+address in the same cycle, dma side wins and eng write is blocked for deterministic behavior

module private_frame_buffer #(
    parameter   DATA_W  = 8,
    parameter   ADDR_W  = 12,
    parameter   BANKS   = 4,
    parameter   IMG_W   = 128,
    parameter   IMG_H   = 128
) (
    input   wire    clk,
    input   wire    rst_n,

    // DMA Port
    input   wire                    dma_req,
    input   wire                    dma_we,
    input   wire    [ADDR_W - 1:0]  dma_addr,
    input   wire    [DATA_W - 1:0]  dma_wdata,
    output  reg     [DATA_W - 1:0]  dma_rdata,
    output  reg                     dma_rvalid,
    output  wire                    dma_ready,

    // Engine Port
    input   wire                    eng_req,
    input   wire                    eng_we,
    input   wire    [ADDR_W - 1:0]  eng_addr,
    input   wire    [DATA_W - 1:0]  eng_wdata,
    output  reg     [DATA_W - 1:0]  eng_rdata,
    output  reg                     eng_rvalid,
    output  wire                    eng_ready,

    output  reg                     wr_conflict

);
    localparam integer PIXELS       = IMG_W * IMG_H;
    localparam integer BANK_BITS    = $clog2(BANKS);           // log2, round up
    localparam integer LOCAL_ADDR_W = ADDR_W - BANK_BITS;

    // dma_addr → dma_bank_sel + dma_local_addr
    // eng_addr → eng_bank_sel + eng_local_addr
    wire [BANK_BITS-1   :0]     dma_bank_sel;
    wire [LOCAL_ADDR_W-1:0]     dma_local_addr;
    wire [BANK_BITS-1   :0]     eng_bank_sel;
    wire [LOCAL_ADDR_W-1:0]     eng_local_addr;

    ptf_addr_map #(
        .ADDR_W     (ADDR_W),
        .BANK_BITS  (BANK_BITS)
    ) dma_mapper (
        .global_addr(dma_addr),
        .bank_sel   (dma_bank_sel),     // output
        .local_addr (dma_local_addr)    // output
    );

    ptf_addr_map #(
        .ADDR_W     (ADDR_W),
        .BANK_BITS  (BANK_BITS)
    ) eng_mapper (
        .global_addr(eng_addr),
        .bank_sel   (eng_bank_sel),     // output
        .local_addr (eng_local_addr)    // output
    );

    // Generate an enable/write enable signal for each bank and instantiate BANKS RAMs
    // Only when both parties are present at the same location and both parties sign simultaneously does it constitute a conflict
    // During conflicts: DMA takes precedence; engine writes are blocked
    wire [BANKS-1       :0]     a_en_vec;   // a_en_vec[i]：Has port A of the i-th bank been selected and requires action
    wire [BANKS-1       :0]     a_we_vec;
    wire [BANKS-1       :0]     b_en_vec;
    wire [BANKS-1       :0]     b_we_vec;
    wire [BANKS*DATA_W-1:0]     a_rdata_vec;
    wire [BANKS*DATA_W-1:0]     b_rdata_vec;
    wire [BANKS-1       :0]     a_rvalid_vec;
    wire [BANKS-1       :0]     b_rvalid_vec;
    wire                        same_addr_wr_conflict;
    
    /* If DMA and the Engine initiate write requests simultaneously within the same clock cycle 
       and both write to the same local address within the same bank, then same_addr_wr_conflict = 1 */
    // Otherwise it is 0
    assign same_addr_wr_conflict = dma_req && dma_we && eng_req && eng_we && (dma_bank_sel == eng_bank_sel) && (dma_local_addr == eng_local_addr);

    // Using generate for to copy 4 identical pfb_bank_ram
    genvar i;
    generate
        for (i = 0; i < BANKS; i = i + 1) begin : g_banks
        assign  a_en_vec[i] = dma_req && (dma_bank_sel == i[BANK_BITS-1:0]);    // Only when a DMA request is present and the requested bank happens to be i will a_en_vec[i] be set to 1. Otherwise, it remains 0
        assign  a_we_vec[i] = a_en_vec[i] && dma_we;                            /* When DMA selects the i-th bank and dma_we=1, it performs a write operation on that bank
                                                                                   If dma_we=0, it performs a read operation (write-enabled is 0, but enable may still be 1) */
        assign  b_en_vec[i] = eng_req && (eng_bank_sel == i[BANK_BITS-1:0]);
        assign  b_we_vec[i] = (b_en_vec[i]) && (eng_we) && (!same_addr_wr_conflict);

        pfb_bank_ram #(
                .DATA_W     (DATA_W),
                .ADDR_W     (LOCAL_ADDR_W)
            ) u_bank (
                .clk        (clk),
                .rst_n      (rst_n),
                .a_en       (a_en_vec[i]),
                .a_we       (a_we_vec[i]),
                .a_addr     (dma_local_addr),
                .a_wdata    (dma_wdata),
                .a_rdata    (a_rdata_vec[i*DATA_W + DATA_W - 1 : i*DATA_W]),    // output
                .a_rvalid   (a_rvalid_vec[i]),                                  // output
                .b_en       (b_en_vec[i]),
                .b_we       (b_we_vec[i]),
                .b_addr     (eng_local_addr),
                .b_wdata    (eng_wdata),
                .b_rdata    (b_rdata_vec[i*DATA_W + DATA_W - 1 : i*DATA_W]),    // output
                .b_rvalid   (b_rvalid_vec[i])                                   // output
            );
        end
    endgenerate

    integer k;
    always @(*) begin
        dma_rdata  = {DATA_W{1'b0}};
        dma_rvalid = 1'b0;
        eng_rdata  = {DATA_W{1'b0}};
        eng_rvalid = 1'b0;

        for (k = 0; k < BANKS; k = k + 1) begin
            if (a_rvalid_vec[k]) begin
                dma_rdata  = a_rdata_vec[k*DATA_W +: DATA_W];
                dma_rvalid = 1'b1;
            end
            if (b_rvalid_vec[k]) begin
                eng_rdata  = b_rdata_vec[k*DATA_W +: DATA_W];
                eng_rvalid = 1'b1;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_conflict <= 1'b0;
        end else begin
            wr_conflict <= same_addr_wr_conflict;
        end
    end

    assign dma_ready = 1'b1;
    assign eng_ready = 1'b1;

endmodule