`timescale 1ns / 1ps

// DMA Module
// Author: Junze Jiang
// 17/3/2026

// Functions:
// This DMA module bridges the AXI4 external memory bus and the Private Frame Buffer (PFB),
// acting as the data mover for the entire Sobel accelerator system.
 
// Two core data-path operations:
// 1. LOAD  (AXI → PFB) — Unpack:
//  Read one AXI word (32-bit) per beat from external memory,
//  then write each byte within that word into the PFB one pixel at a time.
//  AXI word (4 bytes) → 4 individual pixel writes to PFB.
//
// 2. STORE (PFB → AXI) — Pack:
//  Read individual pixels (bytes) from the PFB one by one,
//  assemble them into a full AXI word, then burst-write back to external memory.
//  4 PFB pixel reads → 1 AXI word write.

/* 
Overall Transaction Flow:
    Host/Control block asserts `start`
    → DMA issues AXI burst reads to fetch the raw image        (LOAD phase)
    → DMA unpacks each word and pushes bytes into the PFB
    → DMA asserts `engine_start` to kick off the Sobel engine  (ENGINE phase)
    → Engine processes in-place and returns `engine_done`
    → DMA reads processed pixels back from PFB                 (STORE phase)
    → DMA packs bytes into AXI words and burst-writes to dst
    → DMA asserts `done`, transaction complete
*/

/* 
Burst Strategy:
    - Uses AXI4 INCR bursts (arburst/awburst = 2'b01)
    - Maximum burst length capped by MAX_BURST_BEATS (default 16)
    - Remaining words handled by issuing successive bursts
*/

module dma_module #(
    parameter   AXI_ADDR_W      = 32,
    parameter   AXI_DATA_W      = 32,
    parameter   PFB_ADDR_W      = 14,
    parameter   PIXELS_MAX_W    = 16,
    parameter   MAX_BURST_BEATS = 16
) (
    input   wire                                clk,
    input   wire                                rst_n,

    // Control register side (from control&state block)
    input   wire                                start,
    input   wire    [AXI_ADDR_W   - 1:0]        src_base_addr,
    input   wire    [AXI_ADDR_W   - 1:0]        dst_base_addr,
    input   wire    [PIXELS_MAX_W - 1:0]        pixel_count,
    output  reg                                 busy,
    output  reg                                 done,
    output  reg                                 error,

    // Engine control handshake
    output  reg                                 engine_start,
    input   wire                                engine_done,

    // PFB DMA-side interface (to enigne_pfb_top)
    output  reg                                 pfb_dma_rd_req,
    output  reg     [PFB_ADDR_W - 1:0]          pfb_dma_rd_addr,
    input   wire    [7:0]                       pfb_dma_rd_data,
    input   wire                                pfb_dma_rd_valid,
    input   wire                                pfb_dma_rd_ready,

    output  reg                                 pfb_dma_wr_req,
    output  reg     [PFB_ADDR_W - 1:0]          pfb_dma_wr_addr,
    output  reg     [7:0]                       pfb_dma_wr_data,
    input   wire                                pfb_dma_wr_ready,

    // AXI4 read address channel
    output  reg     [AXI_ADDR_W - 1:0]          m_axi_araddr,
    output  reg     [7:0]                       m_axi_arlen,
    output  reg     [2:0]                       m_axi_arsize,
    output  reg     [1:0]                       m_axi_arburst,
    output  reg                                 m_axi_arvalid,
    input   wire                                m_axi_arready,

    // AXI4 read data channel
    input   wire    [AXI_DATA_W - 1:0]          m_axi_rdata,
    input   wire    [1:0]                       m_axi_rresp,
    input   wire                                m_axi_rlast,
    input   wire                                m_axi_rvalid,
    output  reg                                 m_axi_rready,

    // AXI4 write address channel
    output  reg     [AXI_ADDR_W - 1:0]          m_axi_awaddr,
    output  reg     [7:0]                       m_axi_awlen,
    output  reg     [2:0]                       m_axi_awsize,
    output  reg     [1:0]                       m_axi_awburst,
    output  reg                                 m_axi_awvalid,
    input   wire                                m_axi_awready,

    // AXI4 write data channel
    output  reg     [AXI_DATA_W       - 1:0]    m_axi_wdata,
    output  reg     [(AXI_DATA_W / 8) - 1:0]    m_axi_wstrb,
    output  reg                                 m_axi_wlast,
    output  reg                                 m_axi_wvalid,
    input   wire                                m_axi_wready,

    // AXI4 write response channel
    input   wire [1:0]                          m_axi_bresp,
    input   wire                                m_axi_bvalid,
    output  reg                                 m_axi_bready
);
    localparam integer BYTES_PER_BEAT = AXI_DATA_W / 8;
    localparam integer BYTE_IDX_W     = (BYTES_PER_BEAT <= 1) ? 1 : $clog2(BYTES_PER_BEAT);

    localparam [4:0]
        S_IDLE              = 5'd0,
        S_LOAD_AR           = 5'd1,
        S_LOAD_R            = 5'd2,
        S_LOAD_PUSH_SETUP   = 5'd3,
        S_START_ENG         = 5'd4,
        S_WAIT_ENG          = 5'd5,
        S_STORE_AW          = 5'd6,
        S_STORE_PREP_BEAT   = 5'd7,
        S_STORE_REQ_PFB     = 5'd8,
        S_STORE_WAIT_PFB    = 5'd9,
        S_STORE_SEND_W      = 5'd10,
        S_STORE_WAIT_B      = 5'd11,
        S_DONE              = 5'd12,
        S_ERR               = 5'd13,
        S_LOAD_PUSH_WAIT    = 5'd14,
        S_STORE_REQ_WAIT    = 5'd15,
        S_LOAD_ERR_DRAIN    = 5'd16;

    reg [4:0]                   state;

    reg [PIXELS_MAX_W - 1:0]    load_pixel_idx;
    reg [PIXELS_MAX_W - 1:0]    store_pixel_idx;

    reg [PIXELS_MAX_W - 1:0]    total_words;
    reg [PIXELS_MAX_W - 1:0]    load_words_done;
    reg [PIXELS_MAX_W - 1:0]    store_words_done;

    reg [7:0]                   burst_beats;
    reg [7:0]                   burst_beat_idx;

    reg [AXI_DATA_W - 1:0]      load_word_buf;
    reg [BYTE_IDX_W - 1:0]      load_byte_idx;
    reg                         load_word_valid;

    reg [AXI_DATA_W - 1:0]      store_word_buf;
    reg [BYTE_IDX_W - 1:0]      store_byte_idx;
    reg [BYTE_IDX_W:0]          store_bytes_valid;
    reg                         store_word_valid;
    reg                         store_pfb_req_inflight;

    integer words_left_i;
    integer this_burst_i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            busy  <= 1'b0;
            done  <= 1'b0;
            error <= 1'b0;

            engine_start <= 1'b0;

            pfb_dma_rd_req  <= 1'b0;
            pfb_dma_rd_addr <= {PFB_ADDR_W{1'b0}};
            pfb_dma_wr_req  <= 1'b0;
            pfb_dma_wr_addr <= {PFB_ADDR_W{1'b0}};
            pfb_dma_wr_data <= 8'd0;

            m_axi_araddr  <= {AXI_ADDR_W{1'b0}};
            m_axi_arlen   <= 8'd0;
            m_axi_arsize  <= (AXI_DATA_W == 64) ? 3'd3 : 3'd2;
            m_axi_arburst <= 2'b01;
            m_axi_arvalid <= 1'b0;
            m_axi_rready  <= 1'b0;

            m_axi_awaddr  <= {AXI_ADDR_W{1'b0}};
            m_axi_awlen   <= 8'd0;
            m_axi_awsize  <= (AXI_DATA_W == 64) ? 3'd3 : 3'd2;
            m_axi_awburst <= 2'b01;
            m_axi_awvalid <= 1'b0;
            m_axi_wdata   <= {AXI_DATA_W{1'b0}};
            m_axi_wstrb   <= {AXI_DATA_W/8{1'b0}};
            m_axi_wlast   <= 1'b0;
            m_axi_wvalid  <= 1'b0;
            m_axi_bready  <= 1'b0;

            load_pixel_idx   <= {PIXELS_MAX_W{1'b0}};
            store_pixel_idx  <= {PIXELS_MAX_W{1'b0}};
            total_words      <= {PIXELS_MAX_W{1'b0}};
            load_words_done  <= {PIXELS_MAX_W{1'b0}};
            store_words_done <= {PIXELS_MAX_W{1'b0}};
            burst_beats      <= 8'd0;
            burst_beat_idx   <= 8'd0;

            load_word_buf   <= {AXI_DATA_W{1'b0}};
            load_byte_idx   <= {BYTE_IDX_W{1'b0}};
            load_word_valid <= 1'b0;

            store_word_buf         <= {AXI_DATA_W{1'b0}};
            store_byte_idx         <= {BYTE_IDX_W{1'b0}};
            store_bytes_valid      <= {(BYTE_IDX_W+1){1'b0}};
            store_word_valid       <= 1'b0;
            store_pfb_req_inflight <= 1'b0;
        end else begin
            // default pulses / one-cycle intents
            done         <= 1'b0;
            engine_start <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    error <= 1'b0;

                    pfb_dma_rd_req <= 1'b0;
                    pfb_dma_wr_req <= 1'b0;

                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b0;
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b0;
                    m_axi_bready  <= 1'b0;

                    if (start) begin
                        busy <= 1'b1;

                        load_pixel_idx   <= 0;
                        store_pixel_idx  <= 0;
                        load_words_done  <= 0;
                        store_words_done <= 0;
                        load_word_valid  <= 1'b0;
                        store_word_valid <= 1'b0;
                        store_pfb_req_inflight <= 1'b0;

                        total_words <= (pixel_count + BYTES_PER_BEAT - 1) / BYTES_PER_BEAT;

                        if (pixel_count == 0)
                            state <= S_DONE;
                        else
                            state <= S_LOAD_AR;
                    end
                end

                // --------------------- LOAD PHASE (AXI burst read -> PFB byte write)
                S_LOAD_AR: begin
                    words_left_i = total_words - load_words_done;
                    this_burst_i = (words_left_i > MAX_BURST_BEATS) ? MAX_BURST_BEATS : words_left_i;

                    m_axi_araddr  <= src_base_addr + (load_words_done * BYTES_PER_BEAT);
                    m_axi_arlen   <= this_burst_i - 1;
                    m_axi_arsize  <= (AXI_DATA_W == 64) ? 3'd3 : 3'd2;
                    m_axi_arburst <= 2'b01;
                    m_axi_arvalid <= 1'b1;

                    burst_beats    <= this_burst_i[7:0];
                    burst_beat_idx <= 8'd0;

                    if (m_axi_arvalid && m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                        m_axi_rready  <= 1'b1;
                        state         <= S_LOAD_R;
                    end
                end

                S_LOAD_R: begin
                    pfb_dma_wr_req <= 1'b0;
                    if (!load_word_valid)
                        m_axi_rready <= 1'b1;
                    else
                        m_axi_rready <= 1'b0;

                    if (m_axi_rvalid && m_axi_rready) begin
                        load_word_buf   <= m_axi_rdata;
                        load_byte_idx   <= 0;
                        load_word_valid <= 1'b1;
                        m_axi_rready    <= 1'b0;

                        if (m_axi_rresp != 2'b00) begin
                            error <= 1'b1;
                            if (m_axi_rlast) begin
                                state <= S_ERR;
                            end else begin
                                state <= S_LOAD_ERR_DRAIN;
                            end
                        end else begin
                            burst_beat_idx <= burst_beat_idx + 1'b1;
                            if (((burst_beat_idx + 1'b1) == burst_beats) && !m_axi_rlast) begin
                                error <= 1'b1;
                                state <= S_ERR;
                            end else begin
                                state <= S_LOAD_PUSH_SETUP;
                            end
                        end
                    end
                end

                S_LOAD_PUSH_SETUP: begin
                    if (load_word_valid) begin
                        pfb_dma_wr_req  <= 1'b1;
                        pfb_dma_wr_addr <= load_pixel_idx[PFB_ADDR_W-1:0];
                        pfb_dma_wr_data <= load_word_buf[8*load_byte_idx +: 8];
                        state <= S_LOAD_PUSH_WAIT;
                    end
                end

                S_LOAD_PUSH_WAIT: begin
                    if (load_word_valid) begin
                        // 保持请求和数据稳定，直到 ready
                        pfb_dma_wr_req  <= 1'b1;
                        pfb_dma_wr_addr <= load_pixel_idx[PFB_ADDR_W-1:0];
                        pfb_dma_wr_data <= load_word_buf[8*load_byte_idx +: 8];

                        if (pfb_dma_wr_ready) begin
                            pfb_dma_wr_req <= 1'b0;
                            load_pixel_idx <= load_pixel_idx + 1'b1;

                            if ((load_pixel_idx + 1'b1) >= pixel_count) begin
                                load_word_valid <= 1'b0;
                                load_words_done <= load_words_done + 1'b1;
                                state <= S_START_ENG;
                            end
                            else if (load_byte_idx == BYTES_PER_BEAT-1) begin
                                load_word_valid <= 1'b0;
                                load_words_done <= load_words_done + 1'b1;

                                if ((burst_beat_idx >= burst_beats) && ((load_words_done + 1'b1) < total_words))
                                    state <= S_LOAD_AR;
                                else if ((burst_beat_idx >= burst_beats) && ((load_words_done + 1'b1) == total_words))
                                    state <= S_START_ENG;
                                else
                                    state <= S_LOAD_R;
                            end
                            else begin
                                load_byte_idx <= load_byte_idx + 1'b1;
                                state <= S_LOAD_PUSH_SETUP;
                            end
                        end
                    end
                end

                // --------------------- LOAD ERROR DRAIN
                // AXI protocol: master must accept all R beats until rlast.
                // After rresp error mid-burst, drain remaining beats here.
                S_LOAD_ERR_DRAIN: begin
                    pfb_dma_wr_req  <= 1'b0;
                    load_word_valid <= 1'b0;
                    m_axi_rready    <= 1'b1;
                    if (m_axi_rvalid && m_axi_rready && m_axi_rlast) begin
                        m_axi_rready <= 1'b0;
                        state        <= S_ERR;
                    end
                end

                // --------------------- ENGINE PHASE
                S_START_ENG: begin
                    pfb_dma_wr_req <= 1'b0;
                    m_axi_rready   <= 1'b0;
                    engine_start   <= 1'b1;
                    state          <= S_WAIT_ENG;
                end

                S_WAIT_ENG: begin
                    if (engine_done) begin
                        store_pixel_idx  <= 0;
                        store_words_done <= 0;
                        store_word_valid <= 1'b0;
                        store_pfb_req_inflight <= 1'b0;
                        state <= S_STORE_AW;
                    end
                end

                // --------------------- STORE PHASE (PFB byte read -> AXI burst write)
                S_STORE_AW: begin
                    words_left_i = total_words - store_words_done;
                    this_burst_i = (words_left_i > MAX_BURST_BEATS) ? MAX_BURST_BEATS : words_left_i;

                    m_axi_awaddr  <= dst_base_addr + (store_words_done * BYTES_PER_BEAT);
                    m_axi_awlen   <= this_burst_i - 1;
                    m_axi_awsize  <= (AXI_DATA_W == 64) ? 3'd3 : 3'd2;
                    m_axi_awburst <= 2'b01;
                    m_axi_awvalid <= 1'b1;

                    burst_beats    <= this_burst_i[7:0];
                    burst_beat_idx <= 8'd0;

                    if (m_axi_awvalid && m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                        state         <= S_STORE_PREP_BEAT;
                    end
                end

                S_STORE_PREP_BEAT: begin
                    store_word_buf    <= {AXI_DATA_W{1'b0}};
                    store_byte_idx    <= 0;
                    store_bytes_valid <= 0;
                    store_word_valid  <= 1'b0;
                    state             <= S_STORE_REQ_PFB;
                end

                S_STORE_REQ_PFB: begin
                    if (!store_pfb_req_inflight) begin
                        pfb_dma_rd_req  <= 1'b1;
                        pfb_dma_rd_addr <= store_pixel_idx[PFB_ADDR_W-1:0];
                        state <= S_STORE_REQ_WAIT;
                    end
                end

                S_STORE_REQ_WAIT: begin
                    pfb_dma_rd_req  <= 1'b1;
                    pfb_dma_rd_addr <= store_pixel_idx[PFB_ADDR_W-1:0];

                    if (pfb_dma_rd_ready) begin
                        pfb_dma_rd_req <= 1'b0;
                        store_pfb_req_inflight <= 1'b1;
                        state <= S_STORE_WAIT_PFB;
                    end
                end

                S_STORE_WAIT_PFB: begin
                    pfb_dma_rd_req <= 1'b0;
                    if (pfb_dma_rd_valid) begin
                        store_pfb_req_inflight <= 1'b0;
                        store_word_buf[8*store_byte_idx +: 8] <= pfb_dma_rd_data;
                        store_byte_idx    <= store_byte_idx + 1'b1;
                        store_bytes_valid <= store_bytes_valid + 1'b1;
                        store_pixel_idx   <= store_pixel_idx + 1'b1;

                        if ((store_byte_idx == BYTES_PER_BEAT-1) ||
                            ((store_pixel_idx + 1'b1) >= pixel_count)) begin
                            store_word_valid <= 1'b1;
                            state <= S_STORE_SEND_W;
                        end else begin
                            state <= S_STORE_REQ_PFB;
                        end
                    end
                end
                
                S_STORE_SEND_W: begin
                    if (store_word_valid) begin
                        m_axi_wdata  <= store_word_buf;
                        m_axi_wstrb  <= ({(AXI_DATA_W/8){1'b1}} >> (BYTES_PER_BEAT - store_bytes_valid));
                        m_axi_wlast  <= ((burst_beat_idx + 1'b1) == burst_beats);
                        m_axi_wvalid <= 1'b1;

                        if (m_axi_wvalid && m_axi_wready) begin
                            m_axi_wvalid   <= 1'b0;
                            store_word_valid <= 1'b0;
                            burst_beat_idx <= burst_beat_idx + 1'b1;
                            store_words_done <= store_words_done + 1'b1;

                            if ((burst_beat_idx + 1'b1) == burst_beats) begin
                                m_axi_bready <= 1'b1;
                                state <= S_STORE_WAIT_B;
                            end else begin
                                state <= S_STORE_PREP_BEAT;
                            end
                        end
                    end
                end

                S_STORE_WAIT_B: begin
                    if (m_axi_bvalid && m_axi_bready) begin
                        m_axi_bready <= 1'b0;
                        if (m_axi_bresp != 2'b00) begin
                            error <= 1'b1;
                            state <= S_ERR;
                        end else if (store_words_done >= total_words) begin
                            state <= S_DONE;
                        end else begin
                            state <= S_STORE_AW;
                        end
                    end
                end

                S_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= S_IDLE;
                end

                S_ERR: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule