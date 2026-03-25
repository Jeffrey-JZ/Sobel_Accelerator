`timescale 1ns / 1ps

// Testbench for dma_csr_top
// Author: Junze Jiang
// Date: 18/3/2026

// This testbench exercises the integrated DMA+CSR subsystem end-to-end
// through the Wishbone bus interface.  All AXI, PFB, and engine ports are
// driven by behavioural models embedded in this file.

// CSR Register Map (byte addresses, reg_idx = adr[5:2]):
//   0x00 CTRL        [0]=start(W1P), [1]=clear done_latched, [2]=clear err_latched
//   0x04 SRC_BASE    source AXI base address
//   0x08 DST_BASE    destination AXI base address
//   0x0C PIXEL_COUNT number of pixels to transfer
//   0x10 STATUS      [0]=busy, [1]=done|done_latched, [2]=error|err_latched

// Test Cases (25 total):
//
//  --- Core function tests ---
//  CASE  1 : Reset state verification (WB outputs, no spurious activity)
//  CASE  2 : CSR register write & readback (SRC / DST / COUNT)
//  CASE  3 : Full DMA flow — 16 pixels, word-aligned
//  CASE  4 : Status polling during active DMA
//  CASE  5 : Done-latch & clear via CTRL write
//  CASE  6 : Error-latch & clear via CTRL write (AXI read error)
//
//  --- Boundary / alignment ---
//  CASE  7 : pixel_count = 0  (immediate done, no AXI activity)
//  CASE  8 : pixel_count = 1  (single pixel — partial word)
//  CASE  9 : pixel_count = 3  (less than one AXI word)
//  CASE 10 : pixel_count = 7  (non-word-aligned, two AXI words)
//  CASE 11 : pixel_count = 64 (exact one-burst boundary, 16 words)
//
//  --- Multi-burst ---
//  CASE 12 : pixel_count = 100 (25 words = 16+9, two LOAD + two STORE bursts)
//  CASE 13 : pixel_count = 256 (4 full bursts)
//
//  --- Error injection ---
//  CASE 14 : AXI write error (bresp=SLVERR)
//  CASE 15 : AXI read error followed by re-run with no error
//
//  --- Backpressure / latency ---
//  CASE 16 : PFB write backpressure (slow wr_ready during LOAD)
//  CASE 17 : PFB read backpressure (slow rd_ready during STORE)
//  CASE 18 : Combined PFB backpressure (both slow)
//  CASE 19 : AXI slave latency (delayed arready/awready)
//
//  --- Sequencing / corner ---
//  CASE 20 : Back-to-back DMA runs without reset
//  CASE 21 : Non-zero base addresses (offset src & dst)
//  CASE 22 : WB sel gating — start NOT fired when sel[0]=0
//  CASE 23 : Write to invalid CSR address (no side-effect)
//  CASE 24 : Read from invalid CSR address (returns 0)
//  CASE 25 : engine_start pulse verification (observed between LOAD and STORE)

module tb_dma_csr_top();

    // ================================================================
    //  PARAMETERS
    // ================================================================
    parameter WB_ADDR_W      = 8;
    parameter WB_DATA_W      = 32;
    parameter AXI_ADDR_W     = 32;
    parameter AXI_DATA_W     = 32;
    parameter PFB_ADDR_W     = 14;
    parameter PIXELS_MAX_W   = 16;

    localparam BYTES_PER_BEAT  = AXI_DATA_W / 8;           // 4
    localparam MAX_BURST_BEATS = 16;                        // must match DMA default
    localparam CLK_P           = 10;                        // 100 MHz

    // CSR byte addresses
    localparam [WB_ADDR_W-1:0] ADDR_CTRL   = 8'h00;
    localparam [WB_ADDR_W-1:0] ADDR_SRC    = 8'h04;
    localparam [WB_ADDR_W-1:0] ADDR_DST    = 8'h08;
    localparam [WB_ADDR_W-1:0] ADDR_COUNT  = 8'h0C;
    localparam [WB_ADDR_W-1:0] ADDR_STATUS = 8'h10;
    localparam [WB_ADDR_W-1:0] ADDR_INVAL  = 8'h14;

    // Memory model depths
    localparam AXI_MEM_DEPTH = 65536;
    localparam PFB_MEM_DEPTH = 16384;

    // ================================================================
    //  DUT SIGNALS
    // ================================================================
    reg                         clk;
    reg                         rst_n;

    // Wishbone
    reg  [WB_ADDR_W-1:0]        wb_adr_i;
    reg  [WB_DATA_W-1:0]        wb_dat_i;
    wire [WB_DATA_W-1:0]        wb_dat_o;
    reg  [WB_DATA_W/8-1:0]      wb_sel_i;
    reg                         wb_we_i;
    reg                         wb_stb_i;
    reg                         wb_cyc_i;
    wire                        wb_ack_o;

    // Engine
    wire                        engine_start;
    reg                         engine_done;

    // Engine configuration (threshold from CSR)
    wire [7:0]                  threshold;

    // PFB DMA-side
    wire                        pfb_dma_rd_req;
    wire [PFB_ADDR_W-1:0]       pfb_dma_rd_addr;
    reg  [7:0]                  pfb_dma_rd_data;
    reg                         pfb_dma_rd_valid;
    reg                         pfb_dma_rd_ready;

    wire                        pfb_dma_wr_req;
    wire [PFB_ADDR_W-1:0]       pfb_dma_wr_addr;
    wire [7:0]                  pfb_dma_wr_data;
    reg                         pfb_dma_wr_ready;

    // AXI4 master
    wire [AXI_ADDR_W-1:0]       m_axi_araddr;
    wire [7:0]                  m_axi_arlen;
    wire [2:0]                  m_axi_arsize;
    wire [1:0]                  m_axi_arburst;
    wire                        m_axi_arvalid;
    reg                         m_axi_arready;

    reg  [AXI_DATA_W-1:0]       m_axi_rdata;
    reg  [1:0]                  m_axi_rresp;
    reg                         m_axi_rlast;
    reg                         m_axi_rvalid;
    wire                        m_axi_rready;

    wire [AXI_ADDR_W-1:0]       m_axi_awaddr;
    wire [7:0]                  m_axi_awlen;
    wire [2:0]                  m_axi_awsize;
    wire [1:0]                  m_axi_awburst;
    wire                        m_axi_awvalid;
    reg                         m_axi_awready;

    wire [AXI_DATA_W-1:0]       m_axi_wdata;
    wire [AXI_DATA_W/8-1:0]     m_axi_wstrb;
    wire                        m_axi_wlast;
    wire                        m_axi_wvalid;
    reg                         m_axi_wready;

    reg  [1:0]                  m_axi_bresp;
    reg                         m_axi_bvalid;
    wire                        m_axi_bready;

    // ================================================================
    //  MEMORY MODELS
    // ================================================================
    reg [7:0] axi_src_mem  [0:AXI_MEM_DEPTH-1];
    reg [7:0] axi_dst_mem  [0:AXI_MEM_DEPTH-1];
    reg [7:0] pfb_mem      [0:PFB_MEM_DEPTH-1];

    // ================================================================
    //  AXI READ SLAVE STATE
    // ================================================================
    reg  [AXI_ADDR_W-1:0]   ar_addr_latched;
    reg  [7:0]              ar_len_latched;
    reg  [7:0]              ar_beat_cnt;
    reg                     ar_active;
    reg                     r_beat_pending;

    // ================================================================
    //  AXI WRITE SLAVE STATE
    // ================================================================
    reg  [AXI_ADDR_W-1:0]   aw_addr_latched;
    reg  [7:0]              aw_len_latched;
    reg  [7:0]              aw_beat_cnt;
    reg                     aw_active;

    // ================================================================
    //  TEST CONTROL
    // ================================================================
    integer total_errors;
    integer case_errors;
    integer i;
    integer timed_out;
    integer tmp_errs;
    integer saw_engine_start;
    reg [WB_DATA_W-1:0] rd_data;

    // ================================================================
    //  CONFIGURABLE BEHAVIOUR KNOBS
    // ================================================================
    reg                     inject_rresp_err;
    reg                     inject_bresp_err;
    integer                 pfb_wr_ready_delay;
    integer                 pfb_rd_ready_delay;
    integer                 engine_proc_cycles;
    integer                 axi_ar_accept_delay;   // extra cycles before arready
    integer                 axi_aw_accept_delay;   // extra cycles before awready

    // ================================================================
    //  DUT INSTANTIATION
    // ================================================================
    dma_csr_top #(
        .WB_ADDR_W          (WB_ADDR_W),
        .WB_DATA_W          (WB_DATA_W),
        .AXI_ADDR_W         (AXI_ADDR_W),
        .AXI_DATA_W         (AXI_DATA_W),
        .PFB_ADDR_W         (PFB_ADDR_W),
        .PIXELS_MAX_W       (PIXELS_MAX_W)
    ) dut (
        .clk                (clk),
        .rst_n              (rst_n),
        // Wishbone
        .wb_adr_i           (wb_adr_i),
        .wb_dat_i           (wb_dat_i),
        .wb_dat_o           (wb_dat_o),
        .wb_sel_i           (wb_sel_i),
        .wb_we_i            (wb_we_i),
        .wb_stb_i           (wb_stb_i),
        .wb_cyc_i           (wb_cyc_i),
        .wb_ack_o           (wb_ack_o),
        // Engine
        .engine_start       (engine_start),
        .engine_done        (engine_done),
        // PFB
        .pfb_dma_rd_req     (pfb_dma_rd_req),
        .pfb_dma_rd_addr    (pfb_dma_rd_addr),
        .pfb_dma_rd_data    (pfb_dma_rd_data),
        .pfb_dma_rd_valid   (pfb_dma_rd_valid),
        .pfb_dma_rd_ready   (pfb_dma_rd_ready),
        .pfb_dma_wr_req     (pfb_dma_wr_req),
        .pfb_dma_wr_addr    (pfb_dma_wr_addr),
        .pfb_dma_wr_data    (pfb_dma_wr_data),
        .pfb_dma_wr_ready   (pfb_dma_wr_ready),
        // AXI
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

        .threshold          (threshold)
    );

    // ================================================================
    //  CLOCK GENERATION
    // ================================================================
    initial begin
        clk = 0;
        forever #(CLK_P / 2) clk = ~clk;
    end

    // ================================================================
    //  GLOBAL TIMEOUT WATCHDOG
    // ================================================================
    initial begin
        #(CLK_P * 1000000);
        $display("\n[FATAL] Global timeout - simulation killed at time %0t.", $time);
        $finish;
    end

    // ================================================================
    //  AXI READ SLAVE MODEL
    //  Supports configurable accept delay via axi_ar_accept_delay.
    // ================================================================
    reg [7:0] ar_delay_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_axi_arready    <= 1'b0;
            m_axi_rdata      <= {AXI_DATA_W{1'b0}};
            m_axi_rresp      <= 2'b00;
            m_axi_rlast      <= 1'b0;
            m_axi_rvalid     <= 1'b0;
            ar_addr_latched  <= {AXI_ADDR_W{1'b0}};
            ar_len_latched   <= 8'd0;
            ar_beat_cnt      <= 8'd0;
            ar_active        <= 1'b0;
            r_beat_pending   <= 1'b0;
            ar_delay_cnt     <= 8'd0;
        end else begin
            // ---- AR channel with configurable delay ----
            if (m_axi_arvalid && !ar_active) begin
                if (ar_delay_cnt >= axi_ar_accept_delay) begin
                    m_axi_arready <= 1'b1;
                    ar_delay_cnt  <= 8'd0;
                end else begin
                    m_axi_arready <= 1'b0;
                    ar_delay_cnt  <= ar_delay_cnt + 1'b1;
                end
            end else begin
                m_axi_arready <= 1'b0;
                if (!m_axi_arvalid)
                    ar_delay_cnt <= 8'd0;
            end

            if (m_axi_arvalid && m_axi_arready) begin
                ar_addr_latched <= m_axi_araddr;
                ar_len_latched  <= m_axi_arlen;
                ar_beat_cnt     <= 8'd0;
                ar_active       <= 1'b1;
                r_beat_pending  <= 1'b1;
                m_axi_arready   <= 1'b0;
                m_axi_rvalid    <= 1'b0;
            end

            // ---- Present beat data ----
            if (ar_active && r_beat_pending && !m_axi_rvalid) begin
                m_axi_rdata <= {
                    axi_src_mem[ar_addr_latched + ar_beat_cnt * BYTES_PER_BEAT + 3],
                    axi_src_mem[ar_addr_latched + ar_beat_cnt * BYTES_PER_BEAT + 2],
                    axi_src_mem[ar_addr_latched + ar_beat_cnt * BYTES_PER_BEAT + 1],
                    axi_src_mem[ar_addr_latched + ar_beat_cnt * BYTES_PER_BEAT + 0]
                };
                m_axi_rvalid <= 1'b1;
                m_axi_rresp  <= inject_rresp_err ? 2'b10 : 2'b00;
                m_axi_rlast  <= (ar_beat_cnt == ar_len_latched);
                r_beat_pending <= 1'b0;
            end

            // ---- R handshake ----
            if (m_axi_rvalid && m_axi_rready) begin
                m_axi_rvalid <= 1'b0;
                if (ar_beat_cnt == ar_len_latched) begin
                    ar_active <= 1'b0;
                end else begin
                    ar_beat_cnt    <= ar_beat_cnt + 1'b1;
                    r_beat_pending <= 1'b1;
                end
            end
        end
    end

    // ================================================================
    //  AXI WRITE SLAVE MODEL
    //  Supports configurable accept delay via axi_aw_accept_delay.
    // ================================================================
    reg [7:0] aw_delay_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_axi_awready    <= 1'b0;
            m_axi_wready     <= 1'b0;
            m_axi_bresp      <= 2'b00;
            m_axi_bvalid     <= 1'b0;
            aw_addr_latched  <= {AXI_ADDR_W{1'b0}};
            aw_len_latched   <= 8'd0;
            aw_beat_cnt      <= 8'd0;
            aw_active        <= 1'b0;
            aw_delay_cnt     <= 8'd0;
        end else begin
            // AW channel with configurable delay
            if (m_axi_awvalid && !aw_active && !m_axi_bvalid) begin
                if (aw_delay_cnt >= axi_aw_accept_delay) begin
                    m_axi_awready <= 1'b1;
                    aw_delay_cnt  <= 8'd0;
                end else begin
                    m_axi_awready <= 1'b0;
                    aw_delay_cnt  <= aw_delay_cnt + 1'b1;
                end
            end else begin
                m_axi_awready <= 1'b0;
                if (!m_axi_awvalid)
                    aw_delay_cnt <= 8'd0;
            end

            if (m_axi_awvalid && m_axi_awready) begin
                aw_addr_latched <= m_axi_awaddr;
                aw_len_latched  <= m_axi_awlen;
                aw_beat_cnt     <= 8'd0;
                aw_active       <= 1'b1;
                m_axi_awready   <= 1'b0;
                m_axi_wready    <= 1'b1;
            end

            // W channel
            if (aw_active && m_axi_wvalid && m_axi_wready) begin
                if (m_axi_wstrb[0]) axi_dst_mem[aw_addr_latched + aw_beat_cnt * BYTES_PER_BEAT + 0] <= m_axi_wdata[ 7: 0];
                if (m_axi_wstrb[1]) axi_dst_mem[aw_addr_latched + aw_beat_cnt * BYTES_PER_BEAT + 1] <= m_axi_wdata[15: 8];
                if (m_axi_wstrb[2]) axi_dst_mem[aw_addr_latched + aw_beat_cnt * BYTES_PER_BEAT + 2] <= m_axi_wdata[23:16];
                if (m_axi_wstrb[3]) axi_dst_mem[aw_addr_latched + aw_beat_cnt * BYTES_PER_BEAT + 3] <= m_axi_wdata[31:24];

                if (m_axi_wlast || (aw_beat_cnt == aw_len_latched)) begin
                    aw_active     <= 1'b0;
                    m_axi_wready  <= 1'b0;
                    m_axi_bvalid  <= 1'b1;
                    m_axi_bresp   <= inject_bresp_err ? 2'b10 : 2'b00;
                end else begin
                    aw_beat_cnt <= aw_beat_cnt + 1'b1;
                end
            end

            // B channel
            if (m_axi_bvalid && m_axi_bready) begin
                m_axi_bvalid <= 1'b0;
            end
        end
    end

    // ================================================================
    //  PFB WRITE MODEL  (DMA LOAD phase writes here)
    // ================================================================
    reg [7:0] pfb_wr_delay_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pfb_dma_wr_ready <= 1'b0;
            pfb_wr_delay_cnt <= 8'd0;
        end else begin
            if (pfb_dma_wr_req) begin
                if (pfb_wr_delay_cnt >= pfb_wr_ready_delay) begin
                    pfb_dma_wr_ready <= 1'b1;
                    if (pfb_dma_wr_ready) begin
                        pfb_mem[pfb_dma_wr_addr] <= pfb_dma_wr_data;
                        pfb_wr_delay_cnt <= 8'd0;
                    end
                end else begin
                    pfb_dma_wr_ready <= 1'b0;
                    pfb_wr_delay_cnt <= pfb_wr_delay_cnt + 1'b1;
                end
            end else begin
                pfb_dma_wr_ready <= 1'b0;
                pfb_wr_delay_cnt <= 8'd0;
            end
        end
    end

    // ================================================================
    //  PFB READ MODEL  (DMA STORE phase reads here)
    // ================================================================
    reg [7:0]               pfb_rd_delay_cnt;
    reg                     pfb_rd_pending;
    reg [PFB_ADDR_W-1:0]   pfb_rd_addr_lat;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pfb_dma_rd_ready <= 1'b0;
            pfb_dma_rd_valid <= 1'b0;
            pfb_dma_rd_data  <= 8'd0;
            pfb_rd_delay_cnt <= 8'd0;
            pfb_rd_pending   <= 1'b0;
            pfb_rd_addr_lat  <= {PFB_ADDR_W{1'b0}};
        end else begin
            pfb_dma_rd_valid <= 1'b0;

            if (pfb_dma_rd_req && !pfb_rd_pending) begin
                if (pfb_rd_delay_cnt >= pfb_rd_ready_delay) begin
                    pfb_dma_rd_ready <= 1'b1;
                    if (pfb_dma_rd_ready) begin
                        pfb_rd_addr_lat  <= pfb_dma_rd_addr;
                        pfb_rd_pending   <= 1'b1;
                        pfb_dma_rd_ready <= 1'b0;
                        pfb_rd_delay_cnt <= 8'd0;
                    end
                end else begin
                    pfb_dma_rd_ready <= 1'b0;
                    pfb_rd_delay_cnt <= pfb_rd_delay_cnt + 1'b1;
                end
            end else if (!pfb_dma_rd_req && !pfb_rd_pending) begin
                pfb_dma_rd_ready <= 1'b0;
                pfb_rd_delay_cnt <= 8'd0;
            end

            if (pfb_rd_pending) begin
                pfb_dma_rd_data  <= pfb_mem[pfb_rd_addr_lat];
                pfb_dma_rd_valid <= 1'b1;
                pfb_rd_pending   <= 1'b0;
                pfb_dma_rd_ready <= 1'b0;
            end
        end
    end

    // ================================================================
    //  ENGINE STUB MODEL
    //  On engine_start: inverts all PFB bytes (simulates processing),
    //  then asserts engine_done after engine_proc_cycles clocks.
    // ================================================================
    reg         engine_running;
    integer     engine_cycle_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            engine_done      <= 1'b0;
            engine_running   <= 1'b0;
            engine_cycle_cnt <= 0;
        end else begin
            engine_done <= 1'b0;

            if (engine_start && !engine_running) begin
                engine_running   <= 1'b1;
                engine_cycle_cnt <= 0;
                for (i = 0; i < PFB_MEM_DEPTH; i = i + 1)
                    pfb_mem[i] <= ~pfb_mem[i];
            end

            if (engine_running) begin
                if (engine_cycle_cnt >= engine_proc_cycles) begin
                    engine_done    <= 1'b1;
                    engine_running <= 1'b0;
                end else begin
                    engine_cycle_cnt <= engine_cycle_cnt + 1;
                end
            end
        end
    end

    // ================================================================
    //  REUSABLE TASKS
    // ================================================================

    task do_reset;
        begin
            rst_n = 0;
            wb_adr_i  = 0;
            wb_dat_i  = 0;
            wb_sel_i  = 4'hF;
            wb_we_i   = 0;
            wb_stb_i  = 0;
            wb_cyc_i  = 0;

            inject_rresp_err     = 0;
            inject_bresp_err     = 0;
            pfb_wr_ready_delay   = 0;
            pfb_rd_ready_delay   = 0;
            engine_proc_cycles   = 5;
            axi_ar_accept_delay  = 0;
            axi_aw_accept_delay  = 0;

            #(CLK_P * 3);
            rst_n = 1;
            #(CLK_P * 2);
        end
    endtask

    // Wishbone single write
    task wb_write;
        input [WB_ADDR_W-1:0]     addr;
        input [WB_DATA_W-1:0]     data;
        input [WB_DATA_W/8-1:0]   sel;
        begin
            @(posedge clk); #1;
            wb_cyc_i = 1;
            wb_stb_i = 1;
            wb_we_i  = 1;
            wb_adr_i = addr;
            wb_dat_i = data;
            wb_sel_i = sel;
            @(posedge clk);
            while (!wb_ack_o) @(posedge clk);
            #1;
            wb_cyc_i = 0;
            wb_stb_i = 0;
            wb_we_i  = 0;
        end
    endtask

    // Wishbone single read
    task wb_read;
        input  [WB_ADDR_W-1:0]    addr;
        output [WB_DATA_W-1:0]    data;
        begin
            @(posedge clk); #1;
            wb_cyc_i = 1;
            wb_stb_i = 1;
            wb_we_i  = 0;
            wb_adr_i = addr;
            wb_sel_i = 4'hF;
            @(posedge clk);
            while (!wb_ack_o) @(posedge clk);
            data = wb_dat_o;
            #1;
            wb_cyc_i = 0;
            wb_stb_i = 0;
        end
    endtask

    // Configure CSR registers and issue start command
    task configure_and_start;
        input [AXI_ADDR_W-1:0]  src;
        input [AXI_ADDR_W-1:0]  dst;
        input [PIXELS_MAX_W-1:0] count;
        begin
            wb_write(ADDR_SRC,   src,                          4'hF);
            wb_write(ADDR_DST,   dst,                          4'hF);
            wb_write(ADDR_COUNT, {{(WB_DATA_W-PIXELS_MAX_W){1'b0}}, count}, 4'hF);
            wb_write(ADDR_CTRL,  32'h0000_0001,                4'hF);  // start
        end
    endtask

    // Wait for STATUS[1] (done_latched) to become 1 via polling
    task wait_dma_done_poll;
        input integer timeout_cycles;
        integer cnt;
        reg [WB_DATA_W-1:0] stat;
        begin
            timed_out = 0;
            cnt = 0;
            stat = 0;
            while (!stat[1] && (cnt < timeout_cycles)) begin
                wb_read(ADDR_STATUS, stat);
                cnt = cnt + 1;
            end
            if (!stat[1]) timed_out = 1;
        end
    endtask

    // Fill source AXI memory
    task fill_src_mem;
        input integer count;
        input integer seed;
        integer idx;
        begin
            for (idx = 0; idx < AXI_MEM_DEPTH; idx = idx + 1)
                axi_src_mem[idx] = 8'h00;
            for (idx = 0; idx < count; idx = idx + 1)
                axi_src_mem[idx] = (idx + seed) & 8'hFF;
        end
    endtask

    // Fill source memory at offset
    task fill_src_mem_at_offset;
        input integer offset;
        input integer count;
        input integer seed;
        integer idx;
        begin
            for (idx = 0; idx < AXI_MEM_DEPTH; idx = idx + 1)
                axi_src_mem[idx] = 8'h00;
            for (idx = 0; idx < count; idx = idx + 1)
                axi_src_mem[offset + idx] = (idx + seed) & 8'hFF;
        end
    endtask

    // Clear destination and PFB
    task clear_dst_and_pfb;
        integer idx;
        begin
            for (idx = 0; idx < AXI_MEM_DEPTH; idx = idx + 1)
                axi_dst_mem[idx] = 8'hXX;
            for (idx = 0; idx < PFB_MEM_DEPTH; idx = idx + 1)
                pfb_mem[idx] = 8'h00;
        end
    endtask

    // Verify destination data (engine inverts bytes)
    task verify_store_data;
        input integer count;
        input integer seed;
        input integer expect_inverted;
        integer idx;
        integer errs;
        reg [7:0] expected;
        begin
            errs = 0;
            for (idx = 0; idx < count; idx = idx + 1) begin
                if (expect_inverted)
                    expected = ~((idx + seed) & 8'hFF);
                else
                    expected = (idx + seed) & 8'hFF;

                if (axi_dst_mem[idx] !== expected) begin
                    if (errs < 10)
                        $display("    [DATA ERR] dst[%0d] expected=0x%02h got=0x%02h",
                                 idx, expected, axi_dst_mem[idx]);
                    errs = errs + 1;
                end
            end
            if (errs == 0)
                $display("    [PASS] All %0d destination bytes match expected.", count);
            else
                $display("    [FAIL] %0d byte mismatches out of %0d.", errs, count);
            tmp_errs = errs;
        end
    endtask

    // Verify destination data at offset
    task verify_store_data_at_offset;
        input integer base_offset;
        input integer count;
        input integer seed;
        input integer expect_inverted;
        integer idx;
        integer errs;
        reg [7:0] expected;
        begin
            errs = 0;
            for (idx = 0; idx < count; idx = idx + 1) begin
                if (expect_inverted)
                    expected = ~((idx + seed) & 8'hFF);
                else
                    expected = (idx + seed) & 8'hFF;

                if (axi_dst_mem[base_offset + idx] !== expected) begin
                    if (errs < 10)
                        $display("    [DATA ERR] dst[%0d] expected=0x%02h got=0x%02h",
                                 base_offset + idx, expected, axi_dst_mem[base_offset + idx]);
                    errs = errs + 1;
                end
            end
            if (errs == 0)
                $display("    [PASS] All %0d bytes at offset %0d match.", count, base_offset);
            else
                $display("    [FAIL] %0d byte mismatches out of %0d.", errs, count);
            tmp_errs = errs;
        end
    endtask

    // Check helper
    task check_cond;
        input condition;
        input [800:0] msg;
        begin
            if (!condition) begin
                $display("    [FAIL] %0s  (time %0t)", msg, $time);
                case_errors = case_errors + 1;
            end else begin
                $display("    [PASS] %0s", msg);
            end
        end
    endtask

    // Run a full DMA transaction using WB polling for done status
    task run_dma_via_wb;
        input [AXI_ADDR_W-1:0]     src;
        input [AXI_ADDR_W-1:0]     dst;
        input [PIXELS_MAX_W-1:0]   count;
        input integer               timeout;
        begin
            configure_and_start(src, dst, count);
            wait_dma_done_poll(timeout);
        end
    endtask

    // ================================================================
    //  MAIN TEST SEQUENCE
    // ================================================================
    initial begin
        total_errors = 0;
        tmp_errs     = 0;

        $display("\n");
        $display("##########################################################");
        $display("#    DMA_CSR_TOP INTEGRATION TESTBENCH                   #");
        $display("#    AXI_DATA_W=%0d  BYTES_PER_BEAT=%0d  MAX_BURST=%0d   #",
                 AXI_DATA_W, BYTES_PER_BEAT, MAX_BURST_BEATS);
        $display("##########################################################\n");

        // ============================================================
        //  CASE 1: Reset State Verification
        // ============================================================
        $display("==========================================================");
        $display(" CASE 1: Reset State Verification");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        check_cond(wb_ack_o === 1'b0,       "wb_ack_o low after reset");
        check_cond(engine_start === 1'b0,    "engine_start low after reset");
        check_cond(pfb_dma_rd_req === 1'b0,  "pfb_dma_rd_req low after reset");
        check_cond(pfb_dma_wr_req === 1'b0,  "pfb_dma_wr_req low after reset");
        check_cond(m_axi_arvalid === 1'b0,   "m_axi_arvalid low after reset");
        check_cond(m_axi_awvalid === 1'b0,   "m_axi_awvalid low after reset");
        check_cond(m_axi_wvalid === 1'b0,    "m_axi_wvalid low after reset");

        // Read all CSR registers - expect zeros
        wb_read(ADDR_CTRL, rd_data);
        check_cond(rd_data === 32'd0, "CTRL register reads 0 after reset");
        wb_read(ADDR_SRC, rd_data);
        check_cond(rd_data === 32'd0, "SRC register reads 0 after reset");
        wb_read(ADDR_DST, rd_data);
        check_cond(rd_data === 32'd0, "DST register reads 0 after reset");
        wb_read(ADDR_COUNT, rd_data);
        check_cond(rd_data === 32'd0, "COUNT register reads 0 after reset");
        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data === 32'd0, "STATUS register reads 0 after reset");

        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 2: CSR Register Write & Readback
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 2: CSR Register Write & Readback (SRC/DST/COUNT)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        wb_write(ADDR_SRC,   32'hDEAD_BEEF, 4'hF);
        wb_write(ADDR_DST,   32'hCAFE_BABE, 4'hF);
        wb_write(ADDR_COUNT, 32'h0000_ABCD, 4'hF);

        wb_read(ADDR_SRC, rd_data);
        check_cond(rd_data === 32'hDEAD_BEEF, "SRC readback matches written value");
        wb_read(ADDR_DST, rd_data);
        check_cond(rd_data === 32'hCAFE_BABE, "DST readback matches written value");
        wb_read(ADDR_COUNT, rd_data);
        check_cond(rd_data === 32'h0000_ABCD, "COUNT readback matches written value (16-bit)");

        // Overwrite and re-read
        wb_write(ADDR_SRC, 32'h1111_1111, 4'hF);
        wb_read(ADDR_SRC, rd_data);
        check_cond(rd_data === 32'h1111_1111, "SRC overwrite & readback correct");

        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 3: Full DMA Flow (16 pixels, word-aligned)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 3: Full DMA Flow (16 pixels, word-aligned)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(16, 0);
        clear_dst_and_pfb();

        engine_proc_cycles = 5;
        run_dma_via_wb(32'h0000_0000, 32'h0000_0000, 16'd16, 5000);

        check_cond(!timed_out, "DMA completed (STATUS[1]=1 polled) without timeout");

        // Verify STATUS shows done
        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[0] === 1'b0, "STATUS[0] busy=0 after completion");
        check_cond(rd_data[1] === 1'b1, "STATUS[1] done_latched=1");
        check_cond(rd_data[2] === 1'b0, "STATUS[2] no error");

        verify_store_data(16, 0, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 4: Status Polling During Active DMA
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 4: Status Polling During Active DMA");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(32, 0);
        clear_dst_and_pfb();
        engine_proc_cycles = 20;  // long engine phase to allow polling

        configure_and_start(32'h0000_0000, 32'h0000_0000, 16'd32);

        // Poll status and expect busy=1 while running
        repeat(3) @(posedge clk);
        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[0] === 1'b1, "STATUS[0] busy=1 during active DMA");

        // Now wait for completion
        wait_dma_done_poll(50000);
        check_cond(!timed_out, "DMA eventually completed");

        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[1] === 1'b1, "STATUS[1] done_latched after completion");

        verify_store_data(32, 0, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 5: Done-Latch & Clear via CTRL Write
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 5: Done-Latch & Clear via CTRL Write");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(4, 0);
        clear_dst_and_pfb();
        engine_proc_cycles = 2;

        run_dma_via_wb(32'h0000_0000, 32'h0000_0000, 16'd4, 5000);
        check_cond(!timed_out, "DMA completed");

        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[1] === 1'b1, "done_latched=1 before clear");

        // Clear done latch by writing bit[1] to CTRL
        wb_write(ADDR_CTRL, 32'h0000_0002, 4'hF);
        @(posedge clk); #1;

        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[1] === 1'b0, "done_latched=0 after CTRL clear");
        check_cond(rd_data === 32'd0,   "STATUS fully clear after done latch cleared");

        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 6: Error-Latch & Clear (AXI read error)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 6: Error-Latch & Clear via CTRL Write (AXI read err)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(16, 0);
        clear_dst_and_pfb();
        inject_rresp_err = 1;
        engine_proc_cycles = 3;

        run_dma_via_wb(32'h0000_0000, 32'h0000_0000, 16'd16, 5000);
        check_cond(!timed_out, "DMA completed (with error) without timeout");

        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[2] === 1'b1, "STATUS[2] error latched after AXI read error");

        // Clear error latch: write bit[2] to CTRL
        wb_write(ADDR_CTRL, 32'h0000_0004, 4'hF);
        @(posedge clk); #1;

        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[2] === 1'b0, "err_latched=0 after CTRL clear");

        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 7: pixel_count = 0 (immediate done)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 7: Zero Pixel Count (immediate done)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        run_dma_via_wb(32'h0000_1000, 32'h0000_2000, 16'd0, 200);
        check_cond(!timed_out, "DMA completed for pixel_count=0");

        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[1] === 1'b1, "done_latched=1 for zero-pixel transfer");
        check_cond(rd_data[2] === 1'b0, "No error for zero-pixel transfer");

        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 8: pixel_count = 1 (single pixel)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 8: Single Pixel (pixel_count=1)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(1, 42);
        clear_dst_and_pfb();
        engine_proc_cycles = 2;

        run_dma_via_wb(32'h0000_0000, 32'h0000_0000, 16'd1, 5000);
        check_cond(!timed_out, "DMA completed for 1-pixel transfer");

        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[2] === 1'b0, "No error");

        verify_store_data(1, 42, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 9: pixel_count = 3 (less than one AXI word)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 9: Sub-Word Pixel Count (3 pixels)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(3, 100);
        clear_dst_and_pfb();
        engine_proc_cycles = 2;

        run_dma_via_wb(32'h0000_0000, 32'h0000_0000, 16'd3, 5000);
        check_cond(!timed_out, "DMA completed for 3-pixel transfer");

        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[2] === 1'b0, "No error");

        verify_store_data(3, 100, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 10: pixel_count = 7 (non-word-aligned, 2 AXI words)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 10: Non-Aligned Pixel Count (7 pixels)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(7, 10);
        clear_dst_and_pfb();
        engine_proc_cycles = 3;

        run_dma_via_wb(32'h0000_0000, 32'h0000_0000, 16'd7, 5000);
        check_cond(!timed_out, "DMA completed for 7-pixel transfer");

        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[2] === 1'b0, "No error");

        verify_store_data(7, 10, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 11: pixel_count = 64 (exact one-burst boundary)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 11: Exact Burst Boundary (64 pixels = 16 words)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(64, 0);
        clear_dst_and_pfb();
        engine_proc_cycles = 5;

        run_dma_via_wb(32'h0000_0000, 32'h0000_0000, 16'd64, 20000);
        check_cond(!timed_out, "DMA completed for 64-pixel transfer");

        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[2] === 1'b0, "No error");

        verify_store_data(64, 0, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 12: Multi-Burst (100 pixels = 25 words)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 12: Multi-Burst Transfer (100 pixels = 25 words)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(100, 42);
        clear_dst_and_pfb();
        engine_proc_cycles = 5;

        run_dma_via_wb(32'h0000_0000, 32'h0000_0000, 16'd100, 50000);
        check_cond(!timed_out, "DMA completed for 100-pixel transfer");

        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[2] === 1'b0, "No error");

        verify_store_data(100, 42, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 13: Large Transfer (256 pixels = 4 full bursts)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 13: Large Transfer (256 pixels = 4 full bursts)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(256, 0);
        clear_dst_and_pfb();
        engine_proc_cycles = 5;

        run_dma_via_wb(32'h0000_0000, 32'h0000_0000, 16'd256, 100000);
        check_cond(!timed_out, "DMA completed for 256-pixel transfer");

        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[2] === 1'b0, "No error");

        verify_store_data(256, 0, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 14: AXI Write Error (bresp=SLVERR)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 14: AXI Write Error (bresp=SLVERR)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(16, 0);
        clear_dst_and_pfb();
        inject_bresp_err = 1;
        engine_proc_cycles = 3;

        run_dma_via_wb(32'h0000_0000, 32'h0000_0000, 16'd16, 10000);
        check_cond(!timed_out, "DMA completed (with error) without timeout");

        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[2] === 1'b1, "STATUS[2] error latched after AXI write error");

        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 15: AXI Read Error then Re-run Clean
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 15: AXI Read Error -> Clear -> Re-run Clean");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(16, 0);
        clear_dst_and_pfb();
        inject_rresp_err = 1;
        engine_proc_cycles = 3;

        run_dma_via_wb(32'h0000_0000, 32'h0000_0000, 16'd16, 5000);
        check_cond(!timed_out, "DMA (error run) completed");

        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[2] === 1'b1, "Error latched after read error");

        // Clear latches
        wb_write(ADDR_CTRL, 32'h0000_0006, 4'hF);  // clear done + error
        @(posedge clk); #1;

        // Re-run with no error
        inject_rresp_err = 0;
        fill_src_mem(16, 50);
        clear_dst_and_pfb();

        run_dma_via_wb(32'h0000_0000, 32'h0000_0000, 16'd16, 10000);
        check_cond(!timed_out, "DMA (clean re-run) completed");

        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[2] === 1'b0, "No error on clean re-run");

        verify_store_data(16, 50, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 16: PFB Write Backpressure (LOAD phase)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 16: PFB Write Backpressure (slow wr_ready)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(16, 77);
        clear_dst_and_pfb();
        pfb_wr_ready_delay = 3;
        engine_proc_cycles = 3;

        run_dma_via_wb(32'h0000_0000, 32'h0000_0000, 16'd16, 20000);
        check_cond(!timed_out, "DMA completed with PFB write backpressure");

        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[2] === 1'b0, "No error");

        verify_store_data(16, 77, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 17: PFB Read Backpressure (STORE phase)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 17: PFB Read Backpressure (slow rd_ready)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(16, 33);
        clear_dst_and_pfb();
        pfb_rd_ready_delay = 3;
        engine_proc_cycles = 3;

        run_dma_via_wb(32'h0000_0000, 32'h0000_0000, 16'd16, 20000);
        check_cond(!timed_out, "DMA completed with PFB read backpressure");

        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[2] === 1'b0, "No error");

        verify_store_data(16, 33, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 18: Combined PFB Backpressure
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 18: Combined PFB Backpressure (WR+RD slow, 20 px)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(20, 88);
        clear_dst_and_pfb();
        pfb_wr_ready_delay = 2;
        pfb_rd_ready_delay = 2;
        engine_proc_cycles = 3;

        run_dma_via_wb(32'h0000_0000, 32'h0000_0000, 16'd20, 50000);
        check_cond(!timed_out, "DMA completed with combined backpressure");

        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[2] === 1'b0, "No error");

        verify_store_data(20, 88, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 19: AXI Slave Latency (delayed arready/awready)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 19: AXI Slave Latency (AR+AW accept delay=3)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(32, 0);
        clear_dst_and_pfb();
        axi_ar_accept_delay = 3;
        axi_aw_accept_delay = 3;
        engine_proc_cycles  = 3;

        run_dma_via_wb(32'h0000_0000, 32'h0000_0000, 16'd32, 30000);
        check_cond(!timed_out, "DMA completed with AXI slave latency");

        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[2] === 1'b0, "No error");

        verify_store_data(32, 0, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 20: Back-to-Back DMA Runs Without Reset
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 20: Back-to-Back DMA Runs Without Reset");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        // --- Run A: 8 pixels, seed=0 ---
        fill_src_mem(8, 0);
        clear_dst_and_pfb();
        engine_proc_cycles = 2;

        run_dma_via_wb(32'h0000_0000, 32'h0000_0000, 16'd8, 10000);
        check_cond(!timed_out, "Run A completed");

        verify_store_data(8, 0, 1);
        case_errors = case_errors + tmp_errs;

        // Clear done latch before next run
        wb_write(ADDR_CTRL, 32'h0000_0002, 4'hF);
        @(posedge clk); #1;

        // --- Run B: 12 pixels, seed=200 ---
        fill_src_mem(12, 200);
        clear_dst_and_pfb();
        engine_proc_cycles = 3;

        run_dma_via_wb(32'h0000_0000, 32'h0000_0000, 16'd12, 10000);
        check_cond(!timed_out, "Run B completed");

        verify_store_data(12, 200, 1);
        case_errors = case_errors + tmp_errs;

        // Clear done latch before next run
        wb_write(ADDR_CTRL, 32'h0000_0002, 4'hF);
        @(posedge clk); #1;

        // --- Run C: 20 pixels, seed=50 ---
        fill_src_mem(20, 50);
        clear_dst_and_pfb();
        engine_proc_cycles = 2;

        run_dma_via_wb(32'h0000_0000, 32'h0000_0000, 16'd20, 20000);
        check_cond(!timed_out, "Run C completed");

        verify_store_data(20, 50, 1);
        case_errors = case_errors + tmp_errs;

        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 21: Non-Zero Base Addresses
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 21: Non-Zero Base Addresses (offset src & dst)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem_at_offset(256, 16, 5);
        clear_dst_and_pfb();
        engine_proc_cycles = 3;

        run_dma_via_wb(32'h0000_0100, 32'h0000_0200, 16'd16, 10000);
        check_cond(!timed_out, "DMA completed with offset addresses");

        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data[2] === 1'b0, "No error");

        verify_store_data_at_offset(512, 16, 5, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 22: WB sel Gating - Start NOT Fired When sel[0]=0
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 22: WB sel Gating - No Start When sel[0]=0");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(4, 0);
        clear_dst_and_pfb();

        // Configure registers normally
        wb_write(ADDR_SRC,   32'h0000_0000, 4'hF);
        wb_write(ADDR_DST,   32'h0000_0000, 4'hF);
        wb_write(ADDR_COUNT, 32'h0000_0004, 4'hF);

        // Write CTRL with sel[0]=0 -> start should NOT fire
        wb_write(ADDR_CTRL, 32'h0000_0001, 4'hE);  // sel = 4'b1110

        // Allow a few cycles for any spurious activity
        repeat(10) @(posedge clk);
        #1;

        // STATUS should still be idle (no busy, no done)
        wb_read(ADDR_STATUS, rd_data);
        check_cond(rd_data === 32'd0, "STATUS idle - DMA not started when sel[0]=0");

        // Now start properly with sel[0]=1
        wb_write(ADDR_CTRL, 32'h0000_0001, 4'hF);
        engine_proc_cycles = 2;

        wait_dma_done_poll(5000);
        check_cond(!timed_out, "DMA completed after proper start");

        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 23: Write to Invalid CSR Address (no side-effect)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 23: Write to Invalid CSR Address");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        // Pre-load known values
        wb_write(ADDR_SRC,   32'hAAAA_AAAA, 4'hF);
        wb_write(ADDR_DST,   32'hBBBB_BBBB, 4'hF);
        wb_write(ADDR_COUNT, 32'h0000_5678, 4'hF);

        // Write to invalid address
        wb_write(ADDR_INVAL, 32'hFFFF_FFFF, 4'hF);

        // Verify nothing changed
        wb_read(ADDR_SRC, rd_data);
        check_cond(rd_data === 32'hAAAA_AAAA, "SRC unchanged after invalid write");
        wb_read(ADDR_DST, rd_data);
        check_cond(rd_data === 32'hBBBB_BBBB, "DST unchanged after invalid write");
        wb_read(ADDR_COUNT, rd_data);
        check_cond(rd_data === 32'h0000_5678, "COUNT unchanged after invalid write");

        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 24: Read from Invalid CSR Address
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 24: Read from Invalid CSR Address (returns 0)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        wb_read(ADDR_INVAL, rd_data);
        check_cond(rd_data === 32'd0, "Invalid register read returns 0x0000_0000");

        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 25: engine_start Pulse Verification
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 25: engine_start Pulse Verification");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(4, 0);
        clear_dst_and_pfb();
        engine_proc_cycles = 10;

        saw_engine_start = 0;

        configure_and_start(32'h0000_0000, 32'h0000_0000, 16'd4);

        // Monitor engine_start while waiting for done
        begin : wait_done_c25
            integer wcnt;
            reg [WB_DATA_W-1:0] stat;
            stat = 0;
            wcnt = 0;
            while (!stat[1] && (wcnt < 50000)) begin
                @(posedge clk); #1;
                if (engine_start) saw_engine_start = 1;
                // Poll occasionally
                if (wcnt % 20 == 0) begin
                    wb_read(ADDR_STATUS, stat);
                end
                wcnt = wcnt + 1;
            end
            if (!stat[1]) timed_out = 1;
            else          timed_out = 0;
        end

        check_cond(saw_engine_start == 1, "engine_start pulse observed during LOAD->ENGINE transition");
        check_cond(!timed_out,            "DMA completed");

        total_errors = total_errors + case_errors;

        // ============================================================
        //  FINAL SUMMARY
        // ============================================================
        $display("\n");
        $display("##########################################################");
        if (total_errors == 0) begin
            $display("#                                                        #");
            $display("#   ALL 25 TESTS PASSED!  (0 errors)                     #");
            $display("#                                                        #");
        end else begin
            $display("#                                                        #");
            $display("#   TESTS COMPLETED WITH %0d TOTAL ERROR(S)              #", total_errors);
            $display("#                                                        #");
        end
        $display("##########################################################\n");

        $finish;
    end

    // ================================================================
    //  OPTIONAL VERBOSE MONITOR  (enable with +define+DMA_CSR_TB_VERBOSE)
    // ================================================================
`ifdef DMA_CSR_TB_VERBOSE
    always @(posedge clk) begin
        if (rst_n) begin
            if (wb_ack_o)
                $display("  [WB] adr=0x%02h we=%0b dat_i=0x%08h dat_o=0x%08h  t=%0t",
                         wb_adr_i, wb_we_i, wb_dat_i, wb_dat_o, $time);
            if (m_axi_arvalid && m_axi_arready)
                $display("  [AXI-AR] addr=0x%08h len=%0d  t=%0t", m_axi_araddr, m_axi_arlen, $time);
            if (m_axi_rvalid && m_axi_rready)
                $display("  [AXI-R]  data=0x%08h last=%0b resp=%0b  t=%0t",
                         m_axi_rdata, m_axi_rlast, m_axi_rresp, $time);
            if (m_axi_awvalid && m_axi_awready)
                $display("  [AXI-AW] addr=0x%08h len=%0d  t=%0t", m_axi_awaddr, m_axi_awlen, $time);
            if (m_axi_wvalid && m_axi_wready)
                $display("  [AXI-W]  data=0x%08h strb=%04b last=%0b  t=%0t",
                         m_axi_wdata, m_axi_wstrb, m_axi_wlast, $time);
            if (m_axi_bvalid && m_axi_bready)
                $display("  [AXI-B]  resp=%0b  t=%0t", m_axi_bresp, $time);
            if (pfb_dma_wr_req && pfb_dma_wr_ready)
                $display("  [PFB-W]  addr=%0d data=0x%02h  t=%0t", pfb_dma_wr_addr, pfb_dma_wr_data, $time);
            if (pfb_dma_rd_valid)
                $display("  [PFB-R]  data=0x%02h  t=%0t", pfb_dma_rd_data, $time);
            if (engine_start)
                $display("  [ENG]    engine_start pulse  t=%0t", $time);
            if (engine_done)
                $display("  [ENG]    engine_done  pulse  t=%0t", $time);
        end
    end
`endif

endmodule
