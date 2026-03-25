`timescale 1ns / 1ps

// Testbench for dma_module
// Author: Junze Jiang
// Date: 17/3/2026

// Test Cases:
//   CASE 1 : Basic full flow (16 pixels, word-aligned)
//   CASE 2 : pixel_count = 0 (immediate done, no transfer)
//   CASE 3 : Pixel count not word-aligned (7 pixels)
//   CASE 4 : Multi-burst transfer (100 pixels = 25 words > 16 max burst)
//   CASE 5 : AXI read error response (rresp != OKAY)
//   CASE 6 : AXI write error response (bresp != OKAY)
//   CASE 7 : PFB write backpressure (pfb_dma_wr_ready delayed)
//   CASE 8 : PFB read backpressure (pfb_dma_rd_ready delayed)
//   CASE 9 : AXI transfer with default slave latencies (32 pixels)
//   CASE 10: Back-to-back runs without reset
//   CASE 11: Exact one-burst boundary (64 pixels = 16 words)
//   CASE 12: Single pixel transfer (pixel_count = 1)
//   CASE 13: Non-zero base addresses (offset test)
//   CASE 14: Large transfer (256 pixels = 4 full bursts)
//   CASE 15: Combined PFB backpressure (WR + RD both slow)
//   CASE 16: Busy / Done signal timing verification
//   CASE 17: Engine start pulse verification

module tb_dma_module();

    // ---- Parameters ----
    parameter AXI_ADDR_W      = 32;
    parameter AXI_DATA_W      = 32;
    parameter PFB_ADDR_W      = 14;
    parameter PIXELS_MAX_W    = 16;
    parameter MAX_BURST_BEATS = 16;

    localparam BYTES_PER_BEAT = AXI_DATA_W / 8;    // 4
    localparam CLK_P          = 10;                 // 100 MHz

    // ---- Memory Model Sizes ----
    localparam AXI_MEM_DEPTH = 65536;   // bytes
    localparam PFB_MEM_DEPTH = 16384;   // pixels

    // ================================================================
    //  DUT SIGNAL DECLARATIONS
    // ================================================================
    reg                         clk;
    reg                         rst_n;

    reg                         start;
    reg  [AXI_ADDR_W-1:0]      src_base_addr;
    reg  [AXI_ADDR_W-1:0]      dst_base_addr;
    reg  [PIXELS_MAX_W-1:0]    pixel_count;
    wire                        busy;
    wire                        done;
    wire                        error;

    wire                        engine_start;
    reg                         engine_done;

    wire                        pfb_dma_rd_req;
    wire [PFB_ADDR_W-1:0]      pfb_dma_rd_addr;
    reg  [7:0]                  pfb_dma_rd_data;
    reg                         pfb_dma_rd_valid;
    reg                         pfb_dma_rd_ready;

    wire                        pfb_dma_wr_req;
    wire [PFB_ADDR_W-1:0]      pfb_dma_wr_addr;
    wire [7:0]                  pfb_dma_wr_data;
    reg                         pfb_dma_wr_ready;

    wire [AXI_ADDR_W-1:0]      m_axi_araddr;
    wire [7:0]                  m_axi_arlen;
    wire [2:0]                  m_axi_arsize;
    wire [1:0]                  m_axi_arburst;
    wire                        m_axi_arvalid;
    reg                         m_axi_arready;

    reg  [AXI_DATA_W-1:0]      m_axi_rdata;
    reg  [1:0]                  m_axi_rresp;
    reg                         m_axi_rlast;
    reg                         m_axi_rvalid;
    wire                        m_axi_rready;

    wire [AXI_ADDR_W-1:0]      m_axi_awaddr;
    wire [7:0]                  m_axi_awlen;
    wire [2:0]                  m_axi_awsize;
    wire [1:0]                  m_axi_awburst;
    wire                        m_axi_awvalid;
    reg                         m_axi_awready;

    wire [AXI_DATA_W-1:0]      m_axi_wdata;
    wire [AXI_DATA_W/8-1:0]    m_axi_wstrb;
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
    //  AXI READ SLAVE STATE   [FIX-1]
    // ================================================================
    reg  [AXI_ADDR_W-1:0]  ar_addr_latched;
    reg  [7:0]              ar_len_latched;
    reg  [7:0]              ar_beat_cnt;
    reg                     ar_active;
    reg                     r_beat_pending;

    // ================================================================
    //  AXI WRITE SLAVE STATE
    // ================================================================
    reg  [AXI_ADDR_W-1:0]  aw_addr_latched;
    reg  [7:0]              aw_len_latched;
    reg  [7:0]              aw_beat_cnt;
    reg                     aw_active;

    // ================================================================
    //  TEST CONTROL VARIABLES
    // ================================================================
    integer total_errors;
    integer case_errors;
    integer i;
    integer timed_out;
    integer tmp_errs;
    integer saw_engine_start;
    reg     latched_error;          // [FIX-2]

    // ================================================================
    //  CONFIGURABLE BEHAVIOUR KNOBS
    // ================================================================
    reg                     inject_rresp_err;
    reg                     inject_bresp_err;
    reg                     inject_rlast_mismatch;
    integer                 pfb_wr_ready_delay;
    integer                 pfb_rd_ready_delay;
    integer                 engine_proc_cycles;

    // ================================================================
    //  DUT INSTANTIATION
    // ================================================================
    dma_module #(
        .AXI_ADDR_W      (AXI_ADDR_W),
        .AXI_DATA_W      (AXI_DATA_W),
        .PFB_ADDR_W      (PFB_ADDR_W),
        .PIXELS_MAX_W    (PIXELS_MAX_W),
        .MAX_BURST_BEATS (MAX_BURST_BEATS)
    ) dut (
        .clk                (clk),
        .rst_n              (rst_n),
        .start              (start),
        .src_base_addr      (src_base_addr),
        .dst_base_addr      (dst_base_addr),
        .pixel_count        (pixel_count),
        .busy               (busy),
        .done               (done),
        .error              (error),
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

    // ================================================================
    //  CLOCK GENERATION
    // ================================================================
    initial begin
        clk = 0;
        forever #(CLK_P / 2) clk = ~clk;
    end

    // ================================================================
    //  TIMEOUT WATCHDOG
    // ================================================================
    initial begin
        #(CLK_P * 500000);
        $display("\n[FATAL] Global timeout �? simulation killed at time %0t.", $time);
        $finish;
    end

    // ================================================================
    //  [FIX-1] AXI READ SLAVE MODEL  �?  COMPLETE REWRITE
    //
    //  Protocol:
    //    1. AR handshake -> latch addr/len, mark first beat pending
    //    2. When beat is pending & rvalid is low: load fresh rdata/rlast/rresp
    //       for current ar_beat_cnt, assert rvalid.
    //    3. R handshake (rvalid & rready): deassert rvalid.
    //       - Last beat:  deactivate burst.
    //       - Otherwise:  increment beat counter, mark next beat pending.
    //
    //  This guarantees rdata is ALWAYS fresh before the handshake.
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_axi_arready   <= 1'b0;
            m_axi_rdata     <= {AXI_DATA_W{1'b0}};
            m_axi_rresp     <= 2'b00;
            m_axi_rlast     <= 1'b0;
            m_axi_rvalid    <= 1'b0;
            ar_addr_latched  <= {AXI_ADDR_W{1'b0}};
            ar_len_latched   <= 8'd0;
            ar_beat_cnt      <= 8'd0;
            ar_active        <= 1'b0;
            r_beat_pending   <= 1'b0;
        end else begin

            // ---- AR channel ----
            m_axi_arready <= (m_axi_arvalid && !ar_active) ? 1'b1 : 1'b0;

            if (m_axi_arvalid && m_axi_arready) begin
                ar_addr_latched <= m_axi_araddr;
                ar_len_latched  <= m_axi_arlen;
                ar_beat_cnt     <= 8'd0;
                ar_active       <= 1'b1;
                r_beat_pending  <= 1'b1;
                m_axi_arready   <= 1'b0;
                m_axi_rvalid    <= 1'b0;
            end

            // ---- Present beat data (only when pending and rvalid is low) ----
            if (ar_active && r_beat_pending && !m_axi_rvalid) begin
                m_axi_rdata <= {
                    axi_src_mem[ar_addr_latched + ar_beat_cnt * BYTES_PER_BEAT + 3],
                    axi_src_mem[ar_addr_latched + ar_beat_cnt * BYTES_PER_BEAT + 2],
                    axi_src_mem[ar_addr_latched + ar_beat_cnt * BYTES_PER_BEAT + 1],
                    axi_src_mem[ar_addr_latched + ar_beat_cnt * BYTES_PER_BEAT + 0]
                };
                m_axi_rvalid <= 1'b1;
                m_axi_rresp  <= inject_rresp_err ? 2'b10 : 2'b00;
                m_axi_rlast  <= (ar_beat_cnt == ar_len_latched) && !inject_rlast_mismatch;
                r_beat_pending <= 1'b0;
            end

            // ---- R handshake: master consumed the beat ----
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
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_axi_awready   <= 1'b0;
            m_axi_wready    <= 1'b0;
            m_axi_bresp     <= 2'b00;
            m_axi_bvalid    <= 1'b0;
            aw_addr_latched  <= {AXI_ADDR_W{1'b0}};
            aw_len_latched   <= 8'd0;
            aw_beat_cnt      <= 8'd0;
            aw_active        <= 1'b0;
        end else begin
            // AW handshake
            if (m_axi_awvalid && !aw_active && !m_axi_bvalid) begin
                m_axi_awready <= 1'b1;
            end else begin
                m_axi_awready <= 1'b0;
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
    //  PFB WRITE MODEL
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
    //  PFB READ MODEL
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
                for (i = 0; i < PFB_MEM_DEPTH; i = i + 1) begin
                    pfb_mem[i] <= ~pfb_mem[i];
                end
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
    //  [FIX-2] LATCH ERROR FLAG
    //  The DMA clears `error` when it returns to S_IDLE (one cycle
    //  after done).  This register captures it reliably.
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            latched_error <= 1'b0;
        end else begin
            if (done)
                latched_error <= error;
            else if (start)
                latched_error <= 1'b0;
        end
    end

    // ================================================================
    //  REUSABLE TASKS
    // ================================================================

    task do_reset;
        begin
            rst_n = 0;
            start = 0;
            src_base_addr = {AXI_ADDR_W{1'b0}};
            dst_base_addr = {AXI_ADDR_W{1'b0}};
            pixel_count   = {PIXELS_MAX_W{1'b0}};

            inject_rresp_err      = 0;
            inject_bresp_err      = 0;
            inject_rlast_mismatch = 0;
            pfb_wr_ready_delay    = 0;
            pfb_rd_ready_delay    = 0;
            engine_proc_cycles    = 5;

            #(CLK_P * 3);
            rst_n = 1;
            #(CLK_P * 2);
        end
    endtask

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

    task clear_dst_and_pfb;
        integer idx;
        begin
            for (idx = 0; idx < AXI_MEM_DEPTH; idx = idx + 1)
                axi_dst_mem[idx] = 8'hXX;
            for (idx = 0; idx < PFB_MEM_DEPTH; idx = idx + 1)
                pfb_mem[idx] = 8'h00;
        end
    endtask

    // [FIX-2] Removed extra @(posedge clk); use #1 settle only
    task run_dma;
        input integer timeout_cycles;
        integer cnt;
        begin
            timed_out = 0;
            @(posedge clk); #1;
            start = 1;
            @(posedge clk); #1;
            start = 0;

            cnt = 0;
            while (!done && (cnt < timeout_cycles)) begin
                @(posedge clk);
                cnt = cnt + 1;
            end
            if (!done) timed_out = 1;
            #1;
            // Now: done=1 and error are both still valid.
            // latched_error also captured error on this same edge.
            // Allow one more cycle for latched_error to settle
            @(posedge clk); #1;
        end
    endtask

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

    task check_cond;
        input condition;
        input [800:0] msg;
        begin
            if (!condition) begin
                $display("    [FAIL] %0s", msg);
                case_errors = case_errors + 1;
            end else begin
                $display("    [PASS] %0s", msg);
            end
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
        $display("#     DMA MODULE TESTBENCH (Verilog-2001)  [FIXED]       #");
        $display("#     AXI_DATA_W=%0d  BYTES_PER_BEAT=%0d  MAX_BURST=%0d  #",
                 AXI_DATA_W, BYTES_PER_BEAT, MAX_BURST_BEATS);
        $display("##########################################################\n");

        // ============================================================
        //  CASE 1: Basic Full Flow (16 pixels, word-aligned)
        // ============================================================
        $display("==========================================================");
        $display(" CASE 1: Basic Full Flow (16 pixels, word-aligned)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(16, 0);
        clear_dst_and_pfb();

        src_base_addr = 32'h0000_0000;
        dst_base_addr = 32'h0000_0000;
        pixel_count   = 16'd16;
        engine_proc_cycles = 5;

        run_dma(5000);
        check_cond(!timed_out,              "DMA completed without timeout");
        check_cond(latched_error === 1'b0,  "No error flag raised");
        check_cond(busy === 1'b0,           "Busy deasserted after done");

        verify_store_data(16, 0, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 2: pixel_count = 0 (immediate done)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 2: Zero Pixel Count (immediate done)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        src_base_addr = 32'h0000_1000;
        dst_base_addr = 32'h0000_2000;
        pixel_count   = 16'd0;

        run_dma(100);
        check_cond(!timed_out,              "DMA completed without timeout");
        check_cond(latched_error === 1'b0,  "No error flag raised");
        $display("    [INFO] pixel_count=0 correctly goes straight to S_DONE.");
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 3: Non-aligned Pixel Count (7 pixels)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 3: Non-Aligned Pixel Count (7 pixels)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(7, 10);
        clear_dst_and_pfb();

        src_base_addr = 32'h0000_0000;
        dst_base_addr = 32'h0000_0000;
        pixel_count   = 16'd7;
        engine_proc_cycles = 3;

        run_dma(5000);
        check_cond(!timed_out,              "DMA completed without timeout");
        check_cond(latched_error === 1'b0,  "No error flag raised");

        verify_store_data(7, 10, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 4: Multi-Burst (100 pixels = 25 words > 16 max burst)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 4: Multi-Burst Transfer (100 pixels = 25 words)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(100, 42);
        clear_dst_and_pfb();

        src_base_addr = 32'h0000_0000;
        dst_base_addr = 32'h0000_0000;
        pixel_count   = 16'd100;
        engine_proc_cycles = 5;

        run_dma(50000);
        check_cond(!timed_out,              "DMA completed without timeout");
        check_cond(latched_error === 1'b0,  "No error flag raised");

        verify_store_data(100, 42, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 5: AXI Read Error Response (rresp=SLVERR)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 5: AXI Read Error Response (rresp=SLVERR)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(16, 0);
        clear_dst_and_pfb();

        src_base_addr      = 32'h0000_0000;
        dst_base_addr      = 32'h0000_0000;
        pixel_count        = 16'd16;
        inject_rresp_err   = 1;

        run_dma(5000);
        check_cond(!timed_out,              "DMA completed (with error) without timeout");
        check_cond(latched_error === 1'b1,  "Error flag correctly raised on rresp error");
        check_cond(busy === 1'b0,           "Busy deasserted after error");

        inject_rresp_err = 0;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 6: AXI Write Error Response (bresp=SLVERR)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 6: AXI Write Error Response (bresp=SLVERR)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(16, 0);
        clear_dst_and_pfb();

        src_base_addr      = 32'h0000_0000;
        dst_base_addr      = 32'h0000_0000;
        pixel_count        = 16'd16;
        inject_bresp_err   = 1;
        engine_proc_cycles = 3;

        run_dma(10000);
        check_cond(!timed_out,              "DMA completed (with error) without timeout");
        check_cond(latched_error === 1'b1,  "Error flag correctly raised on bresp error");

        inject_bresp_err = 0;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 7: PFB Write Backpressure (3-cycle delay during load)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 7: PFB Write Backpressure (3-cycle delay)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(16, 77);
        clear_dst_and_pfb();

        src_base_addr      = 32'h0000_0000;
        dst_base_addr      = 32'h0000_0000;
        pixel_count        = 16'd16;
        pfb_wr_ready_delay = 3;
        engine_proc_cycles = 3;

        run_dma(20000);
        check_cond(!timed_out,              "DMA completed with PFB write backpressure");
        check_cond(latched_error === 1'b0,  "No error flag raised");

        verify_store_data(16, 77, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 8: PFB Read Backpressure (3-cycle delay during store)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 8: PFB Read Backpressure (3-cycle delay)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(20, 55);
        clear_dst_and_pfb();

        src_base_addr      = 32'h0000_0000;
        dst_base_addr      = 32'h0000_0000;
        pixel_count        = 16'd20;
        pfb_rd_ready_delay = 3;
        engine_proc_cycles = 3;

        run_dma(30000);
        check_cond(!timed_out,              "DMA completed with PFB read backpressure");
        check_cond(latched_error === 1'b0,  "No error flag raised");

        verify_store_data(20, 55, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 9: Larger Transfer with Default Latencies (32 pixels)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 9: AXI Transfer with Default Slave Latencies (32px)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(32, 99);
        clear_dst_and_pfb();

        src_base_addr      = 32'h0000_0000;
        dst_base_addr      = 32'h0000_0000;
        pixel_count        = 16'd32;
        engine_proc_cycles = 2;

        run_dma(20000);
        check_cond(!timed_out,              "DMA completed with default AXI latencies");
        check_cond(latched_error === 1'b0,  "No error flag raised");

        verify_store_data(32, 99, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 10: Back-to-Back Runs Without Full Reset
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 10: Back-to-Back Runs (no reset between)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        // --- Run A ---
        fill_src_mem(8, 11);
        clear_dst_and_pfb();
        src_base_addr = 32'h0000_0000;
        dst_base_addr = 32'h0000_0000;
        pixel_count   = 16'd8;
        engine_proc_cycles = 3;

        run_dma(5000);
        check_cond(!timed_out && (latched_error === 1'b0), "Run A completed OK");
        verify_store_data(8, 11, 1);
        case_errors = case_errors + tmp_errs;

        // --- Run B (no reset!) ---
        $display("    [INFO] Starting Run B without reset...");
        fill_src_mem(12, 22);
        clear_dst_and_pfb();
        src_base_addr = 32'h0000_0000;
        dst_base_addr = 32'h0000_0000;
        pixel_count   = 16'd12;

        run_dma(5000);
        check_cond(!timed_out && (latched_error === 1'b0), "Run B completed OK");
        verify_store_data(12, 22, 1);
        case_errors = case_errors + tmp_errs;

        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 11: Exact One-Burst Boundary (64 pixels = 16 words)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 11: Exact Burst Boundary (64 pixels = 16 words)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(64, 33);
        clear_dst_and_pfb();

        src_base_addr = 32'h0000_0000;
        dst_base_addr = 32'h0000_0000;
        pixel_count   = 16'd64;
        engine_proc_cycles = 3;

        run_dma(20000);
        check_cond(!timed_out,              "DMA completed at exact burst boundary");
        check_cond(latched_error === 1'b0,  "No error flag raised");

        verify_store_data(64, 33, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 12: Single Pixel Transfer (pixel_count=1)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 12: Single Pixel Transfer (pixel_count=1)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(1, 200);
        clear_dst_and_pfb();

        src_base_addr = 32'h0000_0000;
        dst_base_addr = 32'h0000_0000;
        pixel_count   = 16'd1;
        engine_proc_cycles = 2;

        run_dma(5000);
        check_cond(!timed_out,              "DMA completed for single pixel");
        check_cond(latched_error === 1'b0,  "No error flag raised");

        verify_store_data(1, 200, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 13: Non-Zero Base Addresses (offset test)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 13: Non-Zero Base Addresses");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem_at_offset(256, 16, 5);
        clear_dst_and_pfb();

        src_base_addr = 32'h0000_0100;
        dst_base_addr = 32'h0000_0200;
        pixel_count   = 16'd16;
        engine_proc_cycles = 3;

        run_dma(10000);
        check_cond(!timed_out,              "DMA completed with offset addresses");
        check_cond(latched_error === 1'b0,  "No error flag raised");

        verify_store_data_at_offset(512, 16, 5, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 14: Large Transfer (256 pixels = 64 words = 4 bursts)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 14: Large Transfer (256 pixels = 4 full bursts)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(256, 0);
        clear_dst_and_pfb();

        src_base_addr = 32'h0000_0000;
        dst_base_addr = 32'h0000_0000;
        pixel_count   = 16'd256;
        engine_proc_cycles = 5;

        run_dma(100000);
        check_cond(!timed_out,              "DMA completed for 256-pixel transfer");
        check_cond(latched_error === 1'b0,  "No error flag raised");

        verify_store_data(256, 0, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 15: Combined PFB Backpressure (WR + RD both slow)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 15: Combined PFB Backpressure (WR+RD slow, 20px)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(20, 88);
        clear_dst_and_pfb();

        src_base_addr      = 32'h0000_0000;
        dst_base_addr      = 32'h0000_0000;
        pixel_count        = 16'd20;
        pfb_wr_ready_delay = 2;
        pfb_rd_ready_delay = 2;
        engine_proc_cycles = 3;

        run_dma(50000);
        check_cond(!timed_out,              "DMA completed under combined backpressure");
        check_cond(latched_error === 1'b0,  "No error flag raised");

        verify_store_data(20, 88, 1);
        case_errors = case_errors + tmp_errs;
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 16: Busy / Done Signal Timing Verification
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 16: Busy / Done Signal Timing");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(4, 0);
        clear_dst_and_pfb();

        src_base_addr = 32'h0000_0000;
        dst_base_addr = 32'h0000_0000;
        pixel_count   = 16'd4;
        engine_proc_cycles = 2;

        check_cond(busy === 1'b0, "Busy is low before start");
        check_cond(done === 1'b0, "Done is low before start");

        @(posedge clk); #1;
        start = 1;
        @(posedge clk); #1;
        start = 0;

        @(posedge clk); #1;
        check_cond(busy === 1'b1, "Busy asserted after start");

        // Wait for done
        begin : wait_done_c16
            integer wcnt;
            wcnt = 0;
            while (!done && (wcnt < 10000)) begin
                @(posedge clk); #1;
                wcnt = wcnt + 1;
            end
        end
        #1;     // settle �? done is valid here
        check_cond(done === 1'b1,  "Done asserted");

        @(posedge clk); #1;
        check_cond(busy === 1'b0,  "Busy deasserted after done");
        check_cond(done === 1'b0,  "Done is one-shot pulse (deasserted next cycle)");

        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 17: Engine Start Pulse Verification
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 17: Engine Start Pulse Verification");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        fill_src_mem(4, 0);
        clear_dst_and_pfb();

        src_base_addr = 32'h0000_0000;
        dst_base_addr = 32'h0000_0000;
        pixel_count   = 16'd4;
        engine_proc_cycles = 10;

        saw_engine_start = 0;

        @(posedge clk); #1;
        start = 1;
        @(posedge clk); #1;
        start = 0;

        begin : wait_done_c17
            integer wcnt;
            wcnt = 0;
            while (!done && (wcnt < 50000)) begin
                @(posedge clk); #1;
                if (engine_start) saw_engine_start = 1;
                wcnt = wcnt + 1;
            end
        end

        check_cond(saw_engine_start == 1, "engine_start pulse was observed during load->engine transition");
        check_cond(done === 1'b1,         "DMA eventually completed");

        total_errors = total_errors + case_errors;

        // ============================================================
        //  FINAL SUMMARY
        // ============================================================
        $display("\n");
        $display("##########################################################");
        if (total_errors == 0) begin
            $display("#   ALL TESTS PASSED! (0 errors across 17 cases)        #");
        end else begin
            $display("#   TESTS COMPLETED WITH %0d TOTAL ERRORS               #", total_errors);
        end
        $display("##########################################################\n");

        $finish;
    end

    // ================================================================
    //  OPTIONAL VERBOSE MONITOR (enable with +define+DMA_TB_VERBOSE)
    // ================================================================
`ifdef DMA_TB_VERBOSE
    always @(posedge clk) begin
        if (rst_n) begin
            if (m_axi_arvalid && m_axi_arready)
                $display("  [AXI-AR] addr=0x%08h len=%0d  t=%0t", m_axi_araddr, m_axi_arlen, $time);
            if (m_axi_rvalid && m_axi_rready)
                $display("  [AXI-R]  data=0x%08h last=%0b resp=%0b  t=%0t", m_axi_rdata, m_axi_rlast, m_axi_rresp, $time);
            if (m_axi_awvalid && m_axi_awready)
                $display("  [AXI-AW] addr=0x%08h len=%0d  t=%0t", m_axi_awaddr, m_axi_awlen, $time);
            if (m_axi_wvalid && m_axi_wready)
                $display("  [AXI-W]  data=0x%08h strb=%04b last=%0b  t=%0t", m_axi_wdata, m_axi_wstrb, m_axi_wlast, $time);
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
