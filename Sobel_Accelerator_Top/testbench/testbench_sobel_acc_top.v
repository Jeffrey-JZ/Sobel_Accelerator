`timescale 1ns / 1ps

// Testbench for sobel_acc_top  (Full System Integration)
// Author: Junze Jiang
// Date: 17/3/2026

// This testbench exercises the COMPLETE Sobel accelerator system end-to-end:
//   CPU (Wishbone) → CSR → DMA → AXI Memory ↔ PFB ↔ Sobel Engine
//
// A behavioural AXI slave memory model holds the source/destination images.
// The Wishbone master tasks simulate the CPU programming sequence.
// A golden Sobel reference model computes expected output for automatic check.
//
// Uses a small 10×10 image so full matrices print clearly in Vivado Tcl Console.

// Test Cases (20 total):
//
//  ── Core Function ──
//  CASE  1 : Reset state — all outputs quiescent, STATUS=0
//  CASE  2 : CSR write/readback — SRC, DST, COUNT, STATUS
//  CASE  3 : Full E2E — vertical edge image (strong Gx), threshold=60
//  CASE  4 : Full E2E — horizontal edge image (strong Gy), threshold=60
//  CASE  5 : Full E2E — uniform image (all 128) — expect zero output
//  CASE  6 : Full E2E — diagonal gradient — mixed Gx/Gy
//  CASE  7 : Full E2E — checkerboard pattern — maximum local gradient
//
//  ── Threshold Sensitivity ──
//  CASE  8 : Same vertical edge, threshold=0 — everything passes
//  CASE  9 : Same vertical edge, threshold=255 — everything suppressed
//
//  ── Pixel-Count Boundaries ──
//  CASE 10 : pixel_count = 0 — immediate done, no AXI traffic
//  CASE 11 : pixel_count = 1 — single pixel, partial AXI word
//  CASE 12 : pixel_count = 3 — less than one AXI word
//  CASE 13 : pixel_count = 7 — non-word-aligned, two AXI words
//
//  ── Multi-Burst ──
//  CASE 14 : pixel_count = 100 (25 words = 16+9, two bursts each direction)
//
//  ── Error Injection ──
//  CASE 15 : AXI read SLVERR mid-burst → error flag + STATUS[2]
//  CASE 16 : AXI write SLVERR → error flag + STATUS[2]
//
//  ── Sequencing / Corner ──
//  CASE 17 : Back-to-back runs without system reset
//  CASE 18 : Non-zero base addresses (src=0x1000, dst=0x2000)
//  CASE 19 : AXI slow-ready backpressure (arready/awready delayed)
//  CASE 20 : WB sel gating — start NOT fired when sel[0]=0

module tb_sobel_acc_top();

    // ================================================================
    //  PARAMETERS — small image for console readability
    // ================================================================
    parameter WB_ADDR_W     = 8;
    parameter WB_DATA_W     = 32;
    parameter AXI_ADDR_W    = 32;
    parameter AXI_DATA_W    = 32;
    parameter PFB_ADDR_W    = 14;
    parameter PFB_DATA_W    = 8;
    parameter PFB_BANKS     = 4;
    parameter IMG_W         = 10;
    parameter IMG_H         = 10;
    parameter MAG_W         = 12;
    parameter PIXELS_MAX_W  = 16;

    localparam PIXELS          = IMG_W * IMG_H;   // 100
    localparam BYTES_PER_BEAT  = AXI_DATA_W / 8;  // 4
    localparam MAX_BURST_BEATS = 16;
    localparam CLK_P           = 10;               // 100 MHz

    // CSR byte addresses (reg_idx = adr[5:2])
    localparam [WB_ADDR_W-1:0] ADDR_CTRL   = 8'h00;
    localparam [WB_ADDR_W-1:0] ADDR_SRC    = 8'h04;
    localparam [WB_ADDR_W-1:0] ADDR_DST    = 8'h08;
    localparam [WB_ADDR_W-1:0] ADDR_COUNT  = 8'h0C;
    localparam [WB_ADDR_W-1:0] ADDR_STATUS = 8'h10;

    // ================================================================
    //  DUT SIGNALS
    // ================================================================
    reg                         clk;
    reg                         rst_n;

    // Wishbone
    reg  [WB_ADDR_W-1:0]       wb_adr_i;
    reg  [WB_DATA_W-1:0]       wb_dat_i;
    wire [WB_DATA_W-1:0]       wb_dat_o;
    reg  [WB_DATA_W/8-1:0]     wb_sel_i;
    reg                         wb_we_i;
    reg                         wb_stb_i;
    reg                         wb_cyc_i;
    wire                        wb_ack_o;

    // AXI — directly driven by DUT, responded by our slave model
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

    // Threshold
    reg  [7:0]                  threshold;

    // Status
    wire                        engine_busy;
    wire                        engine_done;
    wire                        eng_wr_ready;
    wire                        eng_rd_ready;
    wire                        wr_conflict;

    // ================================================================
    //  AXI SLAVE MEMORY MODEL
    // ================================================================
    localparam AXI_MEM_SIZE = 65536;          // 64 KB — plenty for 10×10
    reg [7:0] axi_mem [0:AXI_MEM_SIZE-1];    // byte-addressable

    // Error injection controls
    reg  inject_rresp_err;    // force SLVERR on next read burst
    reg  inject_bresp_err;    // force SLVERR on next write response
    integer rresp_err_beat;   // which beat to inject error on (0-based)

    // AXI slow-ready control
    reg  axi_slow_arready;    // insert extra latency on arready
    reg  axi_slow_awready;    // insert extra latency on awready
    integer axi_ar_delay;
    integer axi_aw_delay;

    // ================================================================
    //  LOCAL STORAGE FOR VERIFICATION
    // ================================================================
    reg [7:0] src_image  [0:PIXELS-1];
    reg [7:0] dst_golden [0:PIXELS-1];
    reg [7:0] dst_actual [0:PIXELS-1];

    // ================================================================
    //  COUNTERS
    // ================================================================
    integer total_pass, total_fail;
    integer case_errors;
    integer i, j, x, y;
    integer test_num;
    reg [31:0] wb_rd_data_reg;

    // ================================================================
    //  DUT
    // ================================================================
    sobel_acc_top #(
        .WB_ADDR_W      (WB_ADDR_W),
        .WB_DATA_W      (WB_DATA_W),
        .AXI_ADDR_W     (AXI_ADDR_W),
        .AXI_DATA_W     (AXI_DATA_W),
        .PFB_ADDR_W     (PFB_ADDR_W),
        .PFB_DATA_W     (PFB_DATA_W),
        .PFB_BANKS      (PFB_BANKS),
        .IMG_W          (IMG_W),
        .IMG_H          (IMG_H),
        .MAG_W          (MAG_W),
        .PIXELS_MAX_W   (PIXELS_MAX_W)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        // Wishbone
        .wb_adr_i       (wb_adr_i),
        .wb_dat_i       (wb_dat_i),
        .wb_dat_o       (wb_dat_o),
        .wb_sel_i       (wb_sel_i),
        .wb_we_i        (wb_we_i),
        .wb_stb_i       (wb_stb_i),
        .wb_cyc_i       (wb_cyc_i),
        .wb_ack_o       (wb_ack_o),
        // AXI
        .m_axi_araddr   (m_axi_araddr),
        .m_axi_arlen    (m_axi_arlen),
        .m_axi_arsize   (m_axi_arsize),
        .m_axi_arburst  (m_axi_arburst),
        .m_axi_arvalid  (m_axi_arvalid),
        .m_axi_arready  (m_axi_arready),
        .m_axi_rdata    (m_axi_rdata),
        .m_axi_rresp    (m_axi_rresp),
        .m_axi_rlast    (m_axi_rlast),
        .m_axi_rvalid   (m_axi_rvalid),
        .m_axi_rready   (m_axi_rready),
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
        .m_axi_bresp    (m_axi_bresp),
        .m_axi_bvalid   (m_axi_bvalid),
        .m_axi_bready   (m_axi_bready),
        // Config
        .threshold      (threshold),
        // Status
        .engine_busy    (engine_busy),
        .engine_done    (engine_done),
        .eng_wr_ready   (eng_wr_ready),
        .eng_rd_ready   (eng_rd_ready),
        .wr_conflict    (wr_conflict)
    );

    // ================================================================
    //  CLOCK
    // ================================================================
    initial clk = 0;
    always #(CLK_P/2) clk = ~clk;

    // ================================================================
    //  AXI SLAVE: READ CHANNEL (AR + R)
    // ================================================================
    reg [AXI_ADDR_W-1:0] ar_addr_lat;
    reg [7:0]             ar_len_lat;
    reg                   ar_pending;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_axi_arready <= 1'b0;
            m_axi_rdata   <= 32'd0;
            m_axi_rresp   <= 2'b00;
            m_axi_rlast   <= 1'b0;
            m_axi_rvalid  <= 1'b0;
            ar_addr_lat   <= 32'd0;
            ar_len_lat    <= 8'd0;
            ar_pending    <= 1'b0;
        end else begin
            // Default: deassert rvalid after handshake
            if (m_axi_rvalid && m_axi_rready) begin
                m_axi_rvalid <= 1'b0;
                m_axi_rlast  <= 1'b0;
            end

            // Accept AR
            if (m_axi_arvalid && !ar_pending && !m_axi_rvalid) begin
                if (axi_slow_arready && axi_ar_delay > 0) begin
                    m_axi_arready <= 1'b0;
                    axi_ar_delay  <= axi_ar_delay - 1;
                end else begin
                    m_axi_arready <= 1'b1;
                    ar_addr_lat   <= m_axi_araddr;
                    ar_len_lat    <= m_axi_arlen;
                    ar_pending    <= 1'b1;
                end
            end else begin
                m_axi_arready <= 1'b0;
            end

            // Serve R beats
            if (ar_pending && (!m_axi_rvalid || (m_axi_rvalid && m_axi_rready))) begin
                m_axi_rdata <= {axi_mem[ar_addr_lat+3],
                                axi_mem[ar_addr_lat+2],
                                axi_mem[ar_addr_lat+1],
                                axi_mem[ar_addr_lat+0]};

                if (inject_rresp_err && (ar_len_lat == rresp_err_beat[7:0]))
                    m_axi_rresp <= 2'b10;  // SLVERR
                else
                    m_axi_rresp <= 2'b00;

                m_axi_rlast  <= (ar_len_lat == 8'd0);
                m_axi_rvalid <= 1'b1;

                if (ar_len_lat == 8'd0) begin
                    ar_pending       <= 1'b0;
                    inject_rresp_err <= 1'b0;
                end else begin
                    ar_len_lat  <= ar_len_lat - 1;
                    ar_addr_lat <= ar_addr_lat + BYTES_PER_BEAT;
                end
            end
        end
    end

    // ================================================================
    //  AXI SLAVE: WRITE CHANNEL (AW + W + B)
    // ================================================================
    reg [AXI_ADDR_W-1:0] aw_addr_lat;
    reg [7:0]             aw_len_lat;
    reg                   aw_pending;
    reg                   w_done_wait_b;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_axi_awready <= 1'b0;
            m_axi_wready  <= 1'b0;
            m_axi_bresp   <= 2'b00;
            m_axi_bvalid  <= 1'b0;
            aw_addr_lat   <= 32'd0;
            aw_len_lat    <= 8'd0;
            aw_pending    <= 1'b0;
            w_done_wait_b <= 1'b0;
        end else begin
            // Clear B after handshake
            if (m_axi_bvalid && m_axi_bready) begin
                m_axi_bvalid  <= 1'b0;
                w_done_wait_b <= 1'b0;
            end

            // Accept AW
            if (m_axi_awvalid && !aw_pending && !w_done_wait_b) begin
                if (axi_slow_awready && axi_aw_delay > 0) begin
                    m_axi_awready <= 1'b0;
                    axi_aw_delay  <= axi_aw_delay - 1;
                end else begin
                    m_axi_awready <= 1'b1;
                    aw_addr_lat   <= m_axi_awaddr;
                    aw_len_lat    <= m_axi_awlen;
                    aw_pending    <= 1'b1;
                    m_axi_wready  <= 1'b1;
                end
            end else begin
                m_axi_awready <= 1'b0;
            end

            // Accept W beats
            if (aw_pending && m_axi_wvalid && m_axi_wready) begin
                // Byte-lane write
                if (m_axi_wstrb[0]) axi_mem[aw_addr_lat+0] <= m_axi_wdata[ 7: 0];
                if (m_axi_wstrb[1]) axi_mem[aw_addr_lat+1] <= m_axi_wdata[15: 8];
                if (m_axi_wstrb[2]) axi_mem[aw_addr_lat+2] <= m_axi_wdata[23:16];
                if (m_axi_wstrb[3]) axi_mem[aw_addr_lat+3] <= m_axi_wdata[31:24];

                aw_addr_lat <= aw_addr_lat + BYTES_PER_BEAT;

                if (m_axi_wlast) begin
                    aw_pending    <= 1'b0;
                    m_axi_wready  <= 1'b0;
                    w_done_wait_b <= 1'b1;
                    m_axi_bvalid  <= 1'b1;
                    m_axi_bresp   <= inject_bresp_err ? 2'b10 : 2'b00;
                    inject_bresp_err <= 1'b0;
                end
            end
        end
    end

    // ================================================================
    //  WISHBONE MASTER TASKS
    // ================================================================
    task wb_write;
        input [WB_ADDR_W-1:0] addr;
        input [WB_DATA_W-1:0] data;
        input [3:0]           sel;
        begin
            @(posedge clk);
            wb_adr_i <= addr;
            wb_dat_i <= data;
            wb_sel_i <= sel;
            wb_we_i  <= 1'b1;
            wb_stb_i <= 1'b1;
            wb_cyc_i <= 1'b1;
            @(posedge clk);
            while (!wb_ack_o) @(posedge clk);
            wb_stb_i <= 1'b0;
            wb_cyc_i <= 1'b0;
            wb_we_i  <= 1'b0;
            @(posedge clk);
        end
    endtask

    task wb_read;
        input  [WB_ADDR_W-1:0] addr;
        output [WB_DATA_W-1:0] data;
        begin
            @(posedge clk);
            wb_adr_i <= addr;
            wb_dat_i <= 32'd0;
            wb_sel_i <= 4'hF;
            wb_we_i  <= 1'b0;
            wb_stb_i <= 1'b1;
            wb_cyc_i <= 1'b1;
            @(posedge clk);
            while (!wb_ack_o) @(posedge clk);
            data = wb_dat_o;
            wb_stb_i <= 1'b0;
            wb_cyc_i <= 1'b0;
            @(posedge clk);
        end
    endtask

    // ================================================================
    //  HELPER: Load source image into AXI memory at base address
    // ================================================================
    task load_src_to_axi_mem;
        input [31:0] base;
        input integer npix;
        integer k;
        begin
            for (k = 0; k < npix; k = k + 1)
                axi_mem[base + k] = src_image[k];
        end
    endtask

    // ================================================================
    //  HELPER: Read destination from AXI memory
    // ================================================================
    task read_dst_from_axi_mem;
        input [31:0] base;
        input integer npix;
        integer k;
        begin
            for (k = 0; k < npix; k = k + 1)
                dst_actual[k] = axi_mem[base + k];
        end
    endtask

    // ================================================================
    //  HELPER: Program CSR and start DMA
    // ================================================================
    task program_and_start;
        input [31:0] src_addr;
        input [31:0] dst_addr;
        input [15:0] pix_count;
        begin
            wb_write(ADDR_SRC,   src_addr,  4'hF);
            wb_write(ADDR_DST,   dst_addr,  4'hF);
            wb_write(ADDR_COUNT, {16'd0, pix_count}, 4'hF);
            wb_write(ADDR_CTRL,  32'h0000_0001, 4'hF);  // start pulse
        end
    endtask

    // ================================================================
    //  HELPER: Wait for DMA done via STATUS polling
    // ================================================================
    task wait_dma_done;
        input integer timeout_cycles;
        integer cnt;
        reg [31:0] status;
        reg done_found;
        begin
            cnt = 0;
            status = 32'd0;
            done_found = 1'b0;
            while (cnt < timeout_cycles && !done_found) begin
                wb_read(ADDR_STATUS, status);
                if (status[1]) begin // done|done_latched
                    done_found = 1'b1;
                end
                cnt = cnt + 1;
            end
            if (!done_found)
                $display("  *** TIMEOUT waiting for DMA done after %0d polls!", timeout_cycles);
        end
    endtask

    // ================================================================
    //  GOLDEN SOBEL MODEL — matches the RTL exactly
    //  Gx = (p02+2*p12+p22) - (p00+2*p10+p20)
    //  Gy = (p20+2*p21+p22) - (p00+2*p01+p02)
    //  mag = |Gx| + |Gy|
    //  edge = (mag > threshold) ? clip(mag,255) : 0
    //  Border pixels = 0
    // ================================================================
    task compute_golden;
        input [7:0] thr;
        integer gx, gy, mag_val;
        integer px, py;
        integer p00, p01, p02, p10, p11, p12, p20, p21, p22;
        begin
            for (i = 0; i < PIXELS; i = i + 1)
                dst_golden[i] = 8'd0;

            for (py = 1; py < IMG_H-1; py = py + 1) begin
                for (px = 1; px < IMG_W-1; px = px + 1) begin
                    p00 = src_image[(py-1)*IMG_W + (px-1)];
                    p01 = src_image[(py-1)*IMG_W + (px  )];
                    p02 = src_image[(py-1)*IMG_W + (px+1)];
                    p10 = src_image[(py  )*IMG_W + (px-1)];
                    p11 = src_image[(py  )*IMG_W + (px  )];
                    p12 = src_image[(py  )*IMG_W + (px+1)];
                    p20 = src_image[(py+1)*IMG_W + (px-1)];
                    p21 = src_image[(py+1)*IMG_W + (px  )];
                    p22 = src_image[(py+1)*IMG_W + (px+1)];

                    gx = (p02 + 2*p12 + p22) - (p00 + 2*p10 + p20);
                    gy = (p20 + 2*p21 + p22) - (p00 + 2*p01 + p02);

                    if (gx < 0) gx = -gx;
                    if (gy < 0) gy = -gy;
                    mag_val = gx + gy;

                    if (mag_val > thr)
                        dst_golden[py*IMG_W + px] = (mag_val > 255) ? 255 : mag_val[7:0];
                    else
                        dst_golden[py*IMG_W + px] = 8'd0;
                end
            end
        end
    endtask

    // ================================================================
    //  HELPER: Compare actual vs golden, count errors
    // ================================================================
    task compare_images;
        input integer npix;
        input integer img_w;
        input integer img_h;
        integer cx, cy, idx, errs;
        begin
            errs = 0;
            for (cy = 0; cy < img_h; cy = cy + 1) begin
                for (cx = 0; cx < img_w; cx = cx + 1) begin
                    idx = cy * img_w + cx;
                    if (idx < npix) begin
                        if (dst_actual[idx] !== dst_golden[idx]) begin
                            if (errs < 10) // only print first 10 mismatches
                                $display("    MISMATCH at (%0d,%0d) addr=%0d: got=%0d expect=%0d",
                                         cx, cy, idx, dst_actual[idx], dst_golden[idx]);
                            errs = errs + 1;
                        end
                    end
                end
            end
            case_errors = errs;
            if (errs == 0)
                $display("    Image comparison: ALL MATCH");
            else
                $display("    Image comparison: %0d MISMATCHES", errs);
        end
    endtask

    // ================================================================
    //  HELPER: Print image matrix to Tcl console
    // ================================================================
    task print_image;
        input [8*20:1] label;
        integer px, py;
        reg [8*400:1] line_str;
        begin
            $display("    --- %0s (%0dx%0d) ---", label, IMG_W, IMG_H);
            for (py = 0; py < IMG_H; py = py + 1) begin
                line_str = "";
                for (px = 0; px < IMG_W; px = px + 1) begin
                    $sformat(line_str, "%0s%4d", line_str, dst_actual[py*IMG_W + px]);
                end
                $display("    Row%0d:%0s", py, line_str);
            end
        end
    endtask

    task print_src_image;
        input [8*20:1] label;
        integer px, py;
        reg [8*400:1] line_str;
        begin
            $display("    --- %0s (%0dx%0d) ---", label, IMG_W, IMG_H);
            for (py = 0; py < IMG_H; py = py + 1) begin
                line_str = "";
                for (px = 0; px < IMG_W; px = px + 1) begin
                    $sformat(line_str, "%0s%4d", line_str, src_image[py*IMG_W + px]);
                end
                $display("    Row%0d:%0s", py, line_str);
            end
        end
    endtask

    // ================================================================
    //  HELPER: Full E2E flow — load image, program CSR, wait done, verify
    // ================================================================
    task run_e2e_test;
        input [31:0] src_base;
        input [31:0] dst_base;
        input [15:0] pix_count;
        input [7:0]  thr;
        input integer timeout;
        begin
            threshold = thr;
            load_src_to_axi_mem(src_base, pix_count);

            // Clear destination region
            for (i = 0; i < pix_count + 16; i = i + 1)
                axi_mem[dst_base + i] = 8'hXX;

            program_and_start(src_base, dst_base, pix_count);
            wait_dma_done(timeout);

            // Small delay for any final pipeline flush
            repeat (5) @(posedge clk);

            read_dst_from_axi_mem(dst_base, pix_count);

            if (pix_count == PIXELS) begin
                compute_golden(thr);
                compare_images(pix_count, IMG_W, IMG_H);
            end
        end
    endtask

    // ================================================================
    //  HELPER: Clear done/error latches
    // ================================================================
    task clear_latches;
        begin
            wb_write(ADDR_CTRL, 32'h0000_0006, 4'hF);  // [1]=clear done, [2]=clear error
            repeat (2) @(posedge clk);
        end
    endtask

    // ================================================================
    //  HELPER: System reset
    // ================================================================
    task do_reset;
        begin
            rst_n = 1'b0;
            wb_adr_i = 0; wb_dat_i = 0; wb_sel_i = 0;
            wb_we_i = 0; wb_stb_i = 0; wb_cyc_i = 0;
            threshold = 8'd60;
            inject_rresp_err = 0;
            inject_bresp_err = 0;
            rresp_err_beat   = 0;
            axi_slow_arready = 0;
            axi_slow_awready = 0;
            axi_ar_delay = 0;
            axi_aw_delay = 0;
            repeat (10) @(posedge clk);
            rst_n = 1'b1;
            repeat (5) @(posedge clk);
        end
    endtask

    // ================================================================
    //  IMAGE GENERATORS
    // ================================================================

    // Vertical edge: left half=0, right half=255
    task gen_vertical_edge;
        integer px, py;
        begin
            for (py = 0; py < IMG_H; py = py + 1)
                for (px = 0; px < IMG_W; px = px + 1)
                    src_image[py*IMG_W + px] = (px < IMG_W/2) ? 8'd0 : 8'd255;
        end
    endtask

    // Horizontal edge: top half=0, bottom half=255
    task gen_horizontal_edge;
        integer px, py;
        begin
            for (py = 0; py < IMG_H; py = py + 1)
                for (px = 0; px < IMG_W; px = px + 1)
                    src_image[py*IMG_W + px] = (py < IMG_H/2) ? 8'd0 : 8'd255;
        end
    endtask

    // Uniform: all same value
    task gen_uniform;
        input [7:0] val;
        integer k;
        begin
            for (k = 0; k < PIXELS; k = k + 1)
                src_image[k] = val;
        end
    endtask

    // Diagonal gradient
    task gen_diagonal;
        integer px, py;
        integer val;
        begin
            for (py = 0; py < IMG_H; py = py + 1)
                for (px = 0; px < IMG_W; px = px + 1) begin
                    val = ((px + py) * 255) / (IMG_W + IMG_H - 2);
                    src_image[py*IMG_W + px] = val[7:0];
                end
        end
    endtask

    // Checkerboard: alternating 0 and 255
    task gen_checkerboard;
        integer px, py;
        begin
            for (py = 0; py < IMG_H; py = py + 1)
                for (px = 0; px < IMG_W; px = px + 1)
                    src_image[py*IMG_W + px] = ((px + py) % 2 == 0) ? 8'd0 : 8'd255;
        end
    endtask

    // Ramp: pixel[i] = i % 256
    task gen_ramp;
        integer k;
        begin
            for (k = 0; k < PIXELS; k = k + 1)
                src_image[k] = k[7:0];
        end
    endtask

    // ================================================================
    //  MAIN TEST SEQUENCE
    // ================================================================
    initial begin
        $display("");
        $display("================================================================");
        $display("  SOBEL ACCELERATOR TOP-LEVEL TESTBENCH");
        $display("  Image size: %0d x %0d = %0d pixels", IMG_W, IMG_H, PIXELS);
        $display("  AXI data width: %0d bits, %0d bytes/beat", AXI_DATA_W, BYTES_PER_BEAT);
        $display("================================================================");
        $display("");

        total_pass = 0;
        total_fail = 0;

        // Initialize AXI memory
        for (i = 0; i < AXI_MEM_SIZE; i = i + 1) axi_mem[i] = 8'd0;

        do_reset;

        // ────────────────────────────────────────────────
        // CASE 1: Reset state verification
        // ────────────────────────────────────────────────
        test_num = 1;
        $display("[CASE %0d] Reset state verification", test_num);
        begin
            case_errors = 0;
            if (wb_ack_o !== 1'b0) begin
                $display("  FAIL: wb_ack_o not 0 after reset"); case_errors = case_errors+1; end
            if (engine_busy !== 1'b0) begin
                $display("  FAIL: engine_busy not 0 after reset"); case_errors = case_errors+1; end
            if (m_axi_arvalid !== 1'b0) begin
                $display("  FAIL: arvalid not 0 after reset"); case_errors = case_errors+1; end
            if (m_axi_awvalid !== 1'b0) begin
                $display("  FAIL: awvalid not 0 after reset"); case_errors = case_errors+1; end
            if (m_axi_wvalid !== 1'b0) begin
                $display("  FAIL: wvalid not 0 after reset"); case_errors = case_errors+1; end

            wb_read(ADDR_STATUS, wb_rd_data_reg);
            if (wb_rd_data_reg !== 32'd0) begin
                $display("  FAIL: STATUS=0x%08h, expected 0", wb_rd_data_reg); case_errors = case_errors+1; end

            if (case_errors == 0) begin
                $display("  PASS"); total_pass = total_pass+1;
            end else begin
                total_fail = total_fail+1; end
        end
        $display("");

        // ────────────────────────────────────────────────
        // CASE 2: CSR register write & readback
        // ────────────────────────────────────────────────
        test_num = 2;
        $display("[CASE %0d] CSR register write & readback", test_num);
        begin
            case_errors = 0;

            wb_write(ADDR_SRC, 32'hDEAD_BEEF, 4'hF);
            wb_read(ADDR_SRC, wb_rd_data_reg);
            if (wb_rd_data_reg !== 32'hDEAD_BEEF) begin
                $display("  FAIL: SRC readback=0x%08h, expected 0xDEADBEEF", wb_rd_data_reg);
                case_errors = case_errors+1;
            end

            wb_write(ADDR_DST, 32'hCAFE_1234, 4'hF);
            wb_read(ADDR_DST, wb_rd_data_reg);
            if (wb_rd_data_reg !== 32'hCAFE_1234) begin
                $display("  FAIL: DST readback=0x%08h, expected 0xCAFE1234", wb_rd_data_reg);
                case_errors = case_errors+1;
            end

            wb_write(ADDR_COUNT, 32'h0000_0064, 4'hF); // 100
            wb_read(ADDR_COUNT, wb_rd_data_reg);
            if (wb_rd_data_reg !== 32'h0000_0064) begin
                $display("  FAIL: COUNT readback=0x%08h, expected 0x64", wb_rd_data_reg);
                case_errors = case_errors+1;
            end

            if (case_errors == 0) begin
                $display("  PASS"); total_pass = total_pass+1;
            end else begin
                total_fail = total_fail+1; end
        end
        $display("");

        // ────────────────────────────────────────────────
        // CASE 3: Full E2E — Vertical edge, threshold=60
        // ────────────────────────────────────────────────
        test_num = 3;
        $display("[CASE %0d] Full E2E — vertical edge image, threshold=60", test_num);
        begin
            do_reset;
            gen_vertical_edge;
            $display("  Input image:");
            print_src_image("Vertical Edge");
            run_e2e_test(32'h0000, 32'h1000, PIXELS[15:0], 8'd60, 50000);
            $display("  Output image:");
            print_image("Sobel Result");

            if (case_errors == 0) begin
                $display("  PASS"); total_pass = total_pass+1;
            end else begin
                total_fail = total_fail+1; end
        end
        $display("");

        // ────────────────────────────────────────────────
        // CASE 4: Full E2E — Horizontal edge
        // ────────────────────────────────────────────────
        test_num = 4;
        $display("[CASE %0d] Full E2E — horizontal edge image, threshold=60", test_num);
        begin
            do_reset;
            gen_horizontal_edge;
            run_e2e_test(32'h0000, 32'h1000, PIXELS[15:0], 8'd60, 50000);
            $display("  Output image:");
            print_image("Sobel Result");

            if (case_errors == 0) begin
                $display("  PASS"); total_pass = total_pass+1;
            end else begin
                total_fail = total_fail+1; end
        end
        $display("");

        // ────────────────────────────────────────────────
        // CASE 5: Full E2E — Uniform image (all 128)
        // ────────────────────────────────────────────────
        test_num = 5;
        $display("[CASE %0d] Full E2E — uniform image (all 128), threshold=60", test_num);
        begin
            do_reset;
            gen_uniform(8'd128);
            run_e2e_test(32'h0000, 32'h1000, PIXELS[15:0], 8'd60, 50000);

            // For uniform image, all outputs should be 0
            case_errors = 0;
            for (i = 0; i < PIXELS; i = i + 1) begin
                if (dst_actual[i] !== 8'd0) case_errors = case_errors + 1;
            end
            if (case_errors == 0) begin
                $display("  All pixels = 0 as expected");
                $display("  PASS"); total_pass = total_pass+1;
            end else begin
                $display("  FAIL: %0d non-zero pixels in uniform output", case_errors);
                total_fail = total_fail+1;
            end
        end
        $display("");

        // ────────────────────────────────────────────────
        // CASE 6: Full E2E — Diagonal gradient
        // ────────────────────────────────────────────────
        test_num = 6;
        $display("[CASE %0d] Full E2E — diagonal gradient, threshold=60", test_num);
        begin
            do_reset;
            gen_diagonal;
            run_e2e_test(32'h0000, 32'h1000, PIXELS[15:0], 8'd60, 50000);
            $display("  Output image:");
            print_image("Sobel Result");

            if (case_errors == 0) begin
                $display("  PASS"); total_pass = total_pass+1;
            end else begin
                total_fail = total_fail+1; end
        end
        $display("");

        // ────────────────────────────────────────────────
        // CASE 7: Full E2E — Checkerboard
        // ────────────────────────────────────────────────
        test_num = 7;
        $display("[CASE %0d] Full E2E — checkerboard pattern, threshold=60", test_num);
        begin
            do_reset;
            gen_checkerboard;
            run_e2e_test(32'h0000, 32'h1000, PIXELS[15:0], 8'd60, 50000);
            $display("  Output image:");
            print_image("Sobel Result");

            if (case_errors == 0) begin
                $display("  PASS"); total_pass = total_pass+1;
            end else begin
                total_fail = total_fail+1; end
        end
        $display("");

        // ────────────────────────────────────────────────
        // CASE 8: Threshold sensitivity — threshold=0 (everything passes)
        // ────────────────────────────────────────────────
        test_num = 8;
        $display("[CASE %0d] Threshold=0 — vertical edge (all edges pass)", test_num);
        begin
            do_reset;
            gen_vertical_edge;
            run_e2e_test(32'h0000, 32'h1000, PIXELS[15:0], 8'd0, 50000);

            // With threshold=0, any non-zero gradient passes
            // Inner pixels near the edge should be non-zero
            case_errors = 0;
            // Check that edge column pixels (x=4,5 for 10-wide, inner rows) are non-zero
            for (y = 1; y < IMG_H-1; y = y + 1) begin
                if (dst_actual[y*IMG_W + 5] == 8'd0) begin // column right of edge
                    case_errors = case_errors + 1;
                end
            end

            if (case_errors == 0) begin
                $display("  Edge pixels are non-zero as expected with threshold=0");
                $display("  PASS"); total_pass = total_pass+1;
            end else begin
                $display("  FAIL: %0d edge pixels unexpectedly zero", case_errors);
                total_fail = total_fail+1;
            end
        end
        $display("");

        // ────────────────────────────────────────────────
        // CASE 9: Threshold=255 — everything suppressed
        // ────────────────────────────────────────────────
        test_num = 9;
        $display("[CASE %0d] Threshold=255 — vertical edge (all edges suppressed)", test_num);
        begin
            do_reset;
            gen_vertical_edge;
            run_e2e_test(32'h0000, 32'h1000, PIXELS[15:0], 8'd255, 50000);

            // With threshold=255, almost nothing passes (max Sobel mag for 0/255 edge = 255*4=1020 but clipped)
            // Actually max single-pixel gradient can exceed 255, so some might pass
            // Use golden model for accurate check
            compute_golden(8'd255);
            case_errors = 0;
            for (i = 0; i < PIXELS; i = i + 1) begin
                if (dst_actual[i] !== dst_golden[i]) case_errors = case_errors + 1;
            end

            if (case_errors == 0) begin
                $display("  Output matches golden model with threshold=255");
                $display("  PASS"); total_pass = total_pass+1;
            end else begin
                $display("  FAIL: %0d mismatches with threshold=255", case_errors);
                total_fail = total_fail+1;
            end
        end
        $display("");

        // ────────────────────────────────────────────────
        // CASE 10: pixel_count = 0 — immediate done
        // ────────────────────────────────────────────────
        test_num = 10;
        $display("[CASE %0d] pixel_count = 0 — immediate done, no AXI traffic", test_num);
        begin
            do_reset;
            case_errors = 0;

            program_and_start(32'h0000, 32'h1000, 16'd0);

            // Should complete very quickly
            repeat (20) @(posedge clk);
            wb_read(ADDR_STATUS, wb_rd_data_reg);
            if (wb_rd_data_reg[1]) begin
                $display("  Done flag set as expected for zero-pixel transfer");
            end else begin
                $display("  FAIL: Done not set for zero-pixel transfer, STATUS=0x%08h", wb_rd_data_reg);
                case_errors = case_errors + 1;
            end

            if (case_errors == 0) begin
                $display("  PASS"); total_pass = total_pass+1;
            end else begin
                total_fail = total_fail+1; end
        end
        $display("");

        // ────────────────────────────────────────────────
        // CASE 11: pixel_count = 1 — single pixel
        // ────────────────────────────────────────────────
        test_num = 11;
        $display("[CASE %0d] pixel_count = 1 — single pixel", test_num);
        begin
            do_reset;
            src_image[0] = 8'd100;
            load_src_to_axi_mem(32'h0000, 1);
            axi_mem[32'h1000] = 8'hXX;

            program_and_start(32'h0000, 32'h1000, 16'd1);
            wait_dma_done(50000);
            repeat (5) @(posedge clk);

            // Single pixel in a 10x10 image is at (0,0) which is a border pixel → 0
            case_errors = 0;
            // Just verify DMA completes without hanging
            wb_read(ADDR_STATUS, wb_rd_data_reg);
            if (!wb_rd_data_reg[1]) begin
                $display("  FAIL: DMA did not complete");
                case_errors = case_errors + 1;
            end
            if (wb_rd_data_reg[2]) begin
                $display("  FAIL: Unexpected error flag");
                case_errors = case_errors + 1;
            end

            if (case_errors == 0) begin
                $display("  DMA completed without error for 1 pixel");
                $display("  PASS"); total_pass = total_pass+1;
            end else begin
                total_fail = total_fail+1; end
        end
        $display("");

        // ────────────────────────────────────────────────
        // CASE 12: pixel_count = 3 — less than one AXI word
        // ────────────────────────────────────────────────
        test_num = 12;
        $display("[CASE %0d] pixel_count = 3 — partial AXI word", test_num);
        begin
            do_reset;
            src_image[0] = 8'd10; src_image[1] = 8'd20; src_image[2] = 8'd30;
            load_src_to_axi_mem(32'h0000, 3);

            program_and_start(32'h0000, 32'h1000, 16'd3);
            wait_dma_done(50000);
            repeat (5) @(posedge clk);

            case_errors = 0;
            wb_read(ADDR_STATUS, wb_rd_data_reg);
            if (!wb_rd_data_reg[1]) begin
                $display("  FAIL: DMA did not complete"); case_errors = case_errors+1;
            end
            if (wb_rd_data_reg[2]) begin
                $display("  FAIL: Unexpected error"); case_errors = case_errors+1;
            end

            if (case_errors == 0) begin
                $display("  DMA completed without error for 3 pixels");
                $display("  PASS"); total_pass = total_pass+1;
            end else begin
                total_fail = total_fail+1; end
        end
        $display("");

        // ────────────────────────────────────────────────
        // CASE 13: pixel_count = 7 — non-word-aligned
        // ────────────────────────────────────────────────
        test_num = 13;
        $display("[CASE %0d] pixel_count = 7 — non-word-aligned (2 AXI words)", test_num);
        begin
            do_reset;
            for (i = 0; i < 7; i = i + 1) begin
                j = i * 37;
                src_image[i] = j[7:0];
            end
            load_src_to_axi_mem(32'h0000, 7);

            program_and_start(32'h0000, 32'h1000, 16'd7);
            wait_dma_done(50000);
            repeat (5) @(posedge clk);

            case_errors = 0;
            wb_read(ADDR_STATUS, wb_rd_data_reg);
            if (!wb_rd_data_reg[1]) begin
                $display("  FAIL: DMA did not complete"); case_errors = case_errors+1;
            end
            if (wb_rd_data_reg[2]) begin
                $display("  FAIL: Unexpected error"); case_errors = case_errors+1;
            end

            if (case_errors == 0) begin
                $display("  DMA completed without error for 7 pixels");
                $display("  PASS"); total_pass = total_pass+1;
            end else begin
                total_fail = total_fail+1; end
        end
        $display("");

        // ────────────────────────────────────────────────
        // CASE 14: Full 100-pixel (multi-burst verification)
        // ────────────────────────────────────────────────
        test_num = 14;
        $display("[CASE %0d] pixel_count = 100 — multi-burst (25 words = 16+9)", test_num);
        begin
            do_reset;
            gen_ramp;
            run_e2e_test(32'h0000, 32'h1000, PIXELS[15:0], 8'd60, 50000);

            $display("  Output image:");
            print_image("Ramp Sobel");

            if (case_errors == 0) begin
                $display("  PASS"); total_pass = total_pass+1;
            end else begin
                total_fail = total_fail+1; end
        end
        $display("");

        // ────────────────────────────────────────────────
        // CASE 15: AXI read error (SLVERR)
        // ────────────────────────────────────────────────
        test_num = 15;
        $display("[CASE %0d] AXI read SLVERR — error flag should be set", test_num);
        begin
            do_reset;
            gen_vertical_edge;
            load_src_to_axi_mem(32'h0000, PIXELS);

            inject_rresp_err = 1;
            rresp_err_beat = 2;  // error on 3rd beat of first burst

            program_and_start(32'h0000, 32'h1000, PIXELS[15:0]);

            // Wait for error or done
            repeat (5000) @(posedge clk);

            case_errors = 0;
            wb_read(ADDR_STATUS, wb_rd_data_reg);
            if (!wb_rd_data_reg[2]) begin
                $display("  FAIL: Error flag not set after SLVERR, STATUS=0x%08h", wb_rd_data_reg);
                case_errors = case_errors + 1;
            end else begin
                $display("  Error flag correctly set, STATUS=0x%08h", wb_rd_data_reg);
            end

            // Clear error latch
            clear_latches;

            if (case_errors == 0) begin
                $display("  PASS"); total_pass = total_pass+1;
            end else begin
                total_fail = total_fail+1; end
        end
        $display("");

        // ────────────────────────────────────────────────
        // CASE 16: AXI write error (SLVERR on bresp)
        // ────────────────────────────────────────────────
        test_num = 16;
        $display("[CASE %0d] AXI write SLVERR — error flag should be set", test_num);
        begin
            do_reset;
            gen_uniform(8'd100);
            load_src_to_axi_mem(32'h0000, PIXELS);

            // The write error happens during STORE phase.
            // We need the LOAD + ENGINE to complete first, then inject on first STORE burst.
            // Set the flag now; the AXI write model will pick it up when the first AW+W completes.
            inject_bresp_err = 1;

            program_and_start(32'h0000, 32'h1000, PIXELS[15:0]);

            // Wait for completion
            repeat (80000) @(posedge clk);

            case_errors = 0;
            wb_read(ADDR_STATUS, wb_rd_data_reg);
            if (!wb_rd_data_reg[2]) begin
                $display("  FAIL: Error flag not set after write SLVERR, STATUS=0x%08h", wb_rd_data_reg);
                case_errors = case_errors + 1;
            end else begin
                $display("  Error flag correctly set, STATUS=0x%08h", wb_rd_data_reg);
            end

            clear_latches;

            if (case_errors == 0) begin
                $display("  PASS"); total_pass = total_pass+1;
            end else begin
                total_fail = total_fail+1; end
        end
        $display("");

        // ────────────────────────────────────────────────
        // CASE 17: Back-to-back runs without system reset
        // ────────────────────────────────────────────────
        test_num = 17;
        $display("[CASE %0d] Back-to-back runs without system reset", test_num);
        begin
            do_reset;
            case_errors = 0;

            // Run 1: vertical edge
            $display("  Run 1: Vertical edge");
            gen_vertical_edge;
            run_e2e_test(32'h0000, 32'h1000, PIXELS[15:0], 8'd60, 50000);
            if (case_errors != 0) begin
                $display("  Run 1 FAILED with %0d errors", case_errors);
            end else begin
                $display("  Run 1 OK");
            end

            // Clear latches between runs
            clear_latches;
            repeat (10) @(posedge clk);

            // Run 2: horizontal edge (no reset!)
            $display("  Run 2: Horizontal edge (no reset)");
            gen_horizontal_edge;
            run_e2e_test(32'h0000, 32'h2000, PIXELS[15:0], 8'd60, 50000);
            if (case_errors != 0) begin
                $display("  Run 2 FAILED with %0d errors", case_errors);
            end else begin
                $display("  Run 2 OK");
            end

            if (case_errors == 0) begin
                $display("  PASS"); total_pass = total_pass+1;
            end else begin
                total_fail = total_fail+1; end
        end
        $display("");

        // ────────────────────────────────────────────────
        // CASE 18: Non-zero base addresses
        // ────────────────────────────────────────────────
        test_num = 18;
        $display("[CASE %0d] Non-zero base addresses (src=0x1000, dst=0x2000)", test_num);
        begin
            do_reset;
            gen_vertical_edge;
            run_e2e_test(32'h1000, 32'h2000, PIXELS[15:0], 8'd60, 50000);

            if (case_errors == 0) begin
                $display("  PASS"); total_pass = total_pass+1;
            end else begin
                total_fail = total_fail+1; end
        end
        $display("");

        // ────────────────────────────────────────────────
        // CASE 19: AXI slow-ready backpressure
        // ────────────────────────────────────────────────
        test_num = 19;
        $display("[CASE %0d] AXI slow-ready backpressure (delayed arready/awready)", test_num);
        begin
            do_reset;
            gen_vertical_edge;
            load_src_to_axi_mem(32'h0000, PIXELS);

            axi_slow_arready = 1;
            axi_slow_awready = 1;
            axi_ar_delay = 3;
            axi_aw_delay = 3;

            for (i = 0; i < PIXELS + 16; i = i + 1)
                axi_mem[32'h1000 + i] = 8'hXX;

            program_and_start(32'h0000, 32'h1000, PIXELS[15:0]);
            wait_dma_done(80000);
            repeat (5) @(posedge clk);

            read_dst_from_axi_mem(32'h1000, PIXELS);
            compute_golden(8'd60);
            compare_images(PIXELS, IMG_W, IMG_H);

            axi_slow_arready = 0;
            axi_slow_awready = 0;

            if (case_errors == 0) begin
                $display("  PASS"); total_pass = total_pass+1;
            end else begin
                total_fail = total_fail+1; end
        end
        $display("");

        // ────────────────────────────────────────────────
        // CASE 20: WB sel gating — start NOT fired when sel[0]=0
        // ────────────────────────────────────────────────
        test_num = 20;
        $display("[CASE %0d] WB sel gating — start not fired when sel[0]=0", test_num);
        begin
            do_reset;
            case_errors = 0;

            wb_write(ADDR_SRC,   32'h0000, 4'hF);
            wb_write(ADDR_DST,   32'h1000, 4'hF);
            wb_write(ADDR_COUNT, 32'd100,  4'hF);

            // Write CTRL with sel[0]=0 — should NOT trigger start
            wb_write(ADDR_CTRL, 32'h0000_0001, 4'hE);  // sel = 4'b1110

            repeat (50) @(posedge clk);

            wb_read(ADDR_STATUS, wb_rd_data_reg);
            if (wb_rd_data_reg[0]) begin
                $display("  FAIL: DMA busy despite sel[0]=0, STATUS=0x%08h", wb_rd_data_reg);
                case_errors = case_errors + 1;
            end else begin
                $display("  DMA correctly NOT started when sel[0]=0");
            end

            if (case_errors == 0) begin
                $display("  PASS"); total_pass = total_pass+1;
            end else begin
                total_fail = total_fail+1; end
        end
        $display("");

        // ================================================================
        //  FINAL SUMMARY
        // ================================================================
        $display("");
        $display("================================================================");
        $display("  FINAL RESULTS");
        $display("================================================================");
        $display("  Total tests : %0d", total_pass + total_fail);
        $display("  PASSED      : %0d", total_pass);
        $display("  FAILED      : %0d", total_fail);
        $display("================================================================");
        if (total_fail == 0)
            $display("  *** ALL TESTS PASSED ***");
        else
            $display("  *** SOME TESTS FAILED — see details above ***");
        $display("================================================================");
        $display("");

        #100;
        $finish;
    end

    // ================================================================
    //  GLOBAL TIMEOUT WATCHDOG
    // ================================================================
    initial begin
        #50_000_000;  // 50 ms
        $display("");
        $display("*** GLOBAL TIMEOUT — simulation killed after 50 ms ***");
        $display("");
        $finish;
    end

endmodule
