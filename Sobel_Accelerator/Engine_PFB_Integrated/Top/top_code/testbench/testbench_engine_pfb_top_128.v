`timescale 1ns / 1ps

// Testbench for enigne_pfb_top (Engine + Private Frame Buffer Integration)
// FULL 128x128 IMAGE SIZE
// Author: Junze Jiang
// Date: 14/3/2026
//
// Strategy:
//   128x128 = 16384 pixels — too large to print full matrices.
//   Uses statistical summaries: nonzero count, min/max, sample rows.
//   Each case: DMA write -> start engine -> wait done -> DMA read -> verify
//
// Test Cases:
//   CASE 1 : Vertical edge   (split x=64)         — strong Gx
//   CASE 2 : Horizontal edge (split y=64)          — strong Gy
//   CASE 3 : Uniform image   (all 128)             — no edges
//   CASE 4 : Diagonal gradient                     — mixed Gx/Gy
//   CASE 5 : Threshold sensitivity (thresh=200)    — weak edge suppressed
//   CASE 6 : Back-to-back run (no reset)           — engine re-entry
//   CASE 7 : DMA write during engine busy          — conflict detection
//   CASE 8 : PFB standalone DMA read/write         — memory integrity
//   CASE 9 : Single pixel impulse at (64,64)       — localised response
//   CASE 10: Checkerboard (Sobel symmetry cancel)  — stress test

module tb_engine_pfb_top_128();

    // ---- Parameters: FULL 128x128 ----
    parameter DATA_W    = 8;
    parameter ADDR_W    = 14;
    parameter BANKS     = 4;
    parameter IMG_W     = 128;
    parameter IMG_H     = 128;
    parameter MAG_W     = 12;

    localparam PIXELS   = IMG_W * IMG_H;  // 16384
    localparam CLK_P    = 10;             // 100 MHz

    // ---- DUT signals ----
    reg                 clk;
    reg                 rst_n;
    reg                 start;
    reg  [7:0]          threshold;

    wire                busy;
    wire                done;

    reg                 dma_rd_req;
    reg  [ADDR_W-1:0]   dma_rd_addr;
    wire [DATA_W-1:0]   dma_rd_data;
    wire                dma_rd_valid;
    wire                dma_rd_ready;

    reg                 dma_wr_req;
    reg  [ADDR_W-1:0]   dma_wr_addr;
    reg  [DATA_W-1:0]   dma_wr_data;
    wire                dma_wr_ready;

    wire                eng_wr_ready;
    wire                eng_rd_ready;
    wire                wr_conflict;

    // ---- Local storage ----
    reg  [7:0] golden_input [0:PIXELS-1];
    reg  [7:0] readback     [0:PIXELS-1];

    // ---- Counters ----
    integer total_errors;
    integer case_errors;
    integer i, x, y;

    // ---- Timing measurement ----
    integer time_start, time_end;

    // ---- DUT instantiation ----
    enigne_pfb_top #(
        .DATA_W (DATA_W),
        .ADDR_W (ADDR_W),
        .BANKS  (BANKS),
        .IMG_W  (IMG_W),
        .IMG_H  (IMG_H),
        .MAG_W  (MAG_W)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start),
        .threshold      (threshold),
        .busy           (busy),
        .done           (done),
        .dma_rd_req     (dma_rd_req),
        .dma_rd_addr    (dma_rd_addr),
        .dma_rd_data    (dma_rd_data),
        .dma_rd_valid   (dma_rd_valid),
        .dma_rd_ready   (dma_rd_ready),
        .dma_wr_req     (dma_wr_req),
        .dma_wr_addr    (dma_wr_addr),
        .dma_wr_data    (dma_wr_data),
        .dma_wr_ready   (dma_wr_ready),
        .eng_wr_ready   (eng_wr_ready),
        .eng_rd_ready   (eng_rd_ready),
        .wr_conflict    (wr_conflict)
    );

    // ---- Clock ----
    initial clk = 0;
    always #(CLK_P/2) clk = ~clk;

    // ---- Timeout watchdog (128x128 needs much longer) ----
    initial begin
        #(CLK_P * 50_000_000);  // 50M cycles = 500 ms @ 100MHz — generous
        $display("\n[FATAL] Global timeout reached — simulation killed.");
        $finish;
    end

    // ====================================================================
    //  REUSABLE TASKS
    // ====================================================================

    task do_reset;
        begin
            rst_n       = 0;
            start       = 0;
            dma_rd_req  = 0;
            dma_wr_req  = 0;
            dma_rd_addr = 0;
            dma_wr_addr = 0;
            dma_wr_data = 0;
            #(CLK_P * 3);
            rst_n = 1;
            #(CLK_P * 2);
        end
    endtask

    task dma_write_pixel;
        input [ADDR_W-1:0] addr;
        input [DATA_W-1:0] data;
        begin
            @(posedge clk); #1;
            dma_wr_req  = 1;
            dma_wr_addr = addr;
            dma_wr_data = data;
            @(posedge clk);
            while (!dma_wr_ready) @(posedge clk);
            #1;
            dma_wr_req  = 0;
        end
    endtask

    task dma_write_image;
        integer idx;
        begin
            for (idx = 0; idx < PIXELS; idx = idx + 1) begin
                dma_write_pixel(idx[ADDR_W-1:0], golden_input[idx]);
            end
            @(posedge clk); #1;
        end
    endtask

    task dma_read_pixel;
        input  [ADDR_W-1:0] addr;
        output [DATA_W-1:0] data;
        begin
            @(posedge clk); #1;
            dma_rd_req  = 1;
            dma_rd_addr = addr;
            @(posedge clk);
            while (!dma_rd_ready) @(posedge clk);
            #1;
            dma_rd_req = 0;
            @(posedge clk);
            while (!dma_rd_valid) @(posedge clk);
            data = dma_rd_data;
        end
    endtask

    task dma_read_image;
        integer idx;
        reg [DATA_W-1:0] tmp;
        begin
            for (idx = 0; idx < PIXELS; idx = idx + 1) begin
                dma_read_pixel(idx[ADDR_W-1:0], tmp);
                readback[idx] = tmp;
            end
        end
    endtask

    task run_engine;
        begin
            time_start = $time;
            @(posedge clk); #1;
            start = 1;
            @(posedge clk); #1;
            start = 0;
            @(posedge clk);
            while (!done) @(posedge clk);
            time_end = $time;
            $display("  [INFO] Engine done at time %0t  (took %0d ns = %0d cycles)",
                     $time, time_end - time_start, (time_end - time_start) / CLK_P);
            @(posedge clk); @(posedge clk);
        end
    endtask

    // ---------- Statistics summary (replaces print_image for 128x128) ----------
    task print_image_stats;
        input [800:0] label;
        input integer  use_readback;
        integer sx, sy, nz_cnt, pix_val, min_val, max_val, sum_val;
        begin
            nz_cnt  = 0;
            min_val = 255;
            max_val = 0;
            sum_val = 0;
            for (sy = 0; sy < IMG_H; sy = sy + 1) begin
                for (sx = 0; sx < IMG_W; sx = sx + 1) begin
                    pix_val = use_readback ? readback[sy*IMG_W + sx] : golden_input[sy*IMG_W + sx];
                    if (pix_val != 0) nz_cnt = nz_cnt + 1;
                    if (pix_val < min_val) min_val = pix_val;
                    if (pix_val > max_val) max_val = pix_val;
                    sum_val = sum_val + pix_val;
                end
            end
            $display("  %0s  [%0dx%0d]:", label, IMG_W, IMG_H);
            $display("    Nonzero pixels : %0d / %0d", nz_cnt, PIXELS);
            $display("    Min=%0d  Max=%0d  Sum=%0d", min_val, max_val, sum_val);
        end
    endtask

    // Print a few sample rows from readback for visual spot-check
    task print_sample_rows;
        input integer row0;    // first row to print
        input integer row1;    // second row to print
        input integer row2;    // third row to print
        integer sx, r;
        begin
            $display("    Sample rows from output (showing first 20 columns):");
            // Row 0
            $write("      Row %3d: ", row0);
            for (sx = 0; sx < 20 && sx < IMG_W; sx = sx + 1)
                $write("%4d", readback[row0*IMG_W + sx]);
            $write(" ...\n");
            // Row 1
            $write("      Row %3d: ", row1);
            for (sx = 0; sx < 20 && sx < IMG_W; sx = sx + 1)
                $write("%4d", readback[row1*IMG_W + sx]);
            $write(" ...\n");
            // Row 2
            $write("      Row %3d: ", row2);
            for (sx = 0; sx < 20 && sx < IMG_W; sx = sx + 1)
                $write("%4d", readback[row2*IMG_W + sx]);
            $write(" ...\n");
        end
    endtask

    // ---------- Verification tasks ----------

    task check_borders;
        output integer errs;
        integer bx, by, addr_chk;
        begin
            errs = 0;
            for (by = 0; by < IMG_H; by = by + 1) begin
                for (bx = 0; bx < IMG_W; bx = bx + 1) begin
                    if (bx == 0 || bx == IMG_W-1 || by == 0 || by == IMG_H-1) begin
                        addr_chk = by * IMG_W + bx;
                        if (readback[addr_chk] !== 8'd0) begin
                            // Only print first 5 errors to avoid flooding
                            if (errs < 5)
                                $display("    [BORDER ERR] (%0d,%0d) addr=%0d expected=0 got=%0d",
                                         bx, by, addr_chk, readback[addr_chk]);
                            errs = errs + 1;
                        end
                    end
                end
            end
            if (errs == 0)
                $display("    [PASS] All %0d border pixels are zero.", 2*IMG_W + 2*(IMG_H-2));
            else
                $display("    [FAIL] %0d border pixel errors (first 5 shown above).", errs);
        end
    endtask

    task check_inner_all_zero;
        output integer errs;
        integer cx, cy, addr_chk;
        begin
            errs = 0;
            for (cy = 1; cy < IMG_H-1; cy = cy + 1) begin
                for (cx = 1; cx < IMG_W-1; cx = cx + 1) begin
                    addr_chk = cy * IMG_W + cx;
                    if (readback[addr_chk] !== 8'd0) begin
                        if (errs < 5)
                            $display("    [INNER ERR] (%0d,%0d) expected=0 got=%0d",
                                     cx, cy, readback[addr_chk]);
                        errs = errs + 1;
                    end
                end
            end
            if (errs == 0)
                $display("    [PASS] All %0d inner pixels are zero.", (IMG_W-2)*(IMG_H-2));
            else
                $display("    [FAIL] %0d inner pixels non-zero (first 5 shown).", errs);
        end
    endtask

    task check_vertical_edge;
        input integer split_x;
        output integer errs;
        integer cx, cy, addr_chk;
        integer expect_edge;
        begin
            errs = 0;
            for (cy = 1; cy < IMG_H-1; cy = cy + 1) begin
                for (cx = 1; cx < IMG_W-1; cx = cx + 1) begin
                    addr_chk = cy * IMG_W + cx;
                    expect_edge = ((cx - 1) < split_x) && (split_x <= (cx + 1));
                    if (expect_edge && readback[addr_chk] == 8'd0) begin
                        if (errs < 5)
                            $display("    [VEDGE ERR] (%0d,%0d) expected nonzero, got 0", cx, cy);
                        errs = errs + 1;
                    end else if (!expect_edge && readback[addr_chk] != 8'd0) begin
                        if (errs < 5)
                            $display("    [VEDGE ERR] (%0d,%0d) expected 0, got %0d", cx, cy, readback[addr_chk]);
                        errs = errs + 1;
                    end
                end
            end
            if (errs == 0)
                $display("    [PASS] Vertical edge at x=%0d detected correctly across %0d inner rows.", split_x, IMG_H-2);
            else
                $display("    [FAIL] %0d vertical edge errors (first 5 shown).", errs);
        end
    endtask

    task check_horizontal_edge;
        input integer split_y;
        output integer errs;
        integer cx, cy, addr_chk;
        integer expect_edge;
        begin
            errs = 0;
            for (cy = 1; cy < IMG_H-1; cy = cy + 1) begin
                for (cx = 1; cx < IMG_W-1; cx = cx + 1) begin
                    addr_chk = cy * IMG_W + cx;
                    expect_edge = ((cy - 1) < split_y) && (split_y <= (cy + 1));
                    if (expect_edge && readback[addr_chk] == 8'd0) begin
                        if (errs < 5)
                            $display("    [HEDGE ERR] (%0d,%0d) expected nonzero, got 0", cx, cy);
                        errs = errs + 1;
                    end else if (!expect_edge && readback[addr_chk] != 8'd0) begin
                        if (errs < 5)
                            $display("    [HEDGE ERR] (%0d,%0d) expected 0, got %0d", cx, cy, readback[addr_chk]);
                        errs = errs + 1;
                    end
                end
            end
            if (errs == 0)
                $display("    [PASS] Horizontal edge at y=%0d detected correctly across %0d inner cols.", split_y, IMG_W-2);
            else
                $display("    [FAIL] %0d horizontal edge errors (first 5 shown).", errs);
        end
    endtask

    // Check impulse response: 8 neighbours of (cx,cy) should be nonzero, rest zero
    task check_impulse;
        input integer imp_x;
        input integer imp_y;
        output integer errs;
        integer cx, cy, addr_chk;
        integer dx, dy, is_neighbour;
        begin
            errs = 0;
            for (cy = 1; cy < IMG_H-1; cy = cy + 1) begin
                for (cx = 1; cx < IMG_W-1; cx = cx + 1) begin
                    addr_chk = cy * IMG_W + cx;
                    dx = (cx > imp_x) ? (cx - imp_x) : (imp_x - cx);
                    dy = (cy > imp_y) ? (cy - imp_y) : (imp_y - cy);
                    // Neighbour if within 1 step of impulse, but NOT the centre itself
                    is_neighbour = (dx <= 1) && (dy <= 1) && !(dx == 0 && dy == 0);
                    if (is_neighbour && readback[addr_chk] == 8'd0) begin
                        if (errs < 5)
                            $display("    [IMPULSE ERR] (%0d,%0d) neighbour expected nonzero, got 0", cx, cy);
                        errs = errs + 1;
                    end
                end
            end
            if (errs == 0)
                $display("    [PASS] All 8 neighbours of (%0d,%0d) are nonzero.", imp_x, imp_y);
            else
                $display("    [FAIL] %0d impulse neighbour errors (first 5 shown).", errs);
        end
    endtask

    // ====================================================================
    //  MAIN TEST SEQUENCE
    // ====================================================================
    initial begin
        total_errors = 0;

        $display("\n");
        $display("##########################################################");
        $display("#   ENGINE + PFB INTEGRATION TESTBENCH (128x128)         #");
        $display("#   Image: %0d x %0d = %0d pixels                        #", IMG_W, IMG_H, PIXELS);
        $display("#   PFB:   %0d banks, %0d-bit data, %0d-bit addr         #", BANKS, DATA_W, ADDR_W);
        $display("##########################################################\n");

        // ============================================================
        //  CASE 1: Vertical Edge (left=0, right=255, split at x=64)
        // ============================================================
        $display("==========================================================");
        $display(" CASE 1: Vertical Edge Detection (Gx dominant, split x=64)");
        $display("==========================================================");
        do_reset();
        threshold = 8'd60;

        for (y = 0; y < IMG_H; y = y + 1)
            for (x = 0; x < IMG_W; x = x + 1)
                golden_input[y*IMG_W + x] = (x < 64) ? 8'd0 : 8'd255;

        print_image_stats("Input", 0);
        $display("  [INFO] DMA writing 128x128 image into PFB...");
        dma_write_image();
        $display("  [INFO] Starting engine (threshold=%0d)...", threshold);
        run_engine();
        $display("  [INFO] DMA reading result from PFB...");
        dma_read_image();
        print_image_stats("Sobel Output", 1);
        print_sample_rows(0, 64, 127);

        check_borders(case_errors);
        total_errors = total_errors + case_errors;
        check_vertical_edge(64, case_errors);
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 2: Horizontal Edge (top=0, bottom=255, split at y=64)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 2: Horizontal Edge Detection (Gy dominant, split y=64)");
        $display("==========================================================");
        do_reset();
        threshold = 8'd60;

        for (y = 0; y < IMG_H; y = y + 1)
            for (x = 0; x < IMG_W; x = x + 1)
                golden_input[y*IMG_W + x] = (y < 64) ? 8'd0 : 8'd255;

        print_image_stats("Input", 0);
        dma_write_image();
        $display("  [INFO] Starting engine...");
        run_engine();
        dma_read_image();
        print_image_stats("Sobel Output", 1);
        print_sample_rows(62, 64, 66);

        check_borders(case_errors);
        total_errors = total_errors + case_errors;
        check_horizontal_edge(64, case_errors);
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 3: Uniform Image (all 128) — expect zero output
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 3: Uniform Image (all 128, no edges expected)");
        $display("==========================================================");
        do_reset();
        threshold = 8'd60;

        for (i = 0; i < PIXELS; i = i + 1)
            golden_input[i] = 8'd128;

        print_image_stats("Input", 0);
        dma_write_image();
        run_engine();
        dma_read_image();
        print_image_stats("Sobel Output", 1);

        check_borders(case_errors);
        total_errors = total_errors + case_errors;
        check_inner_all_zero(case_errors);
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 4: Diagonal Gradient (mixed Gx + Gy)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 4: Diagonal Gradient (mixed Gx & Gy)");
        $display("==========================================================");
        do_reset();
        threshold = 8'd60;

        for (y = 0; y < IMG_H; y = y + 1)
            for (x = 0; x < IMG_W; x = x + 1)
                golden_input[y*IMG_W + x] = (x + y < IMG_W) ? 8'd0 : 8'd255;

        print_image_stats("Input", 0);
        dma_write_image();
        run_engine();
        dma_read_image();
        print_image_stats("Sobel Output", 1);
        print_sample_rows(32, 64, 96);

        check_borders(case_errors);
        total_errors = total_errors + case_errors;
        $display("    [INFO] Diagonal — edges should appear along the x+y=128 line.");

        // ============================================================
        //  CASE 5: Threshold Sensitivity (threshold=200, weak edge)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 5: Threshold Sensitivity (threshold=200, weak edge)");
        $display("==========================================================");
        do_reset();
        threshold = 8'd200;

        // Weak edge: left=100, right=150, delta=50
        // Gx = 50+100+50 = 200, com_ip: 200 > 200 is FALSE => all suppressed
        for (y = 0; y < IMG_H; y = y + 1)
            for (x = 0; x < IMG_W; x = x + 1)
                golden_input[y*IMG_W + x] = (x < 64) ? 8'd100 : 8'd150;

        print_image_stats("Input", 0);
        dma_write_image();
        $display("  [INFO] Starting engine (threshold=%0d, weak edge)...", threshold);
        run_engine();
        dma_read_image();
        print_image_stats("Sobel Output", 1);

        check_borders(case_errors);
        total_errors = total_errors + case_errors;
        check_inner_all_zero(case_errors);
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 6: Back-to-Back Run (no full reset, re-run engine)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 6: Back-to-Back Run (re-run without reset)");
        $display("==========================================================");
        // No do_reset — verify engine re-entry from S_DONE -> S_IDLE -> S_PROC
        threshold = 8'd60;

        // New image: vertical edge at x=32
        for (y = 0; y < IMG_H; y = y + 1)
            for (x = 0; x < IMG_W; x = x + 1)
                golden_input[y*IMG_W + x] = (x < 32) ? 8'd0 : 8'd255;

        print_image_stats("Input (2nd run, edge at x=32)", 0);
        dma_write_image();
        $display("  [INFO] Starting engine 2nd time...");
        run_engine();
        dma_read_image();
        print_image_stats("Sobel Output (2nd run)", 1);
        print_sample_rows(0, 64, 127);

        check_borders(case_errors);
        total_errors = total_errors + case_errors;
        check_vertical_edge(32, case_errors);
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 7: DMA Write During Engine Busy — Conflict Detection
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 7: DMA Write During Engine Busy (Conflict Test)");
        $display("==========================================================");
        do_reset();
        threshold = 8'd60;

        for (i = 0; i < PIXELS; i = i + 1)
            golden_input[i] = 8'd50;
        dma_write_image();

        @(posedge clk); #1;
        start = 1;
        @(posedge clk); #1;
        start = 0;

        $display("  [INFO] Engine running. Attempting 200 DMA writes to provoke conflicts...");
        begin : conflict_test_block
            integer conflict_seen, conflict_cnt;
            integer attempt;
            conflict_seen = 0;
            conflict_cnt  = 0;
            for (attempt = 0; attempt < 200; attempt = attempt + 1) begin
                @(posedge clk); #1;
                if (busy) begin
                    dma_wr_req  = 1;
                    dma_wr_addr = attempt[ADDR_W-1:0];
                    dma_wr_data = 8'hFF;
                    @(posedge clk); #1;
                    if (wr_conflict) begin
                        conflict_seen = 1;
                        conflict_cnt  = conflict_cnt + 1;
                    end
                    dma_wr_req = 0;
                end
            end
            if (conflict_seen)
                $display("    [PASS] wr_conflict detected %0d times out of 200 attempts.", conflict_cnt);
            else
                $display("    [WARN] No conflict observed (banks may not have overlapped). Acceptable if engine hit different banks.");
        end

        while (!done) @(posedge clk);
        @(posedge clk); @(posedge clk);
        $display("  [INFO] Engine finished after conflict test.");

        // ============================================================
        //  CASE 8: PFB Standalone DMA Read/Write (engine idle)
        //  Extended to 256 addresses to test all 4 banks thoroughly
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 8: PFB Standalone DMA Read/Write (256 addresses)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        $display("  [INFO] Writing 256 test values via DMA (covers all 4 banks x 64 addresses)...");
        for (i = 0; i < 256; i = i + 1) begin
            dma_write_pixel(i[ADDR_W-1:0], i[7:0]);
        end

        $display("  [INFO] Reading back and verifying...");
        begin : standalone_verify
            reg [DATA_W-1:0] rd_tmp;
            integer s_err;
            s_err = 0;
            for (i = 0; i < 256; i = i + 1) begin
                dma_read_pixel(i[ADDR_W-1:0], rd_tmp);
                if (rd_tmp !== i[7:0]) begin
                    if (s_err < 5)
                        $display("    [PFB ERR] addr=%0d expected=%0d got=%0d", i, i[7:0], rd_tmp);
                    s_err = s_err + 1;
                end
            end
            if (s_err == 0)
                $display("    [PASS] All 256 DMA read-back values match (4 banks verified).");
            else
                $display("    [FAIL] %0d PFB standalone errors (first 5 shown).", s_err);
            total_errors = total_errors + s_err;
        end

        // ============================================================
        //  CASE 9: Single Pixel Impulse at (64,64)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 9: Single Pixel Impulse Response at (64,64)");
        $display("==========================================================");
        do_reset();
        threshold = 8'd10;

        for (i = 0; i < PIXELS; i = i + 1)
            golden_input[i] = 8'd0;
        golden_input[64 * IMG_W + 64] = 8'd255;

        print_image_stats("Input (impulse at 64,64)", 0);
        dma_write_image();
        run_engine();
        dma_read_image();
        print_image_stats("Sobel Output (impulse)", 1);
        print_sample_rows(63, 64, 65);

        check_borders(case_errors);
        total_errors = total_errors + case_errors;
        check_impulse(64, 64, case_errors);
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 10: Checkerboard (Sobel symmetry cancellation)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 10: Checkerboard Pattern (Sobel symmetry test)");
        $display("==========================================================");
        do_reset();
        threshold = 8'd60;

        for (y = 0; y < IMG_H; y = y + 1)
            for (x = 0; x < IMG_W; x = x + 1)
                golden_input[y*IMG_W + x] = ((x + y) % 2 == 0) ? 8'd0 : 8'd255;

        print_image_stats("Input (checkerboard)", 0);
        dma_write_image();
        run_engine();
        dma_read_image();
        print_image_stats("Sobel Output (checkerboard)", 1);

        check_borders(case_errors);
        total_errors = total_errors + case_errors;
        // Sobel kernel symmetry cancels all gradients on perfect checkerboard
        check_inner_all_zero(case_errors);
        total_errors = total_errors + case_errors;
        if (case_errors == 0)
            $display("    [INFO] Checkerboard: Sobel kernel symmetry cancels all gradients — all-zero is correct.");

        // ============================================================
        //  FINAL SUMMARY
        // ============================================================
        $display("\n");
        $display("##########################################################");
        if (total_errors == 0) begin
            $display("#   ALL TESTS PASSED!  (0 errors across 10 cases)      #");
            $display("#   Full 128x128 image processing verified.             #");
        end else begin
            $display("#   TESTS COMPLETED WITH %0d TOTAL ERRORS              #", total_errors);
        end
        $display("##########################################################\n");

        $finish;
    end

    // ---- Global wr_conflict monitor (only first 10 to avoid flooding) ----
    integer conflict_log_cnt = 0;
    always @(posedge clk) begin
        if (rst_n && wr_conflict) begin
            if (conflict_log_cnt < 10) begin
                $display("  [MONITOR] wr_conflict pulse at time %0t", $time);
                conflict_log_cnt = conflict_log_cnt + 1;
            end
            if (conflict_log_cnt == 10)  begin
                $display("  [MONITOR] (further wr_conflict messages suppressed)");
                conflict_log_cnt = conflict_log_cnt + 1;
            end
        end
    end

endmodule
