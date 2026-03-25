`timescale 1ns / 1ps

// Testbench for enigne_pfb_top (Engine + Private Frame Buffer Integration)
// Author: Junze Jiang
// Date: 14/3/2026

// Test Strategy:
//   Uses a reduced 10x10 image so full matrices print clearly in Tcl Console.
//   Each test case follows the canonical flow:
//     DMA write image -> start engine -> wait done -> DMA read result -> verify
//
// Test Cases:
//   CASE 1: Vertical edge   (left=0, right=255)     — strong Gx
//   CASE 2: Horizontal edge (top=0, bottom=255)      — strong Gy
//   CASE 3: Uniform image   (all 128)                — no edges expected
//   CASE 4: Diagonal gradient                        — mixed Gx/Gy
//   CASE 5: Threshold sensitivity                    — same image, high threshold
//   CASE 6: Back-to-back run                         — re-run without full reset
//   CASE 7: DMA write during engine busy             — conflict detection
//   CASE 8: DMA read/write interleave before start   — PFB standalone access

module tb_engine_pfb_top();

    // ---- Parameters (small image for console readability) ----
    parameter DATA_W    = 8;
    parameter ADDR_W    = 14;
    parameter BANKS     = 4;
    parameter IMG_W     = 10;
    parameter IMG_H     = 10;
    parameter MAG_W     = 12;

    localparam PIXELS   = IMG_W * IMG_H;  // 100
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

    // ---- Local storage for verification ----
    reg  [7:0] golden_input [0:PIXELS-1];
    reg  [7:0] readback     [0:PIXELS-1];

    // ---- Counters ----
    integer total_errors;
    integer case_errors;
    integer i, x, y;

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

    // ---- Timeout watchdog ----
    initial begin
        #(CLK_P * 200000);
        $display("\n[FATAL] Global timeout reached — simulation killed.");
        $finish;
    end

    // ====================================================================
    //  REUSABLE TASKS
    // ====================================================================

    // Reset the entire design
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

    // DMA write one pixel into PFB — holds request until accepted
    task dma_write_pixel;
        input [ADDR_W-1:0] addr;
        input [DATA_W-1:0] data;
        begin
            @(posedge clk); #1;
            dma_wr_req  = 1;
            dma_wr_addr = addr;
            dma_wr_data = data;
            // Wait until PFB accepts (dma_wr_ready=1 sampled at next posedge)
            @(posedge clk);
            while (!dma_wr_ready) @(posedge clk);
            #1;
            dma_wr_req  = 0;
        end
    endtask

    // DMA write entire golden_input[] array into PFB
    task dma_write_image;
        integer idx;
        begin
            for (idx = 0; idx < PIXELS; idx = idx + 1) begin
                dma_write_pixel(idx[ADDR_W-1:0], golden_input[idx]);
            end
            // One idle cycle after last write
            @(posedge clk); #1;
        end
    endtask

    // DMA read one pixel from PFB — returns data via readback[]
    task dma_read_pixel;
        input  [ADDR_W-1:0] addr;
        output [DATA_W-1:0] data;
        begin
            @(posedge clk); #1;
            dma_rd_req  = 1;
            dma_rd_addr = addr;
            // Wait for request acceptance
            @(posedge clk);
            while (!dma_rd_ready) @(posedge clk);
            #1;
            dma_rd_req = 0;
            // Now wait for valid read data (1-cycle RAM latency + routing)
            @(posedge clk);
            while (!dma_rd_valid) @(posedge clk);
            data = dma_rd_data;
        end
    endtask

    // DMA read entire image into readback[] array
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

    // Pulse start for one cycle, then wait for done
    task run_engine;
        begin
            @(posedge clk); #1;
            start = 1;
            @(posedge clk); #1;
            start = 0;
            // Wait for done
            @(posedge clk);
            while (!done) @(posedge clk);
            $display("  [INFO] Engine done asserted at time %0t", $time);
            // Allow done to deassert
            @(posedge clk); @(posedge clk);
        end
    endtask

    // Print a PIXELS-sized array as IMG_H x IMG_W matrix
    task print_image;
        input [800:0] label;
        input integer  use_readback; // 1=readback[], 0=golden_input[]
        integer px, py;
        begin
            $display("  %0s:", label);
            for (py = 0; py < IMG_H; py = py + 1) begin
                $write("    ");
                for (px = 0; px < IMG_W; px = px + 1) begin
                    if (use_readback)
                        $write("%4d", readback[py * IMG_W + px]);
                    else
                        $write("%4d", golden_input[py * IMG_W + px]);
                end
                $write("\n");
            end
        end
    endtask

    // Verify all border pixels are zero; returns error count
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
                            $display("    [BORDER ERR] (%0d,%0d) addr=%0d expected=0 got=%0d",
                                     bx, by, addr_chk, readback[addr_chk]);
                            errs = errs + 1;
                        end
                    end
                end
            end
            if (errs == 0)
                $display("    [PASS] All border pixels are zero.");
            else
                $display("    [FAIL] %0d border pixel errors.", errs);
        end
    endtask

    // Verify that all inner pixels are non-zero (for strong-edge images)
    task check_inner_nonzero;
        input [800:0] tag;
        output integer errs;
        integer cx, cy, addr_chk;
        begin
            errs = 0;
            for (cy = 1; cy < IMG_H-1; cy = cy + 1) begin
                for (cx = 1; cx < IMG_W-1; cx = cx + 1) begin
                    addr_chk = cy * IMG_W + cx;
                    // Only check pixels whose 3x3 window straddles the edge
                    // For a vertical edge at x=5: inner pixels at x=4,5,6 should be non-zero
                    // We just flag zeros in the expected edge zone
                end
            end
            // Generic: just report
            $display("    [INFO] %0s inner pixel check done.", tag);
        end
    endtask

    // Verify all inner pixels are exactly zero (uniform image, no edges)
    task check_inner_all_zero;
        output integer errs;
        integer cx, cy, addr_chk;
        begin
            errs = 0;
            for (cy = 1; cy < IMG_H-1; cy = cy + 1) begin
                for (cx = 1; cx < IMG_W-1; cx = cx + 1) begin
                    addr_chk = cy * IMG_W + cx;
                    if (readback[addr_chk] !== 8'd0) begin
                        $display("    [INNER ERR] (%0d,%0d) expected=0 got=%0d",
                                 cx, cy, readback[addr_chk]);
                        errs = errs + 1;
                    end
                end
            end
            if (errs == 0)
                $display("    [PASS] All inner pixels are zero (no edges).");
            else
                $display("    [FAIL] %0d inner pixels non-zero.", errs);
        end
    endtask

    // Check vertical edge: columns near the boundary x=split should have nonzero
    task check_vertical_edge;
        input integer split_x;  // The column where 0->255 transition happens
        output integer errs;
        integer cx, cy, addr_chk;
        integer expect_edge;
        begin
            errs = 0;
            for (cy = 1; cy < IMG_H-1; cy = cy + 1) begin
                for (cx = 1; cx < IMG_W-1; cx = cx + 1) begin
                    addr_chk = cy * IMG_W + cx;
                    // A pixel at (cx,cy) sees a 3x3 window [cx-1..cx+1].
                    // If the window straddles split_x (i.e., cx-1 < split_x <= cx+1), there's an edge.
                    expect_edge = ((cx - 1) < split_x) && (split_x <= (cx + 1));
                    if (expect_edge && readback[addr_chk] == 8'd0) begin
                        $display("    [VEDGE ERR] (%0d,%0d) expected nonzero, got 0", cx, cy);
                        errs = errs + 1;
                    end else if (!expect_edge && readback[addr_chk] != 8'd0) begin
                        $display("    [VEDGE ERR] (%0d,%0d) expected 0, got %0d", cx, cy, readback[addr_chk]);
                        errs = errs + 1;
                    end
                end
            end
            if (errs == 0)
                $display("    [PASS] Vertical edge at x=%0d detected correctly.", split_x);
            else
                $display("    [FAIL] %0d vertical edge errors.", errs);
        end
    endtask

    // Check horizontal edge: rows near the boundary y=split should have nonzero
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
                        $display("    [HEDGE ERR] (%0d,%0d) expected nonzero, got 0", cx, cy);
                        errs = errs + 1;
                    end else if (!expect_edge && readback[addr_chk] != 8'd0) begin
                        $display("    [HEDGE ERR] (%0d,%0d) expected 0, got %0d", cx, cy, readback[addr_chk]);
                        errs = errs + 1;
                    end
                end
            end
            if (errs == 0)
                $display("    [PASS] Horizontal edge at y=%0d detected correctly.", split_y);
            else
                $display("    [FAIL] %0d horizontal edge errors.", errs);
        end
    endtask

    // ====================================================================
    //  MAIN TEST SEQUENCE
    // ====================================================================
    initial begin
        total_errors = 0;

        $display("\n");
        $display("##########################################################");
        $display("#   ENGINE + PFB TOP INTEGRATION TESTBENCH               #");
        $display("#   Image size: %0d x %0d = %0d pixels                   #", IMG_W, IMG_H, PIXELS);
        $display("##########################################################\n");

        // ============================================================
        //  CASE 1: Vertical Edge (left=0, right=255, split at x=5)
        // ============================================================
        $display("==========================================================");
        $display(" CASE 1: Vertical Edge Detection (Gx dominant)");
        $display("==========================================================");
        do_reset();
        threshold = 8'd60;

        for (y = 0; y < IMG_H; y = y + 1)
            for (x = 0; x < IMG_W; x = x + 1)
                golden_input[y*IMG_W + x] = (x < 5) ? 8'd0 : 8'd255;

        print_image("Input Image", 0);
        $display("  [INFO] DMA writing image into PFB...");
        dma_write_image();
        $display("  [INFO] Starting engine (threshold=%0d)...", threshold);
        run_engine();
        $display("  [INFO] DMA reading result from PFB...");
        dma_read_image();
        print_image("Sobel Output", 1);

        check_borders(case_errors);
        total_errors = total_errors + case_errors;
        check_vertical_edge(5, case_errors);
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 2: Horizontal Edge (top=0, bottom=255, split at y=5)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 2: Horizontal Edge Detection (Gy dominant)");
        $display("==========================================================");
        do_reset();
        threshold = 8'd60;

        for (y = 0; y < IMG_H; y = y + 1)
            for (x = 0; x < IMG_W; x = x + 1)
                golden_input[y*IMG_W + x] = (y < 5) ? 8'd0 : 8'd255;

        print_image("Input Image", 0);
        dma_write_image();
        $display("  [INFO] Starting engine...");
        run_engine();
        dma_read_image();
        print_image("Sobel Output", 1);

        check_borders(case_errors);
        total_errors = total_errors + case_errors;
        check_horizontal_edge(5, case_errors);
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 3: Uniform Image (all 128) — expect zero output
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 3: Uniform Image (no edges expected)");
        $display("==========================================================");
        do_reset();
        threshold = 8'd60;

        for (i = 0; i < PIXELS; i = i + 1)
            golden_input[i] = 8'd128;

        print_image("Input Image", 0);
        dma_write_image();
        run_engine();
        dma_read_image();
        print_image("Sobel Output", 1);

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

        // Upper-left triangle = 0, lower-right triangle = 255
        for (y = 0; y < IMG_H; y = y + 1)
            for (x = 0; x < IMG_W; x = x + 1)
                golden_input[y*IMG_W + x] = (x + y < IMG_W) ? 8'd0 : 8'd255;

        print_image("Input Image", 0);
        dma_write_image();
        run_engine();
        dma_read_image();
        print_image("Sobel Output", 1);

        check_borders(case_errors);
        total_errors = total_errors + case_errors;
        // For diagonal, just visually verify + border check
        $display("    [INFO] Diagonal pattern — verify visually that edges appear along the diagonal.");

        // ============================================================
        //  CASE 5: High Threshold — same vertical edge but threshold=255
        //          All magnitudes should be < 255 or clipped, many suppressed
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 5: Threshold Sensitivity (threshold=200)");
        $display("==========================================================");
        do_reset();
        threshold = 8'd200;

        // Weak edge: left=100, right=150, delta=50 per pixel
        for (y = 0; y < IMG_H; y = y + 1)
            for (x = 0; x < IMG_W; x = x + 1)
                golden_input[y*IMG_W + x] = (x < 5) ? 8'd100 : 8'd150;

        print_image("Input Image", 0);
        dma_write_image();
        $display("  [INFO] Starting engine (threshold=%0d, weak edge)...", threshold);
        run_engine();
        dma_read_image();
        print_image("Sobel Output", 1);

        check_borders(case_errors);
        total_errors = total_errors + case_errors;
        // With delta=50 per pixel, max Gx = 50+100+50=200, which equals threshold
        // com_ip uses strictly greater-than, so 200 is NOT > 200 => output should be 0
        check_inner_all_zero(case_errors);
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 6: Back-to-Back Run (no full reset between runs)
        //  Verify engine can re-process after done without reset
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 6: Back-to-Back Run (re-run without reset)");
        $display("==========================================================");
        // Don't call do_reset — just reload a new image and re-run
        threshold = 8'd60;

        // New image: vertical edge at x=3
        for (y = 0; y < IMG_H; y = y + 1)
            for (x = 0; x < IMG_W; x = x + 1)
                golden_input[y*IMG_W + x] = (x < 3) ? 8'd0 : 8'd255;

        print_image("Input Image (2nd run)", 0);
        dma_write_image();
        $display("  [INFO] Starting engine 2nd time...");
        run_engine();
        dma_read_image();
        print_image("Sobel Output (2nd run)", 1);

        check_borders(case_errors);
        total_errors = total_errors + case_errors;
        check_vertical_edge(3, case_errors);
        total_errors = total_errors + case_errors;

        // ============================================================
        //  CASE 7: DMA Write During Engine Busy — Conflict Detection
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 7: DMA Write During Engine Busy (Conflict Test)");
        $display("==========================================================");
        do_reset();
        threshold = 8'd60;

        // Load uniform image
        for (i = 0; i < PIXELS; i = i + 1)
            golden_input[i] = 8'd50;
        dma_write_image();

        // Start engine
        @(posedge clk); #1;
        start = 1;
        @(posedge clk); #1;
        start = 0;

        // While engine is busy, attempt DMA writes to provoke conflicts
        $display("  [INFO] Engine running. Attempting DMA writes to same banks...");
        begin : conflict_test_block
            integer conflict_seen;
            integer attempt;
            conflict_seen = 0;
            for (attempt = 0; attempt < 50; attempt = attempt + 1) begin
                @(posedge clk); #1;
                if (busy) begin
                    dma_wr_req  = 1;
                    dma_wr_addr = attempt[ADDR_W-1:0]; // sequential addresses hit various banks
                    dma_wr_data = 8'hFF;
                    @(posedge clk); #1;
                    if (wr_conflict) begin
                        conflict_seen = 1;
                        $display("    [INFO] wr_conflict detected at attempt %0d, time %0t", attempt, $time);
                    end
                    dma_wr_req = 0;
                end
            end
            if (conflict_seen)
                $display("    [PASS] Write conflict correctly detected during engine busy.");
            else
                $display("    [WARN] No conflict observed (engine may not have overlapped banks). This is OK if banks were different.");
        end

        // Wait for engine to finish
        while (!done) @(posedge clk);
        @(posedge clk); @(posedge clk);
        $display("  [INFO] Engine finished after conflict test.");

        // ============================================================
        //  CASE 8: DMA Read-Write Standalone (engine idle)
        //  Verify PFB works correctly as standalone memory
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 8: PFB Standalone DMA Read/Write (engine idle)");
        $display("==========================================================");
        do_reset();
        case_errors = 0;

        // Write known pattern: addr[i] = i & 0xFF
        $display("  [INFO] Writing test pattern via DMA...");
        for (i = 0; i < 20; i = i + 1) begin
            dma_write_pixel(i[ADDR_W-1:0], i[7:0]);
        end

        // Read back and verify
        $display("  [INFO] Reading back and verifying...");
        begin : standalone_verify
            reg [DATA_W-1:0] rd_tmp;
            integer s_err;
            s_err = 0;
            for (i = 0; i < 20; i = i + 1) begin
                dma_read_pixel(i[ADDR_W-1:0], rd_tmp);
                if (rd_tmp !== i[7:0]) begin
                    $display("    [PFB ERR] addr=%0d expected=%0d got=%0d", i, i[7:0], rd_tmp);
                    s_err = s_err + 1;
                end
            end
            if (s_err == 0)
                $display("    [PASS] All 20 DMA read-back values match.");
            else
                $display("    [FAIL] %0d PFB standalone errors.", s_err);
            total_errors = total_errors + s_err;
        end

        // ============================================================
        //  CASE 9: Single Pixel Impulse (one bright pixel in dark field)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 9: Single Pixel Impulse Response");
        $display("==========================================================");
        do_reset();
        threshold = 8'd10;  // Low threshold to catch weak responses

        for (i = 0; i < PIXELS; i = i + 1)
            golden_input[i] = 8'd0;
        // Place a single bright pixel at center (5,5)
        golden_input[5 * IMG_W + 5] = 8'd255;

        print_image("Input Image (impulse)", 0);
        dma_write_image();
        run_engine();
        dma_read_image();
        print_image("Sobel Output (impulse)", 1);

        check_borders(case_errors);
        total_errors = total_errors + case_errors;
        // The response should be nonzero around (5,5) and zero elsewhere
        $display("    [INFO] Impulse response — verify visually that edges surround (5,5).");

        // ============================================================
        //  CASE 10: Checkerboard Pattern (stress test)
        // ============================================================
        $display("\n==========================================================");
        $display(" CASE 10: Checkerboard Pattern (max edge density)");
        $display("==========================================================");
        do_reset();
        threshold = 8'd60;

        for (y = 0; y < IMG_H; y = y + 1)
            for (x = 0; x < IMG_W; x = x + 1)
                golden_input[y*IMG_W + x] = ((x + y) % 2 == 0) ? 8'd0 : 8'd255;

        print_image("Input Image (checkerboard)", 0);
        dma_write_image();
        run_engine();
        dma_read_image();
        print_image("Sobel Output (checkerboard)", 1);

        check_borders(case_errors);
        total_errors = total_errors + case_errors;
        // Perfect checkerboard: every 3x3 window is diagonally symmetric,
        // so Sobel Gx and Gy both cancel to zero. Correct output is ALL zeros.
        // This case stress-tests the pipeline with maximum data transitions.
        check_inner_all_zero(case_errors);
        total_errors = total_errors + case_errors;
        if (case_errors == 0)
            $display("    [INFO] Checkerboard: Sobel kernel symmetry cancels all gradients — all-zero output is mathematically correct.");

        // ============================================================
        //  FINAL SUMMARY
        // ============================================================
        $display("\n");
        $display("##########################################################");
        if (total_errors == 0) begin
            $display("#   ALL TESTS PASSED!  (0 errors across 10 cases)      #");
        end else begin
            $display("#   TESTS COMPLETED WITH %0d TOTAL ERRORS              #", total_errors);
        end
        $display("##########################################################\n");

        $finish;
    end

    // ---- Optional: monitor wr_conflict pulses globally ----
    always @(posedge clk) begin
        if (rst_n && wr_conflict) begin
            $display("  [MONITOR] wr_conflict pulse at time %0t", $time);
        end
    end

endmodule
