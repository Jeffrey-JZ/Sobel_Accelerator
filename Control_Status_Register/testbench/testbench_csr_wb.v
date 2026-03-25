`timescale 1ns / 1ps

// Testbench for csr_wb (Wishbone CSR block)
// Author: Junze Jiang
// Date: 17/3/2026

// Register Map:
//   0x00 CTRL        [0]=start(W1P), write [1]=clear done_latched, write [2]=clear err_latched
//   0x04 SRC_BASE    source base address
//   0x08 DST_BASE    destination base address
//   0x0C PIXEL_COUNT pixel count
//   0x10 STATUS      [0]=busy, [1]=done|done_latched, [2]=error|err_latched
//   0x14 THRESHOLD  [7:0]=sobel threshold

// Test Cases:
//   CASE  1: Reset state verification
//   CASE  2: Write & readback SRC_BASE register
//   CASE  3: Write & readback DST_BASE register
//   CASE  4: Write & readback PIXEL_COUNT register
//   CASE  5: CTRL write-1-pulse start (dma_start_pulse is one-cycle)
//   CASE  6: Read CTRL returns zero
//   CASE  7: STATUS read reflects live dma_busy/dma_done/dma_error
//   CASE  8: done_latched sticky behavior (pulse -> latch -> read -> clear)
//   CASE  9: err_latched sticky behavior
//   CASE 10: Clear done_latched and err_latched via CTRL write
//   CASE 11: wb_ack_o is exactly one-cycle per transaction
//   CASE 12: No ack when cyc or stb deasserted
//   CASE 13: Back-to-back write-read transactions
//   CASE 14: wb_sel_i byte-enable gating on CTRL start
//   CASE 15: Write to invalid register address (no side-effect)
//   CASE 16: Read from invalid register address (returns zero)
//   CASE 17: Start pulse NOT generated when writing 0 to CTRL[0]
//   CASE 18: Simultaneous dma_done pulse during STATUS read
//   CASE 19: Overwrite registers while DMA busy
//   CASE 20: Full transaction flow simulation
//   CASE 21: Write & Readback THRESHOLD Register

module tb_csr_wb();

    // Parameters
    parameter WB_ADDR_W     = 8;
    parameter WB_DATA_W     = 32;
    parameter AXI_ADDR_W    = 32;
    parameter PIXELS_MAX_W  = 16;
    parameter CLK_P         = 10;

    // Address byte offsets (matching reg_idx = wb_adr_i[5:2])
    localparam [WB_ADDR_W-1:0] ADDR_CTRL   = 8'h00;  // reg_idx = 0
    localparam [WB_ADDR_W-1:0] ADDR_SRC    = 8'h04;  // reg_idx = 1
    localparam [WB_ADDR_W-1:0] ADDR_DST    = 8'h08;  // reg_idx = 2
    localparam [WB_ADDR_W-1:0] ADDR_COUNT  = 8'h0C;  // reg_idx = 3
    localparam [WB_ADDR_W-1:0] ADDR_STATUS = 8'h10;  // reg_idx = 4
    localparam [WB_ADDR_W-1:0] ADDR_THRESH = 8'h14;  // reg_idx = 5 (threshold)
    localparam [WB_ADDR_W-1:0] ADDR_INVAL  = 8'h18;  // reg_idx = 6 (invalid)

    // DUT signals
    reg                         wb_clk_i;
    reg                         wb_rst_i;
    reg  [WB_ADDR_W     - 1:0]  wb_adr_i;
    reg  [WB_DATA_W     - 1:0]  wb_dat_i;
    wire [WB_DATA_W     - 1:0]  wb_dat_o;
    reg  [(WB_DATA_W/8) - 1:0]  wb_sel_i;
    reg                         wb_we_i;
    reg                         wb_stb_i;
    reg                         wb_cyc_i;
    wire                        wb_ack_o;

    wire                        dma_start_pulse;
    wire [AXI_ADDR_W - 1:0]     dma_src_base_addr;
    wire [AXI_ADDR_W - 1:0]     dma_dst_base_addr;
    wire [PIXELS_MAX_W - 1:0]   dma_pixel_count;
    wire [7:0]                  threshold;
    reg                         dma_busy;
    reg                         dma_done;
    reg                         dma_error;

    // Error tracking
    integer error_cnt;
    integer case_errors;

    // DUT instantiation
    csr_wb #(
        .WB_ADDR_W          (WB_ADDR_W),
        .WB_DATA_W          (WB_DATA_W),
        .AXI_ADDR_W         (AXI_ADDR_W),
        .PIXELS_MAX_W       (PIXELS_MAX_W)
    ) dut (
        .wb_clk_i           (wb_clk_i),
        .wb_rst_i           (wb_rst_i),
        .wb_adr_i           (wb_adr_i),
        .wb_dat_i           (wb_dat_i),
        .wb_dat_o           (wb_dat_o),
        .wb_sel_i           (wb_sel_i),
        .wb_we_i            (wb_we_i),
        .wb_stb_i           (wb_stb_i),
        .wb_cyc_i           (wb_cyc_i),
        .wb_ack_o           (wb_ack_o),
        .dma_start_pulse    (dma_start_pulse),
        .dma_src_base_addr  (dma_src_base_addr),
        .dma_dst_base_addr  (dma_dst_base_addr),
        .dma_pixel_count    (dma_pixel_count),
        .dma_busy           (dma_busy),
        .dma_done           (dma_done),
        .dma_error          (dma_error),
        .threshold          (threshold)
    );

    // Clock
    initial wb_clk_i = 0;
    always #(CLK_P/2) wb_clk_i = ~wb_clk_i;

    // Timeout watchdog
    initial begin
        #(CLK_P * 5000);
        $display("\n[FATAL] Global timeout — simulation killed.");
        $finish;
    end

    // ================================================================
    //  REUSABLE TASKS
    // ================================================================

    task do_reset;
        begin
            wb_rst_i  = 1;
            wb_adr_i  = 0;
            wb_dat_i  = 0;
            wb_sel_i  = 4'hF;
            wb_we_i   = 0;
            wb_stb_i  = 0;
            wb_cyc_i  = 0;
            dma_busy  = 0;
            dma_done  = 0;
            dma_error = 0;
            #(CLK_P * 3);
            wb_rst_i = 0;
            #(CLK_P);
        end
    endtask

    // Wishbone single write cycle: assert cyc+stb+we, wait for ack, deassert
    task wb_write;
        input [WB_ADDR_W-1:0]     addr;
        input [WB_DATA_W-1:0]     data;
        input [WB_DATA_W/8-1:0]   sel;
        begin
            @(posedge wb_clk_i); #1;
            wb_cyc_i = 1;
            wb_stb_i = 1;
            wb_we_i  = 1;
            wb_adr_i = addr;
            wb_dat_i = data;
            wb_sel_i = sel;
            // Wait for ack
            @(posedge wb_clk_i);
            while (!wb_ack_o) @(posedge wb_clk_i);
            #1;
            // Deassert bus after ack
            wb_cyc_i = 0;
            wb_stb_i = 0;
            wb_we_i  = 0;
        end
    endtask

    // Wishbone single read cycle: assert cyc+stb, wait for ack, capture data
    task wb_read;
        input  [WB_ADDR_W-1:0]    addr;
        output [WB_DATA_W-1:0]    data;
        begin
            @(posedge wb_clk_i); #1;
            wb_cyc_i = 1;
            wb_stb_i = 1;
            wb_we_i  = 0;
            wb_adr_i = addr;
            wb_sel_i = 4'hF;
            // Wait for ack
            @(posedge wb_clk_i);
            while (!wb_ack_o) @(posedge wb_clk_i);
            data = wb_dat_o;
            #1;
            wb_cyc_i = 0;
            wb_stb_i = 0;
        end
    endtask

    // Check helper
    task check;
        input           condition;
        input [800:0]   msg;
        begin
            if (!condition) begin
                $display("    [FAIL] %0s  (time %0t)", msg, $time);
                error_cnt = error_cnt + 1;
                case_errors = case_errors + 1;
            end else begin
                $display("    [PASS] %0s", msg);
            end
        end
    endtask

    task case_header;
        input [800:0] title;
        begin
            $display("\n----------------------------------------------------------");
            $display(" %0s", title);
            $display("----------------------------------------------------------");
            case_errors = 0;
        end
    endtask

    // ================================================================
    //  MAIN TEST
    // ================================================================
    reg [WB_DATA_W-1:0] rd_data;

    initial begin
        error_cnt = 0;

        $display("\n");
        $display("##########################################################");
        $display("#   CSR WISHBONE MODULE TESTBENCH                        #");
        $display("##########################################################\n");

        // ============================================================
        //  CASE 1: Reset State Verification
        // ============================================================
        case_header("CASE 1: Reset State Verification");
        do_reset();
        check(wb_ack_o          === 1'b0, "wb_ack_o = 0 after reset");
        check(dma_start_pulse   === 1'b0, "dma_start_pulse = 0 after reset");
        check(dma_src_base_addr === 32'd0, "dma_src_base_addr = 0 after reset");
        check(dma_dst_base_addr === 32'd0, "dma_dst_base_addr = 0 after reset");
        check(dma_pixel_count   === 16'd0, "dma_pixel_count = 0 after reset");
        check(threshold         === 8'd0,  "threshold = 0 after reset");

        // ============================================================
        //  CASE 2: Write & Readback SRC_BASE
        // ============================================================
        case_header("CASE 2: Write & Readback SRC_BASE Register");
        do_reset();
        wb_write(ADDR_SRC, 32'hDEAD_BEEF, 4'hF);
        check(dma_src_base_addr === 32'hDEAD_BEEF, "SRC_BASE output matches written value");
        wb_read(ADDR_SRC, rd_data);
        check(rd_data === 32'hDEAD_BEEF, "SRC_BASE readback matches 0xDEAD_BEEF");

        // ============================================================
        //  CASE 3: Write & Readback DST_BASE
        // ============================================================
        case_header("CASE 3: Write & Readback DST_BASE Register");
        do_reset();
        wb_write(ADDR_DST, 32'hCAFE_1234, 4'hF);
        check(dma_dst_base_addr === 32'hCAFE_1234, "DST_BASE output matches written value");
        wb_read(ADDR_DST, rd_data);
        check(rd_data === 32'hCAFE_1234, "DST_BASE readback matches 0xCAFE_1234");

        // ============================================================
        //  CASE 4: Write & Readback PIXEL_COUNT
        // ============================================================
        case_header("CASE 4: Write & Readback PIXEL_COUNT Register");
        do_reset();
        wb_write(ADDR_COUNT, 32'h0000_4000, 4'hF);  // 16384 pixels = 128x128
        check(dma_pixel_count === 16'h4000, "PIXEL_COUNT output matches 16384");
        wb_read(ADDR_COUNT, rd_data);
        check(rd_data === 32'h0000_4000, "PIXEL_COUNT readback correct (upper bits zero-padded)");

        // Write max 16-bit value
        wb_write(ADDR_COUNT, 32'h0000_FFFF, 4'hF);
        check(dma_pixel_count === 16'hFFFF, "PIXEL_COUNT accepts max 16-bit value");

        // Write with upper bits set — only lower 16 bits should be stored
        wb_write(ADDR_COUNT, 32'hFFFF_1234, 4'hF);
        check(dma_pixel_count === 16'h1234, "PIXEL_COUNT ignores upper 16 bits of write data");
        wb_read(ADDR_COUNT, rd_data);
        check(rd_data === 32'h0000_1234, "PIXEL_COUNT readback has upper bits zeroed");

        // ============================================================
        //  CASE 5: CTRL Write-1-Pulse Start
        // ============================================================
        case_header("CASE 5: CTRL Start Pulse (W1P behavior)");
        do_reset();

        // Write 1 to CTRL[0] -> dma_start_pulse should go high for exactly 1 cycle
        // Manual bus timing (avoid wb_write task's while-loop which misses the pulse)
        @(posedge wb_clk_i); #1;
        wb_cyc_i = 1; wb_stb_i = 1; wb_we_i = 1;
        wb_adr_i = ADDR_CTRL; wb_dat_i = 32'h0000_0001; wb_sel_i = 4'hF;

        @(posedge wb_clk_i); #1;
        // After this posedge + NBA resolve: ack=1, start_pulse=1
        check(dma_start_pulse === 1'b1, "dma_start_pulse HIGH on ack cycle");
        wb_cyc_i = 0; wb_stb_i = 0; wb_we_i = 0;

        @(posedge wb_clk_i); #1;
        // Default clears start_pulse to 0
        check(dma_start_pulse === 1'b0, "dma_start_pulse LOW one cycle after (W1P)");

        // ============================================================
        //  CASE 6: Read CTRL Returns Zero
        // ============================================================
        case_header("CASE 6: Read CTRL Returns Zero");
        do_reset();
        wb_read(ADDR_CTRL, rd_data);
        check(rd_data === 32'd0, "CTRL read returns 0x0000_0000");

        // ============================================================
        //  CASE 7: STATUS Read — Live Signals
        // ============================================================
        case_header("CASE 7: STATUS Read Reflects Live dma_busy/done/error");
        do_reset();

        // All idle
        wb_read(ADDR_STATUS, rd_data);
        check(rd_data === 32'd0, "STATUS=0 when all idle");

        // Assert dma_busy
        dma_busy = 1;
        @(posedge wb_clk_i); #1;
        wb_read(ADDR_STATUS, rd_data);
        check(rd_data[0] === 1'b1, "STATUS[0]=1 when dma_busy=1");
        check(rd_data[1] === 1'b0, "STATUS[1]=0 when dma_done=0");
        check(rd_data[2] === 1'b0, "STATUS[2]=0 when dma_error=0");
        dma_busy = 0;

        // Assert dma_done (live, not latched yet by edge)
        dma_done = 1;
        @(posedge wb_clk_i); #1;  // let done_latched capture
        wb_read(ADDR_STATUS, rd_data);
        check(rd_data[1] === 1'b1, "STATUS[1]=1 when dma_done=1");
        dma_done = 0;

        // Assert dma_error
        dma_error = 1;
        @(posedge wb_clk_i); #1;
        wb_read(ADDR_STATUS, rd_data);
        check(rd_data[2] === 1'b1, "STATUS[2]=1 when dma_error=1");
        dma_error = 0;

        // ============================================================
        //  CASE 8: done_latched Sticky Behavior
        // ============================================================
        case_header("CASE 8: done_latched Sticky Behavior");
        do_reset();

        // Pulse dma_done for 1 cycle
        @(posedge wb_clk_i); #1;
        dma_done = 1;
        @(posedge wb_clk_i); #1;
        dma_done = 0;
        @(posedge wb_clk_i); #1;

        // done_latched should keep STATUS[1] high even after dma_done goes low
        wb_read(ADDR_STATUS, rd_data);
        check(rd_data[1] === 1'b1, "STATUS[1]=1 after done pulse (latched)");

        // Wait a few cycles, re-read: still latched
        @(posedge wb_clk_i); @(posedge wb_clk_i); #1;
        wb_read(ADDR_STATUS, rd_data);
        check(rd_data[1] === 1'b1, "STATUS[1]=1 still latched after multiple cycles");

        // ============================================================
        //  CASE 9: err_latched Sticky Behavior
        // ============================================================
        case_header("CASE 9: err_latched Sticky Behavior");
        do_reset();

        // Pulse dma_error for 1 cycle
        @(posedge wb_clk_i); #1;
        dma_error = 1;
        @(posedge wb_clk_i); #1;
        dma_error = 0;
        @(posedge wb_clk_i); #1;

        wb_read(ADDR_STATUS, rd_data);
        check(rd_data[2] === 1'b1, "STATUS[2]=1 after error pulse (latched)");

        // ============================================================
        //  CASE 10: Clear done_latched & err_latched via CTRL Write
        // ============================================================
        case_header("CASE 10: Clear Latched Status via CTRL Write");
        do_reset();

        // Latch both done and error
        @(posedge wb_clk_i); #1;
        dma_done = 1; dma_error = 1;
        @(posedge wb_clk_i); #1;
        dma_done = 0; dma_error = 0;
        @(posedge wb_clk_i); #1;

        wb_read(ADDR_STATUS, rd_data);
        check(rd_data[2:1] === 2'b11, "Both done_latched and err_latched are set");

        // Clear done_latched by writing bit[1]=1 to CTRL
        wb_write(ADDR_CTRL, 32'h0000_0002, 4'hF);
        @(posedge wb_clk_i); #1;
        wb_read(ADDR_STATUS, rd_data);
        check(rd_data[1] === 1'b0, "done_latched cleared after CTRL write bit[1]");
        check(rd_data[2] === 1'b1, "err_latched still set (only cleared bit[1])");

        // Clear err_latched by writing bit[2]=1 to CTRL
        wb_write(ADDR_CTRL, 32'h0000_0004, 4'hF);
        @(posedge wb_clk_i); #1;
        wb_read(ADDR_STATUS, rd_data);
        check(rd_data[2] === 1'b0, "err_latched cleared after CTRL write bit[2]");

        // Clear both simultaneously
        dma_done = 1; dma_error = 1;
        @(posedge wb_clk_i); #1;
        dma_done = 0; dma_error = 0;
        @(posedge wb_clk_i); #1;

        wb_write(ADDR_CTRL, 32'h0000_0006, 4'hF);  // bits [2:1] = 11
        @(posedge wb_clk_i); #1;
        wb_read(ADDR_STATUS, rd_data);
        check(rd_data[2:1] === 2'b00, "Both latches cleared simultaneously");

        // ============================================================
        //  CASE 11: wb_ack_o One-Cycle Assertion
        // ============================================================
        case_header("CASE 11: wb_ack_o Exactly One Cycle Per Transaction");
        do_reset();

        @(posedge wb_clk_i); #1;
        wb_cyc_i = 1; wb_stb_i = 1; wb_we_i = 0;
        wb_adr_i = ADDR_SRC; wb_sel_i = 4'hF;

        // Cycle 1: ack should rise
        @(posedge wb_clk_i); #1;
        check(wb_ack_o === 1'b1, "ack HIGH on first cycle after request");

        // Cycle 2: ack must drop (even if cyc+stb still asserted)
        @(posedge wb_clk_i); #1;
        check(wb_ack_o === 1'b0, "ack LOW on second cycle (single-cycle ack)");

        wb_cyc_i = 0; wb_stb_i = 0;

        // ============================================================
        //  CASE 12: No Ack When CYC or STB Deasserted
        // ============================================================
        case_header("CASE 12: No Ack When CYC or STB Deasserted");
        do_reset();

        // Only CYC, no STB
        @(posedge wb_clk_i); #1;
        wb_cyc_i = 1; wb_stb_i = 0; wb_we_i = 0;
        wb_adr_i = ADDR_SRC;
        @(posedge wb_clk_i); #1;
        check(wb_ack_o === 1'b0, "No ack with CYC=1, STB=0");
        wb_cyc_i = 0;

        // Only STB, no CYC
        @(posedge wb_clk_i); #1;
        wb_cyc_i = 0; wb_stb_i = 1; wb_we_i = 0;
        wb_adr_i = ADDR_SRC;
        @(posedge wb_clk_i); #1;
        check(wb_ack_o === 1'b0, "No ack with CYC=0, STB=1");
        wb_stb_i = 0;

        // Neither
        @(posedge wb_clk_i); #1;
        wb_cyc_i = 0; wb_stb_i = 0;
        @(posedge wb_clk_i); #1;
        check(wb_ack_o === 1'b0, "No ack with CYC=0, STB=0");

        // ============================================================
        //  CASE 13: Back-to-Back Write-Read Transactions
        // ============================================================
        case_header("CASE 13: Back-to-Back Write-Read Transactions");
        do_reset();

        wb_write(ADDR_SRC, 32'h1111_1111, 4'hF);
        wb_write(ADDR_DST, 32'h2222_2222, 4'hF);
        wb_write(ADDR_COUNT, 32'h0000_0064, 4'hF);  // 100 pixels

        wb_read(ADDR_SRC, rd_data);
        check(rd_data === 32'h1111_1111, "Back-to-back: SRC readback correct");
        wb_read(ADDR_DST, rd_data);
        check(rd_data === 32'h2222_2222, "Back-to-back: DST readback correct");
        wb_read(ADDR_COUNT, rd_data);
        check(rd_data === 32'h0000_0064, "Back-to-back: COUNT readback correct");

        // ============================================================
        //  CASE 14: wb_sel_i Byte-Enable Gating on CTRL Start
        // ============================================================
        case_header("CASE 14: wb_sel_i Gating — Start NOT Triggered When sel[0]=0");
        do_reset();

        // Write CTRL with sel[0]=0 -> start should NOT fire even though dat[0]=1
        wb_write(ADDR_CTRL, 32'h0000_0001, 4'hE);  // sel = 4'b1110, byte 0 not selected
        @(posedge wb_clk_i); #1;
        check(dma_start_pulse === 1'b0, "Start pulse NOT generated when wb_sel_i[0]=0");

        // Now write with sel[0]=1 -> should fire
        // Manual bus timing to catch the one-cycle pulse
        @(posedge wb_clk_i); #1;
        wb_cyc_i = 1; wb_stb_i = 1; wb_we_i = 1;
        wb_adr_i = ADDR_CTRL; wb_dat_i = 32'h0000_0001; wb_sel_i = 4'hF;

        @(posedge wb_clk_i); #1;
        // After posedge + NBA: ack=1, start_pulse=1
        check(dma_start_pulse === 1'b1, "Start pulse generated when wb_sel_i[0]=1");
        wb_cyc_i = 0; wb_stb_i = 0; wb_we_i = 0;
        @(posedge wb_clk_i); #1;

        // ============================================================
        //  CASE 15: Write to Invalid Register (No Side-Effect)
        // ============================================================
        case_header("CASE 15: Write to Invalid Register Address");
        do_reset();

        // Pre-load known values
        wb_write(ADDR_SRC, 32'hAAAA_AAAA, 4'hF);
        wb_write(ADDR_DST, 32'hBBBB_BBBB, 4'hF);
        wb_write(ADDR_COUNT, 32'h0000_5678, 4'hF);

        // Write to invalid address
        wb_write(ADDR_INVAL, 32'hFFFF_FFFF, 4'hF);

        // Verify nothing changed
        check(dma_src_base_addr === 32'hAAAA_AAAA, "SRC unchanged after invalid write");
        check(dma_dst_base_addr === 32'hBBBB_BBBB, "DST unchanged after invalid write");
        check(dma_pixel_count   === 16'h5678,       "COUNT unchanged after invalid write");

        // ============================================================
        //  CASE 16: Read from Invalid Register Returns Zero
        // ============================================================
        case_header("CASE 16: Read from Invalid Register Address");
        do_reset();
        wb_read(ADDR_INVAL, rd_data);
        check(rd_data === 32'd0, "Invalid register read returns 0x0000_0000");

        // ============================================================
        //  CASE 17: No Start When Writing 0 to CTRL[0]
        // ============================================================
        case_header("CASE 17: No Start Pulse When Writing 0 to CTRL[0]");
        do_reset();
        wb_write(ADDR_CTRL, 32'h0000_0000, 4'hF);
        @(posedge wb_clk_i); #1;
        check(dma_start_pulse === 1'b0, "No start pulse when CTRL write data[0]=0");

        // Writing only clear bits (no start)
        wb_write(ADDR_CTRL, 32'h0000_0006, 4'hF);
        @(posedge wb_clk_i); #1;
        check(dma_start_pulse === 1'b0, "No start pulse when only clear bits written");

        // ============================================================
        //  CASE 18: dma_done Pulse During STATUS Read
        // ============================================================
        case_header("CASE 18: Simultaneous dma_done Pulse During STATUS Read");
        do_reset();

        // Start a STATUS read, and pulse dma_done on the same clock edge
        @(posedge wb_clk_i); #1;
        wb_cyc_i = 1; wb_stb_i = 1; wb_we_i = 0;
        wb_adr_i = ADDR_STATUS; wb_sel_i = 4'hF;
        dma_done = 1;  // Assert done simultaneously

        @(posedge wb_clk_i);  // ack cycle — done_latched gets set this edge too
        while (!wb_ack_o) @(posedge wb_clk_i);
        rd_data = wb_dat_o;
        #1;
        wb_cyc_i = 0; wb_stb_i = 0;
        dma_done = 0;

        // The read captures dma_done|done_latched — at least one should be 1
        // (depending on whether latch updates in same cycle or status uses live OR)
        check(rd_data[1] === 1'b1, "STATUS[1] reflects dma_done during concurrent read");

        // After done deasserted, latched should still hold
        @(posedge wb_clk_i); @(posedge wb_clk_i); #1;
        wb_read(ADDR_STATUS, rd_data);
        check(rd_data[1] === 1'b1, "done_latched persists after dma_done deasserted");

        // ============================================================
        //  CASE 19: Overwrite Registers While DMA Busy
        // ============================================================
        case_header("CASE 19: Register Overwrite While DMA Busy");
        do_reset();

        wb_write(ADDR_SRC,   32'h0000_1000, 4'hF);
        wb_write(ADDR_DST,   32'h0000_2000, 4'hF);
        wb_write(ADDR_COUNT, 32'h0000_0100, 4'hF);

        // Simulate DMA busy
        dma_busy = 1;
        @(posedge wb_clk_i); #1;

        // Overwrite while busy (CSR doesn't gate writes on busy)
        wb_write(ADDR_SRC,   32'hAAAA_0000, 4'hF);
        wb_write(ADDR_DST,   32'hBBBB_0000, 4'hF);
        wb_write(ADDR_COUNT, 32'h0000_0200, 4'hF);

        check(dma_src_base_addr === 32'hAAAA_0000, "SRC overwritten while busy");
        check(dma_dst_base_addr === 32'hBBBB_0000, "DST overwritten while busy");
        check(dma_pixel_count   === 16'h0200,       "COUNT overwritten while busy");
        dma_busy = 0;

        // ============================================================
        //  CASE 20: Full Transaction Flow Simulation
        // ============================================================
        case_header("CASE 20: Full Transaction Flow (config -> start -> busy -> done -> status)");
        do_reset();

        // Step 1: Configure
        $display("    [INFO] Step 1: Configure registers");
        wb_write(ADDR_SRC,   32'h8000_0000, 4'hF);
        wb_write(ADDR_DST,   32'h9000_0000, 4'hF);
        wb_write(ADDR_COUNT, 32'h0000_4000, 4'hF);  // 16384 pixels

        // Step 2: Verify configuration
        wb_read(ADDR_SRC, rd_data);
        check(rd_data === 32'h8000_0000, "SRC configured correctly");
        wb_read(ADDR_DST, rd_data);
        check(rd_data === 32'h9000_0000, "DST configured correctly");
        wb_read(ADDR_COUNT, rd_data);
        check(rd_data === 32'h0000_4000, "COUNT configured correctly");

        // Step 3: Check status idle
        wb_read(ADDR_STATUS, rd_data);
        check(rd_data === 32'd0, "STATUS idle before start");

        // Step 4: Issue start
        $display("    [INFO] Step 2: Issue start command");
        wb_write(ADDR_CTRL, 32'h0000_0001, 4'hF);

        // Step 5: Simulate DMA busy
        @(posedge wb_clk_i); #1;
        dma_busy = 1;
        @(posedge wb_clk_i); #1;

        wb_read(ADDR_STATUS, rd_data);
        check(rd_data[0] === 1'b1, "STATUS[0]=busy after start");

        // Step 6: Simulate processing (a few cycles)
        repeat(5) @(posedge wb_clk_i);

        // Step 7: DMA completes
        $display("    [INFO] Step 3: DMA completes");
        #1;
        dma_busy = 0;
        dma_done = 1;
        @(posedge wb_clk_i); #1;
        dma_done = 0;
        @(posedge wb_clk_i); #1;

        wb_read(ADDR_STATUS, rd_data);
        check(rd_data[0] === 1'b0, "STATUS[0]=0 (not busy after done)");
        check(rd_data[1] === 1'b1, "STATUS[1]=1 (done latched)");
        check(rd_data[2] === 1'b0, "STATUS[2]=0 (no error)");

        // Step 8: Clear done
        $display("    [INFO] Step 4: Clear done latch");
        wb_write(ADDR_CTRL, 32'h0000_0002, 4'hF);
        @(posedge wb_clk_i); #1;
        wb_read(ADDR_STATUS, rd_data);
        check(rd_data === 32'd0, "STATUS all clear after done latch cleared");

        // ============================================================
        //  CASE 21: Write & Readback THRESHOLD Register
        // ============================================================
        case_header("CASE 21: Write & Readback THRESHOLD Register");
        do_reset();

        // Write threshold = 60
        wb_write(ADDR_THRESH, 32'h0000_003C, 4'hF);
        check(threshold === 8'd60, "threshold output matches written value 60");
        wb_read(ADDR_THRESH, rd_data);
        check(rd_data === 32'h0000_003C, "THRESHOLD readback matches 0x3C");

        // Write max 8-bit value
        wb_write(ADDR_THRESH, 32'h0000_00FF, 4'hF);
        check(threshold === 8'hFF, "threshold accepts max 8-bit value 255");

        // Write with upper bits set — only lower 8 bits should be stored
        wb_write(ADDR_THRESH, 32'hFFFF_FF80, 4'hF);
        check(threshold === 8'h80, "threshold ignores upper 24 bits");
        wb_read(ADDR_THRESH, rd_data);
        check(rd_data === 32'h0000_0080, "THRESHOLD readback has upper bits zeroed");

        // ============================================================
        //  FINAL SUMMARY
        // ============================================================
        $display("\n");
        $display("##########################################################");
        if (error_cnt == 0) begin
            $display("#   ALL TESTS PASSED!  (0 errors across 21 cases)      #");
        end else begin
            $display("#   TESTS COMPLETED WITH %0d TOTAL ERRORS              #", error_cnt);
        end
        $display("##########################################################\n");

        $finish;
    end

    // ================================================================
    //  OPTIONAL MONITORS
    // ================================================================

    // Monitor start pulse
    always @(posedge wb_clk_i) begin
        if (!wb_rst_i && dma_start_pulse)
            $display("  [MONITOR] dma_start_pulse asserted at time %0t", $time);
    end

endmodule
