`timescale 1ns / 1ps

module tb_private_frame_buffer #(
    // Parameters
    parameter DATA_W  = 8,
    parameter ADDR_W  = 14,
    parameter BANKS   = 4,
    parameter IMG_W   = 128,
    parameter IMG_H   = 128
)();
    // Clock and Reset
    reg clk;
    reg rst_n;

    // DMA Port Signals
    reg                   dma_req;
    reg                   dma_we;
    reg  [ADDR_W - 1:0]   dma_addr;
    reg  [DATA_W - 1:0]   dma_wdata;
    wire [DATA_W - 1:0]   dma_rdata;
    wire                  dma_rvalid;
    wire                  dma_ready;

    // Engine Port Signals
    reg                   eng_req;
    reg                   eng_we;
    reg  [ADDR_W - 1:0]   eng_addr;
    reg  [DATA_W - 1:0]   eng_wdata;
    wire [DATA_W - 1:0]   eng_rdata;
    wire                  eng_rvalid;
    wire                  eng_ready;

    // Conflict Signal
    wire                  wr_conflict;

    // Instantiate the Unit Under Test (UUT)
    private_frame_buffer #(
        .DATA_W (DATA_W),
        .ADDR_W (ADDR_W),
        .BANKS  (BANKS),
        .IMG_W  (IMG_W),
        .IMG_H  (IMG_H)
    ) uut (
        .clk        (clk),
        .rst_n      (rst_n),
        .dma_req    (dma_req),
        .dma_we     (dma_we),
        .dma_addr   (dma_addr),
        .dma_wdata  (dma_wdata),
        .dma_rdata  (dma_rdata),
        .dma_rvalid (dma_rvalid),
        .dma_ready  (dma_ready),
        .eng_req    (eng_req),
        .eng_we     (eng_we),
        .eng_addr   (eng_addr),
        .eng_wdata  (eng_wdata),
        .eng_rdata  (eng_rdata),
        .eng_rvalid (eng_rvalid),
        .eng_ready  (eng_ready),
        .wr_conflict(wr_conflict)
    );

    // Clock Generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz Clock
    end

    // Error Counter
    integer error_count = 0;

    // Test Sequence
    initial begin
        // --------------------------------------------------------
        // 1. Initialize and Reset
        // --------------------------------------------------------
        $display("==================================================");
        $display("[INFO] Starting Private Frame Buffer Simulation...");
        $display("==================================================");
        
        dma_req   = 0; dma_we    = 0; dma_addr  = 0; dma_wdata = 0;
        eng_req   = 0; eng_we    = 0; eng_addr  = 0; eng_wdata = 0;
        rst_n     = 0;
        
        #20;
        rst_n = 1;
        #10;

        // --------------------------------------------------------
        // Test 1: Simple Write/Read (DMA writes, Engine reads)
        // --------------------------------------------------------
        $display("[TEST 1] Simple DMA Write and Engine Read");
        
        // DMA Write to Address 0 (Bank 0)
        @(posedge clk);
        dma_req = 1; dma_we = 1; dma_addr = 14'd0; dma_wdata = 8'hAA;
        @(posedge clk);
        dma_req = 0; dma_we = 0;
        
        // Engine Read from Address 0
        @(posedge clk);
        eng_req = 1; eng_we = 0; eng_addr = 14'd0;
        @(posedge clk);
        eng_req = 0;
        
        // Check Read Data (Read latency is 1 cycle)
        #1; 
        if (eng_rdata === 8'hAA && eng_rvalid === 1'b1) 
            $display("  -> [PASS] Engine successfully read DMA written data: %h", eng_rdata);
        else begin
            $display("  -> [FAIL] Expected 8'hAA, got %h. Valid: %b", eng_rdata, eng_rvalid);
            error_count = error_count + 1;
        end

        // --------------------------------------------------------
        // Test 2: Parallel Write to DIFFERENT Banks
        // --------------------------------------------------------
        $display("[TEST 2] Parallel Write to DIFFERENT Banks (No Conflict)");
        // Address 1 -> Bank 1, Address 2 -> Bank 2
        @(posedge clk);
        dma_req = 1; dma_we = 1; dma_addr = 14'd1; dma_wdata = 8'hD1;
        eng_req = 1; eng_we = 1; eng_addr = 14'd2; eng_wdata = 8'hE2;
        @(posedge clk);
        dma_req = 0; eng_req = 0; dma_we = 0; eng_we = 0;

        // Verify Data
        @(posedge clk);
        dma_req = 1; dma_we = 0; dma_addr = 14'd2; // DMA reads Engine's write
        eng_req = 1; eng_we = 0; eng_addr = 14'd1; // Engine reads DMA's write
        @(posedge clk);
        dma_req = 0; eng_req = 0;
        
        #1;
        if (dma_rdata === 8'hE2 && eng_rdata === 8'hD1)
            $display("  -> [PASS] Parallel cross-bank read/write successful.");
        else begin
            $display("  -> [FAIL] Parallel write/read failed. DMA read: %h, ENG read: %h", dma_rdata, eng_rdata);
            error_count = error_count + 1;
        end

        // --------------------------------------------------------
        // Test 3: Parallel Write to SAME Bank but DIFFERENT Addresses
        // --------------------------------------------------------
        $display("[TEST 3] Parallel Write to SAME Bank, DIFFERENT Local Addresses");
        // Address 0 -> Bank 0 (Local 0), Address 4 -> Bank 0 (Local 1)
        @(posedge clk);
        dma_req = 1; dma_we = 1; dma_addr = 14'd0; dma_wdata = 8'h11;
        eng_req = 1; eng_we = 1; eng_addr = 14'd4; eng_wdata = 8'h22;
        @(posedge clk);
        dma_req = 0; eng_req = 0; dma_we = 0; eng_we = 0;
        
        // Since it's a true dual-port RAM, this should work perfectly. Let's verify.
        @(posedge clk);
        dma_req = 1; dma_we = 0; dma_addr = 14'd0;
        eng_req = 1; eng_we = 0; eng_addr = 14'd4;
        @(posedge clk);
        dma_req = 0; eng_req = 0;
        
        #1;
        if (dma_rdata === 8'h11 && eng_rdata === 8'h22)
            $display("  -> [PASS] Dual-port same-bank different-address write successful.");
        else begin
            $display("  -> [FAIL] Same-bank different-address write failed. DMA: %h, ENG: %h", dma_rdata, eng_rdata);
            error_count = error_count + 1;
        end

        // --------------------------------------------------------
        // Test 4: Corner Case - SAME Bank, SAME Address WRITE CONFLICT
        // --------------------------------------------------------
        $display("[TEST 4] Corner Case: Simultaneous Write to SAME Address (DMA Priority Check)");
        
        @(posedge clk);
        // Both write to Address 8 (Bank 0, Local 2)
        dma_req = 1; dma_we = 1; dma_addr = 14'd8; dma_wdata = 8'hDD; // DMA Data
        eng_req = 1; eng_we = 1; eng_addr = 14'd8; eng_wdata = 8'hEE; // Engine Data
        
        @(posedge clk);
        dma_req = 0; eng_req = 0; dma_we = 0; eng_we = 0;
        
        // wr_conflict flag is delayed by 1 cycle in your code, let's check it now
        #1;
        if (wr_conflict === 1'b1)
            $display("  -> [PASS] wr_conflict flag asserted correctly.");
        else begin
            $display("  -> [FAIL] wr_conflict flag NOT asserted!");
            error_count = error_count + 1;
        end
        
        // Verify that DMA won and Engine data was blocked
        @(posedge clk);
        dma_req = 1; dma_we = 0; dma_addr = 14'd8;
        @(posedge clk);
        dma_req = 0;
        
        #1;
        if (dma_rdata === 8'hDD)
            $display("  -> [PASS] DMA data (0xDD) successfully written, Engine data (0xEE) blocked.");
        else begin
            $display("  -> [FAIL] Expected DMA data 8'hDD, but read %h", dma_rdata);
            error_count = error_count + 1;
        end

        // --------------------------------------------------------
        // Finish Simulation
        // --------------------------------------------------------
        $display("==================================================");
        if (error_count == 0)
            $display("[RESULT] All Tests PASSED! Excellent job!");
        else
            $display("[RESULT] Finished with %0d ERRORS. Please check the logs.", error_count);
        $display("==================================================");
        
        $finish;
    end

endmodule