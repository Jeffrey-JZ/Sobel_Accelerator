`timescale 1ns / 1ps

module tb_pfb_top();

    // Parameters
    parameter DATA_W  = 8;
    parameter ADDR_W  = 14;
    parameter BANKS   = 4;
    parameter IMG_W   = 128;
    parameter IMG_H   = 128;

    // Clock and Reset
    reg clk;
    reg rst_n;

    // DMA Read Port
    reg                 dma_rd_req;
    reg  [ADDR_W-1:0]   dma_rd_addr;
    wire [DATA_W-1:0]   dma_rd_data;
    wire                dma_rd_valid;
    wire                dma_rd_ready;

    // DMA Write Port
    reg                 dma_wr_req;
    reg  [ADDR_W-1:0]   dma_wr_addr;
    reg  [DATA_W-1:0]   dma_wr_data;
    wire                dma_wr_ready;

    // Engine Read Port
    reg                 eng_rd_req;
    reg  [ADDR_W-1:0]   eng_rd_addr;
    wire [DATA_W-1:0]   eng_rd_data;
    wire                eng_rd_valid;
    wire                eng_rd_ready;

    // Engine Write Port
    reg                 eng_wr_en;
    reg  [ADDR_W-1:0]   eng_wr_addr;
    reg  [DATA_W-1:0]   eng_wr_data;
    wire                eng_wr_ready;

    // Status
    wire                wr_conflict;

    // Error Counter
    integer error_cnt = 0;

    // Instantiate the DUT (Device Under Test)
    private_frame_buffer #(
        .DATA_W(DATA_W), .ADDR_W(ADDR_W), .BANKS(BANKS), 
        .IMG_W(IMG_W), .IMG_H(IMG_H)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .dma_rd_req(dma_rd_req), .dma_rd_addr(dma_rd_addr), .dma_rd_data(dma_rd_data), .dma_rd_valid(dma_rd_valid), .dma_rd_ready(dma_rd_ready),
        .dma_wr_req(dma_wr_req), .dma_wr_addr(dma_wr_addr), .dma_wr_data(dma_wr_data), .dma_wr_ready(dma_wr_ready),
        .eng_rd_req(eng_rd_req), .eng_rd_addr(eng_rd_addr), .eng_rd_data(eng_rd_data), .eng_rd_valid(eng_rd_valid), .eng_rd_ready(eng_rd_ready),
        .eng_wr_en(eng_wr_en), .eng_wr_addr(eng_wr_addr), .eng_wr_data(eng_wr_data), .eng_wr_ready(eng_wr_ready),
        .wr_conflict(wr_conflict)
    );

    // Clock Generation (10ns period -> 100MHz)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Task to check conditions and print to Vivado Tcl Console
    task check_cond;
        input condition;
        input [800:0] test_name;
        begin
            if (!condition) begin
                $display("[ERROR] %s FAILED at time %0t", test_name, $time);
                error_cnt = error_cnt + 1;
            end else begin
                $display("[PASS]  %s", test_name);
            end
        end
    endtask

    // Reset All Inputs Task
    task clear_inputs;
        begin
            dma_rd_req = 0; dma_rd_addr = 0;
            dma_wr_req = 0; dma_wr_addr = 0; dma_wr_data = 0;
            eng_rd_req = 0; eng_rd_addr = 0;
            eng_wr_en  = 0; eng_wr_addr = 0; eng_wr_data = 0;
        end
    endtask

    // Main Test Sequence
    initial begin
        $display("\n=================================================");
        $display("   PFB TOP MODULE SIMULATION STARTED");
        $display("=================================================\n");

        // Initialize and Reset
        clear_inputs();
        rst_n = 0;
        #20 rst_n = 1;
        #10;

        // ---------------------------------------------------------
        // CORNER CASE 1: Basic sequential writes and reads
        // ---------------------------------------------------------
        $display("\n--- CASE 1: Basic DMA Write & Engine Read ---");
        @(posedge clk); #1; // Add #1 to avoid clock edge contention
        dma_wr_req = 1; dma_wr_addr = 14'd0; dma_wr_data = 8'hAA; 
        #1; // Awaiting the logic circuit's ready signal to go high
        check_cond(dma_wr_ready === 1'b1, "DMA Write to Bank 0 Accepted");

        @(posedge clk); #1;
        clear_inputs(); // Revoke the request after writing is complete

        @(posedge clk); #1;
        eng_rd_req = 1; eng_rd_addr = 14'd0; // Initiate a read request
        
        @(posedge clk); #1;
        clear_inputs(); // Revoke read request

        @(posedge clk); #1; 
        check_cond(eng_rd_valid === 1'b1 && eng_rd_data === 8'hAA, "Engine Read Data Match (8'hAA)");

        // ---------------------------------------------------------
        // CORNER CASE 2: 4-Way No Conflict (Accessing Banks 0, 1, 2, 3)
        // ---------------------------------------------------------
        $display("\n--- CASE 2: 4-Way Simultaneous Access (No Conflict) ---");
        @(posedge clk); #1;
        dma_wr_req = 1; dma_wr_addr = 14'd0; dma_wr_data = 8'h11; // Bank 0
        eng_wr_en  = 1; eng_wr_addr = 14'd1; eng_wr_data = 8'h22; // Bank 1
        dma_rd_req = 1; dma_rd_addr = 14'd2;                      // Bank 2
        eng_rd_req = 1; eng_rd_addr = 14'd3;                      // Bank 3
        
        #1; 
        check_cond({dma_wr_ready, eng_wr_ready, dma_rd_ready, eng_rd_ready} === 4'b1111, "All 4 requests granted to 4 different banks");
        
        @(posedge clk); #1; 
        check_cond(wr_conflict === 1'b0, "No write conflict asserted");
        clear_inputs();

        // ---------------------------------------------------------
        // CORNER CASE 3: 3-Way Bank Conflict 
        // ---------------------------------------------------------
        $display("\n--- CASE 3: 3-Way Bank Conflict (Same Bank) ---");
        @(posedge clk); #1;
        eng_wr_en  = 1; eng_wr_addr = 14'd0; eng_wr_data = 8'h99; // Bank 0
        dma_wr_req = 1; dma_wr_addr = 14'd4; dma_wr_data = 8'h88; // Bank 0
        dma_rd_req = 1; dma_rd_addr = 14'd8;                      // Bank 0
        
        #1;
        check_cond(eng_wr_ready === 1'b1, "Engine Write granted (highest priority Port A)");
        check_cond(dma_wr_ready === 1'b1, "DMA Write granted (Port B)");
        check_cond(dma_rd_ready === 1'b0, "DMA Read denied (no ports left)");
        
        @(posedge clk); #1;
        clear_inputs();

        // ---------------------------------------------------------
        // CORNER CASE 4: 4-Way Extreme Bank Conflict & Write Conflict
        // ---------------------------------------------------------
        $display("\n--- CASE 4: 4-Way Extreme Bank Conflict & wr_conflict Flag ---");
        @(posedge clk); #1;
        eng_wr_en  = 1; eng_wr_addr = 14'd0;  eng_wr_data = 8'h77; // Bank 0
        eng_rd_req = 1; eng_rd_addr = 14'd4;                       // Bank 0
        dma_wr_req = 1; dma_wr_addr = 14'd8;  dma_wr_data = 8'h66; // Bank 0
        dma_rd_req = 1; dma_rd_addr = 14'd12;                      // Bank 0
        
        #1;
        check_cond(eng_wr_ready === 1'b1, "Engine Write granted (Port A)");
        check_cond(eng_rd_ready === 1'b1, "Engine Read granted (Port B priority)");
        check_cond(dma_wr_ready === 1'b0, "DMA Write denied (Starved)");
        check_cond(dma_rd_ready === 1'b0, "DMA Read denied (Starved)");
        
        @(posedge clk); #1;
        check_cond(wr_conflict === 1'b1, "wr_conflict flag correctly triggered by blocked DMA write");
        clear_inputs();

        // Finish Test
        @(posedge clk);
        @(posedge clk);
        $display("\n=================================================");
        if (error_cnt == 0) begin
            $display("   ALL TESTS PASSED SUCCESSFULLY! (0 Errors)");
        end else begin
            $display("   TESTS FAILED WITH %0d ERRORS.", error_cnt);
        end
        $display("=================================================\n");
        
        $finish;
    end

endmodule