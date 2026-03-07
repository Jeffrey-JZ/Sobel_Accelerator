`timescale 1ns / 1ps

module tb_engine_top #(
    // Reduce the size to display the matrix clearly in the Tcl Console
    parameter IMG_W     = 10,
    parameter IMG_H     = 10,
    parameter ADDR_W    = 14,
    parameter MAG_W     = 12,
    parameter clipped_w = 8
) ();
    localparam PIXELS = IMG_W * IMG_H;

    reg                         clk;
    reg                         rst_n;
    reg                         start;
    reg     [clipped_w - 1:0]   threshold;

    wire                        rd_req;
    wire    [ADDR_W-1:0]        rd_addr;
    reg     [7:0]               rd_data;
    reg                         rd_data_valid;

    wire                        wr_en;
    wire    [ADDR_W-1:0]        wr_addr;
    wire    [7:0]               wr_data;

    wire                        busy;
    wire                        done;

    // --- Simulate external memory (Private Frame Buffer) ---
    // To prevent the engine from reading modified data during read operations (write-back conflicts),
    // Here, the input and output video memory are separated, but the physical mapping is managed as a single unit within the test bench.
    reg [7:0] mem_in  [0:PIXELS-1];
    reg [7:0] mem_out [0:PIXELS-1];

    integer i, x, y;

    sobel_engine #(
        .IMG_W      (IMG_W),
        .IMG_H      (IMG_H),
        .ADDR_W     (ADDR_W),
        .MAG_W      (MAG_W),
        .clipped_w  (clipped_w)
    ) uut (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start),
        .threshold      (threshold),
        
        .rd_req         (rd_req),
        .rd_addr        (rd_addr),
        .rd_data        (rd_data),
        .rd_data_valid  (rd_data_valid),
        
        .wr_en          (wr_en),
        .wr_addr        (wr_addr),
        .wr_data        (wr_data),
        
        .busy           (busy),
        .done           (done)
    );

    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz
    end

    // --- Memory Read/Write Response Logic ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_data_valid <= 1'b0;
            rd_data       <= 8'd0;
        end else begin
            // Simulated RAM read latency for one cycle
            rd_data_valid <= rd_req;
            if (rd_req) begin
                rd_data <= mem_in[rd_addr];
            end

            // Write logic
            if (wr_en) begin
                mem_out[wr_addr] <= wr_data;
                // Print only non-zero data to prevent screen flooding.
                if (wr_data != 0) 
                    $display("[Time %0t] Write edge pixels: Addr = %0d (X:%0d, Y:%0d) | Data = %0d", 
                              $time, wr_addr, wr_addr%IMG_W, wr_addr/IMG_W, wr_data);
            end
        end
    end

    // --- Testing the Incentive Mainline ---
    initial begin
        rst_n = 0;
        start = 0;
        threshold = 8'd60; // The threshold corresponding to gaze is 60.
        
        // 2. Construct a test image with vertical edges
        // The left half is entirely black (0), the right half entirely white (255), with a distinct boundary line running through the middle.
        for (y = 0; y < IMG_H; y = y + 1) begin
            for (x = 0; x < IMG_W; x = x + 1) begin
                if (x < 5) 
                    mem_in[y*IMG_W + x] = 8'd0;
                else 
                    mem_in[y*IMG_W + x] = 8'd255;
                
                // Initialise the output memory with random characters (8'hFF) to verify whether S_CLEAR is functioning correctly.
                mem_out[y*IMG_W + x] = 8'hFF; 
            end
        end

        // 3. Print the raw image to the Tcl console
        $display("\n=======================================================");
        $display("                 ORIGINAL INPUT IMAGE                  ");
        $display("=======================================================");
        for (y = 0; y < IMG_H; y = y + 1) begin
            for (x = 0; x < IMG_W; x = x + 1) begin
                $write("%3d ", mem_in[y*IMG_W + x]);
            end
            $display(""); 
        end
        $display("=======================================================\n");

        // 4. Release the reset and start the engine
        #20 rst_n = 1;
        #15 start = 1;
        #10 start = 0;

        $display("[Time %0t] The engine has started and is awaiting completion....", $time);

        // 5. Awaiting the Done signal (Corner 4: Including Pipeline Drain)
        wait(done == 1'b1);
        $display("[Time %0t] The engine triggers the DONE signal, indicating completion of processing!", $time);

        // 6. Print the result image to the Tcl Console to verify the boundaries and processing results.
        $display("\n=======================================================");
        $display("                 SOBEL PROCESSED IMAGE                 ");
        $display("=======================================================");
        for (y = 0; y < IMG_H; y = y + 1) begin
            for (x = 0; x < IMG_W; x = x + 1) begin
                $write("%3d ", mem_out[y*IMG_W + x]);
            end
            $display(""); 
        end
        $display("=======================================================\n");
        
        // 7. Check Corner 2 (whether the boundary is zero)
        check_borders();

        $display("Simulation concluded！");
        $finish;
    end

    // --- Task: Boundary Verification Function ---
    task check_borders;
        integer err_cnt;
        begin
            err_cnt = 0;
            for (i = 0; i < PIXELS; i = i + 1) begin
                x = i % IMG_W;
                y = i / IMG_W;
                // If it is a boundary (top, bottom, left or right edge)
                if (x == 0 || x == (IMG_W-1) || y == 0 || y == (IMG_H-1)) begin
                    if (mem_out[i] !== 8'd0) begin
                        $display(" Boundary error! Addr=%0d, Expectation=0, Actual=%0d", i, mem_out[i]);
                        err_cnt = err_cnt + 1;
                    end
                end
            end
            if (err_cnt == 0)
                $display(" Corner 1 & 2 Verification Passed: S_CLEAR Normal clearing, without dirty data being written back to the boundary.!");
        end
    endtask

endmodule