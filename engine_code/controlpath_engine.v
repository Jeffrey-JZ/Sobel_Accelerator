`timescale 1ns / 1ps

// Control Path of the engine
// How does the engine work
// Function:
//   1) Clear full output frame to zero (so all border pixels become 0 by default)
//   2) Read input image in raster order, one pixel per accepted read beat
//   3) Build 3x3 sliding window with 2 line buffers
//   4) Feed window into pipelined Sobel datapath
//   5) Write inner-pixel Sobel results back to output buffer

module sobel_engine #(
    parameter   IMG_W       = 128,
    parameter   IMG_H       = 128,
    parameter   ADDR_W      = 14,
    parameter   MAG_W       = 12,
    parameter   clipped_w   = 8
) (
    // Regural input
    input   wire                        clk     ,
    input   wire                        rst_n   ,

    input   wire                        start       ,
    input   wire    [clipped_w - 1:0]   threshold   ,   // The horeshold to decide the edge of the image
                                                        // The completed software testbench defined threshold = 60

    // Read port to private frame buffer
    output  reg                         rd_req          ,
    output  reg     [ADDR_W-1:0]        rd_addr         ,
    input   wire    [       7:0]        rd_data         ,
    input   wire                        rd_data_valid   ,

    // Write port to private frame buffer
    output  reg                         wr_en   ,
    output  reg     [ADDR_W-1:0]        wr_addr ,
    output  reg     [       7:0]        wr_data ,

    output  reg                         busy,
    output  reg                         done
);
    localparam integer PIXELS = IMG_W * IMG_H;
    localparam [1:0] S_IDLE  = 2'd0;
    localparam [1:0] S_CLEAR = 2'd1;
    localparam [1:0] S_PROC  = 2'd2;
    localparam [1:0] S_DONE  = 2'd3;

    reg [1:0] state;

    // -------------------- CLEAR phase --------------------
    reg [ADDR_W-1:0] clr_addr;

    // -------------------- PROCESS phase ------------------
    reg [ADDR_W-1:0] rd_addr_cnt;
    reg [7:0]        x_cnt;
    reg [7:0]        y_cnt;

    // Two line buffers: keep previous two image rows.
    reg [7:0] linebuf0 [0:IMG_W-1]; // y-2 row
    reg [7:0] linebuf1 [0:IMG_W-1]; // y-1 row

    // 3-tap horizontal shift registers for each row in the current 3x3 window.
    reg [7:0] r0_s0, r0_s1, r0_s2;
    reg [7:0] r1_s0, r1_s1, r1_s2;
    reg [7:0] r2_s0, r2_s1, r2_s2;

    // Datapath input handshake (window valid + destination address)
    reg                  win_valid;
    reg [ADDR_W-1:0]     win_addr;

    // Datapath output
    wire                    out_valid;
    wire    [ADDR_W-1:0]    out_addr;
    wire    [7:0]           out_edge;

    // Pipeline drain tracking after final read beat
    /* 当引擎的读地址 rd_addr_cnt 扫描到整张图像的最后一个像素（PIXELS-1）时，说明输入数据已经全部读完了 。 
    但此时最后读进来的这几个像素，还卡在 Sobel 流水线里正在进行计算。 
    如果引擎在读完最后一个像素的瞬间，直接跳转到 S_DONE 状态，那么流水线里剩下的这几个像素的计算结果就会被丢弃，导致图像最后几个点没有被正确写回内存。  
    因此，此机制：在读完最后一个像素后，原地等待几个时钟周期，让流水线把数据都计算完成了 这就是排空（Drain）。
    reg final_read_seen:    触发时机：当读地址计数器 rd_addr_cnt 达到 PIXELS-1 时，这个标志位被拉高为 1 (final_read_seen <= 1'b1;) 
                            作用：告诉状态机两件事:不用继续要数据了：if (!final_read_seen) rd_req <= 1'b1; 一旦它为 1，rd_req 就不会再拉高，彻底切断了输入源
                                                 准备开启倒计时：它是启动后续排空逻辑的先决条件
    reg [2:0] drain_cnt:    初始装载:   在 final_read_seen 被拉高的同一瞬间，drain_cnt 被赋予了一个初始值 3'd5（5 个时钟周期）
                                        它是根据你实例化的 sobel_datapath_pipeline 内部的流水线级数（延迟）加上一些握手余量提前估算好的
                            开始倒数：在接下来的每一个时钟周期里，只要 final_read_seen 为 1 且倒计时没归零
                                    drain_cnt 就会不断减 1 (drain_cnt <= drain_cnt - 1'b1;) 
                            完美谢幕：当 drain_cnt 倒数到 0 时，说明理论上流水线已经走完了
                                    此时代码还会做一个双保险的确认 else if (!out_valid) 
                                    意思就是：倒计时结束了，并且确认流水线出口确实没有有效数据 (out_valid 为低) 往外冒了，这个时候，状态机才会安心地进入 S_DONE 状态
    */
    reg final_read_seen;    
    reg [2:0] drain_cnt;

    // Temp wires for incoming row samples and next shift values.
    wire    [7:0]   row0_in = linebuf0[x_cnt];
    wire    [7:0]   row1_in = linebuf1[x_cnt];

    // The data requiring calculation are updated. 
    // After completing the current calculation, these data must be supplied to 3-tap horizontal shift registers(This operation is in the FSM S_PROC phase)
    wire    [7:0]   r0_s0_n = r0_s1, r0_s1_n = r0_s2, r0_s2_n = row0_in; // y-2 row left shift
    wire    [7:0]   r1_s0_n = r1_s1, r1_s1_n = r1_s2, r1_s2_n = row1_in; // y-1 row left shift
    wire    [7:0]   r2_s0_n = r2_s1, r2_s1_n = r2_s2, r2_s2_n = rd_data; // current row left shift

    // Current accepted beat is the 3rd row/3rd column of a valid 3x3 window.
    wire window_ready = (x_cnt >= 8'd2) && (y_cnt >= 8'd2);

    // center pixel address of the 3x3 window = (x-1, y-1)
    wire    [ADDR_W-1:0]    center_addr = (y_cnt - 8'd1) * IMG_W + (x_cnt - 8'd1);

    // Datapath, calculate the results
    sobel_datapath_pipeline #(
        .WIDTH  (MAG_W),
        .ADDR_W (ADDR_W)
    ) u_sobel_pipe (
        .clk        (clk),
        .rst_n      (rst_n),
        .valid_in   (win_valid),
        .addr_in    (win_addr),
        .p00(r0_s0_n), .p01(r0_s1_n), .p02(r0_s2_n),
        .p10(r1_s0_n), .p11(r1_s1_n), .p12(r1_s2_n),
        .p20(r2_s0_n), .p21(r2_s1_n), .p22(r2_s2_n),
        .threshold  (threshold),
        .valid_out  (out_valid),                    // OUTPUT: Valid signal
        .addr_out   (out_addr),                     // OUTPUT: Writeback to wr_addr in FSM
        .edge_out   (out_edge)                      // OUTPUT: Writeback to wr_data in FSM
    );

    // FSM
    /*
    S_IDLE  = 2'd0;
    S_CLEAR = 2'd1;
    S_PROC  = 2'd2;
    S_DONE  = 2'd3;
    */

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            busy            <= 1'b0;
            done            <= 1'b0;

            rd_req          <= 1'b0;
            rd_addr         <=  'd0;
            wr_en           <= 1'b0;
            wr_addr         <=  'd0;
            wr_data         <= 8'd0;

            clr_addr        <=  'd0;
            rd_addr_cnt     <=  'd0;
            x_cnt           <= 8'd0;
            y_cnt           <= 8'd0;

            for (i = 0; i < IMG_W; i = i + 1) begin
                linebuf0[i] <= 8'd0;
                linebuf1[i] <= 8'd0;
            end

            r0_s0 <= 8'd0; r0_s1 <= 8'd0; r0_s2 <= 8'd0;
            r1_s0 <= 8'd0; r1_s1 <= 8'd0; r1_s2 <= 8'd0;
            r2_s0 <= 8'd0; r2_s1 <= 8'd0; r2_s2 <= 8'd0;

            win_valid       <= 1'b0;
            win_addr        <=  'd0;

            final_read_seen <= 1'b0;
            drain_cnt       <= 8'd0;

        end else begin
            // Default outputs each cycle
            rd_req      <= 1'b0;
            wr_en       <= 1'b0;
            done        <= 1'b0;
            win_valid   <= 1'b0;

            // Writeback from datapath
            if (out_valid) begin
                wr_en   <= 1'b1;
                wr_addr <= out_addr;
                wr_data <= out_edge;
            end

            case (state)
                /* 1. Idle state: 
                The engine remains idle until receiving a start signal, after which it will enter the clear phase. */
                S_IDLE: begin
                    busy    <= 1'b0;
                    if (start) begin
                        busy    <= 1'b1;

                        // Start by clearing output area to 0
                        clr_addr <= 1'b0;
                        state    <= S_CLEAR;
                    end
                end

                /* 2. Clear State:  
                The engine traverses from address 0, clearing the entire output image area to zero (to set all boundary pixels to 0 by default). 
                When the clearing address reaches PIXELS-1, the stream state is initialised and the processing phase commences. */
                S_CLEAR: begin
                    // Clear the data stored at the write-back address
                    wr_en   <= 1'b1;
                    wr_addr <= clr_addr;
                    wr_data <= 8'd0;

                    // When the data stored at the write-back address are all cleared, the FSM will entry Processing State
                    if (clr_addr == PIXELS - 1) begin
                        // Initialize the stream state for Processing State
                        rd_addr_cnt     <=  'd0;
                        rd_addr         <=  'd0;
                        x_cnt           <= 8'd0;
                        y_cnt           <= 8'd0;
                        final_read_seen <= 1'b0;
                        drain_cnt       <= 3'd0;

                        r0_s0 <= 8'd0; r0_s1 <= 8'd0; r0_s2 <= 8'd0;
                        r1_s0 <= 8'd0; r1_s1 <= 8'd0; r1_s2 <= 8'd0;
                        r2_s0 <= 8'd0; r2_s1 <= 8'd0; r2_s2 <= 8'd0;

                        state <= S_PROC;
                    end else begin
                        clr_addr <= clr_addr + 1'b1;
                    end
                end

                /* 3. Processing State: 
                This constitutes the engine's core operational state. 
                It continuously issues sequential read requests to acquire input pixels until all pixels have been read. 
                Upon receiving valid read data, it updates the row buffer and shift window registers, updating the XY coordinates in raster order. 
                Once the 3x3 window is ready, it triggers the win_valid signal, passing the centre address and window data to subsequent pipeline modules. 
                After reading the final input sample, a drain counter (drain_cnt) is utilised to clear any residual data from the pipeline.*/
                S_PROC: begin
                    // keep requesting sequential reads until last address is issued
                    if (!final_read_seen) begin
                        rd_req <= 1'b1;
                        rd_addr <= rd_addr_cnt;
                    end

                    if (rd_data_valid) begin

                        r0_s0 <= r0_s0_n; r0_s1 <= r0_s1_n; r0_s2 <= r0_s2_n;
                        r1_s0 <= r1_s0_n; r1_s1 <= r1_s1_n; r1_s2 <= r1_s2_n;
                        r2_s0 <= r2_s0_n; r2_s1 <= r2_s1_n; r2_s2 <= r2_s2_n;

                        // Roll line buffers for this x position
                        linebuf0[x_cnt] <= row1_in;
                        linebuf1[x_cnt] <= rd_data;

                        // Valid 3x3 window
                        if (window_ready) begin         // wire window_ready = (x_cnt >= 8'd2) && (y_cnt >= 8'd2);
                            win_valid   <= 1'b1;
                            win_addr    <= center_addr; // wire    [ADDR_W-1:0]    center_addr = (y_cnt - 8'd1) * IMG_W + (x_cnt - 8'd1);
                        end

                        // Advance x/y counters in raster order
                        if (x_cnt == IMG_W-1) begin
                            x_cnt <= 8'd0;
                            y_cnt <= y_cnt + 1'b1;
                        end else begin
                            x_cnt <= x_cnt + 1'b1;
                        end

                        // advance read address generator
                        if (rd_addr_cnt == PIXELS-1) begin
                            final_read_seen <= 1'b1;    // Stop inputting the data into the engine, just complete the data calculations in pipeline
                            drain_cnt       <= 3'd5;    // pipeline + handshake guard cycles = 3 + 2 = 5 cycles
                        end else begin
                            rd_addr_cnt <= rd_addr_cnt + 1'b1;
                        end
                    end

                    // Drain remaining pipeline outputs after last input sample
                    if (final_read_seen) begin
                        if (drain_cnt != 3'd0) begin
                            drain_cnt <= drain_cnt - 1'b1;
                        end else if (!out_valid) begin  // drain_cnt=0 than check out_valid which is from win_valid 2 cycles ago
                            state <= S_DONE;            // This is double insurance
                        end
                    end
                    
                end

                /* 4. Completed state: 
                Upon completion of processing, the engine will raise the done signal and await the reset of the start signal before returning to the S_IDLE state. */
                S_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    if (!start) begin
                        state <= S_IDLE;
                    end
                end
                
                default: state <= S_IDLE;

            endcase
        end
    end
endmodule