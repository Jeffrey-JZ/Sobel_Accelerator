`timescale 1ns / 1ps

// Sobel accelerator private frame buffer(pfb)
// Author: Junze Jiang
// 4/3/2026
// Rebuilt on 11/3/2026
// Both ports can work in parallel when addressing any bank
// If both ports write the same bank+address in the same cycle, dma side wins and eng write is blocked for deterministic behavior

// - 128x128, 8-bit pixels, 4-bank interleaving
// - Single storage (in-place writeback), no split input/output PFB
// - Exposes independent engine read/write ports and DMA read/write ports
// - Internally arbitrates 4 logical requesters onto 2 physical ports per bank
// Read latency: 1 cycle
// There is no Latency in writing

module private_frame_buffer #(
    parameter DATA_W  = 8,
    parameter ADDR_W  = 14,
    parameter BANKS   = 4,
    parameter IMG_W   = 128,
    parameter IMG_H   = 128
) (
    input  wire                 clk,
    input  wire                 rst_n,

    // DMA read port    DMA read from pfb   pfb -> DMA
    // Read 有 1 cycle latency
    // 拍 T:   DMA 拉高 req + 给 addr   →  PFB 回 ready=1（接受了） 
    // 拍 T+1: PFB 给出 data + valid=1  →  DMA 拿到像素值
    // 如果 ready=0，说明这一拍 DMA 读请求的那个 bank 两个端口都被别人占了，DMA 需要下一拍重试
    input  wire                 dma_rd_req,     // DMA 要读，拉高一拍表示发起一次读请求
    input  wire [ADDR_W - 1:0]  dma_rd_addr,    // DMA 要读这个地址的像素
    output reg  [DATA_W - 1:0]  dma_rd_data,    // PFB 返回的像素值，下一拍才有效
    output reg                  dma_rd_valid,   // PFB 说"这一拍 dma_rd_data 上的数据你可以拿了"
    output reg                  dma_rd_ready,   // PFB 回答"这一拍我收到你的请求了"。注意它不是说数据好了，只是说请求被接受了

    // DMA write port   DMA write to pfb    DMA -> pfb
    // Write 没有Latency
    // Write 没有延迟
    // ready=1 的那一拍，数据就已经在下个时钟沿写入 bank RAM 了
    // ready=0 表示这拍被仲裁拦掉了，数据没写进去，DMA 需要保持请求重试
    input  wire                 dma_wr_req,     // DMA 要写
    input  wire [ADDR_W - 1:0]  dma_wr_addr,    // 要写入的全局地址
    input  wire [DATA_W - 1:0]  dma_wr_data,    // 要写入的像素值
    output reg                  dma_wr_ready,   // PFB 回答"写进去了"

    // Engine read port     Engine read from pfb    pfb -> Engine
    // Read 有 1 cycle latency
    input  wire                 eng_rd_req,     // Engine 说"我要读一个像素"
    input  wire [ADDR_W - 1:0]  eng_rd_addr,    // 读这个地址
    output reg  [DATA_W - 1:0]  eng_rd_data,    // 读回的像素值（下一拍）
    output reg                  eng_rd_valid,   // "数据有效，你拿走"（下一拍）
    output reg                  eng_rd_ready,   // PFB 说"请求收到了"（当拍）

    // Engine writeback port    Engine writeback to pfb Engine -> pfb
    input  wire                 eng_wr_en,      // Engine 说"我要写回一个结果" (功能等同于 req)
    input  wire [ADDR_W - 1:0]  eng_wr_addr,    // "写到这个地址"（就是 Sobel 窗口中心像素的地址）
    input  wire [DATA_W - 1:0]  eng_wr_data,    // "写入这个边缘值"（0 或截断后的梯度幅值）
    output reg                  eng_wr_ready,   // PFB 说"写进去了"

    // Any write requester blocked in this cycle    这一拍只要有任意一个写请求没被接收，就报 conflict
    output reg                  wr_conflict
);
    localparam integer PIXELS       = IMG_W * IMG_H;        // 128 * 128 = 16384 pixels
    localparam integer BANK_BITS    = $clog2(BANKS);        // log2, round up   log2(BANKs) = BANK_BITS = 2
    localparam integer LOCAL_ADDR_W = ADDR_W - BANK_BITS;   // 14 - 2 = 12

    // 标签编码
    localparam [1:0] SRC_NONE   = 2'd0;
    localparam [1:0] SRC_DMA_RD = 2'd1;
    localparam [1:0] SRC_ENG_RD = 2'd2;

    // 由 pfb_addr_map 产生     低 2 位选 bank，高 12 位做 bank 内地址
    wire [BANK_BITS    - 1  :0] dma_rd_bank_sel,    dma_wr_bank_sel,    eng_rd_bank_sel,    eng_wr_bank_sel;    // 落到哪个 bank
    wire [LOCAL_ADDR_W - 1  :0] dma_rd_local_addr,  dma_wr_local_addr,  eng_rd_local_addr,  eng_wr_local_addr;  // 在这个 bank 内部的地址是多少

    // DMA Read Address Mapping  
    pfb_addr_map #(
        .ADDR_W         (ADDR_W), 
        .BANK_BITS      (BANK_BITS)
    ) u_dma_rd_map (
        .global_addr    (dma_rd_addr),      // Input
        .bank_sel       (dma_rd_bank_sel),  // Output
        .local_addr     (dma_rd_local_addr) // Output
    );

    // DMA Write Address Mapping  
    pfb_addr_map #(
        .ADDR_W         (ADDR_W), 
        .BANK_BITS      (BANK_BITS)
    ) u_dma_wr_map (
        .global_addr    (dma_wr_addr),      // Input
        .bank_sel       (dma_wr_bank_sel),  // Output
        .local_addr     (dma_wr_local_addr) // Output
    );

    // Engine Address Mapping  
    pfb_addr_map #(
        .ADDR_W         (ADDR_W), 
        .BANK_BITS      (BANK_BITS)
    ) u_eng_rd_map (
        .global_addr    (eng_rd_addr),      // Input
        .bank_sel       (eng_rd_bank_sel),  // Output
        .local_addr     (eng_rd_local_addr) // Output
    );

    // Engine Address Mapping
    pfb_addr_map #(
        .ADDR_W         (ADDR_W), 
        .BANK_BITS      (BANK_BITS)
    ) u_eng_wr_map (
        .global_addr    (eng_wr_addr),      // Input
        .bank_sel       (eng_wr_bank_sel),  // Output
        .local_addr     (eng_wr_local_addr) // Output
    );

    // 对每个 bank 来说，A 端口和 B 端口这一拍要干什么
    // 对第 i 个 bank
    // a_en_vec  [i]: A 口是否启用                  a_we_vec   [i]: A 口是读还是写，1=写, 0=读
    // a_addr_vec[i]: A 口访问哪个 local address    a_wdata_vec[i]: A 口写入的数据
    // B Port 同理
    reg  [BANKS         - 1:0]  a_en_vec, a_we_vec, b_en_vec, b_we_vec;
    reg  [LOCAL_ADDR_W  - 1:0]  a_addr_vec [0:BANKS - 1];                   // 每个元素是 LOCAL_ADDR_W 位宽的寄存器    数组有 BANKS 个元素
    reg  [LOCAL_ADDR_W  - 1:0]  b_addr_vec [0:BANKS - 1];
    reg  [DATA_W        - 1:0]  a_wdata_vec[0:BANKS - 1];
    reg  [DATA_W        - 1:0]  b_wdata_vec[0:BANKS - 1];

    // 跟踪标签: 这一笔读请求是谁发的
    // 因为bank RAM读数据有 1-cycle latency
    // 这一拍把读地址送进 RAM，下一拍数据回来时，必须知道: 这笔返回数据应该送给 DMA or Engine
    // n: 当前组合逻辑算出来的 next
    // q: 寄存后的当前有效标签
    reg  [1:0]                  a_rd_src_n [0:BANKS - 1];   // 当前这一拍; 某个 bank 的 A 口如果在做读, 那这笔读是谁发起的    e.g. DMA 读走了 bank2 的 A 口, a_rd_src_n[2] = SRC_DMA_RD
    reg  [1:0]                  b_rd_src_n [0:BANKS - 1];

    reg  [1:0]                  a_rd_src_q [0:BANKS - 1];   // 把上一拍的读来源标签寄存下来  等下一拍 RAM 数据回来时，就知道要把数据送给谁
    reg  [1:0]                  b_rd_src_q [0:BANKS - 1];

    wire [BANKS*DATA_W - 1:0]   a_rdata_vec,    b_rdata_vec;
    wire [BANKS        - 1:0]   a_rvalid_vec,   b_rvalid_vec;

    // PFB 核心调度器: 决定每一拍、每个 bank 的 A/B 两个物理端口分别给谁用。
    // 仲裁是按 bank 分开的，不是整个 PFB 只有一套 A/B 口
    // bank0 有自己的 A/B 口  
    // bank1 有自己的 A/B 口  
    // bank2 有自己的 A/B 口  
    // bank3 有自己的 A/B 口  
    // 所以不同 bank 的请求本来就可以并行。
    integer i;
    always @(*) begin
        dma_rd_ready = 1'b0;
        dma_wr_ready = 1'b0;
        eng_rd_ready = 1'b0;
        eng_wr_ready = 1'b0;

        for (i = 0; i < BANKS; i = i + 1) begin
            // Initialisation
            a_en_vec    [i] = 1'b0;
            a_we_vec    [i] = 1'b0;
            b_en_vec    [i] = 1'b0;
            b_we_vec    [i] = 1'b0;
            a_addr_vec  [i] = {LOCAL_ADDR_W {1'b0}};
            b_addr_vec  [i] = {LOCAL_ADDR_W {1'b0}};
            a_wdata_vec [i] = {DATA_W       {1'b0}};
            b_wdata_vec [i] = {DATA_W       {1'b0}};
            a_rd_src_n  [i] = SRC_NONE;
            b_rd_src_n  [i] = SRC_NONE;

            // Port A priority: engine write > DMA write > DMA read > engine read
            // e.g. 某一拍对 bank1 同时来了: eng_wr  dma_rd  eng_rd  那 A 口会先给 eng_wr
            if (eng_wr_en && (eng_wr_bank_sel == i[BANK_BITS-1:0])) begin   // i[BANK_BITS-1:0]表示取低两位 e.g. i = 0 -> 2'd00
                a_en_vec    [i] = 1'b1;
                a_we_vec    [i] = 1'b1;
                a_addr_vec  [i] = eng_wr_local_addr;
                a_wdata_vec [i] = eng_wr_data;
                eng_wr_ready    = 1'b1;
            end else if (dma_wr_req && (dma_wr_bank_sel == i[BANK_BITS-1:0])) begin
                a_en_vec    [i] = 1'b1;
                a_we_vec    [i] = 1'b1;
                a_addr_vec  [i] = dma_wr_local_addr;
                a_wdata_vec [i] = dma_wr_data;
                dma_wr_ready    = 1'b1;
            end else if (dma_rd_req && (dma_rd_bank_sel == i[BANK_BITS-1:0])) begin
                a_en_vec    [i] = 1'b1;
                a_we_vec    [i] = 1'b0;
                a_addr_vec  [i] = dma_rd_local_addr;
                a_rd_src_n  [i] = SRC_DMA_RD;
                dma_rd_ready    = 1'b1;
            end else if (eng_rd_req && (eng_rd_bank_sel == i[BANK_BITS-1:0])) begin
                a_en_vec    [i] = 1'b1;
                a_we_vec    [i] = 1'b0;
                a_addr_vec  [i] = eng_rd_local_addr;
                a_rd_src_n  [i] = SRC_ENG_RD;
                eng_rd_ready    = 1'b1;
            end

            // Port B priority: engine read > DMA write > DMA read > engine write
            // Only requesters not already granted on port A can be granted here.
            if (eng_rd_req && !eng_rd_ready && (eng_rd_bank_sel == i[BANK_BITS-1:0])) begin             // Engine 读请求存在、在 Port A 落选了、而且它要的就是这个 bank -> 允许用Port B
                b_en_vec    [i] = 1'b1;
                b_we_vec    [i] = 1'b0;
                b_addr_vec  [i] = eng_rd_local_addr;
                b_rd_src_n  [i] = SRC_ENG_RD;
                eng_rd_ready    = 1'b1;
            end else if (dma_wr_req && !dma_wr_ready && (dma_wr_bank_sel == i[BANK_BITS-1:0])) begin
                b_en_vec    [i] = 1'b1;
                b_we_vec    [i] = 1'b1;
                b_addr_vec  [i] = dma_wr_local_addr;
                b_wdata_vec [i] = dma_wr_data;
                dma_wr_ready    = 1'b1;
            end else if (dma_rd_req && !dma_rd_ready && (dma_rd_bank_sel == i[BANK_BITS-1:0])) begin
                b_en_vec    [i] = 1'b1;
                b_we_vec    [i] = 1'b0;
                b_addr_vec  [i] = dma_rd_local_addr;
                b_rd_src_n  [i] = SRC_DMA_RD;
                dma_rd_ready    = 1'b1;
            end else if (eng_wr_en && !eng_wr_ready && (eng_wr_bank_sel == i[BANK_BITS-1:0])) begin
                b_en_vec    [i] = 1'b1;
                b_we_vec    [i] = 1'b1;
                b_addr_vec  [i] = eng_wr_local_addr;
                b_wdata_vec [i] = eng_wr_data;
                eng_wr_ready    = 1'b1;
            end
        end
    end

    // Using generate for to copy 4 identical pfb_bank_ram
    genvar b;
    generate
        for (b = 0; b < BANKS; b = b + 1) begin : g_banks
            pfb_bank_ram #(
                .DATA_W(DATA_W),
                .ADDR_W(LOCAL_ADDR_W)
            ) u_bank (
                .clk     (clk),
                .rst_n   (rst_n),
                .a_en    (a_en_vec[b]),
                .a_we    (a_we_vec[b]),
                .a_addr  (a_addr_vec[b]),
                .a_wdata (a_wdata_vec[b]),
                .a_rdata (a_rdata_vec[b*DATA_W +: DATA_W]),
                .a_rvalid(a_rvalid_vec[b]),
                .b_en    (b_en_vec[b]),
                .b_we    (b_we_vec[b]),
                .b_addr  (b_addr_vec[b]),
                .b_wdata (b_wdata_vec[b]),
                .b_rdata (b_rdata_vec[b*DATA_W +: DATA_W]),
                .b_rvalid(b_rvalid_vec[b])
            );
        end
    endgenerate

    integer j;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dma_rd_data  <= {DATA_W{1'b0}};
            dma_rd_valid <= 1'b0;
            eng_rd_data  <= {DATA_W{1'b0}};
            eng_rd_valid <= 1'b0;
            wr_conflict  <= 1'b0;
            for (j = 0; j < BANKS; j = j + 1) begin
                a_rd_src_q[j] <= SRC_NONE;
                b_rd_src_q[j] <= SRC_NONE;
            end
        end else begin
            // 每拍一开始就把两个 valid 默认设为 0
            // 只有后面确实有数据回来时，才会被覆盖为 1
            dma_rd_valid <= 1'b0;
            eng_rd_valid <= 1'b0;

            // Track who issued read on each physical port (1-cycle RAM read latency)   读来源标签寄存
            for (j = 0; j < BANKS; j = j + 1) begin
                a_rd_src_q[j] <= (a_en_vec[j] && !a_we_vec[j]) ? a_rd_src_n[j] : SRC_NONE;
                b_rd_src_q[j] <= (b_en_vec[j] && !b_we_vec[j]) ? b_rd_src_n[j] : SRC_NONE;
            end

            // Return read data to each logical requester   读数据回送
            for (j = 0; j < BANKS; j = j + 1) begin
                if (a_rvalid_vec[j]) begin
                    case (a_rd_src_q[j])
                        SRC_DMA_RD: begin
                            dma_rd_data  <= a_rdata_vec[j*DATA_W +: DATA_W];
                            dma_rd_valid <= 1'b1;
                        end
                        SRC_ENG_RD: begin
                            eng_rd_data  <= a_rdata_vec[j*DATA_W +: DATA_W];
                            eng_rd_valid <= 1'b1;
                        end
                    endcase
                end
                if (b_rvalid_vec[j]) begin
                    case (b_rd_src_q[j])
                        SRC_DMA_RD: begin
                            dma_rd_data  <= b_rdata_vec[j*DATA_W +: DATA_W];
                            dma_rd_valid <= 1'b1;
                        end
                        SRC_ENG_RD: begin
                            eng_rd_data  <= b_rdata_vec[j*DATA_W +: DATA_W];
                            eng_rd_valid <= 1'b1;
                        end
                    endcase
                end
            end

            // 冲突检测: 这一拍只要有任意一个写请求没被接收，就报 conflict
            wr_conflict <= (dma_wr_req && !dma_wr_ready) || (eng_wr_en && !eng_wr_ready);
        end
    end
endmodule