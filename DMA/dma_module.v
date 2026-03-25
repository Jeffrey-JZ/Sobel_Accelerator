`timescale 1ns / 1ps

// DMA Module
// Author: Junze Jiang
// 17/3/2026

// Functions:
// This DMA module bridges the AXI4 external memory bus and the Private Frame Buffer (PFB),
// acting as the data mover for the entire Sobel accelerator system.
 
// Two core data-path operations:
// 1. LOAD  (AXI → PFB) — Unpack:
//  Read one AXI word (32-bit) per beat from external memory,
//  then write each byte within that word into the PFB one pixel at a time.
//  AXI word (4 bytes) → 4 individual pixel writes to PFB.
//
// 2. STORE (PFB → AXI) — Pack:
//  Read individual pixels (bytes) from the PFB one by one,
//  assemble them into a full AXI word, then burst-write back to external memory.
//  4 PFB pixel reads → 1 AXI word write.

/* 
Overall Transaction Flow:
    Host/Control block asserts `start`
    → DMA issues AXI burst reads to fetch the raw image        (LOAD phase)
    → DMA unpacks each word and pushes bytes into the PFB
    → DMA asserts `engine_start` to kick off the Sobel engine  (ENGINE phase)
    → Engine processes in-place and returns `engine_done`
    → DMA reads processed pixels back from PFB                 (STORE phase)
    → DMA packs bytes into AXI words and burst-writes to dst
    → DMA asserts `done`, transaction complete
*/

/* 
Burst Strategy:
    - Uses AXI4 INCR bursts (arburst/awburst = 2'b01)
    - Maximum burst length capped by MAX_BURST_BEATS (default 16) 一个burst 最多传输 16 个 beats, 一个beats最多传输 4 bytes 数据
    - Remaining words handled by issuing successive bursts
*/

// AXI4 有五个独立的通道读和写

// 读操作(2 个通道): 
// 1. AR 通道（Read Address）   ：Master 告诉 Slave 要读哪里
// 2. R  通道（Read Data）      ：Slave 把数据送回给 Master  

// 写操作(3 个通道):
// 1. AW 通道(Write Address)    : Master 告诉 Slave 要写到哪里
// 2. W  通道(Write Data)       : Master 把数据送给 Slave 
// 3. B  通道(Write Response)   : Slave 告诉 Master 写完了

module dma_module #(
    parameter   AXI_ADDR_W      = 32,
    parameter   AXI_DATA_W      = 32,
    parameter   PFB_ADDR_W      = 14,
    parameter   PIXELS_MAX_W    = 16,
    parameter   MAX_BURST_BEATS = 16
) (
    input   wire                                clk,
    input   wire                                rst_n,

    // Control register side (from control&state block)
    input   wire                                start,              // 启动 DMA
    input   wire    [AXI_ADDR_W   - 1:0]        src_base_addr,      // 源地址   DMA 从外部内存的哪个地址开始读数据。
    input   wire    [AXI_ADDR_W   - 1:0]        dst_base_addr,      // 目标地址 DMA 把处理完成后的结果写回外部内存时，从哪个地址开始写
    input   wire    [PIXELS_MAX_W - 1:0]        pixel_count,        // 总共要搬多少个像素 128 * 128 = 16384 bytes
    output  reg                                 busy,               // 告诉外部正在工作
    output  reg                                 done,               // 告诉外部任务已经完成
    output  reg                                 error,

    // Engine control handshake
    output  reg                                 engine_start,       // DMA 通知 engine 开始处理
    input   wire                                engine_done,        // engine 告诉 DMA 数据已经处理完了 DMA 看到这个信号后才开始从 PFB 把处理结果读出来  再写回 AXI 外部内存

    // PFB DMA-side interface (to enigne_pfb_top)   DMA 在 LOAD 阶段把数据写进 PFB，在 STORE 阶段从 PFB 读出数据
    // Read     STORE Phase     DMA 从 PFB 取出 engine 算完的结果
    output  reg                                 pfb_dma_rd_req,
    output  reg     [PFB_ADDR_W - 1:0]          pfb_dma_rd_addr,
    input   wire    [7:0]                       pfb_dma_rd_data,
    input   wire                                pfb_dma_rd_valid,   // 当前 pfb_dma_rd_data 这条线上放着的是否为有效数据
    input   wire                                pfb_dma_rd_ready,   // PFB 告诉 DMA：Read请求能不能接

    // Write    LOAD Phase      DMA 把从 AXI 读到的原始图像写入 PFB
    output  reg                                 pfb_dma_wr_req,
    output  reg     [PFB_ADDR_W - 1:0]          pfb_dma_wr_addr,
    output  reg     [7:0]                       pfb_dma_wr_data,
    input   wire                                pfb_dma_wr_ready,

    // AXI4 read address channel(AR)
    output  reg     [AXI_ADDR_W - 1:0]          m_axi_araddr,       // DMA 要从外部内存哪个地址开始读   src_base_addr + offset
    output  reg     [7:0]                       m_axi_arlen,        // 本次 burst 要读多少个 beat
    output  reg     [2:0]                       m_axi_arsize,       // 每个 beat 的传输大小     1 beat = 32 bits = 4 bytes = 4 pixels
    output  reg     [1:0]                       m_axi_arburst,      // burst 类型   2'b01 = INCR burst  表示每个 beat 地址递增
    output  reg                                 m_axi_arvalid,
    input   wire                                m_axi_arready,

    // AXI4 read data channel(R)
    input   wire    [AXI_DATA_W - 1:0]          m_axi_rdata,        // 读数据 32 bits
    input   wire    [1:0]                       m_axi_rresp,        // 读响应状态   表示这次读返回是否正常  2'b00 = 正常  其他值 = 错误或异常响应
    input   wire                                m_axi_rlast,        // 表示当前这个 beat 是本次 burst 的最后一个 beat
    input   wire                                m_axi_rvalid,
    output  reg                                 m_axi_rready,

    // AXI4 write address channel(AW)
    output  reg     [AXI_ADDR_W - 1:0]          m_axi_awaddr,
    output  reg     [7:0]                       m_axi_awlen,        // 这次 burst 写多少个 beat
    output  reg     [2:0]                       m_axi_awsize,       // 每个 beat 写字节数量     1 beat = 32 bits = 4 bytes = 4 pixels
    output  reg     [1:0]                       m_axi_awburst,      // burst 类型: INCR 连续递增地址写
    output  reg                                 m_axi_awvalid,
    input   wire                                m_axi_awready,

    // AXI4 write data channel(W)
    output  reg     [AXI_DATA_W       - 1:0]    m_axi_wdata,        // 要写出去的数据
    output  reg     [(AXI_DATA_W / 8) - 1:0]    m_axi_wstrb,        // 字节写使能  AXI 数据宽度是 32 bit = 4 byte; 4'b1111：4 个字节全写    但最后一个 word 可能不是 4 个字节都有效 e.g. 总共只剩 2 个像素要写 那这个 32-bit word 里只有低 2 个 byte 有效   4'b0011：只写低两个字节
    output  reg                                 m_axi_wlast,        // 表示当前这个写数据 beat 是本次 burst 的最后一个;  DMA 用它来告诉 main memory 这是这轮 burst 的最后一拍
    output  reg                                 m_axi_wvalid,
    input   wire                                m_axi_wready,

    // AXI4 write response channel(B)
    input   wire    [1:0]                       m_axi_bresp,        // 写响应状态  表示这次 burst 写入是否成功;  2'b00 = OKAY 其他 = 错误;  DMA 收到错误响应时会置 error
    input   wire                                m_axi_bvalid,
    output  reg                                 m_axi_bready
);
    localparam integer BYTES_PER_BEAT = AXI_DATA_W / 8;                                         // 每个 beat 有着 4 bytes 也就是每个 beat 传输 4 pixels
    localparam integer BYTE_IDX_W     = (BYTES_PER_BEAT <= 1) ? 1 : $clog2(BYTES_PER_BEAT);

    localparam [4:0]
        S_IDLE              = 5'd0,
        S_LOAD_AR           = 5'd1,
        S_LOAD_R            = 5'd2,
        S_LOAD_PUSH_SETUP   = 5'd3,
        S_START_ENG         = 5'd4,
        S_WAIT_ENG          = 5'd5,
        S_STORE_AW          = 5'd6,
        S_STORE_PREP_BEAT   = 5'd7,
        S_STORE_REQ_PFB     = 5'd8,
        S_STORE_WAIT_PFB    = 5'd9,
        S_STORE_SEND_W      = 5'd10,
        S_STORE_WAIT_B      = 5'd11,
        S_DONE              = 5'd12,
        S_ERR               = 5'd13,
        S_LOAD_PUSH_WAIT    = 5'd14,
        S_STORE_REQ_WAIT    = 5'd15,
        S_LOAD_ERR_DRAIN    = 5'd16;

    reg [4:0]                   state;

    reg [PIXELS_MAX_W - 1:0]    load_pixel_idx;             // 在 LOAD 阶段，DMA 已经往 PFB 写到第几个像素了
    reg [PIXELS_MAX_W - 1:0]    store_pixel_idx;            // 在 STORE 阶段，DMA 已经从 PFB 读到了第几个像素

    reg [PIXELS_MAX_W - 1:0]    total_words;
    reg [PIXELS_MAX_W - 1:0]    load_words_done;            // 在 LOAD 阶段，已经从 AXI 读完了多少个 word
    reg [PIXELS_MAX_W - 1:0]    store_words_done;           // 在 STORE 阶段，已经往 AXI 写回了多少个 word

    reg [7:0]                   burst_beats;                // 当前这个 burst 总共有多少个 beat
    reg [7:0]                   burst_beat_idx;             // 当前这个 burst 已经进行到第几个 beat

    reg [AXI_DATA_W - 1:0]      load_word_buf;              // 保存刚从 AXI 读回来的那个 32-bit word    LOAD 阶段的临时缓存
    reg [BYTE_IDX_W - 1:0]      load_byte_idx;              // 当前正在从 load_word_buf 里取第几个 byte
    reg                         load_word_valid;

    reg [AXI_DATA_W - 1:0]      store_word_buf;             // 从 PFB 读回来的多个 8-bit pixel 临时拼成一个 32-bit word
    reg [BYTE_IDX_W - 1:0]      store_byte_idx;             // 当前正准备往 store_word_buf 里填第几个 byte
    reg [BYTE_IDX_W:0]          store_bytes_valid;          // 当前 store_word_buf 里面已经装了多少个有效 byte
    reg                         store_word_valid;           // 当前 store_word_buf 里是不是已经拼好了一个可以发往 AXI 的 word
    reg                         store_pfb_req_inflight;     // DMA 已经向 PFB 发出了一个读请求，但是这个读请求对应的数据还没有返回

    integer words_left_i;
    integer this_burst_i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            // Output Reset: Control register side
            busy  <= 1'b0;
            done  <= 1'b0;
            error <= 1'b0;

            // Output Reset: Engine control handshake
            engine_start <= 1'b0;

            // Output Reset: PFB DMA-side interface
            pfb_dma_rd_req  <= 1'b0;
            pfb_dma_rd_addr <= {PFB_ADDR_W{1'b0}};
            pfb_dma_wr_req  <= 1'b0;
            pfb_dma_wr_addr <= {PFB_ADDR_W{1'b0}};
            pfb_dma_wr_data <= 8'd0;

            // Output Reset: AXI4 read address channel(AR) & AXI4 read data channel(R)
            m_axi_araddr  <= {AXI_ADDR_W{1'b0}};
            m_axi_arlen   <= 8'd0;
            m_axi_arsize  <= (AXI_DATA_W == 64) ? 3'd3 : 3'd2;
            m_axi_arburst <= 2'b01;
            m_axi_arvalid <= 1'b0;
            m_axi_rready  <= 1'b0;

            // Output Reset: AXI4 write address channel(AW) & AXI4 write data channel(W) & AXI4 write response channel(B)
            m_axi_awaddr  <= {AXI_ADDR_W{1'b0}};
            m_axi_awlen   <= 8'd0;
            m_axi_awsize  <= (AXI_DATA_W == 64) ? 3'd3 : 3'd2;
            m_axi_awburst <= 2'b01;
            m_axi_awvalid <= 1'b0;
            m_axi_wdata   <= {AXI_DATA_W{1'b0}};
            m_axi_wstrb   <= {AXI_DATA_W/8{1'b0}};
            m_axi_wlast   <= 1'b0;
            m_axi_wvalid  <= 1'b0;
            m_axi_bready  <= 1'b0;

            load_pixel_idx   <= {PIXELS_MAX_W{1'b0}};
            store_pixel_idx  <= {PIXELS_MAX_W{1'b0}};
            total_words      <= {PIXELS_MAX_W{1'b0}};
            load_words_done  <= {PIXELS_MAX_W{1'b0}};
            store_words_done <= {PIXELS_MAX_W{1'b0}};
            burst_beats      <= 8'd0;
            burst_beat_idx   <= 8'd0;

            load_word_buf   <= {AXI_DATA_W{1'b0}};
            load_byte_idx   <= {BYTE_IDX_W{1'b0}};
            load_word_valid <= 1'b0;

            store_word_buf         <= {AXI_DATA_W{1'b0}};
            store_byte_idx         <= {BYTE_IDX_W{1'b0}};
            store_bytes_valid      <= {(BYTE_IDX_W+1){1'b0}};
            store_word_valid       <= 1'b0;
            store_pfb_req_inflight <= 1'b0;
        end else begin
            done         <= 1'b0;
            engine_start <= 1'b0;

            case (state)

                // Phase 0: 空闲状态
                // - 等待 start 清零各种计数器
                // - 计算总共多少个 AXI word 
                // - 如果 pixel_count == 0 不需要传输，直接结束 否则进入 LOAD 阶段
                S_IDLE: begin
                    busy <= 1'b0;
                    error <= 1'b0;

                    // Reset req & handshake regs
                    pfb_dma_rd_req <= 1'b0;
                    pfb_dma_wr_req <= 1'b0;

                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b0;
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b0;
                    m_axi_bready  <= 1'b0;

                    if (start) begin
                        busy <= 1'b1;

                        // 跨多个状态持续保存的事务状态寄存器
                        load_pixel_idx          <= 0;
                        store_pixel_idx         <= 0;
                        load_words_done         <= 0;
                        store_words_done        <= 0;
                        load_word_valid         <= 1'b0;
                        store_word_valid        <= 1'b0;
                        store_pfb_req_inflight  <= 1'b0;

                        total_words <= (pixel_count + BYTES_PER_BEAT - 1) / BYTES_PER_BEAT;     // 整个图片总共需要多少个words
                                                                                                // e.g. pixel_count = 16 每 word 4 字节 => total_words = 4  如果 pixel_count = 10 => total_words = (10+3)/4 = 3  3 个 4-bytes word 才装得下
                        if (pixel_count == 0)
                            state <= S_DONE;
                        else
                            state <= S_LOAD_AR;
                    end
                end

                // Phase 1. LOAD PHASE (AXI burst read -> PFB byte write)
                // 这个阶段的任务是把原始图像从外部内存搬进 PFB，核心难点在于 AXI 每次给你一个 32-bit word（4 个像素），但 PFB 每次只能写 1 个字节，所以需要解包

                // Phase 1.1. Issue AXI burst read address
                // 在 AR 通道上发起一次 burst 读请求
                // 计算这次 burst 要读多少个 beat(最多 16 个, 受 MAX_BURST_BEATS 限制)
                // 设置好 araddr、arlen、arsize、arburst, 等 slave 回 arready 后进入下一状态
                S_LOAD_AR: begin
                    // 计算本次 burst 的长度
                    // e.g. 总共要读 40 个 word，前两轮各读 16，第三轮读剩下的 8
                    words_left_i = total_words - load_words_done;
                    this_burst_i = (words_left_i > MAX_BURST_BEATS) ? MAX_BURST_BEATS : words_left_i;

                    // AR Channel
                    m_axi_araddr  <= src_base_addr + (load_words_done * BYTES_PER_BEAT);    // starting address + offset
                    m_axi_arlen   <= this_burst_i - 1;
                    m_axi_arsize  <= (AXI_DATA_W == 64) ? 3'd3 : 3'd2;                      // 每 beat 4 字节 (2^2 = 4)
                    m_axi_arburst <= 2'b01;                                                 // INCR burst
                    m_axi_arvalid <= 1'b1;

                    burst_beats    <= this_burst_i[7:0];
                    burst_beat_idx <= 8'd0;

                    // 等待握手完成
                    if (m_axi_arvalid && m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                        m_axi_rready  <= 1'b1;
                        state         <= S_LOAD_R;
                    end
                end

                // Phase 1.2. Receive one AXI R-channel beat
                // 在 AXI 的 R 通道上接收 slave 返回的数据，同时做错误检查
                S_LOAD_R: begin
                    // 背压机制(Back Pressure): 当下游处理不过来时, 主动向上游喊停，防止数据/req把系统压崩溃
                    // 如果上一个 word 还在拆，就把 rready 拉低，slave 不再传输数据
                    // 如果已经拆完了，才拉高 rready 允许接收下一个 beat 的数据
                    pfb_dma_wr_req <= 1'b0;
                    if (!load_word_valid)
                        m_axi_rready <= 1'b1;
                    else
                        m_axi_rready <= 1'b0;

                    // 握手成功，锁存数据
                    if (m_axi_rvalid && m_axi_rready) begin
                        load_word_buf   <= m_axi_rdata;
                        load_byte_idx   <= 0;
                        load_word_valid <= 1'b1;
                        m_axi_rready    <= 1'b0;

                        // 读的返回值不是2'b00 有错误 返回error
                        if (m_axi_rresp != 2'b00) begin
                            error <= 1'b1;
                            if (m_axi_rlast) begin      // 如果这恰好是 burst 的最后一拍(rlast=1), burst 已经自然结束，直接进 S_ERR
                                state <= S_ERR;
                            end else begin              // 如果不是最后一拍, AXI 协议要求 master 必须把剩余的 beat 全部接收完才能结束这个 burst, 所以进 S_LOAD_ERR_DRAIN 去把剩下的 beat 接收完
                                state <= S_LOAD_ERR_DRAIN;
                            end
                        end else begin
                            burst_beat_idx <= burst_beat_idx + 1'b1;     // beat counter + 1
                            if (((burst_beat_idx + 1'b1) == burst_beats) && !m_axi_rlast) begin     // DMA counter觉得结束了但 slave 没有拉 rlast, 说明双方对 burst 长度的理解不一致，这是严重的协议违规，直接报错
                                error <= 1'b1;
                                state <= S_ERR;
                            end else begin      // DMA counter的最后一拍和 slave 的 rlast 信号应该对得上
                                state <= S_LOAD_PUSH_SETUP;
                            end
                        end
                    end
                end

                // Phase 1.3. Prepare PFB write (addr + byte select)
                // 从 load_word_buf 中取出当前 load_byte_idx 位置的那个字节，准备好 PFB 写请求(pfb_dma_wr_req、地址、数据), 然后进入等待状态
                S_LOAD_PUSH_SETUP: begin
                    if (load_word_valid) begin  // 确认 load_word_buf 里确实有一个待拆的 word
                        pfb_dma_wr_req  <= 1'b1;                                    // 告诉 PFB 要写一个字节
                        pfb_dma_wr_addr <= load_pixel_idx[PFB_ADDR_W - 1:0];        // 写到 PFB 的哪个地址，就是当前的像素索引 比如第 0 个像素写地址 0，第 1 个写地址 1，依此类推
                        pfb_dma_wr_data <= load_word_buf[8 * load_byte_idx +: 8];
                        state <= S_LOAD_PUSH_WAIT;
                    end
                end

                // Phase 1.4. Wait PFB write handshake, dispatch next
                // LOAD 阶段的等握手 + 决定下一步去哪的状态
                // 保持写请求稳定，等 PFB 回 pfb_dma_wr_ready
                // 一旦握手成功，load_pixel_idx++
                // 然后判断三种情况：
                // 1. 所有像素都写完了 → 进 ENGINE
                // 2. 当前 word 的 4 个字节还没拆完 → 回 PUSH_SETUP 拆下一个字节
                // 3. 当前 word 拆完了 → 回 S_LOAD_R 接收下一个 AXI beat(如果这个 burst 也完了，就回 S_LOAD_AR 发起新 burst)
                S_LOAD_PUSH_WAIT: begin
                    if (load_word_valid) begin
                        // 保持请求和数据稳定，直到 ready
                        pfb_dma_wr_req  <= 1'b1;
                        pfb_dma_wr_addr <= load_pixel_idx[PFB_ADDR_W - 1:0];
                        pfb_dma_wr_data <= load_word_buf[8 * load_byte_idx +: 8];

                        if (pfb_dma_wr_ready) begin
                            pfb_dma_wr_req <= 1'b0;
                            load_pixel_idx <= load_pixel_idx + 1'b1;    // pixels counter +1
                            
                            // 1. 所有像素写完了 → 进 ENGINE
                            if ((load_pixel_idx + 1'b1) >= pixel_count) begin
                                load_word_valid <= 1'b0;
                                load_words_done <= load_words_done + 1'b1;
                                state <= S_START_ENG;
                            end
                            // 2. 当前 word 的 4 个字节都拆完了
                            else if (load_byte_idx == BYTES_PER_BEAT-1) begin
                                load_word_valid <= 1'b0;
                                load_words_done <= load_words_done + 1'b1;

                                if ((burst_beat_idx >= burst_beats) && ((load_words_done + 1'b1) < total_words))            // burst 用完了，还有数据要读 → 发起新 burst
                                    state <= S_LOAD_AR;
                                else if ((burst_beat_idx >= burst_beats) && ((load_words_done + 1'b1) == total_words))      // burst 用完了，数据也读完了 → 进 ENGINE
                                    state <= S_START_ENG;
                                else                                                                                        // burst 还没用完 → 回去收下一个 beat
                                    state <= S_LOAD_R;
                            end
                            // 3. 当前 word 还没拆完
                            else begin
                                load_byte_idx <= load_byte_idx + 1'b1;
                                state <= S_LOAD_PUSH_SETUP;
                            end
                        end
                    end
                end

                // Phase 1.5. Drain remaining R beats after error
                // 在 burst 中途发现错误后，把剩余的 beat 全部处理完
                // AXI protocol: master must accept all R beats until rlast.
                // After rresp error mid-burst, drain remaining beats here.
                S_LOAD_ERR_DRAIN: begin
                    pfb_dma_wr_req  <= 1'b0;    // 不再往 PFB 写任何东西
                    load_word_valid <= 1'b0;    // 丢弃缓存的 word, 反正数据已经不可信了
                    m_axi_rready    <= 1'b1;    // 保持 rready 一直拉高，让 slave 把数据尽快倒完, 把剩余 beat 全部吞掉
                                                // AXI4 协议有一条硬性规定: 
                                                // master 一旦发起了一个 burst 读请求, 就必须把 slave 返回的所有 beat 全部接收完, 直到 rlast=1
                                                // 不能因为中间某一拍报错了就中断; slave 那边的 beat 已经在路上了, 如果 master 不接, 总线就会卡死
                    if (m_axi_rvalid && m_axi_rready && m_axi_rlast) begin  // 当看到 rlast=1 的那一拍握手成功, 说明这个 burst 的所有 beat 都排完了，总线恢复干净
                        m_axi_rready <= 1'b0;
                        state        <= S_ERR;  // 关掉 rready，安全地进入 S_ERR 结束整个 DMA 事务
                    end
                end

                // Phase 2. ENGINE PHASE
                // Phase 2.1. Pulse engine_start for one cycle
                // 1. 把 LOAD 阶段用到的信号全部收干净
                // 2. 是拉高 engine_start 一个周期
                S_START_ENG: begin
                    pfb_dma_wr_req <= 1'b0;
                    m_axi_rready   <= 1'b0;
                    engine_start   <= 1'b1;
                    state          <= S_WAIT_ENG;
                end

                // Phase 2.2. Wait for engine_done, init STORE counters
                // Sobel 引擎在 PFB 上做边缘检测 DMA 每拍检查 engine_done 是否为 1。
                // 一旦引擎处理完了，DMA 立刻初始化 STORE 阶段需要的所有计数器归零，然后跳到 S_STORE_AW 开始把结果写回外部内存
                S_WAIT_ENG: begin
                    if (engine_done) begin
                        store_pixel_idx  <= 0;
                        store_words_done <= 0;
                        store_word_valid <= 1'b0;
                        store_pfb_req_inflight <= 1'b0;
                        state <= S_STORE_AW;
                    end
                end

                // Phase 3. STORE PHASE (PFB byte read -> AXI burst write)
                // Phase 3.1. Issue AXI burst write address
                // 在 AW channel 发写地址
                S_STORE_AW: begin
                    // 计算 burst 长度
                    words_left_i = total_words - store_words_done;
                    this_burst_i = (words_left_i > MAX_BURST_BEATS) ? MAX_BURST_BEATS : words_left_i;

                    // AW channel signals
                    m_axi_awaddr  <= dst_base_addr + (store_words_done * BYTES_PER_BEAT);
                    m_axi_awlen   <= this_burst_i - 1;
                    m_axi_awsize  <= (AXI_DATA_W == 64) ? 3'd3 : 3'd2;
                    m_axi_awburst <= 2'b01;
                    m_axi_awvalid <= 1'b1;

                    burst_beats    <= this_burst_i[7:0];
                    burst_beat_idx <= 8'd0;

                    // 等握手
                    if (m_axi_awvalid && m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                        state         <= S_STORE_PREP_BEAT;
                    end
                end

                // Phase 3.2. Zero out word buffer for new word
                // 准备拼一个新 word
                S_STORE_PREP_BEAT: begin
                    store_word_buf    <= {AXI_DATA_W{1'b0}};    // 32-bit 缓冲区清零
                    store_byte_idx    <= 0;                     // 从 byte[0] 开始填
                    store_bytes_valid <= 0;
                    store_word_valid  <= 1'b0;
                    state             <= S_STORE_REQ_PFB;
                end

                // Phase 3.3. Issue PFB read request
                S_STORE_REQ_PFB: begin
                    if (!store_pfb_req_inflight) begin      // 先检查 store_pfb_req_inflight, 确认没有一个尚未返回的读请求还在路上
                        pfb_dma_rd_req  <= 1'b1;
                        pfb_dma_rd_addr <= store_pixel_idx[PFB_ADDR_W - 1:0];
                        state <= S_STORE_REQ_WAIT;
                    end
                end

                // Phase 3.4. Hold request until PFB accepts
                S_STORE_REQ_WAIT: begin
                    pfb_dma_rd_req  <= 1'b1;
                    pfb_dma_rd_addr <= store_pixel_idx[PFB_ADDR_W - 1:0];

                    if (pfb_dma_rd_ready) begin
                        pfb_dma_rd_req <= 1'b0;
                        store_pfb_req_inflight <= 1'b1;     // 标记 inflight=1(表示数据还在路上)
                        state <= S_STORE_WAIT_PFB;
                    end
                end

                // Phase 3.5. Receive PFB byte, pack into word
                // 等数据回来, 把字节填进 store_word_buf, 然后决定这个 word 是否拼好了
                S_STORE_WAIT_PFB: begin
                    pfb_dma_rd_req <= 1'b0;
                    if (pfb_dma_rd_valid) begin
                        store_pfb_req_inflight                  <= 1'b0;                        // 这次读请求完结
                        store_word_buf[8*store_byte_idx +: 8]   <= pfb_dma_rd_data;             // 把字节填到对应位置   [7:0] 第 1 个像素 [15:8]第 2 个像素 [23:16]第 3 个像素  [31:24]第 4 个像素
                        store_byte_idx                          <= store_byte_idx + 1'b1;       // byte counter +1
                        store_bytes_valid                       <= store_bytes_valid + 1'b1;    // valid byte counter +1
                        store_pixel_idx                         <= store_pixel_idx + 1'b1;      // global pixels counter +1

                        if ((store_byte_idx == BYTES_PER_BEAT-1) || ((store_pixel_idx + 1'b1) >= pixel_count)) begin    // store_byte_idx == 3: 4 个字节都填满了, word 完整
                                                                                                                        // store_pixel_idx + 1 >= pixel_count: 这是最后一个像素了
                            store_word_valid <= 1'b1;
                            state <= S_STORE_SEND_W;
                        end else begin
                            state <= S_STORE_REQ_PFB;
                        end
                    end
                end
                
                // Phase 3.6. Send packed word on AXI W-channel
                // 发数据, 把拼好的 word 通过 AXI 的 Write Data Channel发出去
                S_STORE_SEND_W: begin
                    if (store_word_valid) begin
                        m_axi_wdata  <= store_word_buf;
                        m_axi_wstrb  <= ({(AXI_DATA_W / 8){1'b1}} >> (BYTES_PER_BEAT - store_bytes_valid));     // {(AXI_DATA_W/8){1'b1}} 就是 4'b1111，然后右移 (4 - store_bytes_valid) 位
                                                                                                                // store_bytes_valid=4 => 右移0位 => wstrb=4'b1111 => 4 字节全有效(正常情况)
                                                                                                                // store_bytes_valid=3 => 右移1位 => wstrb=4'b0111 => 只有低 3 字节有效
                                                                                                                // store_bytes_valid=2 => 右移2位 => wstrb=4'b0011 => 只有低 2 字节有效
                        m_axi_wlast  <= ((burst_beat_idx + 1'b1) == burst_beats);   // 标记 burst 的最后一拍
                        m_axi_wvalid <= 1'b1;

                        if (m_axi_wvalid && m_axi_wready) begin     // 握手成功后的清理和分支
                            m_axi_wvalid   <= 1'b0;
                            store_word_valid <= 1'b0;
                            burst_beat_idx <= burst_beat_idx + 1'b1;
                            store_words_done <= store_words_done + 1'b1;

                            if ((burst_beat_idx + 1'b1) == burst_beats) begin   // 检查 burst 是否结束
                                m_axi_bready <= 1'b1;
                                state <= S_STORE_WAIT_B;
                            end else begin
                                state <= S_STORE_PREP_BEAT;
                            end
                        end
                    end
                end

                // Phase 3.7. Wait write response, check bresp
                // AXI 的 Write Response Channel 上等 slave 确认这轮 burst 是否写入成功
                S_STORE_WAIT_B: begin
                    if (m_axi_bvalid && m_axi_bready) begin
                        m_axi_bready <= 1'b0;
                        if (m_axi_bresp != 2'b00) begin
                            error <= 1'b1;
                            state <= S_ERR;
                        end else if (store_words_done >= total_words) begin
                            state <= S_DONE;
                        end else begin
                            state <= S_STORE_AW;
                        end
                    end
                end

                S_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= S_IDLE;
                end

                S_ERR: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule