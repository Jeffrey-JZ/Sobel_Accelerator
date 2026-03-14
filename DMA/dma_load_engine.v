`timescale 1ns / 1ps

// DMA Load Engine
// Author: Junze Jiang
// 14/3/2026
//
// 功能：从 Main Memory 经 AXI4 burst 读取图像数据，拆包后逐像素写入 PFB
//
// 数据流: AXI4 burst read → 32-bit word 拆包为 4 个 8-bit pixel → 逐像素写入 PFB
//
// FSM:
//   S_IDLE     →  等待 start
//   S_AR_ISSUE →  发 AXI4 AR 请求（burst read address）
//   S_R_BEAT   →  接收一个 AXI4 R 数据拍（32-bit word）
//   S_UNPACK   →  将 32-bit word 里的 4 个像素逐一写入 PFB
//   S_DONE     →  传输完成，脉冲 done
//
// 每一拍 AXI 数据 (32 bit) 包含 4 个像素 (8 bit each, little-endian byte order)
//   pixel 0 = word[ 7: 0]
//   pixel 1 = word[15: 8]
//   pixel 2 = word[23:16]
//   pixel 3 = word[31:24]
//
// AXI burst 策略:
//   - INCR burst, 每次 burst 最多 BURST_LEN 拍
//   - 所有 burst 拼起来覆盖整张图像
//
// PFB 写接口时序:
//   - 拉高 dma_wr_req + 给 addr/data
//   - 如果 dma_wr_ready=1 → 数据已写入，下一拍可以推下一个像素
//   - 如果 dma_wr_ready=0 → bank 冲突，保持不变下一拍重试

module dma_load_engine #(
    parameter AXI_ADDR_W    = 32,
    parameter AXI_DATA_W    = 32,
    parameter AXI_ID_W      = 4,
    parameter PIXEL_W       = 8,
    parameter PFB_ADDR_W    = 14,
    parameter BURST_LEN     = 256,          // 每次 burst 的 AXI beat 数 (AXI4 最大 256)
    parameter TOTAL_PIXELS  = 16384         // 128 × 128
) (
    input  wire                     clk,
    input  wire                     rst_n,

    // ---- 控制接口 (来自 dma_top / CSR) ----
    input  wire                     start,          // 脉冲启动
    input  wire [AXI_ADDR_W-1:0]    src_addr,       // Main Memory 起始地址
    output reg                      done,           // 脉冲完成
    output reg                      busy,

    // ---- AXI4 Master Read - Address Channel (AR) ----
    output reg  [AXI_ID_W-1:0]     m_axi_arid,
    output reg  [AXI_ADDR_W-1:0]   m_axi_araddr,
    output reg  [7:0]              m_axi_arlen,     // burst length - 1
    output reg  [2:0]              m_axi_arsize,    // 每拍字节数 = 2^arsize
    output reg  [1:0]              m_axi_arburst,   // 2'b01 = INCR
    output reg                     m_axi_arvalid,
    input  wire                    m_axi_arready,

    // ---- AXI4 Master Read - Data Channel (R) ----
    input  wire [AXI_ID_W-1:0]     m_axi_rid,
    input  wire [AXI_DATA_W-1:0]   m_axi_rdata,
    input  wire [1:0]              m_axi_rresp,
    input  wire                    m_axi_rlast,
    input  wire                    m_axi_rvalid,
    output reg                     m_axi_rready,

    // ---- PFB DMA Write Port ----
    output reg                      dma_wr_req,
    output reg  [PFB_ADDR_W-1:0]   dma_wr_addr,
    output reg  [PIXEL_W-1:0]      dma_wr_data,
    input  wire                     dma_wr_ready
);

    // 常量
    localparam BYTES_PER_BEAT   = AXI_DATA_W / 8;                  // 4
    localparam PIXELS_PER_BEAT  = AXI_DATA_W / PIXEL_W;            // 4
    localparam PIX_IDX_W        = $clog2(PIXELS_PER_BEAT);          // 2
    localparam TOTAL_BEATS      = TOTAL_PIXELS / PIXELS_PER_BEAT;   // 4096

    // FSM 状态编码
    localparam [2:0] S_IDLE     = 3'd0;
    localparam [2:0] S_AR_ISSUE = 3'd1;
    localparam [2:0] S_R_BEAT   = 3'd2;
    localparam [2:0] S_UNPACK   = 3'd3;
    localparam [2:0] S_DONE     = 3'd4;

    reg [2:0]                   state;

    // AXI 地址 & 拍数追踪
    reg [AXI_ADDR_W-1:0]       cur_axi_addr;           // 下一次 burst 的起始地址
    reg [15:0]                  beats_remaining;        // 还有多少 AXI 拍没有读
    reg [7:0]                   cur_burst_len;          // 当前 burst 的长度 (单位: beat)

    // 像素拆包
    reg [AXI_DATA_W-1:0]       beat_data;              // 锁存的 32-bit AXI 数据
    reg                         beat_rlast;             // 锁存的 rlast 标志
    reg [PIX_IDX_W-1:0]        pix_idx;                // 当前正在写第几个像素 (0..3)

    // PFB 地址
    reg [PFB_ADDR_W-1:0]       pfb_addr_cnt;           // 当前 PFB 写地址 (0 ~ TOTAL_PIXELS-1)

    // 从 beat_data 提取当前像素 (组合逻辑)
    // little-endian: pixel 0 = beat_data[7:0], pixel 1 = beat_data[15:8], ...
    wire [PIXEL_W-1:0] cur_pixel = beat_data[pix_idx * PIXEL_W +: PIXEL_W];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            done            <= 1'b0;
            busy            <= 1'b0;

            m_axi_arid      <= {AXI_ID_W{1'b0}};
            m_axi_araddr    <= {AXI_ADDR_W{1'b0}};
            m_axi_arlen     <= 8'd0;
            m_axi_arsize    <= 3'd0;
            m_axi_arburst   <= 2'b01;
            m_axi_arvalid   <= 1'b0;
            m_axi_rready    <= 1'b0;

            dma_wr_req      <= 1'b0;
            dma_wr_addr     <= {PFB_ADDR_W{1'b0}};
            dma_wr_data     <= {PIXEL_W{1'b0}};

            cur_axi_addr    <= {AXI_ADDR_W{1'b0}};
            beats_remaining <= 16'd0;
            cur_burst_len   <= 8'd0;
            beat_data       <= {AXI_DATA_W{1'b0}};
            beat_rlast      <= 1'b0;
            pix_idx         <= {PIX_IDX_W{1'b0}};
            pfb_addr_cnt    <= {PFB_ADDR_W{1'b0}};
        end else begin
            // 默认脉冲信号清零
            done <= 1'b0;

            case (state)

                // ============================================================
                // S_IDLE: 等待 start 脉冲
                // ============================================================
                S_IDLE: begin
                    busy            <= 1'b0;
                    m_axi_arvalid   <= 1'b0;
                    m_axi_rready    <= 1'b0;
                    dma_wr_req      <= 1'b0;

                    if (start) begin
                        busy            <= 1'b1;
                        cur_axi_addr    <= src_addr;
                        pfb_addr_cnt    <= {PFB_ADDR_W{1'b0}};
                        beats_remaining <= TOTAL_BEATS[15:0];
                        state           <= S_AR_ISSUE;
                    end
                end

                // ============================================================
                // S_AR_ISSUE: 发出 AXI4 burst 读地址请求
                //   - 计算 burst 长度 = min(BURST_LEN, beats_remaining)
                //   - arvalid 拉高并保持，直到 arready 握手成功
                //   - 握手成功后推进地址，进入 S_R_BEAT 接收数据
                // ============================================================
                S_AR_ISSUE: begin
                    dma_wr_req <= 1'b0;

                    if (beats_remaining == 16'd0) begin
                        // 所有数据已读完
                        state <= S_DONE;
                    end else begin
                        // 设置 burst 参数
                        m_axi_arvalid   <= 1'b1;
                        m_axi_araddr    <= cur_axi_addr;
                        m_axi_arsize    <= $clog2(BYTES_PER_BEAT);  // 3'd2 for 4 bytes
                        m_axi_arburst   <= 2'b01;                   // INCR
                        m_axi_arid      <= {AXI_ID_W{1'b0}};

                        if (beats_remaining >= BURST_LEN) begin
                            cur_burst_len   <= BURST_LEN[7:0];
                            m_axi_arlen     <= BURST_LEN[7:0] - 8'd1;
                        end else begin
                            cur_burst_len   <= beats_remaining[7:0];
                            m_axi_arlen     <= beats_remaining[7:0] - 8'd1;
                        end

                        // AR 握手成功
                        if (m_axi_arvalid && m_axi_arready) begin
                            m_axi_arvalid   <= 1'b0;
                            m_axi_rready    <= 1'b1;   // 准备接收数据

                            // 提前计算下一次 burst 的起始地址
                            // cur_burst_len 在上面赋值，这里用的是上一拍的值（非阻塞赋值）
                            // 但由于 beats_remaining 不变，值相同，所以正确
                            if (beats_remaining >= BURST_LEN)
                                cur_axi_addr <= cur_axi_addr + (BURST_LEN * BYTES_PER_BEAT);
                            else
                                cur_axi_addr <= cur_axi_addr + (beats_remaining * BYTES_PER_BEAT);

                            state <= S_R_BEAT;
                        end
                    end
                end

                // ============================================================
                // S_R_BEAT: 等待 AXI4 R 通道返回一拍数据
                //   - rready=1 等待 rvalid
                //   - 握手成功: 锁存 rdata + rlast，关闭 rready，进入 S_UNPACK
                // ============================================================
                S_R_BEAT: begin
                    m_axi_rready <= 1'b1;

                    if (m_axi_rvalid && m_axi_rready) begin
                        beat_data       <= m_axi_rdata;
                        beat_rlast      <= m_axi_rlast;
                        m_axi_rready    <= 1'b0;
                        pix_idx         <= {PIX_IDX_W{1'b0}};
                        beats_remaining <= beats_remaining - 16'd1;
                        state           <= S_UNPACK;
                    end
                end

                // ============================================================
                // S_UNPACK: 把 32-bit word 里的 4 个像素逐一写入 PFB
                //   - dma_wr_req=1, 给出 addr + data
                //   - 如果 PFB 返回 dma_wr_ready=1 → 写入成功，推进到下一像素
                //   - 如果 dma_wr_ready=0 → bank 冲突，保持不变下一拍重试
                //   - 4 个像素写完后:
                //       beat_rlast=1 且 beats_remaining=0 → 全部完成 → S_DONE
                //       beat_rlast=1 且还有数据 → 下一个 burst → S_AR_ISSUE
                //       beat_rlast=0 → 同一 burst 内下一拍 → S_R_BEAT
                // ============================================================
                S_UNPACK: begin
                    dma_wr_req  <= 1'b1;
                    dma_wr_addr <= pfb_addr_cnt;
                    dma_wr_data <= cur_pixel;

                    if (dma_wr_ready) begin
                        pfb_addr_cnt <= pfb_addr_cnt + 1'b1;

                        if (pix_idx == PIXELS_PER_BEAT[PIX_IDX_W-1:0] - 1) begin
                            // 当前 beat 的 4 个像素全部写完
                            dma_wr_req <= 1'b0;

                            if (beat_rlast) begin
                                // 当前 burst 结束
                                if (beats_remaining == 16'd0)
                                    state <= S_DONE;        // 全部读完
                                else
                                    state <= S_AR_ISSUE;    // 发下一个 burst
                            end else begin
                                // 同一 burst 内继续接收下一拍
                                state <= S_R_BEAT;
                            end
                        end else begin
                            pix_idx <= pix_idx + 1'b1;
                        end
                    end
                    // dma_wr_ready=0: 保持 req/addr/data 不变，下一拍自动重试
                end

                // ============================================================
                // S_DONE: 传输完成
                // ============================================================
                S_DONE: begin
                    dma_wr_req  <= 1'b0;
                    done        <= 1'b1;
                    busy        <= 1'b0;
                    state       <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
