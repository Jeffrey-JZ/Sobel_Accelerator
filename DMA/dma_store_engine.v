`timescale 1ns / 1ps

// DMA Store Engine
// Author: Junze Jiang
// 14/3/2026
//
// 功能：从 PFB 逐像素读取处理后的图像，打包后经 AXI4 burst 写回 Main Memory
//
// 数据流: PFB read (8-bit pixel) → 4 像素打包为 32-bit word → AXI4 burst write
//
// FSM:
//   S_IDLE       →  等待 start
//   S_AW_ISSUE   →  发 AXI4 AW 请求（burst write address）
//   S_PFB_RD_REQ →  向 PFB 发读请求
//   S_PFB_RD_WAIT→  等待 PFB 返回像素数据 (1-cycle latency)
//   S_W_SEND     →  把打包好的 32-bit word 发到 AXI4 W 通道
//   S_B_WAIT     →  等待 AXI4 写响应 (B channel)
//   S_DONE       →  传输完成，脉冲 done
//
// PFB 读接口时序（参考 pfb_top.v 注释）:
//   拍 T:   DMA 拉高 dma_rd_req + 给 dma_rd_addr → PFB 回 dma_rd_ready=1（接受了）
//   拍 T+1: PFB 给出 dma_rd_data + dma_rd_valid=1 → DMA 拿到像素值
//   如果 dma_rd_ready=0 → 该 bank 端口全被占了，下一拍重试
//
// 像素打包顺序 (little-endian):
//   pixel 0 → word[ 7: 0]
//   pixel 1 → word[15: 8]
//   pixel 2 → word[23:16]
//   pixel 3 → word[31:24]

module dma_store_engine #(
    parameter AXI_ADDR_W    = 32,
    parameter AXI_DATA_W    = 32,
    parameter AXI_ID_W      = 4,
    parameter PIXEL_W       = 8,
    parameter PFB_ADDR_W    = 14,
    parameter BURST_LEN     = 256,          // 每次 burst 的 AXI beat 数
    parameter TOTAL_PIXELS  = 16384         // 128 × 128
) (
    input  wire                     clk,
    input  wire                     rst_n,

    // ---- 控制接口 (来自 dma_top / CSR) ----
    input  wire                     start,
    input  wire [AXI_ADDR_W-1:0]    dst_addr,       // Main Memory 目标起始地址
    output reg                      done,
    output reg                      busy,

    // ---- AXI4 Master Write - Address Channel (AW) ----
    output reg  [AXI_ID_W-1:0]     m_axi_awid,
    output reg  [AXI_ADDR_W-1:0]   m_axi_awaddr,
    output reg  [7:0]              m_axi_awlen,     // burst length - 1
    output reg  [2:0]              m_axi_awsize,    // 每拍字节数 = 2^awsize
    output reg  [1:0]              m_axi_awburst,   // 2'b01 = INCR
    output reg                     m_axi_awvalid,
    input  wire                    m_axi_awready,

    // ---- AXI4 Master Write - Data Channel (W) ----
    output reg  [AXI_DATA_W-1:0]   m_axi_wdata,
    output reg  [AXI_DATA_W/8-1:0] m_axi_wstrb,    // byte strobe
    output reg                     m_axi_wlast,
    output reg                     m_axi_wvalid,
    input  wire                    m_axi_wready,

    // ---- AXI4 Master Write - Response Channel (B) ----
    input  wire [AXI_ID_W-1:0]     m_axi_bid,
    input  wire [1:0]              m_axi_bresp,
    input  wire                    m_axi_bvalid,
    output reg                     m_axi_bready,

    // ---- PFB DMA Read Port ----
    output reg                      dma_rd_req,
    output reg  [PFB_ADDR_W-1:0]   dma_rd_addr,
    input  wire [PIXEL_W-1:0]      dma_rd_data,
    input  wire                     dma_rd_valid,
    input  wire                     dma_rd_ready
);

    // 常量
    localparam BYTES_PER_BEAT   = AXI_DATA_W / 8;                  // 4
    localparam PIXELS_PER_BEAT  = AXI_DATA_W / PIXEL_W;            // 4
    localparam PIX_IDX_W        = $clog2(PIXELS_PER_BEAT);          // 2
    localparam TOTAL_BEATS      = TOTAL_PIXELS / PIXELS_PER_BEAT;   // 4096
    localparam STRB_ALL         = {(AXI_DATA_W/8){1'b1}};          // 4'b1111

    // FSM 状态编码
    localparam [2:0] S_IDLE         = 3'd0;
    localparam [2:0] S_AW_ISSUE     = 3'd1;
    localparam [2:0] S_PFB_RD_REQ   = 3'd2;
    localparam [2:0] S_PFB_RD_WAIT  = 3'd3;
    localparam [2:0] S_W_SEND       = 3'd4;
    localparam [2:0] S_B_WAIT       = 3'd5;
    localparam [2:0] S_DONE         = 3'd6;

    reg [2:0]                   state;

    // AXI 地址 & 拍数追踪
    reg [AXI_ADDR_W-1:0]       cur_axi_addr;           // 下一次 burst 的起始地址
    reg [15:0]                  beats_remaining;        // 还有多少 AXI 拍没有写
    reg [7:0]                   cur_burst_len;          // 当前 burst 的长度
    reg [7:0]                   beats_in_burst;         // 当前 burst 已发送多少拍

    // 像素打包
    reg [AXI_DATA_W-1:0]       pack_reg;               // 打包寄存器
    reg [PIX_IDX_W-1:0]        pix_idx;                // 当前在打包第几个像素 (0..3)

    // PFB 地址
    reg [PFB_ADDR_W-1:0]       pfb_addr_cnt;           // 当前 PFB 读地址

    // 标记 PFB 读请求是否已被接受（用于跟踪 req→valid 的 1-cycle latency）
    reg                         rd_req_accepted;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            done            <= 1'b0;
            busy            <= 1'b0;

            m_axi_awid      <= {AXI_ID_W{1'b0}};
            m_axi_awaddr    <= {AXI_ADDR_W{1'b0}};
            m_axi_awlen     <= 8'd0;
            m_axi_awsize    <= 3'd0;
            m_axi_awburst   <= 2'b01;
            m_axi_awvalid   <= 1'b0;

            m_axi_wdata     <= {AXI_DATA_W{1'b0}};
            m_axi_wstrb     <= {(AXI_DATA_W/8){1'b0}};
            m_axi_wlast     <= 1'b0;
            m_axi_wvalid    <= 1'b0;

            m_axi_bready    <= 1'b0;

            dma_rd_req      <= 1'b0;
            dma_rd_addr     <= {PFB_ADDR_W{1'b0}};

            cur_axi_addr    <= {AXI_ADDR_W{1'b0}};
            beats_remaining <= 16'd0;
            cur_burst_len   <= 8'd0;
            beats_in_burst  <= 8'd0;

            pack_reg        <= {AXI_DATA_W{1'b0}};
            pix_idx         <= {PIX_IDX_W{1'b0}};
            pfb_addr_cnt    <= {PFB_ADDR_W{1'b0}};
            rd_req_accepted <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)

                // ============================================================
                // S_IDLE: 等待 start 脉冲
                // ============================================================
                S_IDLE: begin
                    busy            <= 1'b0;
                    m_axi_awvalid   <= 1'b0;
                    m_axi_wvalid    <= 1'b0;
                    m_axi_bready    <= 1'b0;
                    dma_rd_req      <= 1'b0;

                    if (start) begin
                        busy            <= 1'b1;
                        cur_axi_addr    <= dst_addr;
                        pfb_addr_cnt    <= {PFB_ADDR_W{1'b0}};
                        beats_remaining <= TOTAL_BEATS[15:0];
                        beats_in_burst  <= 8'd0;
                        pix_idx         <= {PIX_IDX_W{1'b0}};
                        pack_reg        <= {AXI_DATA_W{1'b0}};
                        state           <= S_AW_ISSUE;
                    end
                end

                // ============================================================
                // S_AW_ISSUE: 发出 AXI4 burst 写地址请求
                //   - 计算 burst 长度 = min(BURST_LEN, beats_remaining)
                //   - awvalid 拉高并保持，直到 awready 握手成功
                // ============================================================
                S_AW_ISSUE: begin
                    dma_rd_req <= 1'b0;

                    if (beats_remaining == 16'd0) begin
                        state <= S_DONE;
                    end else begin
                        m_axi_awvalid   <= 1'b1;
                        m_axi_awaddr    <= cur_axi_addr;
                        m_axi_awsize    <= $clog2(BYTES_PER_BEAT);
                        m_axi_awburst   <= 2'b01;
                        m_axi_awid      <= {AXI_ID_W{1'b0}};

                        if (beats_remaining >= BURST_LEN) begin
                            cur_burst_len   <= BURST_LEN[7:0];
                            m_axi_awlen     <= BURST_LEN[7:0] - 8'd1;
                        end else begin
                            cur_burst_len   <= beats_remaining[7:0];
                            m_axi_awlen     <= beats_remaining[7:0] - 8'd1;
                        end

                        // AW 握手成功
                        if (m_axi_awvalid && m_axi_awready) begin
                            m_axi_awvalid   <= 1'b0;
                            beats_in_burst  <= 8'd0;
                            pix_idx         <= {PIX_IDX_W{1'b0}};
                            pack_reg        <= {AXI_DATA_W{1'b0}};

                            // 提前计算下一次 burst 的起始地址
                            if (beats_remaining >= BURST_LEN)
                                cur_axi_addr <= cur_axi_addr + (BURST_LEN * BYTES_PER_BEAT);
                            else
                                cur_axi_addr <= cur_axi_addr + (beats_remaining * BYTES_PER_BEAT);

                            state <= S_PFB_RD_REQ;
                        end
                    end
                end

                // ============================================================
                // S_PFB_RD_REQ: 向 PFB 发起一次读请求
                //   - 拉高 dma_rd_req + 给 dma_rd_addr
                //   - 如果 dma_rd_ready=1 → 请求被接受，下一拍等数据
                //   - 如果 dma_rd_ready=0 → 保持请求，下一拍重试
                // ============================================================
                S_PFB_RD_REQ: begin
                    dma_rd_req  <= 1'b1;
                    dma_rd_addr <= pfb_addr_cnt;

                    if (dma_rd_ready) begin
                        dma_rd_req      <= 1'b0;
                        pfb_addr_cnt    <= pfb_addr_cnt + 1'b1;
                        rd_req_accepted <= 1'b1;
                        state           <= S_PFB_RD_WAIT;
                    end
                end

                // ============================================================
                // S_PFB_RD_WAIT: 等待 PFB 返回读数据 (1-cycle latency)
                //   - dma_rd_valid=1 → 拿到像素，打包进 pack_reg
                //   - 打包满 4 个像素 → 送到 AXI W 通道
                // ============================================================
                S_PFB_RD_WAIT: begin
                    dma_rd_req      <= 1'b0;
                    rd_req_accepted <= 1'b0;

                    if (dma_rd_valid) begin
                        // 把像素打包进 pack_reg
                        // little-endian: pix 0 → [7:0], pix 1 → [15:8], ...
                        pack_reg[pix_idx * PIXEL_W +: PIXEL_W] <= dma_rd_data;

                        if (pix_idx == PIXELS_PER_BEAT[PIX_IDX_W-1:0] - 1) begin
                            // 4 个像素收齐，准备发 AXI W
                            pix_idx <= {PIX_IDX_W{1'b0}};
                            state   <= S_W_SEND;
                        end else begin
                            pix_idx <= pix_idx + 1'b1;
                            state   <= S_PFB_RD_REQ;   // 继续读下一个像素
                        end
                    end
                end

                // ============================================================
                // S_W_SEND: 把打包好的 32-bit word 送到 AXI4 W 通道
                //   - 注意：pack_reg 里最后一个像素是上一拍用非阻塞赋值写入的
                //     它在这一拍的寄存器值中已经更新 ✓
                //   - wlast: 当前 burst 的最后一拍
                //   - wready 握手成功后推进 beat 计数
                // ============================================================
                S_W_SEND: begin
                    m_axi_wvalid    <= 1'b1;
                    m_axi_wdata     <= pack_reg;
                    m_axi_wstrb     <= STRB_ALL;
                    m_axi_wlast     <= (beats_in_burst == cur_burst_len - 8'd1) ? 1'b1 : 1'b0;

                    if (m_axi_wvalid && m_axi_wready) begin
                        m_axi_wvalid    <= 1'b0;
                        m_axi_wlast     <= 1'b0;
                        beats_in_burst  <= beats_in_burst + 8'd1;
                        beats_remaining <= beats_remaining - 16'd1;
                        pack_reg        <= {AXI_DATA_W{1'b0}};

                        if (beats_in_burst == cur_burst_len - 8'd1) begin
                            // 当前 burst 最后一拍已发送 → 等 B 响应
                            state <= S_B_WAIT;
                        end else begin
                            // 继续读下一组像素
                            state <= S_PFB_RD_REQ;
                        end
                    end
                end

                // ============================================================
                // S_B_WAIT: 等待 AXI4 写响应
                //   - bready=1 等待 bvalid
                //   - 握手成功后: 如果还有数据 → 下一个 burst; 否则 → 完成
                // ============================================================
                S_B_WAIT: begin
                    m_axi_bready <= 1'b1;

                    if (m_axi_bvalid && m_axi_bready) begin
                        m_axi_bready <= 1'b0;

                        if (beats_remaining == 16'd0)
                            state <= S_DONE;
                        else begin
                            beats_in_burst <= 8'd0;
                            state          <= S_AW_ISSUE;
                        end
                    end
                end

                // ============================================================
                // S_DONE: 传输完成
                // ============================================================
                S_DONE: begin
                    dma_rd_req      <= 1'b0;
                    m_axi_wvalid    <= 1'b0;
                    m_axi_bready    <= 1'b0;
                    done            <= 1'b1;
                    busy            <= 1'b0;
                    state           <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
