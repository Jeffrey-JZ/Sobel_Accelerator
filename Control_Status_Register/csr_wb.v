`timescale 1ns / 1ps

// Wishbone CSR block for Sobel DMA control/state registers
// Register map (word offset):
// 0x00 CTRL   [0]=start (W1P)
// 0x04 SRC_BASE_ADDR
// 0x08 DST_BASE_ADDR
// 0x0C PIXEL_COUNT
// 0x10 STATUS [0]=busy [1]=done [2]=error
// 0x14 THRESHOLD [7:0] =sobel threshold

module csr_wb #(
    parameter   WB_ADDR_W       = 8,
    parameter   WB_DATA_W       = 32,
    parameter   AXI_ADDR_W      = 32,
    parameter   PIXELS_MAX_W    = 16
) (
    // Wishbone bus interface signals
    // Standard Wishbone B4 slave device interface signals
    input   wire                            wb_clk_i,
    input   wire                            wb_rst_i,
    input   wire    [WB_ADDR_W       - 1:0] wb_adr_i,   // CPU 要访问哪个寄存器地址
    input   wire    [WB_DATA_W       - 1:0] wb_dat_i,   // CPU 写进来的数据
    output  reg     [WB_DATA_W       - 1:0] wb_dat_o,   // CSR 返回给 CPU 的读数据
    input   wire    [(WB_DATA_W / 8) - 1:0] wb_sel_i,   // 字节选择信号: 32位数据分4字节, 每一位控制一个字节是否有效(32/8 = 4位)
    input   wire                            wb_we_i,    // Write enable: 1 = 写, 0 = 读
    input   wire                            wb_stb_i,   // 选通信号: 1 = 当前从设备被选中
    input   wire                            wb_cyc_i,   // 总线周期信号: 1 = 一次总线事务正在进行
    output  reg                             wb_ack_o,   // 应答输出: 从设备告诉主设备这次读/写处理好了

    // DMA core interface signals
    output  reg                             dma_start_pulse,    // 单周期启动脉冲, 告诉 DMA 开始工作
    output  reg     [AXI_ADDR_W   - 1:0]    dma_src_base_addr,  // DMA 读数据的起始地址
    output  reg     [AXI_ADDR_W   - 1:0]    dma_dst_base_addr,  // DMA 写数据的目标地址
    output  reg     [PIXELS_MAX_W - 1:0]    dma_pixel_count,    // 要处理的像素数量
    input   wire                            dma_busy,
    input   wire                            dma_done,
    input   wire                            dma_error,

    output  reg     [7:0]                   threshold
);
    localparam [3:0] REG_CTRL   = 4'h0;     // 控制寄存器, 其中 bit[0] 是 start     往 bit[0] 写 1 会产生一个单周期脉冲 dma_start_pulse, 用来启动 DMA
    localparam [3:0] REG_SRC    = 4'h1;     // DMA 读源地址     DMA 从main memory的哪里开始读原始图像
    localparam [3:0] REG_DST    = 4'h2;     // DMA 写目标地址   DMA 最后把处理后的图像写main memory的哪里
    localparam [3:0] REG_COUNT  = 4'h3;     // 要处理的像素总数 128×128 = 16384
    localparam [3:0] REG_STATUS = 4'h4;     // 状态寄存器: bit[0] = busy bit[1] = done bit[2] = error  CPU 通过读这个寄存器来知道 DMA 当前情况
    localparam [3:0] REG_THRESH = 4'h5;     // Threshold 寄存器: 在软件里设置的是 60, 用于engine的计算结果的比较

    wire wb_req;            // 当前是否有有效 Wishbone 请求
    wire [3:0] reg_idx;     // 用地址的 [5:2] 位来选择是哪个 32-bit 寄存器
    assign wb_req  = wb_cyc_i & wb_stb_i;   // 只有当 wb_cyc_i = 1 wb_stb_i = 1  时, 才认为 CPU 真正在访问这个从设备
    assign reg_idx = wb_adr_i[5:2];

    reg done_latched;
    reg err_latched;

    always @(posedge wb_clk_i or posedge wb_rst_i) begin
        if (wb_rst_i) begin
            wb_ack_o            <= 1'b0;
            wb_dat_o            <= {WB_DATA_W   {1'b0}};
            dma_start_pulse     <= 1'b0;
            dma_src_base_addr   <= {AXI_ADDR_W  {1'b0}};
            dma_dst_base_addr   <= {AXI_ADDR_W  {1'b0}};
            dma_pixel_count     <= {PIXELS_MAX_W{1'b0}};
            threshold           <= 8'd0;
            done_latched        <= 1'b0;
            err_latched         <= 1'b0;
        end else begin
            wb_ack_o            <= 1'b0;
            dma_start_pulse     <= 1'b0;

            if (dma_done)
                done_latched    <= 1'b1;
            if (dma_error)
                err_latched     <= 1'b1;

            if (wb_req && !wb_ack_o) begin  // 当检测到有效总线请求(wb_req)且当前没有正在响应(!wb_ack_o)时, 拉高 ACK
                wb_ack_o <= 1'b1;

                if (wb_we_i) begin  // Write
                    case (reg_idx)

                        // 检查 wb_sel_i[0](字节0使能)后: 
                        // bit[0]=1 产生启动脉冲; bit[1]=1 清除 done 锁存; bit[2]=1 清除 error 锁存
                        // 注意这三个动作可以同时发生(CTRL = 0x7 bit[2:0] = 111)
                        REG_CTRL: begin
                            // // bit[0]：启动 DMA      C 语言：CTRL = 0x1
                            if (wb_sel_i[0] && wb_dat_i[0])     // 最低字节被使能 + CPU 写入数据 bit[0]=1 => 拉高 dma_start_pulse 一个周期
                                dma_start_pulse <= 1'b1;
                            // bit[1]：清 done 锁存     C 语言：CTRL = 0x2
                            if (wb_sel_i[0] && wb_dat_i[1])
                                done_latched    <= 1'b0;
                            // bit[2]：清 error 锁存    C 语言：CTRL = 0x4
                            if (wb_sel_i[0] && wb_dat_i[2])
                                err_latched     <= 1'b0;
                        end
                        REG_SRC: begin
                            dma_src_base_addr   <= wb_dat_i[AXI_ADDR_W - 1:0];
                        end
                        REG_DST: begin
                            dma_dst_base_addr   <= wb_dat_i[AXI_ADDR_W - 1:0];
                        end
                        REG_COUNT: begin
                            dma_pixel_count     <= wb_dat_i[PIXELS_MAX_W - 1:0];
                        end
                        REG_THRESH: begin
                            threshold <= wb_dat_i[7:0];
                        end
                        default: ;
                    endcase

                end else begin  // wb_we_i = 0 => Read
                    case (reg_idx)
                        REG_CTRL: begin
                            wb_dat_o <= {WB_DATA_W{1'b0}};  // CTRL 被读出来永远是 0
                        end
                        REG_SRC: begin
                            wb_dat_o <= dma_src_base_addr;  // 返回当前保存的源地址
                        end
                        REG_DST: begin
                            wb_dat_o <= dma_dst_base_addr;  // 返回当前保存的目标地址
                        end
                        REG_COUNT: begin
                            wb_dat_o <= {{(WB_DATA_W - PIXELS_MAX_W){1'b0}}, dma_pixel_count};  // dma_pixel_count 可能只有 16 位, 所以高位补 0, 拼成 32 位返回给 CPU
                        end
                        /*
                        bit[31:3] = 0,  bit[2] = dma_error | err_latched,                               bit[1] = dma_done | done_latched,                           bit[0] = dma_busy
                        
                                        error: 只要当前 error=1, 或者以前 error 过并被锁存了, 就返回 1     done: 只要当前 done=1, 或者以前 done 过并被锁存了, 就返回 1    busy: 看 DMA 当前是否忙
                        */
                        REG_STATUS: begin
                            wb_dat_o <= {29'd0, (dma_error | err_latched), (dma_done | done_latched), dma_busy};
                        end
                        REG_THRESH: begin
                            wb_dat_o <= {24'd0, threshold};
                        end
                        default: begin
                            wb_dat_o <= {WB_DATA_W{1'b0}};
                        end
                    endcase
                end
            end
        end
    end
endmodule
