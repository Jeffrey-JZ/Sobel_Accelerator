`timescale 1ns / 1ps

// The Bank Ram and the Controller for Private Frame Buffer
// Author: Junze Jiang
// 1/3/2026
// Port A: mapped to DMA side in top module
// Port B: mapped to Engine side in top module
// Read latency: 1 cycle

module ptf_bank_ram #(
    parameter   DATA_W = 8,
    parameter   ADDR_W = 12
) (
    input   wire    clk,
    input   wire    rst_n,

    // Port A: DMA Read / Write
    input   wire                    a_en,       // Does this take initiate a visit (request)
    input   wire                    a_we,       // 1=write, 0=read
    input   wire    [ADDR_W - 1:0]  a_addr,     // Access address (local address within the bank)
    input   wire    [DATA_W - 1:0]  a_wdata,    // Write data
    output  reg     [DATA_W - 1:0]  a_rdata,    // Read out data (register output)
    output  reg                     a_rvalid,   // Read data validity flag

    // Port B: Engine Read / Write
    input   wire                    b_en,
    input   wire                    b_we,
    input   wire    [ADDR_W - 1:0]  b_addr,
    input   wire    [DATA_W - 1:0]  b_wdata,
    output  reg     [DATA_W - 1:0]  b_rdata,
    output  reg                     b_rvalid
);
    localparam  DEPTH = (1 << ADDR_W);  // ADDR_W = 12 DEPTH = (1 << 12) = 4096

    // Memory, regs array
    reg [DATA_W - 1:0]  mem [0:DEPTH - 1];  // 4096 8-bit reg

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_rdata     <= {DATA_W{1'b0}};
            a_rvalid    <= 1'b0;

            b_rdata     <= {DATA_W{1'b0}};
            b_rvalid    <= 1'b0;

            for (i = 0; i < DEPTH; i = i + 1) begin
                mem[i]  <= {DATA_W{1'b0}};
            end
        end else begin
            a_rvalid    <= 1'b0;
            b_rvalid   <= 1'b0;

            if (a_en) begin
                if (a_we) begin
                    mem[a_addr] <= a_wdata;
                end else begin
                    a_rdata     <= mem[a_addr];
                    a_rvalid    <= 1'b1;
                end
            end

            if (b_en) begin
                if (b_we) begin
                    mem[b_addr] <= b_wdata;
                end else begin
                    b_rdata     <= mem[b_addr];
                    b_rvalid    <= 1'b1;
                end
            end

        end
    end
endmodule
