`timescale 1ns / 1ps

// Private Frame Buffer Address Mapper
// Author: Junze Jiang
// 1/3/2026
/*  Functions:
    Split a global linear address global_addr into: 
        bank_sel: Selects which RAM bank (which small RAM block) 
        local_addr: The address within this bank (bank-internal index)
    
    Interleave contiguous pixel addresses across multiple banks to enhance parallel access capability 
    particularly when both the engine and DMA ports access simultaneously
*/

// global_addr[1:0] -> bank select
// global_addr[ADDR_W-1:2] -> local row address inside bank
/*
    global_addr     bank_sel(low 2 bits)        local_addr(high 12 bits)
    0   0000            00  bank0                   00  0
    1   0001            01  bank1                   00  0
    2   0010            10  bank2                   00  0
    3   0011            11  bank3                   00  0
    4   0100            00  bank0                   01  1
    5   0101            01  bank1                   01  1
    6   0110            10  bank2                   01  1
    7   0111            11  bank3                   01  1

    Consecutive pixels are written to four banks in rotation: 0→1→2→3→0→1→…
*/

// bank_sel: The width is BANK_BITS. 2 bits can represent banks 0 bank2 bank3, 4 banks totally
// local_addr: ADDR_W(14 bits) - BANK_BITS(2 bits) = 12bits -> Each bank contains 2^12 = 4096 addresses
// 4 banks = 4 * 4096 = 16384 pixels

module pfb_addr_map #(
    parameter ADDR_W    = 14,
    parameter BANK_BITS = 2
) (
    input  wire [ADDR_W - 1             :0] global_addr,
    output wire [BANK_BITS - 1          :0] bank_sel,
    output wire [ADDR_W - BANK_BITS - 1 :0] local_addr
);
    assign bank_sel   = global_addr[BANK_BITS - 1   :0          ];
    assign local_addr = global_addr[ADDR_W - 1      :BANK_BITS  ];
endmodule