# 100 MHz clock (10 ns period) — matches testbench assumption
# "clk" is the port name in enigne_pfb_top
create_clock -period 10.000 -name sys_clk [get_ports clk]
