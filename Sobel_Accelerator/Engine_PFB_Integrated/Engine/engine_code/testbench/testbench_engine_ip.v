`timescale 1ns / 1ps

module tb_basic_ip #(
    parameter WIDTH = 12
) ();
    // 1. Signed Adder signal
    reg  signed [WIDTH-1:0] add_a, add_b;
    wire signed [WIDTH-1:0] add_sum;

    // 2. Signed Subtractor signal
    reg  signed [WIDTH-1:0] sub_a, sub_b;
    wire signed [WIDTH-1:0] sub_res;

    // 3. Left Shifter signal
    reg  signed [WIDTH-1:0] shift_in;
    wire signed [WIDTH-1:0] shift_out;

    // 4. Absolute Value signal
    reg  signed [WIDTH-1:0] abs_in;
    wire        [WIDTH-1:0] abs_out;

    // 5. Comparator signal
    reg  [WIDTH-1:0] com_a, com_b;
    wire             com_out;

    // 6. Clip signal
    reg  [WIDTH-1:0] clip_in;
    wire [7:0]       clip_out;

    // Instantiate all modules
    
    // 1. Adder [cite: 3, 4]
    signed_add_ip #(.WIDTH(WIDTH)) dut_add (
        .a(add_a), .b(add_b), .sum(add_sum)
    );

    // 2. Subtractor [cite: 5]
    signed_sub_ip #(.WIDTH(WIDTH)) dut_sub (
        .a(sub_a), .b(sub_b), .sub(sub_res)
    );

    // 3. Left Shifter (x2) 
    left_shift_ip #(.WIDTH(WIDTH)) dut_shift (
        .a(shift_in), .shifted(shift_out)
    );

    // 4. Absolute Value [cite: 7, 8]
    abs_ip #(.WIDTH(WIDTH)) dut_abs (
        .a(abs_in), .abs(abs_out)
    );

    // 5. Comparator [cite: 9]
    com_ip #(.WIDTH(WIDTH)) dut_com (
        .a(com_a), .b(com_b), .com(com_out)
    );

    // 6. Clip 
    clip_ip #(.WIDTH(WIDTH)) dut_clip (
        .a(clip_in), .clip(clip_out)
    );

    // Test Logic
    initial begin
        $display("===== Start Sobel Basic IP Testing =====");

        // Test 1: Signed Add
        add_a = 12'sd100; add_b = 12'sd200; #10;
        $display("[ADD] %d + %d = %d", add_a, add_b, add_sum);
        add_a = -12'sd50; add_b = 12'sd10;  #10;
        $display("[ADD] %d + %d = %d", add_a, add_b, add_sum);

        // Test 2: Signed Sub
        sub_a = 12'sd500; sub_b = 12'sd100; #10;
        $display("[SUB] %d - %d = %d", sub_a, sub_b, sub_res);
        sub_a = 12'sd10;  sub_b = 12'sd20;  #10;
        $display("[SUB] %d - %d = %d", sub_a, sub_b, sub_res);

        // Test 3: Left Shift
        shift_in = 12'sd15; #10;
        $display("[SHIFT] %d << 1 = %d", shift_in, shift_out);
        shift_in = -12'sd30; #10;
        $display("[SHIFT] %d << 1 = %d", shift_in, shift_out);

        // Test 4: Absolute Value
        abs_in = -12'sd42; #10;
        $display("[ABS] |%d| = %d", abs_in, abs_out);
        abs_in = 12'sd100; #10;
        $display("[ABS] |%d| = %d", abs_in, abs_out);

        // Test 5: Comparator
        com_a = 12'd300; com_b = 12'd255; #10;
        $display("[COM] %d > %d? Result: %b", com_a, com_b, com_out);
        com_a = 12'd100; com_b = 12'd200; #10;
        $display("[COM] %d > %d? Result: %b", com_a, com_b, com_out);

        // Test 6: Clip Output (0-255)
        clip_in = 12'd500; #10; // overflow condition
        $display("[CLIP] Input: %d, Output: %d (Expected: 255)", clip_in, clip_out);
        clip_in = 12'd128; #10; // within the normal range
        $display("[CLIP] Input: %d, Output: %d (Expected: 128)", clip_in, clip_out);

        $display("===== Testing completed =====");
        $finish;
    end

endmodule