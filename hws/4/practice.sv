`timescale 1ns/1ps
`default_nettype none

module practice(rst, clk, ena, seed, out);
    input wire rst;
    input wire clk;
    input wire ena;
    input wire seed;
    output logic out;

    logic XOR_out, d, data0, q0, data1, q1, data2;

    mux2 MUX0 
        (.in0(seed),
        .in1(XOR_out),
        .select(ena),
        .out(d));

	always_comb begin
        XOR_out = data1 ^ data2;
        data0 = ~rst ? d : 1'b0;
        data1 = ~rst ? q0 : 1'b0;
        data2 = ~rst ? q1 : 1'b0;
    end

    always_ff @(posedge clk) begin
        q0 <= data0;
        q1 <= data1;
        out <= data2;
    end
endmodule
