`timescale 1ns/1ps
module decoder_2_to_4(ena, in, out);

input wire ena;
input wire [1:0] in;
output logic [3:0] out;

always_comb begin
    out[3] = ena & in[0] & in[1];
    out[2] = ena & ~in[0] & in[1];
    out[1] = ena & in[0] & ~in[1];
    out[0] = ena & ~in[0] & ~in[1];
end
endmodule