module adder_parallel_3(a,b,c_in,sum,c_out);
input wire [2:0] a,b;
input wire c_in;

output wire [2:0] sum;
output wire c_out;
wire [1:0] c_intermediate;

adder_1 ab1(.a(a[0]),.b(b[0]),.c_in(c_in),.sum(sum[0]),.c_out(c_intermediate[0]));
adder_1 ab2(.a(a[1]),.b(b[1]),.c_in(c_intermediate[0]),.sum(sum[1]),.c_out(c_intermediate[1]));
adder_1 ab3(.a(a[2]),.b(b[2]),.c_in(c_intermediate[1]),.sum(sum[2]),.c_out(c_out));

endmodule