`timescale 1ns/1ps
module decoder_3_to_8(ena, in, out);

  input wire ena;
  input wire [2:0] in;
  output logic [7:0] out;

    
    assign out[0] = ena&~in[0]&~in[1]&~in[2];
    assign out[1] = ena&in[0]&~in[1]&~in[2];
    assign out[2] = ena&~in[0]&in[1]&~in[2];
    assign out[3] = ena&in[0]&in[1]&~in[2]; 

    assign out[4] = ena&~in[0]&~in[1]&in[2]; 
    assign out[5] = ena&in[0]&~in[1]&in[2];
    assign out[6] = ena&~in[0]&in[1]&in[2];
    assign out[7] = ena&in[0]&in[1]&in[2]; 



endmodule
