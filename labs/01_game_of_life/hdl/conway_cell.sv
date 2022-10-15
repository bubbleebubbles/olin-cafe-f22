`default_nettype none
`timescale 1ns/1ps

module conway_cell(clk, rst, ena, state_0, state_d, state_q, neighbors);
input wire clk;
input wire rst;
input wire ena;

input wire state_0;
output logic state_d; // NOTE - this is only an output of the module for debugging purposes. 
output logic state_q;

input wire [7:0] neighbors;

//adder here -- should output sum that is a 4 bit number
//decoder_3_to_8(en, in, out)
//
parameter neighbors[7:0] = 0; 
wire [2:0] adder_output;
assign adder_output[0] = 0; 
assign adder_output[1] = 0; 
assign adder_output[2] = 0; 
wire [7:0] neighbors_alive;
decoder_3_to_8 COUNT(1, adder_output[2:0], neighbors_alive);

wire en1; 
wire en2; 

wire data1; 
wire data2; 
wire data3; 
wire select1; 
wire select2; 

assign select1 = neighbors_alive[0]|neighbors_alive[2]|neighbors_alive[3]|neighbors_alive[4]| neighbors_alive[5]|neighbors_alive[6]|neighbors_alive[7]; 
assign data1 = ~select1; 
assign en1 = (select1 ? 1'b1:1'b0); 
assign data3 = neighbors_alive[2]; 
assign select2 = neighbors_alive[1]; 
assign en2 = (select2 ? 1'b0:1'b1); 

wire en_cell; 
assign en_cell = en1^en2; 
assign rst = en_cell; 

assign state_d = (select1?(~data1):data3); 

//flip flop 


endmodule
