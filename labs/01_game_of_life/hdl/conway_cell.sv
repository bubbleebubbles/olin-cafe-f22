`default_nettype none
`timescale 1ns/1ps

module conway_cell(clk, rst, ena, state_0, state_d, state_q, neighbors);
input wire clk;
input wire rst;
input wire ena;

input wire state_0;
output logic state_d; // NOTE - this is only an output of the module for debugging purposes. 
output logic state_q;
logic next_state; 

//logic live_neighbors_3;
//logic continue_alive;
//logic enabled_output;
//logic reset_output;

input wire [7:0] neighbors;
logic [3:0] live_neighbors;
wire [1:0] n12;
wire [1:0] n34;
wire [1:0] n56;
wire [1:0] n78;
wire [2:0] n1234;
wire [2:0] n5678;

// Sum neighbors into 4 2-bit numbers using 1-bit full adders
adder_1 SUM_N12(.a(neighbors[0]),.b(neighbors[1]),.c_in(1'b0),.sum(n12[0]),.c_out(n12[1]));
adder_1 SUM_N34(.a(neighbors[2]),.b(neighbors[3]),.c_in(1'b0),.sum(n34[0]),.c_out(n34[1]));
adder_1 SUM_N56(.a(neighbors[4]),.b(neighbors[5]),.c_in(1'b0),.sum(n56[0]),.c_out(n56[1]));
adder_1 SUM_N78(.a(neighbors[6]),.b(neighbors[7]),.c_in(1'b0),.sum(n78[0]),.c_out(n78[1]));

// Sum neighbors into 2 4-bit numbers using 2-bit parallel adders
adder_parallel_2 SUM_N1234(.a(n12),.b(n34),.c_in(1'b0),.sum(n1234[1:0]),.c_out(n1234[2]));
adder_parallel_2 SUM_N5678(.a(n56),.b(n78),.c_in(1'b0),.sum(n5678[1:0]),.c_out(n5678[2]));

// Sum neighbors using a 3-bit parallel adder to find the total number of living neighbors
adder_parallel_3 SUM_N(.a(n1234),.b(n5678),.c_in(1'b0),.sum(live_neighbors[2:0]),.c_out(live_neighbors[3]));

always_comb begin: INPUT_LOGIC
    next_state = (~live_neighbors[3]&~live_neighbors[2]&live_neighbors[1])& ((state_q & ~live_neighbors[0])|live_neighbors[0]); 
    state_d = ~rst ? (ena ? next_state : state_q) : state_0;
    /*
    live_neighbors_3 =(~live_neighbors[3]&~live_neighbors[2]&live_neighbors[1]);
    continue_alive=state_q&~live_neighbors[0];
    state_d = ~rst ? (ena ? (live_neighbors_3 | continue_alive): state_q) : state_0;
    */ 
end
/*
always_comb begin
    enabled_output=(ena)?state_d:state_q;
    reset_output=(rst)?state_0:enabled_output;
end
*/

always_ff @(posedge clk) begin: D_FLIP_FLOP
    state_q <= state_d;
end


endmodule
