`timescale 1ns/1ps
`default_nettype none

`include "alu_types.sv"
`include "rv32i_defines.sv"

module rv32i_multicycle_core(
  clk, rst, ena,
  mem_addr, mem_rd_data, mem_wr_data, mem_wr_ena,
  PC
);

parameter PC_START_ADDRESS=0;

// Standard control signals.
input  wire clk, rst, ena; // <- worry about implementing the ena signal last.

// Memory interface.
output logic [31:0] mem_addr, mem_wr_data;
input   wire [31:0] mem_rd_data;
output logic mem_wr_ena;

// Program Counter
output wire [31:0] PC;
wire [31:0] PC_old;
logic PC_ena;
logic [31:0] PC_next; 

// Program Counter Registers
register #(.N(32), .RESET(PC_START_ADDRESS)) PC_REGISTER (
  .clk(clk), .rst(rst), .ena(PC_ena), .d(PC_next), .q(PC)
);
register #(.N(32)) PC_OLD_REGISTER(
  .clk(clk), .rst(rst), .ena(PC_ena), .d(PC), .q(PC_old)
);

//  an example of how to make named inputs for a mux:
/*
    enum logic {MEM_SRC_PC, MEM_SRC_RESULT} mem_src;
    always_comb begin : memory_read_address_mux
      case(mem_src)
        MEM_SRC_RESULT : mem_rd_addr = alu_result;
        MEM_SRC_PC : mem_rd_addr = PC;
        default: mem_rd_addr = 0;
    end
*/

// Register file
logic reg_write;
logic [4:0] rd, rs1, rs2;
logic [31:0] rfile_wr_data;
wire [31:0] reg_data1, reg_data2;
register_file REGISTER_FILE(
  .clk(clk), 
  .wr_ena(reg_write), .wr_addr(rd), .wr_data(rfile_wr_data),
  .rd_addr0(rs1), .rd_addr1(rs2),
  .rd_data0(reg_data1), .rd_data1(reg_data2)
);

// ALU and related control signals
// Feel free to replace with your ALU from the homework.
logic [31:0] src_a, src_b;
alu_control_t alu_control;
wire [31:0] alu_result;
wire overflow, zero, equal;
alu_behavioural ALU (
  .a(src_a), .b(src_b), .result(alu_result),
  .control(alu_control),
  .overflow(overflow), .zero(zero), .equal(equal)
);

// Implement your multicycle rv32i CPU here!

// ALU Muxes
always_comb begin: ALU_MUX_SRC_A
  case (alu_src_a)
    default: src_a=0;
    ALU_SRC_A_PC: src_a = PC;
    ALU_SRC_A_OLD_PC: src_a = PC_old;
    ALU_SRC_A_RF: src_a = reg_A;
  endcase
end

always_comb begin: ALU_MUX_SRC_B
  case (alu_src_b)
    default: src_b=0;
    ALU_SRC_B_IMM: src_b = extended_immediate;
    ALU_SRC_B_4: src_b = 32'd4;
    ALU_SRC_B_RF: src_b = reg_B;
  endcase
end

// Instructions


// Decoding instructions
logic [6:0] op;
logic r_type, i_type, l_type, s_type, b_type, j_type;
enum logic [1:0] {IMM_SRC_I_TYPE, IMM_SRC_S_TYPE, IMM_SRC_B_TYPE, IMM_SRC_J_TYPE} immediate_src;
logic [31:0] extended_immediate; 

// Op type comparators
r_type = (op==OP_R_TYPE);
i_type = (op==OP_I_TYPE);
l_type = (op==OP_L_TYPE);
s_type = (op==OP_S_TYPE);
b_type = (op==OP_B_TYPE);
j_type = (op==OP_JAL) | (op==OP_JALR);

// Decoding immediate
case(op)
  default: immediate_src = IMM_SRC_S_TYPE; // anything can go here, all imediate op codes are implemented here already
  OP_I_TYPE: immediate_src = IMM_SRC_I_TYPE;
  OP_S_TYPE: immediate_src = IMM_SRC_S_TYPE;
  OP_B_TYPE:  immediate_src = IMM_SRC_B_TYPE;
  OP_JAL, OP_JALR:  immediate_src = IMM_SRC_J_TYPE;

// Main FSM

typedef enum [3:0] {S_FETCH, S_DECODE, S_MEMADR, S_EXECUTER, S_EXECUTEI, S_JUMP, S_BRANCH, S_MEMREAD, S_ALUWB, S_MEMREAD, S_MEMWRITE, S_MEMWB, S_ERROR=4'hF } statetype;

statetype state, next_state;
logic [14:0] controls;

// State register
always @(posedge clk or posedge rst) begin : FSM_MULTICYCLE
  if (rst) state <= S_FETCH;
  else state <= next_state;

// Logic for next state
always_comb: begin
  case(state)
    default: next_state = S_ERROR;
    S_FETCH: next_state = S_DECODE;
    S_DECODE: begin
      case(op)
        default: begin
          $display("Error - op %b not implemented", op);
          next_state = S_ERROR;
        end
        OP_R_TYPE: next_state = S_EXECUTER; // R-type
        OP_I_TYPE: next_state = S_EXECUTEI; // I-type ALU
        OP_S_TYPE, OP_L_TYPE, OP_LUI: next_state = S_MEMADR; // lw or sw
        OP_JAL: next_state = S_JUMP;
        OP_JALR: next_state = S_BRANCH;
      endcase
    end
    S_MEMADR: begin
      case(op)
        default: next_state = S_ERROR;
        OP_L_TYPE, OP_LUI: next_state = S_MEMREAD;
        OP_S_TYPE: next_state = S_MEMWRITE;
      endcase
    end
    S_EXECUTEI: next_state = S_ALUWB;
    S_EXECUTER: next_state = S_ALUWB;
    S_ALUWB, S_MEMREAD, S_MEMWRITE: next_state = S_FETCH
    S_JUMP: begin
      $display("Please implement jumps")
      next_state = S_ERROR;
    end
    S_BRANCH: begin
      $display("Please implement branches")
      next_state = S_ERROR;
    end
end

endmodule
