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

//Non-architectural register
wire [31:0] reg_A, reg_B; 

register #(.N(32), REGISTER_A (
  .clk(clk), .rst(rst), .ena(1'b1), .d(reg_data1), .q(reg_A)
);

register #(.N(32), REGISTER_B (
  .clk(clk), .rst(rst), .ena(1'b1), .d(reg_data1), .q(reg_B)
);


always_comb mem_wr_data = reg_B; 

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

//Mux A 
enum logic [1:0] {ALU_SRC_A_PC, ALU_SRC_A_PC_OLD, ALU_SRC_A_RA} alu_src_a; 
always_comb begin: ALU_MUX_A
    case (alu_src_a)
        ALU_SRC_A_PC: src_a = PC; 
        ALU_SRC_A_RA: src_a = reg_A; 
        ALU_SRC_A_PC_OLD: src_a = PC_old; 
        default: src_a = 0; 
    endcase
end




//Mux B
enum logic [1:0] {ALU_SRC_B_4, ALU_SRC_B_IMM_EXT, ALU_SRC_B_RB} alu_src_b; 
always_comb begin: ALU_MUX_B
    case (alu_src_b)
        ALU_SRC_B_RB: src_b = reg_B; 
        ALU_SRC_B_IMM_EXT: src_b = immidiate_extended ; 
        ALU_SRC_B_4: src_b = 32'd4; 
        default: src_a = 0; 
    endcase
end

//Non-architectural Register
//Memory Register
logic mem_data_ena; 
wire [31:0] mem_data;
register #(.N(32)) MEM_DATA_REGISTER(
    .clk(clk), .rst(rst), .ena(mem_data__ena), .d(mem_rd_data), .q(mem_data)
); 


//Memory Mux 
enum logic [1:0] {MEM_ADR_SRC_PC, MEM_ADR_SRC_ALU_RESULT} mem_adr_src
always_comb begin: MEMORY_ADR_MUX
    case(mem_adr_src)
        MEM_ADR_SRC_PC: mem_adr_src = PC; 
        MEM_ADR_SRC_ALU_RESULT: mem_adr_src = result;//ALU Result mux result 
    endcase
end

//Non-architectural register 
//ALU Result Register
logic ALU_ena; 
wire [31:0] alu_last;
register #(.N(32)) ALU_REGISTER(
    .clk(clk), .rst(rst), .ena(alu_ena), .d(alu_result), .q(alu_last)
); 

//Output Mux 
enum logic [1:0] {RESULT_SRC_ALU, RESULT_SRC_MEM_DATA, RESULT_SRC_ALU_LAST} result_src; 
always_comb begin: ALU_RESULT_MUX
    case(result_src)
        RESULT_SRC_ALU: result = alu_result; 
        RESULT_SRC_MEM_DATA: result = mem_data; 
        RESULT_SRC_ALU_LAST: result = alu_last; 
        default: result = alu_result; 
    endcase
end 

always_comb begin: RESULT_ALIASES //for things that are connected/same value
    PC_next = result; 
    rfile_wr_data = result; 
end

// Main FSM

typedef enum [3:0] {S_FETCH, S_DECODE, S_MEMADR, S_EXECUTER, S_EXECUTEI, S_JUMP, S_BRANCH, S_MEMREAD, S_ALUWB, S_MEMREAD, S_MEMWRITE, S_MEMWB, S_BEQ, S_JAL, S_ERROR=4'hF } statetype;

statetype state, next_state;
logic [14:0] controls;

// State register
always @(posedge clk or posedge rst) begin : FSM_MULTICYCLE
  if (rst) state <= S_FETCH;
  else state <= next_state;

// Logic for next state
always_comb: begin
  case(state)
    default: next_state = S_FETCH;
    S_FETCH: next_state = S_DECODE;
    S_DECODE: begin
      casez(op) 
      // casez is a "wildcard" case expression that allows "Z" and "?" to be treated 
      // as don't care values in either the case expression and/or the case item when 
      // doing a comparison
        default: begin
          $display("Error - op %b not implemented", op);
          next_state = S_ERROR;
        end
        /*
        OP_R_TYPE: next_state = S_EXECUTER; // R-type
        OP_I_TYPE: next_state = S_EXECUTEI; // I-type ALU
        OP_S_TYPE, OP_L_TYPE, OP_LUI: next_state = S_MEMADR; // lw or sw
        OP_JAL: next_state = S_JUMP;
        OP_JALR: next_state = S_BRANCH;
        */
        7'b0?00011: next_state = S_MEMADR; // lw or sw
        7'b0110011: next_state = S_EXECUTER; // R-type
        7'b0010011: next_state = S_EXECUTEI; // I-type
        7'b1101111: next_state = S_JAL; // jal
        7'b1100011: next_state = S_BEQ; // beq
      endcase
    end
    S_MEMADR: begin
      case(op)
        default: next_state = S_ERROR;
        /*
        OP_L_TYPE, OP_LUI: next_state = S_MEMREAD;
        OP_S_TYPE: next_state = S_MEMWRITE;
        */
        if (op[5]) next_state = S_MEMWRITE; // sw
        else next_state = S_MEMREAD; // lw
      endcase
    end
    S_MEMREAD: next_state = S_MEMWB;
    S_EXECUTEI, S_EXECUTER, S_JAL: next_state = S_ALUWB;
    S_ALUWB, S_MEMREAD, S_MEMWRITE: next_state = S_FETCH;
    S_JUMP: begin
      $display("Please implement jumps")
      next_state = S_ERROR;
    end
    S_BRANCH: begin
      $display("Please implement branches")
      next_state = S_ERROR;
    end
end


// state output logic
always_comb: begin
  case(state)
    // AdrSrc_IRWrite_ALUSrcA_ALUSrcB_ALUOp_ResultSrc_PCUpdate_RegWrite_MemWrite_Branch
    S_FETCH: controls=15'b0_1_00_10_00_10_1_0_0_0;
    S_DECODE: controls=15'b0_0_01_01_00_00_0_0_0_0;
    S_MEMADR: controls=15'b0_0_10_01_00_00_0_0_0_0;
    S_MEMREAD: controls=15'b1_0_00_00_00_00_0_0_0_0;
    S_MEMWRITE: controls=15'b1_0_00_00_00_00_0_0_1_0;
    S_MEMWB: controls=15'b0_0_00_00_00_01_0_1_0_0;
    S_EXECUTER: controls=15'b0_0_10_00_10_00_0_0_0_0;
    S_EXECUTEI: controls=15'b0_0_10_01_10_00_0_0_0_0;
    S_ALUWB: controls=15'b0_0_00_00_00_00_1_0_0_0;
    S_JAL: controls=15'b0_0_10_00_01_00_0_0_0_1;
    S_BEQ: controls=15'b0_0_01_10_00_00_1_0_0_0;
    default: controls=15'bx_x_xx_xx_xx_xx_x_x_x_x;
  endcase
end

assign {AdrSrc, IRWrite, ALUSrcA, ALUSrcB, ALUOp, ResultSrc, PCUpdate, RegWrite, MemWrite, Branch} = controls;

        // add
        // sub
        // xor
        // or
        // and
        // sll
        // srl
        // sra
        // slt
        // sltu
        // addi
        // xori
        // ori
        // andi
        // slli
        // srli
        // srai
        // slti
        // sltiu
        // jalr
        // bne


endmodule
