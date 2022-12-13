`timescale 1ns/1ps
`default_nettype none

`include "alu_types.sv"
`include "rv32i_defines.sv"

module rv32i_multicycle_core(
  clk, rst, ena,
  mem_addr, mem_rd_data, mem_wr_data, mem_wr_ena,
  PC, instructions_completed
);

parameter PC_START_ADDRESS=0;

// Standard control signals.
input  wire clk, rst, ena; // <- worry about implementing the ena signal last.

// Memory interface.
output logic [31:0] mem_addr, mem_wr_data;
input   wire [31:0] mem_rd_data;
output logic mem_wr_ena;
input logic [31:0] instructions_completed;

// Program Counter
output wire [31:0] PC;
wire [31:0] PC_old;
logic PC_ena, ir_write;
logic [31:0] PC_next; 

// Program Counter Registers
register #(.N(32), .RESET(PC_START_ADDRESS)) PC_REGISTER (
  .clk(clk), .rst(rst), .ena(PC_ena), .d(PC_next), .q(PC)
);
register #(.N(32)) PC_OLD_REGISTER(
  .clk(clk), .rst(rst), .ena(ir_write), .d(PC), .q(PC_old)
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
enum logic [3:0] {S_FETCH, S_DECODE, S_MEMADR, S_EXECUTER, S_EXECUTEI, S_JUMP, S_BRANCH, S_ALUWB, S_MEMREAD, S_MEMWRITE, S_MEMWB, S_BEQ, S_JAL, S_JALR, S_ERROR=4'hF } state;
logic [31:0] instr, imm_ext, alu_out, data, result, A;
logic [1:0] alu_op;
logic [6:0] op;
logic [2:0] funct3;
logic pc_target_src, pc_update, adr_src;
logic branch, branch_taken, cond, funct7b5;

// branches
//assign branch_taken = cond & branch;
/*
always_comb begin
  case(funct3)
    3'b000: cond = zero; // beq
    3'b001: cond = ~zero; // bne
    default: cond = 1'b0;
  endcase
end
*/

// control unit variables?
always_comb begin
  op = instr[6:0];
  funct3 = instr[14:12];
  funct7b5 = instr[30];
  rs1 = instr[19:15];
  rs2 = instr[24:20];
  rd = instr[11:7];
  PC_ena = (branch & (funct3[0] ? ~zero : zero)) | pc_update;
end 

// Muxes
//Mux A 
enum logic [1:0] {ALU_SRC_PC_A,ALU_SRC_RF_A, ALU_SRC_PC_A_OLD} alu_src_a; 
always_comb begin: ALU_A_MUX
    case (alu_src_a)
        ALU_SRC_PC_A: src_a = PC; 
        ALU_SRC_RF_A: src_a = A; 
        ALU_SRC_PC_A_OLD: src_a = PC_old; 
        default: src_a = 0; 
    endcase
end

//Mux B
enum logic [1:0] {ALU_SRC_RF_B, ALU_SRC_IMM_B, ALU_SRC_4_B} alu_src_b; 
always_comb begin: ALU_B_MUX
    case (alu_src_b)
        ALU_SRC_RF_B: src_b = mem_wr_data; 
        ALU_SRC_IMM_B: src_b = imm_ext; 
        ALU_SRC_4_B: src_b = 32'd4; 
        default: src_b = 0; 
    endcase
end

//Memory Mux 
enum logic [1:0] {MEM_SRC_PC, MEM_SRC_RESULT} mem_src; 
always_comb begin: MEMORY_ADR_MUX
    case(mem_src)
        MEM_SRC_PC: mem_addr = PC; 
        MEM_SRC_RESULT: mem_addr = result;//ALU Result mux result 
    endcase
end

enum logic [1:0] {RESULT_SRC_ALU, RESULT_SRC_MEM_DATA, RESULT_SRC_ALU_LAST} result_src;
always_comb begin : final_result_mux
  case(result_src)
    RESULT_SRC_ALU:      result = alu_result;
    RESULT_SRC_MEM_DATA: result = data;
    RESULT_SRC_ALU_LAST: result = alu_last;
    default : result = alu_out;
  endcase
end

always_comb begin: RESULT_ALIASES //for things that are connected/same value
    PC_next = result; 
    rfile_wr_data = result; 
end


enum logic [1:0] {IMM_SRC_I_TYPE, IMM_SRC_B_TYPE, IMM_SRC_S_TYPE, IMM_SRC_J_TYPE} imm_src;
always_comb begin
   // immediates
  case(op)
    default:  imm_src=IMM_SRC_I_TYPE;
    OP_BTYPE: imm_src=IMM_SRC_B_TYPE;
    OP_ITYPE: imm_src=IMM_SRC_I_TYPE;
    OP_STYPE: imm_src=IMM_SRC_S_TYPE;
    OP_JAL:   imm_src=IMM_SRC_J_TYPE;
    OP_JALR:  imm_src=IMM_SRC_J_TYPE;
  endcase
  case(imm_src)
    IMM_SRC_I_TYPE: imm_ext={{20{instr[31]}},instr[31:20]};
    IMM_SRC_S_TYPE: imm_ext={{20{instr[31]}},instr[31:25],instr[11:7]};
    IMM_SRC_B_TYPE: imm_ext={{20{instr[31]}},instr[30:25],instr[11:8],1'b0};
    IMM_SRC_J_TYPE: imm_ext={{20{instr[31]}},instr[19:12],instr[20],instr[30:21],1'b0};
  endcase
end

//Non-architectural registers
//IR write register
register #(.N(32)) INSTRUCTION_REGISTER(
    .clk(clk), .rst(rst), .ena(ir_write), .d(mem_rd_data), .q(instr)
); 

//ALU result register
logic alu_ena; 
wire [31:0] alu_last; 
register #(.N(32)) ALU_RESULT_REGISTER(
    .clk(clk), .rst(rst), .ena(1'b1), .d(alu_result), .q(alu_last)
); 

// control signals
always_comb begin
  case(state)
    S_FETCH: begin
      branch      = 0;
      mem_wr_ena  = 0;
      adr_src     = 0;
      pc_update   = 1;
      ir_write    = 1;
      reg_write   = 0;
      alu_src_a   = ALU_SRC_PC_A;
      alu_src_b   = ALU_SRC_4_B;
      alu_op      = 2'b00;
      result_src  = RESULT_SRC_ALU_LAST;
    end
    S_DECODE: begin
      branch      = 0;
      mem_wr_ena  = 0;
      adr_src     = 0;
      pc_update   = 0;
      ir_write    = 0;
      reg_write   = 0;
      alu_src_a   = ALU_SRC_RF_A;
      alu_src_b   = ALU_SRC_IMM_B;
      alu_op      = 2'b00;
      result_src  = RESULT_SRC_ALU;
    end
    S_EXECUTER: begin
      branch      = 0;
      mem_wr_ena  = 0;
      adr_src     = 0;
      pc_update   = 0;
      ir_write    = 0;
      reg_write   = 0;
      alu_src_a   = ALU_SRC_PC_A_OLD;
      alu_src_b   = ALU_SRC_RF_B;
      alu_op      = 2'b10;
      result_src  = RESULT_SRC_ALU;
    end
    S_EXECUTEI: begin
      branch      = 0;
      mem_wr_ena  = 0;
      adr_src     = 0;
      pc_update   = 0;
      ir_write    = 0;
      reg_write   = 0;
      alu_src_a   = ALU_SRC_PC_A_OLD;
      alu_src_b   = ALU_SRC_IMM_B;
      alu_op      = 2'b10;
      result_src  = RESULT_SRC_ALU;
    end
    S_ALUWB: begin
      branch      = 0;
      mem_wr_ena  = 0;
      adr_src     = 0;
      pc_update   = 0;
      ir_write    = 0;
      reg_write   = 1;
      alu_src_a   = ALU_SRC_PC_A;
      alu_src_b   = ALU_SRC_RF_B;
      alu_op      = 2'b00;
      result_src  = RESULT_SRC_ALU;
    end
    S_MEMADR: begin
      branch      = 0;
      mem_wr_ena  = 0;
      adr_src     = 0;
      pc_update   = 0;
      ir_write    = 0;
      reg_write   = 0;
      alu_src_a   = ALU_SRC_PC_A_OLD;
      alu_src_b   = ALU_SRC_IMM_B;
      alu_op      = 2'b00;
      result_src  = RESULT_SRC_ALU;
    end
    S_MEMREAD: begin
      branch      = 0;
      mem_wr_ena  = 0;
      adr_src     = 1;
      pc_update   = 0;
      ir_write    = 0;
      reg_write   = 0;
      alu_src_a   = ALU_SRC_PC_A;
      alu_src_b   = ALU_SRC_RF_B;
      alu_op      = 2'b00;
      result_src  = RESULT_SRC_ALU;
    end
    S_MEMWRITE: begin // check
        branch      = 0;
        mem_wr_ena  = 1;
        adr_src     = 1;
        pc_update   = 0;
        ir_write    = 0;
        reg_write   = 0;
        alu_src_a   = ALU_SRC_PC_A;
        alu_src_b   = ALU_SRC_4_B;
        alu_op      = 2'b00;
        result_src  = RESULT_SRC_ALU;
    end
    S_MEMWB: begin
      branch      = 0;
      mem_wr_ena  = 0;
      adr_src     = 0;
      pc_update   = 0;
      ir_write    = 0;
      reg_write   = 1;
      alu_src_a   = ALU_SRC_PC_A;
      alu_src_b   = ALU_SRC_RF_B;
      alu_op      = 2'b00;
      result_src  = RESULT_SRC_ALU;
    end
    S_BRANCH: begin
      branch      = 1;
      mem_wr_ena  = 0;
      adr_src     = 0;
      pc_update   = 0;
      ir_write    = 0;
      reg_write   = 0;
      alu_src_a   = ALU_SRC_PC_A_OLD;
      alu_src_b   = ALU_SRC_RF_B;
      alu_op      = 2'b01;
      result_src  = RESULT_SRC_ALU;
    end
    S_JAL: begin
      branch      = 0;
      mem_wr_ena  = 0;
      adr_src     = 0;
      pc_update   = 1;
      ir_write    = 0;
      reg_write   = 0;
      alu_src_a   = ALU_SRC_RF_A;
      alu_src_b   = ALU_SRC_4_B;
      alu_op      = 2'b00;
      result_src  = RESULT_SRC_ALU;
    end
    S_JALR: begin
      branch      = 0;
      mem_wr_ena  = 0;
      adr_src     = 0;
      pc_update   = 1;
      ir_write    = 0;
      reg_write   = 0;
      alu_src_a   = ALU_SRC_PC_A_OLD;
      alu_src_b   = ALU_SRC_IMM_B;
      alu_op      = 2'b00;
      result_src  = RESULT_SRC_ALU_LAST;
    end
    default: begin
      branch      = 0;
      mem_wr_ena  = 0;
      adr_src     = 0;
      pc_update   = 0;
      ir_write    = 0;
      reg_write   = 0;
      alu_src_a   = ALU_SRC_PC_A;
      alu_src_b   = ALU_SRC_RF_B;
      alu_op      = 2'b00;
      result_src  = RESULT_SRC_ALU;
    end
  endcase
end

//ALU Decoding Types of Instructions 
always_comb begin: ALU_DECODE_INSTRUCTION
    case(alu_op)
      2'b00: alu_control = ALU_ADD;
      2'b01: alu_control = ALU_SUB;
      2'b10: begin
        case(funct3)
            FUNCT3_ADD: begin
                if(funct7b5 & op[5])
                    alu_control = ALU_SUB; 
                else
                    alu_control = ALU_ADD; 
                end
            FUNCT3_SLT: alu_control = ALU_SLT; 
            FUNCT3_XOR: alu_control = ALU_XOR;
            FUNCT3_SLL: alu_control = ALU_SLL; 
            FUNCT3_OR:  alu_control = ALU_OR; 
            FUNCT3_AND: alu_control = ALU_AND; 
            FUNCT3_SLTU: alu_control = ALU_SLTU;
            default: alu_control = ALU_INVALID;
            FUNCT3_SHIFT_RIGHT: begin 
                if(funct7b5)
                    alu_control = ALU_SRL;
                else 
                    alu_control = ALU_SRA;
            end
        endcase
      end
      default: alu_control = ALU_INVALID;
    endcase
end 

/*
always @(posedge clk) begin
  if (rst) begin
    instr <= 0;
  end else if (ena) case (state)
    S_FETCH: instr <= mem_rd_data;
  endcase
end
*/
always_ff @( posedge clk ) begin
  if (rst) begin
    state <= S_FETCH;
    reg_write <= 0;
  end else begin
    A <= reg_data1;
    mem_wr_data <= reg_data2;
    alu_out <= alu_result;
    data <= mem_rd_data;
    instr <= ir_write ? mem_rd_data : instr;
  end 
end

always_ff @(posedge clk) begin
  if (rst) begin
    state <= S_FETCH;
  end else if (~ena) begin
    // Do nothing if ena is disabled
  end else case(state)
      S_FETCH: state <= S_DECODE;
      S_DECODE: begin
      case(op)
        //replace with variables from defines file 
        OP_LTYPE: state <= S_MEMADR; // lw
        OP_STYPE: state <= S_MEMADR; // sw
        OP_RTYPE: state <= S_EXECUTER; // R-type
        OP_ITYPE: state <= S_EXECUTEI; // I-type
        OP_JAL: state <= S_JAL; // jal
        OP_JALR: state <= S_JALR; // jalr
        OP_BTYPE: state <= S_BRANCH; // beq, bne
      endcase
      end
      S_MEMADR: begin
        if (op[5]==1'b1)
            state <= S_MEMWRITE; // sw
        else if (op[5]==1'b0) 
            state <= S_MEMREAD; // lw
        else
            state <= S_ERROR;
      end
      S_MEMREAD: state <= S_MEMWB;
      S_EXECUTEI, S_EXECUTER, S_JAL, S_JALR: state <= S_ALUWB;
      S_ALUWB, S_MEMWB, S_MEMWRITE: state <= S_FETCH;
    endcase
end
endmodule
