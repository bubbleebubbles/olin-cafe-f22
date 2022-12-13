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
output logic [31:0] instructions_completed; 


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

register #(.N(32)) REGISTER_A(
  .clk(clk), .rst(rst), .ena(1'b1), .d(reg_data1), .q(reg_A)
);

register #(.N(32)) REGISTER_B (
  .clk(clk), .rst(rst), .ena(1'b1), .d(reg_data1), .q(reg_B)
);


always_comb mem_wr_data = reg_B; 

// ALU and related control signals
// Feel free to replace with your ALU from the homework.
logic [31:0] src_a, src_b;
alu_control_t alu_control, ri_alu_control;
wire [31:0] alu_result;
wire overflow, zero, equal;
alu_behavioural ALU (
  .a(src_a), .b(src_b), .result(alu_result),
  .control(alu_control),
  .overflow(overflow), .zero(zero), .equal(equal)
);

// Implement your multicycle rv32i CPU here!

//Mux A 
enum logic [1:0] {ALU_SRC_PC_A,ALU_SRC_RF_A, ALU_SRC_PC_A_OLD} alu_src_a; 
always_comb begin: ALU_A_MUX
    case (alu_src_a)
        ALU_SRC_PC_A: src_a = PC; 
        ALU_SRC_RF_A: src_a = reg_A; 
        ALU_SRC_PC_A_OLD: src_a = PC_old; 
        default: src_a = 0; 
    endcase
end

//Mux B
enum logic [1:0] {ALU_SRC_RF_B, ALU_SRC_IMM_B, ALU_SRC_4_B} alu_src_b; 
always_comb begin: ALU_B_MUX
    case (alu_src_b)
        ALU_SRC_RF_B: src_b = reg_B; 
        ALU_SRC_IMM_B: src_b = immediate_extended ; 
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

//Non-architectural registers
//IR write register
logic IR_write; 
wire [31:0] instruction;
register #(.N(32)) INSTRUCTION_REGISTER(
    .clk(clk), .rst(rst), .ena(IR_write), .d(mem_rd_data), .q(instruction)
); 

//ALU result register
logic alu_ena; 
wire [31:0] alu_last; 
register #(.N(32)) ALU_RESULT_REGISTER(
    .clk(clk), .rst(rst), .ena(alu_ena), .d(alu_result), .q(alu_last)
); 

//Data memory register
logic mem_data_ena;
wire [31:0] mem_data; 
register #(.N(32)) DATA_MEMORY_REGISTER(
    .clk(clk), .rst(rst), .ena(mem_data_ena), .d(mem_rd_data), .q(mem_data)
); 

//Output Mux 
enum logic [1:0] {RESULT_SRC_ALU, RESULT_SRC_MEM_DATA, RESULT_SRC_ALU_LAST} result_src; 
logic [31:0] result;
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

// Decoding instructions
logic [1:0] alu_op;
logic [6:0] op;
logic [2:0] funct3;
logic [6:0] funct7;
logic [31:0] immediate_extended;
logic rtype;
logic itype;
logic btype;
logic ltype;
logic jtype;
logic stype;
enum logic [1:0] {IMM_SRC_I_TYPE, IMM_SRC_B_TYPE, IMM_SRC_S_TYPE, IMM_SRC_J_TYPE} immediate_src;

always_comb begin: INSTRUCTION_BREAKDOWN

  // type of intruction
  rtype = (op == OP_RTYPE); 
  itype = (op == OP_ITYPE); 
  stype = (op == OP_STYPE); 
  btype = (op == OP_BTYPE); 
  ltype = (op == OP_LTYPE); 
  jtype = (op == OP_JAL) | (op == OP_JALR); 

  // instruction breakdown
  op=instruction[6:0];
  rd=instruction[11:7];
  rs1=instruction[19:15];
  rs2=instruction[24:20];
  funct3=instruction[14:12];
  funct7=instruction[31:25];

  // immediates
  case(op)
    default:  immediate_src=IMM_SRC_I_TYPE;
    OP_BTYPE: immediate_src=IMM_SRC_B_TYPE;
    OP_ITYPE: immediate_src=IMM_SRC_I_TYPE;
    OP_STYPE: immediate_src=IMM_SRC_S_TYPE;
    OP_JAL:   immediate_src=IMM_SRC_J_TYPE;
    OP_JALR:  immediate_src=IMM_SRC_J_TYPE;
  endcase
  case(immediate_src)
    2'b00: immediate_extended={{20{instruction[31]}},instruction[31:20]};
    2'b01: immediate_extended={{20{instruction[31]}},instruction[31:25],instruction[11:7]};
    2'b10: immediate_extended={{20{instruction[31]}},instruction[30:25],instruction[11:8],1'b0};
    2'b11: immediate_extended={{20{instruction[31]}},instruction[19:12],instruction[20],instruction[30:21],1'b0};
  endcase

end 

/*
logic branch; // check if using
logic [3:0] flags;
//logic v, c, n, zero; // Flags: overflow, carry out, negative, zero
logic cond; // cond is 1 when condition for branch met
//assign {v, c, n, zero} = flags; // check if using
logic PCTargetSrc;
logic [2:0] ImmSrc;
*/
logic cond; // cond is 1 when condition for branch met

logic sub_true; 
//ALU Decoding Types of Instructions 
always_comb begin: ALU_DECODE_INSTRUCTION
    case(funct3)
        FUNCT3_ADD: begin
            if(funct7[5] && op==OP_RTYPE)
                ri_alu_control = ALU_SUB; 
            else
                ri_alu_control = ALU_ADD; 
        end 

        FUNCT3_SLT: ri_alu_control = ALU_SLT; 
        FUNCT3_XOR: ri_alu_control = ALU_XOR;
        FUNCT3_SLL: ri_alu_control = ALU_SLL; 
        FUNCT3_OR:  ri_alu_control = ALU_OR; 
        FUNCT3_AND: ri_alu_control = ALU_AND; 
        FUNCT3_SLTU: ri_alu_control = ALU_SLTU;
        FUNCT3_SHIFT_RIGHT: begin 
            if(funct7[5])
                ri_alu_control = ALU_SRA;
            else 
                ri_alu_control = ALU_SRL;
        end 
        FUNCT3_BNE: begin
          if(cond != 1'b0)
            PC_ena = 1;
        end
        FUNCT3_BEQ: begin
          if(cond == 1'b0)
            PC_ena = 1;
        end 
    endcase 
end 
        

// Main FSM

enum logic [3:0] {S_FETCH, S_DECODE, S_MEMADR, S_EXECUTER, S_EXECUTEI, S_JUMP, S_BRANCH, S_ALUWB, S_MEMREAD, S_MEMWRITE, S_MEMWB, S_BEQ, S_JAL, S_JALR, S_ERROR=4'hF } state, next_state;


logic [14:0] controls;

// State register
always @(posedge clk or posedge rst) begin : FSM_MULTICYCLE
  if (rst) state <= S_FETCH;
  else state <= next_state;
end

// Logic for next state
always_comb begin: NEXT_STATE_LOGIC
  case(state)
    default: next_state = S_FETCH;
    S_FETCH: next_state = S_DECODE;
    S_DECODE: begin
      case(op)
        default: begin
          $display("Error - %b op not implemented", op);
          next_state = S_ERROR;
        end
        //replace with variables from defines file 
        OP_LTYPE: next_state = S_MEMADR; // lw
        OP_STYPE: next_state = S_MEMADR; // sw
        OP_RTYPE: next_state = S_EXECUTER; // R-type
        OP_ITYPE: next_state = S_EXECUTEI; // I-type
        OP_JAL: next_state = S_JAL; // jal
        OP_JALR: next_state = S_JALR; // jalr
        OP_BTYPE: next_state = S_BRANCH; // beq, bne
      endcase
    end
    S_MEMADR: begin
        if (op[5]==1'b1)
            next_state = S_MEMWRITE; // sw
        else if (op[5]==1'b0) 
            next_state = S_MEMREAD; // lw
        else
            next_state = S_ERROR; 
    end
    S_MEMREAD: next_state = S_MEMWB;
    S_EXECUTEI, S_EXECUTER, S_JAL: next_state = S_ALUWB;
    S_ALUWB, S_MEMREAD, S_MEMWRITE: next_state = S_FETCH;
    S_JUMP: begin
      /*
      $display("Please implement jumps")
      next_state = S_ERROR;
      */

    end
    S_BRANCH: begin
      /*
      $display("Please implement branches")
      next_state = S_ERROR;
      */
        alu_src_a = ALU_SRC_RF_A;
        alu_src_b = ALU_SRC_RF_B;
        ri_alu_control = ALU_SUB;
        result_src = RESULT_SRC_ALU;
        next_state = S_FETCH;
    end
    endcase 
end

// state output logic
always_comb begin: STATE_OUTPUT_LOGIC
    case(state)
    /*
    // AdrSrc_IRWrite_ALUSrcA_ALUSrcB_ALUOp_ResultSrc_PCUpdate_RegWrite_MemWrite_Branch
    S_FETCH:    controls=15'b0_1_00_10_00_10_1_0_0_0;
    S_DECODE:   controls=15'b0_0_01_01_00_00_0_0_0_0;
    S_MEMADR:   controls=15'b0_0_10_01_00_00_0_0_0_0;
    S_MEMREAD:  controls=15'b1_0_00_00_00_00_0_0_0_0;
    S_MEMWRITE: controls=15'b1_0_00_00_00_00_0_0_1_0;
    S_MEMWB:    controls=15'b0_0_00_00_00_01_0_1_0_0;
    S_EXECUTER: controls=15'b0_0_10_00_10_00_0_0_0_0;
    S_EXECUTEI: controls=15'b0_0_10_01_10_00_0_0_0_0;
    S_ALUWB:    controls=15'b0_0_00_00_00_00_0_1_0_0;
    S_JAL:      controls=15'b0_0_01_10_00_00_1_0_0_0;
    S_BEQ:      controls=15'b0_0_10_00_01_00_1_0_0_1;
    S_BNE:      controls=15'b0_0_10_00_01_00_1_0_0_1;
    default:    controls=15'bx_x_xx_xx_xx_xx_x_x_x_x;
    */

    S_FETCH: begin
        mem_wr_ena      = 0;
        reg_write       = 0;
        PC_ena          = 1;
        result_src      = RESULT_SRC_ALU;
        IR_write        = 1;
        alu_src_a       = ALU_SRC_PC_A;
        alu_src_b       = ALU_SRC_4_B;
        alu_control     = ALU_ADD;
        alu_ena         = 0;
        mem_data_ena = 0;
        mem_src         = MEM_SRC_PC;
    end
    S_DECODE: begin
        mem_wr_ena      = 0;
        reg_write       = 0;
        PC_ena          = 0;
        result_src      = RESULT_SRC_ALU;
        IR_write        = 0;
        alu_src_a       = ALU_SRC_RF_A;
        alu_src_b       = ALU_SRC_RF_B;
        alu_control     = ALU_INVALID;
        alu_ena         = 0;
        mem_data_ena = 0;
        mem_src         = MEM_SRC_PC;
    end
    S_EXECUTER: begin
      mem_wr_ena      = 0;
      reg_write       = 0;
      PC_ena          = 0;
      result_src      = RESULT_SRC_ALU;
      IR_write        = 0;
      alu_src_a       = ALU_SRC_RF_A;
      alu_src_b       = ALU_SRC_RF_B;
      alu_control     = ri_alu_control;
      alu_ena         = 1;
      mem_data_ena = 0;
      mem_src         = MEM_SRC_PC;
    end
    S_EXECUTEI: begin
      mem_wr_ena      = 0;
      reg_write       = 0;
      PC_ena          = 0;
      result_src      = RESULT_SRC_ALU;
      IR_write        = 0;
      alu_src_a       = ALU_SRC_RF_A;
      alu_src_b       = ALU_SRC_IMM_B;
      alu_control     = ri_alu_control;
      alu_ena         = 1;
      mem_data_ena = 0;
      mem_src         = MEM_SRC_PC;
    end
    S_ALUWB: begin
      mem_wr_ena      = 0;
      reg_write       = 1;
      PC_ena          = 0;
      result_src      = RESULT_SRC_ALU_LAST;
      IR_write        = 0;
      alu_src_a       = ALU_SRC_RF_A;
      alu_src_b       = ALU_SRC_RF_B;
      alu_control     = ALU_INVALID;
      alu_ena         = 0;
      mem_data_ena = 0;
      mem_src         = MEM_SRC_PC;
    end
    S_MEMADR: begin
      mem_wr_ena      = 0;
      reg_write       = 0;
      PC_ena          = 0;
      result_src      = RESULT_SRC_ALU;
      IR_write        = 0;
      alu_src_a       = ALU_SRC_RF_A;
      alu_src_b       = ALU_SRC_IMM_B;
      alu_control     = ALU_ADD;
      alu_ena         = 1;
      mem_data_ena = 0;
      mem_src         = MEM_SRC_RESULT;
    end
    S_MEMREAD: begin
      mem_wr_ena      = 0;
      reg_write       = 0;
      PC_ena          = 0;
      result_src      = RESULT_SRC_ALU_LAST;
      IR_write        = 0;
      alu_src_a       = ALU_SRC_RF_A;
      alu_src_b       = ALU_SRC_IMM_B;
      alu_control     = ALU_INVALID;
      alu_ena         = 0;
      mem_data_ena = 0;
      mem_src         = MEM_SRC_PC;
    end
    S_MEMWRITE: begin // check
      mem_wr_ena      = 1;
      reg_write       = 0;
      PC_ena          = 0;
      result_src      = RESULT_SRC_ALU;
      IR_write        = 0;
      alu_src_a       = ALU_SRC_RF_A;
      alu_src_b       = ALU_SRC_IMM_B;
      alu_control     = ALU_INVALID;
      alu_ena         = 0;
      mem_data_ena = 1;
      mem_src         = MEM_SRC_RESULT;
    end
    S_MEMWB: begin
      mem_wr_ena      = 0;
      reg_write       = 0;
      PC_ena          = 0;
      result_src      = RESULT_SRC_ALU;
      IR_write        = 0;
      alu_src_a       = ALU_SRC_RF_A;
      alu_src_b       = ALU_SRC_IMM_B;
      alu_control     = ALU_INVALID;
      alu_ena         = 1;
      mem_data_ena = 0;
      mem_src         = MEM_SRC_RESULT;
    end
   // S_BEQ: begin 
    //    mem_wr_ena      = 0;
     //   reg_write       = 0;
     //   PC_ena          = 0;
     //   result_src      = RESULT_SRC_ALU;
     //   IR_write        = 0;
     //   alu_src_a       = ALU_SRC_RF_A;
     //   alu_src_b       = ALU_SRC_IMM_B;
     //   alu_control     = ALU_INVALID;
     //   alu_ena         = 1;
     //   mem_data_ena = 0;
    //    mem_src         = MEM_SRC_RESULT;
   // end
    // S_BNE: begin 
     //   mem_wr_ena      = 0;
      //  reg_write       = 0;
       // PC_ena          = 0;
       // result_src      = RESULT_SRC_ALU;
       // IR_write        = 0;
       // alu_src_a       = ALU_SRC_RF_A;
       // alu_src_b       = ALU_SRC_IMM_B;
       // alu_control     = ALU_INVALID;
        //alu_ena         = 1;
       // mem_data_ena = 0;
       // mem_src         = MEM_SRC_RESULT;
   // end
    
    default: begin
      mem_wr_ena      = 0;
      reg_write       = 0;
      PC_ena          = 0;
      result_src      = RESULT_SRC_ALU;
      IR_write        = 0;
      alu_src_a       = ALU_SRC_RF_A;
      alu_src_b       = ALU_SRC_RF_B;
      alu_control     = ALU_INVALID;
      alu_ena         = 0;
      mem_data_ena = 0;
      mem_src         = MEM_SRC_PC;
    end
  endcase
end

//assign {AdrSrc, IRWrite, ALUSrcA, ALUSrcB, ALUOp, ResultSrc, PCUpdate, RegWrite, MemWrite, Branch} = controls;

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
