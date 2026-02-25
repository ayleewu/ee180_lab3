//=============================================================================
// EE180 Lab 3
//
// Decode module. Determines what to do with an instruction.
//=============================================================================

`include "mips_defines.v"

module decode (
    input [31:0] pc,
    input [31:0] instr,
    input [31:0] rs_data_in,
    input [31:0] rt_data_in,

    output wire [4:0] reg_write_addr,
    output wire jump_branch,
    output wire jump_target,
    output wire jump_reg,
    output wire [31:0] jr_pc,
    output reg [3:0] alu_opcode,
    output wire [31:0] alu_op_x,
    output wire [31:0] alu_op_y,
    output wire mem_we,
    output wire [31:0] mem_write_data,
    output wire mem_read,
    output wire mem_byte,
    output wire mem_signextend,
    output wire reg_we,
    output wire movn,
    output wire movz,
    output wire [4:0] rs_addr,
    output wire [4:0] rt_addr,
    output wire atomic_id,
    input  atomic_ex,
    output wire mem_sc_mask_id,
    output wire mem_sc_id,

    output wire stall,

    input reg_we_ex,
    input [4:0] reg_write_addr_ex,
    input [31:0] alu_result_ex,
    input mem_read_ex,

    input reg_we_mem,
    input [4:0] reg_write_addr_mem,
    input [31:0] reg_write_data_mem,

    // Added wires
    output wire mem_half
);
//******************************************************************************
// instruction field
//******************************************************************************

    wire [5:0] op = instr[31:26];
    assign rs_addr = instr[25:21];
    assign rt_addr = instr[20:16];
    wire [4:0] rd_addr = instr[15:11];
    wire [4:0] shamt = instr[10:6];
    wire [5:0] funct = instr[5:0];
    wire [15:0] immediate = instr[15:0];

    wire [31:0] rs_data, rt_data;

//******************************************************************************
// branch instructions decode
//******************************************************************************

    wire isBEQ    = (op == `BEQ);
    wire isBGEZNL = (op == `BLTZ_GEZ) & (rt_addr == 5'b00001);
    wire isBGEZAL = (op == `BLTZ_GEZ) & (rt_addr == `BGEZAL);
    wire isBGTZ   = (op == `BGTZ) & (rt_addr == 5'b00000);
    wire isBLEZ   = (op == `BLEZ) & (rt_addr == 5'b00000);
    wire isBLTZNL = (op == `BLTZ_GEZ) & (rt_addr == `BLTZ);
    wire isBLTZAL = (op == `BLTZ_GEZ) & (rt_addr == `BLTZAL);
    wire isBNE    = (op == `BNE);
    wire isBranchLink = (isBGEZAL | isBLTZAL);

    // Added wires
    wire isBGEZ   = isBGEZNL | isBGEZAL;

//******************************************************************************
// load instructions decode
//******************************************************************************

    wire isLB  = (op == `LB);
    wire isLBU = (op == `LBU);
    wire isLH  = (op == `LH);
    wire isSH  = (op == `SH);
    wire isLL  = (op == `LL);

//******************************************************************************
// jump instructions decode
//******************************************************************************

    wire isJ    = (op == `J);
    wire isJAL  = (op == `JAL);

    wire isJR   = (op == `SPECIAL) & (funct == `JR);
    wire isJALR = (op == `SPECIAL) & (funct == `JALR);

    assign jump_target = isJ | isJAL;
    assign jump_reg    = isJR | isJALR;
    assign jr_pc       = rs_data;

//******************************************************************************
// shift instruction decode
//******************************************************************************

    wire isSLL = (op == `SPECIAL) & (funct == `SLL);
    wire isSRL = (op == `SPECIAL) & (funct == `SRL);
    wire isSLLV = (op == `SPECIAL) & (funct == `SLLV);
    wire isSRLV = (op == `SPECIAL) & (funct == `SRLV);

    // Added Instructions
    wire isSRAV = (op == `SPECIAL) & (funct == `SRAV);
    wire isSRA = (op == `SPECIAL) & (funct == `SRA);

    wire isShiftImm = isSLL | isSRL | isSRA;
    wire isShift = isShiftImm | isSLLV | isSRLV | isSRAV;

//******************************************************************************
// multiplication instruction decode
//******************************************************************************

    wire isMUL = (op == `SPECIAL2) & (funct == `MUL);
//******************************************************************************
// ALU instructions decode / control signal for ALU datapath
//******************************************************************************

       always @* begin
        casex({op, funct})
            {`ADDI, `DC6}:      alu_opcode = `ALU_ADD;
            {`ADDIU, `DC6}:     alu_opcode = `ALU_ADDU;
            {`SLTI, `DC6}:      alu_opcode = `ALU_SLT;
            {`SLTIU, `DC6}:     alu_opcode = `ALU_SLTU;
            {`ANDI, `DC6}:      alu_opcode = `ALU_AND;
            {`ORI, `DC6}:       alu_opcode = `ALU_OR;
            {`LB, `DC6}:        alu_opcode = `ALU_ADD;
            {`LW, `DC6}:        alu_opcode = `ALU_ADD;
            {`LBU, `DC6}:       alu_opcode = `ALU_ADD;
            {`SB, `DC6}:        alu_opcode = `ALU_ADD;
            {`SW, `DC6}:        alu_opcode = `ALU_ADD;
            {`BEQ, `DC6}:       alu_opcode = `ALU_SUBU;
            {`BNE, `DC6}:       alu_opcode = `ALU_SUBU;
            {`SPECIAL, `ADD}:   alu_opcode = `ALU_ADD;
            {`SPECIAL, `ADDU}:  alu_opcode = `ALU_ADDU;
            {`SPECIAL, `SUB}:   alu_opcode = `ALU_SUB;
            {`SPECIAL, `SUBU}:  alu_opcode = `ALU_SUBU;
            {`SPECIAL, `AND}:   alu_opcode = `ALU_AND;
            {`SPECIAL, `OR}:    alu_opcode = `ALU_OR;
            {`SPECIAL, `MOVN}:  alu_opcode = `ALU_PASSX;
            {`SPECIAL, `MOVZ}:  alu_opcode = `ALU_PASSX;
            {`SPECIAL, `SLT}:   alu_opcode = `ALU_SLT;
            {`SPECIAL, `SLTU}:  alu_opcode = `ALU_SLTU;
            {`SPECIAL, `SLL}:   alu_opcode = `ALU_SLL;
            {`SPECIAL, `SRL}:   alu_opcode = `ALU_SRL;
            {`SPECIAL, `SLLV}:  alu_opcode = `ALU_SLL;
            {`SPECIAL, `SRLV}:  alu_opcode = `ALU_SRL;
            // Added instructions
            {`XORI, `DC6}:      alu_opcode = `ALU_XOR;
            {`SPECIAL, `XOR}:   alu_opcode = `ALU_XOR;
            {`SPECIAL2, `MUL}:  alu_opcode = `ALU_MUL;
            {`LH, `DC6}:        alu_opcode = `ALU_ADD;
            {`SH, `DC6}:        alu_opcode = `ALU_ADD;
            {`SPECIAL, `SRA}:   alu_opcode = `ALU_SRA;
            {`SPECIAL, `SRAV}:  alu_opcode = `ALU_SRA;
            // compare rs data to 0, only care about 1 operand
            {`BGTZ, `DC6}:      alu_opcode = `ALU_PASSX;
            {`BLEZ, `DC6}:      alu_opcode = `ALU_PASSX;
            {`BLTZ_GEZ, `DC6}: begin
                if (isBranchLink)
                    alu_opcode = `ALU_PASSY; // pass link address for mem stage
                else
                    alu_opcode = `ALU_PASSX;
            end
            // pass link address to be stored in $ra
            {`JAL, `DC6}:       alu_opcode = `ALU_PASSY;
            {`SPECIAL, `JALR}:  alu_opcode = `ALU_PASSY;
            // or immediate with 0
            {`LUI, `DC6}:       alu_opcode = `ALU_PASSY;
            default:            alu_opcode = `ALU_PASSX;
        endcase
    end

//******************************************************************************
// Compute value for 32 bit immediate data
//******************************************************************************
    wire use_imm = &{op != `SPECIAL, op != `SPECIAL2, op != `BNE, op != `BEQ}; // where to get 2nd ALU operand from: 0 for RtData, 1 for Immediate

    wire [31:0] imm_sign_extend = {{16{immediate[15]}}, immediate};
    wire [31:0] imm_zero_extend = {16'b0, immediate};
    wire [31:0] imm_upper = {immediate, 16'b0};

    wire isLogicImm = (op == `ANDI) | (op == `ORI) | (op == `XORI);

    wire [31:0] imm = (op == `LUI) ? imm_upper : isLogicImm ?  imm_zero_extend : imm_sign_extend;

//******************************************************************************
// forwarding and stalling logic
//******************************************************************************
    // Forward rs & rt from EX
    wire forward_rs_ex = reg_we_ex & ~mem_read_ex & (rs_addr == reg_write_addr_ex) & (rs_addr != `ZERO);
    wire forward_rt_ex = reg_we_ex & ~mem_read_ex &  (rt_addr == reg_write_addr_ex) & (rt_addr != `ZERO);

    // Forward rs & rt from MEM
    wire forward_rs_mem = &{rs_addr == reg_write_addr_mem, rs_addr != `ZERO, reg_we_mem};
    wire forward_rt_mem = &{rt_addr == reg_write_addr_mem, rt_addr != `ZERO, reg_we_mem};
    assign rs_data = forward_rs_ex ? alu_result_ex : forward_rs_mem ? reg_write_data_mem : rs_data_in;
    assign rt_data = forward_rt_ex ? alu_result_ex : forward_rt_mem ? reg_write_data_mem : rt_data_in;

    wire isLUI = op == `LUI;
    wire read_from_rs = ~|{isLUI, jump_target, isShiftImm};

    wire isALUImm = |{op == `ADDI, op == `ADDIU, op == `SLTI, op == `SLTIU, op == `ANDI, op == `ORI};
    wire read_from_rt = ~|{isLUI, jump_target, isALUImm, mem_read};

    // Check load use hazard and stall if so
    wire rs_load_use = mem_read_ex & (rs_addr == reg_write_addr_ex) & (rs_addr != `ZERO);
    wire rt_load_use = mem_read_ex & (rt_addr == reg_write_addr_ex) & (rt_addr != `ZERO);

    // Check if stall branches on EX dependency 
    wire isBranch = isBEQ | isBNE | isBGEZAL | isBGEZNL | isBLTZNL | isBLTZAL | isBGTZ | isBLEZ;
    wire branch_uses_rt = isBEQ | isBNE;
    wire ex_hazard_rs = reg_we_ex & (rs_addr == reg_write_addr_ex) & (reg_write_addr_ex != `ZERO);
    wire ex_hazard_rt = reg_we_ex & (rt_addr == reg_write_addr_ex) & (reg_write_addr_ex != `ZERO);
    wire stall_ex_branch = isBranch & (ex_hazard_rs | (branch_uses_rt & ex_hazard_rt));

    assign stall = stall_ex_branch |
            ((rs_load_use & read_from_rs) | (rt_load_use & read_from_rt));
    assign mem_write_data = rt_data;
//******************************************************************************
// Determine ALU inputs and register writeback address
//******************************************************************************

    // for shift operations, use either shamt field or lower 5 bits of rs
    // otherwise use rs
    // is shamt an intermediate part tho?
    wire [31:0] shift_amount = isShiftImm ? shamt : rs_data[4:0];
    assign alu_op_x = isShift ? shift_amount : rs_data;

    // for link operations, use next pc (current pc + 8)
    // for immediate operations, use Imm
    // otherwise use rt

    assign alu_op_y = (use_imm) ? imm : rt_data;
    assign reg_write_addr = (use_imm) ? rt_addr : rd_addr;

    // determine when to write back to a register (any operation that isn't an
    // unconditional store, non-linking branch, or non-linking jump)
    assign reg_we = ~|{(mem_we & (op != `SC)), isJ, isBGEZNL, isBGTZ, isBLEZ, isBLTZNL, isBNE, isBEQ};

    // determine whether a register write is conditional
    assign movn = &{op == `SPECIAL, funct == `MOVN};
    assign movz = &{op == `SPECIAL, funct == `MOVZ};

//******************************************************************************
// Memory control
//******************************************************************************

    assign mem_we = |{op == `SW, op == `SB, op == `SH,  op == `SC};    // write to memory
    assign mem_read = isLB | isLBU | isLH | (op == `LW);                     // use memory data for writing to a register
    assign mem_byte = |{op == `SB, op == `LB, op == `LBU};    // memory operations use only one byte
    assign mem_signextend = isLB | isLH;     // sign extend sub-word memory reads
    assign mem_half = (op == `SH) | (op == `LH);

//******************************************************************************
// Load linked / Store conditional
//******************************************************************************
    
    // not complete but doesn't affect any tests right now because we haven't tested stores
    wire nc_store_happens;
    assign mem_sc_id = (op == `SC);
    assign nc_store_happens = |{(op == `SW), (op == `SB), (op == `SH)};

    // 'mem_sc_mask_id' is high when a store conditional should not store.
    assign mem_sc_mask_id = (nc_store_happens & atomic_ex) ? 1'b1 : 1'b0;

    // 'atomic_id' is high when a load-linked has not been followed by
    // a store.
    assign atomic_id = (op == `LL);



//******************************************************************************
// Branch resolution
//******************************************************************************

    wire isEqual = rs_data == rt_data;

    // Checks if reg_sign is positive or negative 
    wire not_negative = ~rs_data[31];
    assign jump_branch = |{(isBEQ & isEqual),
                           (isBNE & ~isEqual),
                           (isBGEZ & not_negative),
                           ((isBLTZNL | isBLTZAL) & ~not_negative),
                           (isBGTZ & (~rs_data[31] & (rs_data != 32'b0))),
                           (isBLEZ & (rs_data[31] | (rs_data == 32'b0)))};

endmodule
                                                                                                                                                                                                 297,1         Bot
