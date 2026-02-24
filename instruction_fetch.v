//=============================================================================
// EE180 Lab 3
//
// Instruction fetch module. Maintains PC and updates it. Reads from the
// instruction ROM.
//=============================================================================

module instruction_fetch (
    input clk,
    input rst,
    input en,               // enable bit for stalls
    input jump_target,
    input jump_branch,
    input jump_reg,
    input [31:0] jr_pc,
    input [31:0] pc_id,
    input [25:0] instr_id,  // Lower 26 bits of the instruction

    output [31:0] pc
);


    wire [31:0] pc_id_p4 = pc_id + 3'h4;

    // J-type instruction
    wire [31:0] j_addr = {pc_id_p4[31:28], instr_id[25:0], 2'b0};

    // Branch displacement 
    wire [31:0] br_disp = {{14{instr_id[15]}}, instr_id[15:0], 2'b00};
    wire [31:0] br_addr = pc_id_p4 + br_disp;

    // Compute next PC
    wire [31:0] pc_next = jump_reg ? jr_pc : jump_target ? j_addr : jump_branch ? br_addr : (pc + 32'd4);

    // it'll take one clock cycle to update pc
    dffare #(32) pc_reg (.clk(clk), .r(rst), .en(en), .d(pc_next), .q(pc));


endmodule
~                             
