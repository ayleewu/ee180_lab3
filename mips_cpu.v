module mips_cpu (
    input clk,
    input rst,
    input en,

    output wire [3:0] mem_write_en,
    output wire mem_read_en,
    output wire [31:0] mem_addr,
    output wire [31:0] mem_write_data,
    input [31:0] mem_read_data,
    output wire [31:0] pc,
    input [31:0] instr
);

    wire [31:0] pc_if, pc_id;
    wire [31:0] instr_sav;
    wire [31:0] instr_id;
    wire jump_branch_id, jump_target_id, jump_reg_id;
    wire [4:0] rs_addr_id, rt_addr_id;
    wire [31:0] rs_data_id, rt_data_id;
    wire [31:0] mem_write_data_id, mem_write_data_ex;
    wire [31:0] jr_pc_id;
    wire [4:0] reg_write_addr_id, reg_write_addr_ex, reg_write_addr_mem, reg_write_addr_wb;
    wire [31:0] mem_out;
    wire [31:0] reg_write_data_mem, reg_write_data_wb;
    wire reg_we_id, reg_we_cond_ex, reg_we_ex, reg_we_mem, reg_we_wb;
    wire movn_id, movn_ex, movz_id, movz_ex;
    wire atomic_id, atomic_ex;
    wire [3:0] alu_opcode_id, alu_opcode_ex;
    wire [31:0] alu_op_x_id, alu_op_y_id, alu_op_x_ex, alu_op_y_ex;
    wire [31:0] alu_result_ex, alu_result_mem;
    wire alu_op_y_zero_ex;
    wire mem_we_id, mem_we_ex;
    wire mem_read_id, mem_read_ex, mem_read_mem;
    wire mem_byte_id, mem_byte_ex, mem_byte_mem;
    wire mem_signextend_id, mem_signextend_ex, mem_signextend_mem;
    wire [7:0] mem_read_data_byte_select;
    wire [31:0] mem_read_data_byte_extend;
    wire mem_atomic_id, mem_atomic_ex, mem_atomic_en, mem_sc_mask_id;
    wire mem_sc_id, mem_sc_ex;
    wire alu_overflow;
    wire stall, stall_r;
    wire en_if = ~stall & en;
    wire rst_id = stall & en;

    // Added wires
    wire mem_half_id;
    wire mem_half_ex;
    wire mem_half_mem;

    // Added Regs
    reg resv_valid;

//=====================================================
// IF STAGE
// ====================================================

instruction_fetch if_stage (
        .clk            (clk),
        .rst            (rst),
        .en             (en_if),
        .jump_target    (jump_target_id),
        .jump_reg       (jump_reg_id),
        .jump_branch    (jump_branch_id),
        .jr_pc          (jr_pc_id),
        .pc_id          (pc_id),
        .instr_id       (instr_id[25:0]),
        .pc             (pc_if)
    );


    assign pc = pc_if; // output pc to parent module

    // needed for D stage
    dffare #(32) pc_if2id (.clk(clk), .r(rst), .en(en_if), .d(pc_if), .q(pc_id));

    // Saved ID instruction after a stall
    dffare #(32) instr_sav_dff (.clk(clk), .r(rst), .en(en), .d(instr), .q(instr_sav));
    dffare #(1) stall_f_dff (.clk(clk), .r(rst), .en(en), .d(stall), .q(stall_r));
    assign instr_id = (stall_r) ? instr_sav : instr;

    wire [29:0] instr_number_id = pc_id[31:2]; // useful for viewing waveforms

    // needed for D stage
    dffare #(32) pc_if2id (.clk(clk), .r(rst), .en(en_if), .d(pc_if), .q(pc_id));

    // Saved ID instruction after a stall
    dffare #(32) instr_sav_dff (.clk(clk), .r(rst), .en(en), .d(instr), .q(instr_sav));
    dffare #(1) stall_f_dff (.clk(clk), .r(rst), .en(en), .d(stall), .q(stall_r));
    assign instr_id = (stall_r) ? instr_sav : instr;

    wire [29:0] instr_number_id = pc_id[31:2]; // useful for viewing waveforms

//=====================================================
// ID STAGE
// ====================================================

decode d_stage (
        // inputs
        .pc                 (pc_id),
        .instr              (instr_id),
        .rs_data_in         (rs_data_id),
        .rt_data_in         (rt_data_id),

        .reg_write_addr     (reg_write_addr_id),
        .jump_branch        (jump_branch_id),
        .jump_target        (jump_target_id),
        .jump_reg           (jump_reg_id),
        .jr_pc              (jr_pc_id),
        .alu_opcode         (alu_opcode_id),
        .alu_op_x           (alu_op_x_id),
        .alu_op_y           (alu_op_y_id),
        .mem_we             (mem_we_id),
        .mem_write_data     (mem_write_data_id),
        .mem_read           (mem_read_id),
        .mem_byte           (mem_byte_id),
        .mem_signextend     (mem_signextend_id),
        .reg_we             (reg_we_id),
        .movn               (movn_id),
        .movz               (movz_id),
        .rs_addr            (rs_addr_id),
        .rt_addr            (rt_addr_id),
        .atomic_id          (mem_atomic_id),
        .atomic_ex          (mem_atomic_ex),
        .mem_sc_mask_id     (mem_sc_mask_id),
        .mem_sc_id          (mem_sc_id),
        .stall              (stall),

        // inputs for forwarding/stalling from X
        .reg_we_ex          (reg_we_ex),
        .reg_write_addr_ex  (reg_write_addr_ex),
        .alu_result_ex      (alu_result_ex),
        .mem_read_ex        (mem_read_ex),

        // inputs for forwarding/stalling from M
        .reg_we_mem         (reg_we_mem),
        .reg_write_addr_mem (reg_write_addr_mem),
        .reg_write_data_mem (reg_write_data_mem),

        // Added wires
        .mem_half(mem_half_id)
    );

// Load-linked / Store-conditional
    dffarre       atomic  (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(mem_atomic_id), .q(mem_atomic_ex));
    dffarre       sc      (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(mem_sc_id), .q(mem_sc_ex));

    // needed for X stage
    dffarre #(32) alu_op_x_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(alu_op_x_id), .q(alu_op_x_ex));
    dffarre #(32) alu_op_y_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(alu_op_y_id), .q(alu_op_y_ex));
    dffarre #(4)  alu_opcode_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(alu_opcode_id), .q(alu_opcode_ex));
    dffarre       movn (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(movn_id), .q(movn_ex));
    dffarre       movz (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(movz_id), .q(movz_ex));

    // Added piepline ID 2 EX
    dffarre mem_half_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(mem_half_id), .q(mem_half_ex));
    dffarre mem_read_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(mem_read_id), .q(mem_read_ex));

    // needed for M stage
    dffarre #(32) mem_write_data_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(mem_write_data_id), .q(mem_write_data_ex));
    dffarre mem_we_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(mem_we_id), .q(mem_we_ex));
    //dffarre mem_read_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(1'b0), .q());
    dffarre mem_byte_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(mem_byte_id), .q(mem_byte_ex));
    dffarre mem_signextend_id2ex (.clk(clk), .r(rst_id), .en(en), .d(mem_signextend_id), .q(mem_signextend_ex));

    // needed for W stage
    dffarre #(5) reg_write_addr_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(reg_write_addr_id), .q(reg_write_addr_ex));
    dffarre reg_we_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(reg_we_id), .q(reg_we_cond_ex));

    assign reg_we_ex = reg_we_cond_ex & |{movz_ex & alu_op_y_zero_ex, movn_ex & ~alu_op_y_zero_ex, ~movz_ex & ~movn_ex};

//=====================================================
// EX STAGE
// ====================================================
    
        alu x_stage (
        .alu_opcode     (alu_opcode_ex),
        .alu_op_x       (alu_op_x_ex),
        .alu_op_y       (alu_op_y_ex),

        .alu_result     (alu_result_ex),
        .alu_op_y_zero  (alu_op_y_zero_ex),
        .alu_overflow   (alu_overflow) // maybe do something creative with this
    );

    // needed for M stage
    wire [31:0] sc_result = {{31{1'b0}},(mem_sc_ex & mem_we_ex)};

    // Maybe need to check if we violated any ll/sc pair

    dffare #(32) alu_result_ex2mem (.clk(clk), .r(rst), .en(en), .d(alu_result_ex), .q(alu_result_mem));
    //dffare mem_read_ex2mem (.clk(clk), .r(rst), .en(en), .d(1'b0), .q());
    dffare mem_byte_ex2mem (.clk(clk), .r(rst), .en(en), .d(mem_byte_ex), .q(mem_byte_mem));
    dffare mem_signextend_ex2mem (.clk(clk), .r(rst), .en(en), .d(mem_signextend_ex), .q(mem_signextend_mem));

    // Added pipeline EX 2 MEM
    dffare mem_half_ex2mem (.clk(clk), .r(rst), .en(en), .d(mem_half_ex), .q(mem_half_mem));
    dffare mem_read_ex2mem (.clk(clk), .r(rst), .en(en), .d(mem_read_ex), .q(mem_read_mem));

    // needed for W stage
    dffare #(5) reg_write_addr_ex2mem (.clk(clk), .r(rst), .en(en), .d(reg_write_addr_ex), .q(reg_write_addr_mem));
    dffare reg_we_ex2mem (.clk(clk), .r(rst), .en(en), .d(reg_we_ex), .q(reg_we_mem));

    // LL & SC Handling 

    wire is_sc_ex = mem_sc_ex;
    wire is_non_sc_store_ex = mem_we_ex & ~is_sc_ex;
    wire sc_success_ex = is_sc_ex & resv_valid;
    wire mem_we_ex_final = mem_we_ex & (~is_sc_ex | sc_success_ex);
    wire is_ll_ex = mem_atomic_ex; // LL in EX

    always @(posedge clk) begin
            if (rst) begin
                    resv_valid <= 1'b0;

            end else if (en) begin
                    //kill reservation on any non-SC store
                    if (is_non_sc_store_ex)
                            resv_valid <= 1'b0;
                    if (is_sc_ex)
                            resv_valid <= 1'b0;
                    if (is_ll_ex) begin
                            resv_valid <= 1'b1;
                 end
              end
    end
    // Added for sh and lh instructions 
    wire [1:0] a = alu_result_ex[1:0];
    wire sw = mem_we_ex_final & ~mem_byte_ex & ~mem_half_ex;
    wire sb = mem_we_ex_final & mem_byte_ex;
    wire sh = mem_we_ex_final & mem_half_ex;
    wire sh_ok = sh & ~a[0];

    //assign mem_read_ex = 1'b0;
    //assign mem_read_mem = 1'b0;
    assign mem_read_en = mem_read_ex;
    assign mem_write_en = sw ? 4'b1111 : sb ? (4'b1000 >> a) : sh_ok ? (a[1] ? 4'b0011 : 4'b1100) : 4'b0000;
    wire [31:0] sb_data = {4{mem_write_data_ex[7:0]}};
    wire [31:0] sh_data = (alu_result_ex[1] == 1'b0) ?
            {mem_write_data_ex[15:0], 16'h0000} :
            {16'h0000, mem_write_data_ex[15:0]};
    assign mem_write_data = mem_byte_ex ? sb_data : mem_half_ex ? sh_data : mem_write_data_ex;

    assign mem_addr = alu_result_ex;

    // Added for LL / SC Handling, sc write back
    wire [31:0] alu_sc_result_ex = is_sc_ex ? {31'b0, sc_success_ex} : alu_result_ex;
   // assign mem_write_data = (mem_byte_ex) ? {4{mem_write_data_ex[7:0]}} : mem_half_ex ? {2{mem_write_data_ex[15:0]}} : mem_write_data_ex;
    /=====================================================
// MEM STAGE
// ====================================================
    // assign mem_write_data = (mem_byte_ex) ? {4{mem_write_data_ex[7:0]}} : mem_half_ex ? {2{mem_write_data_ex[15:0]}} : mem_write_data_ex;
    assign mem_read_data_byte_select =  (alu_result_mem[1:0] == 2'b00) ? mem_read_data[31:24] :
                                       ((alu_result_mem[1:0] == 2'b01) ? mem_read_data[23:16] :
                                       ((alu_result_mem[1:0] == 2'b10) ? mem_read_data[15:8] : mem_read_data[7:0]));
    assign mem_read_data_byte_extend = {{24{mem_signextend_mem & mem_read_data_byte_select[7]}}, mem_read_data_byte_select};
    wire [15:0] mem_read_data_half_select = (alu_result_mem[1] == 1'b0) ? mem_read_data[31:16] : mem_read_data[15:0];
    wire [31:0] mem_read_data_half_extend = {{16{mem_signextend_mem & mem_read_data_half_select[15]}}, mem_read_data_half_select};

    assign mem_out = (mem_byte_mem) ? mem_read_data_byte_extend : mem_half_mem ? mem_read_data_half_extend : mem_read_data;
    assign reg_write_data_mem = mem_read_mem ? mem_out : alu_result_mem;

    // needed for W stage
    dffare #(32) reg_write_data_mem2wb (.clk(clk), .r(rst), .en(en), .d(reg_write_data_mem), .q(reg_write_data_wb));
    dffare #(5) reg_write_addr_mem2wb (.clk(clk), .r(rst), .en(en), .d(reg_write_addr_mem), .q(reg_write_addr_wb));
    dffare reg_we_mem2wb (.clk(clk), .r(rst), .en(en), .d(reg_we_mem), .q(reg_we_wb));

//=====================================================
// WB STAGE
// ====================================================

    regfile w_stage (
        .clk            (clk),
        .en             (en),
        .reg_write_data (reg_write_data_wb),
        .reg_write_addr (reg_write_addr_wb),
        .reg_we         (reg_we_wb),
        .rs_addr        (rs_addr_id),
        .rt_addr        (rt_addr_id),

        .rs_data        (rs_data_id),
        .rt_data        (rt_data_id)
    );

endmodule
                                                                                          261,7         Bot
                                                                                                        
                                                                                          261,7         Bot
