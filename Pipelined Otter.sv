`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/04/2019 04:32:12 PM
// Design Name: 
// Module Name: PIPELINED_OTTER_CPU
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

  typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BRANCH   = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           OP_IMM   = 7'b0010011,
           OP_RG3   = 7'b0110011,
           SYSTEM   = 7'b1110011
 } opcode_t;
        
typedef struct packed{
    opcode_t opcode;
    logic [2:0] func3;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr;
    logic [4:0] rd_addr;
    logic rs1_used;
    logic rs2_used;
    logic rd_used;
    logic [3:0] alu_fun;
    logic memWrite;
    logic memRead2;
    logic [31:0]MEM_ADDR2;
    logic [31:0] DOUT2;
    logic RF_WE;
    logic [1:0] RF_SEL;
    logic [2:0] mem_type;  //sign, size
    logic [31:0] pc;
    logic [31:0]rs1;
    logic [31:0]rs2;
    logic [31:0]ALU_srcA;
    logic [31:0]ALU_srcB;
    logic [1:0]srcA_SEL;
    logic [1:0]srcB_SEL;
    logic [31:0]ALU_result;
    logic [31:0]ir;
    logic load_hazard;
    logic control_haz;
    logic flushed;
    logic [31:0]J_TYPE;
    logic [31:0]B_TYPE;
    logic [31:0]I_TYPE;
    logic [31:0]U_TYPE;
    logic [31:0]S_TYPE;
} instr_t;

module OTTER_MCU(input CLK,
                input INTR,
                input RESET,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR 
);           
   
    //PC
    logic [31:0]pc_out;
    logic [31:0]pc_in;
    logic stall;
    logic PC_WE;
    //MEMORY
    //INSTRUCTION MEMORY
    logic [31:0] ir;
    logic [31:0] MEM_ADDR1;
    //DATA MEMORY
    logic [31:0] DOUT2;
    logic dmem_WE;
    logic dmem_RE;
    logic IO_RD;
    
    logic [31:0] dmem_addr;
    logic [31:0] dmem_w0_in;
    logic [31:0] dmem_w0_out;
    logic [31:0] dmem_w1_out;
    logic [31:0] dmem_w2_out;
    logic [31:0] dmem_w3_out;
    //IMMEDGEN
    logic [31:0]J_TYPE;
    logic [31:0]I_TYPE;
    logic [31:0]B_TYPE;
    logic [31:0]U_TYPE;
    logic [31:0]S_TYPE;
    //BRANCHADDR
    logic [31:0]jal;
    logic [31:0]branch;
    logic [31:0]jalr;
    //CUFSM
    logic RF_WE;
    logic memWE2;
    logic memRDEN1;
    logic memRDEN2;
    logic reset;
//    logic int_taken;
//    logic csr_WE;
//    logic mret_exec;
    //CUDCDR
    logic [3:0]ALU_FUN;
    logic [1:0]srcA_SEL;
    logic [1:0]srcB_SEL;
    logic [2:0]PC_SEL;
    logic [1:0]RF_SEL;
    //REGFILE 
    logic [31:0]w_data;
    logic [31:0]rs1;
    logic [31:0]rs2;
    //ALU 
    logic [31:0] ALU_srcA;
    logic [31:0] ALU_srcB;
    logic [31:0] ALU_result;
    //BRANC_COND_GEN
    logic br_eq;
    logic br_lt;
    logic br_ltu;
    //CSR
//    logic CSRmstatus3;
//    logic [31:0]mepc;
//    logic [31:0]mtvec;
//    logic [31:0]csr_RD;
    //HAZARDUNIT
    logic [1:0] fselA;
    logic [1:0] fselB;
    logic control_haz;
    logic flush;
    logic hazard_stall;
    logic load_hazard;
    logic fMemResult;
    logic rs1_used;
    logic rs2_used;
    //forwardconnections
    logic [31:0] rs1muxout;
    logic [31:0] rs2muxout;
    logic load_hazard_rs1_sel;
    logic load_hazard_rs2_sel;
    // CacheFSM
    logic miss_stall;
    logic update;
    logic [1:0] dmem_WE_sel;
    logic [1:0] dmem_RE_sel;
    //Cache
    logic miss;
    logic hit;
    logic evict;
    logic addr_is_io;
    logic [31:0] evicted_addr;
    logic [31:0] cache_w0_in;
    logic [31:0] cache_w0_out;
    logic [31:0] cache_w1_out;
    logic [31:0] cache_w2_out;
    logic [31:0] cache_w3_out;
    
//      CSR  my_csr (
//        .CLK        (CLK),
//        .RST        (RESET),
//        .MRET_EXEC  (mret_exec),
//        .INT_TAKEN  (int_taken),
//        .ADDR       (ir[31:20]),
//        .PC         (pc_out),
//        .WD         (ALU_result),
//        .WR_EN      (csr_WE),
//        .RD         (csr_RD),
//        .CSR_MEPC   (mepc),
//        .CSR_MTVEC  (mtvec),
//        .CSR_MSTATUS_MIE (CSRmstatus3));  
        
      HazardUnit Hazardunit (
        .de_rs1                     (ir[19:15]),
        .de_rs2                     (ir[24:20]),
        .de_rs1_used                (rs1_used),
        .de_rs2_used                (rs2_used),
        .de_ex_rs1_used             (de_ex_inst.rs1_used),
        .de_ex_rs2_used             (de_ex_inst.rs2_used),
        .de_ex_rs1                  (de_ex_inst.rs1_addr),
        .de_ex_rs2                  (de_ex_inst.rs2_addr),
        .de_ex_rd                   (de_ex_inst.rd_addr),
        .ex_mem_rd                  (ex_mem_inst.rd_addr),
        .mem_wb_rd                  (mem_wb_inst.rd_addr),
        .PC_SEL                     (PC_SEL),
        .ex_mem_RF_WE               (ex_mem_inst.RF_WE),
        .de_ex_memRead2             (de_ex_inst.memRead2),
        .ex_mem_control_haz         (ex_mem_inst.control_haz),
        .mem_wb_RF_WE               (mem_wb_inst.RF_WE),
        .load_hazard                (load_hazard),
        .fselA                      (fselA),
        .fselB                      (fselB),
        .control_haz                (control_haz),
        .flush                      (flush),
        .hazard_stall               (hazard_stall));
        
       
//==== Instruction Fetch ===========================================

     logic control;
     instr_t de_inst;
     
     assign memRDEN1 = 1'b1; 	//Fetch new instruction every cycle
     
     mux_8t1_nb #(.n(32)) PCMUX (
        .SEL   (PC_SEL), 
        .D0    (pc_out + 3'b100), // PC output + 4
        .D1    (jalr),  //jalr from BAG
        .D2    (branch),  //branch from BAG
        .D3    (jal), //jal from BAG
        .D4    (32'b0), //ISR vector address
        .D5    (32'b0),   //resume address
        .D6    (32'b0), //grounded
        .D7    (32'b0),  //grounded
        .D_OUT (pc_in) ); //32 bit mux output into PC
     
     reg_nb_sclr #(.n(32)) PC (
      .data_in  (pc_in), //input from the mux
      .ld       (PC_WE), //synchronus write enable
      .clk      (CLK), 
      .clr      (RESET), //synchronus reset/clr
      .data_out (pc_out));  //output into module ALU
      
     assign pc_stall = hazard_stall || miss_stall;
     assign PC_WE = !pc_stall;

     always_ff @(posedge CLK) begin
            if (!pc_stall)
                de_inst.pc <= pc_out;
            else
                de_inst.pc <= de_inst.pc;
            
//            if (load_hazard)
//                de_inst.pc <= pc_out - 3'b100;

     end
    
//==== Instruction Decode ===========================================

    instr_t de_ex_inst;
   
    opcode_t OPCODE;
    assign OPCODE = opcode_t'(ir[6:0]);
    
    CU_DCDR CONTROLDCDR(
        .opcode    (ir[6:0]),    //opcode 
        .func7     (ir[30]),    //func7 code
        .func3     (ir[14:12]),     //func3 code
        .int_taken (0),
        .ALU_FUN   (ALU_FUN), //select for ALU
        .flush     (flush),
        .srcA_SEL  (srcA_SEL), //alu SRCA sel
        .srcB_SEL  (srcB_SEL),  //alu SRCA sel
        .RF_SEL    (RF_SEL), //reg file mux sel   
        .RF_WE     (RF_WE),
        .memRDEN2  (memRDEN2), 
        .memWE2    (memWE2));
                             
     //IMMED_GEN
    assign I_TYPE = {
        {21{ir[31]}}, 
        ir[30:25],
        ir[24:20]};
    assign S_TYPE = {
        {21{ir[31]}}, 
        ir[30:25],
        ir[11:7]};
    assign B_TYPE = {
        {20{ir[31]}}, 
        ir[7], 
        ir[30:25],  
        ir[11:8],
        1'b0};
    assign U_TYPE = {
        ir[31:12], 
        12'b0};
    assign J_TYPE = {
        {12{ir[31]}},
        ir[19:12],
        ir[20],
        ir[30:21],
        1'b0};           
   
   assign rs1_used =       ir[19:15] != 0
                                        && OPCODE != LUI
                                        && OPCODE != AUIPC
                                        && OPCODE != JAL;
                                        
   assign rs2_used =       ir[24:20] != 0 &&
                                        (OPCODE == BRANCH
                                        || OPCODE == STORE
                                        || OPCODE == OP_RG3);
        
   assign MEM_ADDR1 = pc_stall ? de_inst.pc[31:0] : pc_out[31:0];            
    
    logic err;
    
    OTTER_mem_byte otter_memory(
        .MEM_CLK (CLK),
        .MEM_READ1 (memRDEN1),
        .MEM_READ2 (1'b0),
        .MEM_WRITE2 (1'b0),
        .MEM_ADDR1 (MEM_ADDR1), // 14-bit signal
        .MEM_ADDR2 (32'hDEADBEEF),
        .MEM_DIN2 (32'hDEADBEEF),
        .MEM_SIZE (ex_mem_inst.mem_type[1:0]),
        .MEM_SIGN (ex_mem_inst.mem_type[2]),
        .ERR      (err),
        .IO_IN (IOBUS_IN),
        .IO_WR (IOBUS_WR), 
        .MEM_DOUT1 (ir), // 32-bit signal
        .MEM_DOUT2 ());  
        
            
    always_ff @(posedge CLK) begin
        if (miss_stall) begin
            de_ex_inst <= de_ex_inst;
        end else begin
            de_ex_inst <= de_inst;
            de_ex_inst.opcode <= OPCODE;
            if (flush) begin
                de_ex_inst.RF_WE <= 0;
                de_ex_inst.memWrite <= 0;
                de_ex_inst.flushed <= 1;
                de_ex_inst.ir <= 0; 
                de_ex_inst.rs1_addr <= 0;
                de_ex_inst.rs2_addr <= 0;
                de_ex_inst.rd_addr <= 0;
                de_ex_inst.func3 <= 0;
                de_ex_inst.alu_fun <= 0;
                de_ex_inst.memRead2 <= 0;
                de_ex_inst.RF_SEL <= 0;
                de_ex_inst.rs1 <= 0;
                de_ex_inst.rs2 <= 0;
                de_ex_inst.ALU_srcA <= 0;
                de_ex_inst.ALU_srcB <= 0;
                de_ex_inst.srcA_SEL <= 0;
                de_ex_inst.srcB_SEL <= 0;
                de_ex_inst.rs1_used <= 0;
                                            
                de_ex_inst.rs2_used <= 0;
                   
                de_ex_inst.rd_used <=  0;
                de_ex_inst.mem_type <= 0;
                de_ex_inst.J_TYPE <= 0;
                de_ex_inst.B_TYPE <= 0;
                de_ex_inst.I_TYPE <= 0;
                de_ex_inst.U_TYPE <= 0;
                de_ex_inst.S_TYPE <= 0;
            end else begin
                de_ex_inst.RF_WE <= RF_WE;
                de_ex_inst.memWrite <= memWE2;
                de_ex_inst.flushed <= 0;
                de_ex_inst.ir <= ir; 
                de_ex_inst.rs1_addr <= ir[19:15];
                de_ex_inst.rs2_addr <= ir[24:20];
                de_ex_inst.rd_addr <= ir[11:7];
                de_ex_inst.func3 <= ir[14:12];
                de_ex_inst.alu_fun <= ALU_FUN;
                de_ex_inst.memRead2 <= memRDEN2;
                de_ex_inst.RF_SEL <= RF_SEL;
                de_ex_inst.rs1 <= rs1;
                de_ex_inst.rs2 <= rs2;
                de_ex_inst.ALU_srcA <= ALU_srcA;
                de_ex_inst.ALU_srcB <= ALU_srcB;
                de_ex_inst.srcA_SEL <= srcA_SEL;
                de_ex_inst.srcB_SEL <= srcB_SEL;
                de_ex_inst.rs1_used <= rs1_used;
                de_ex_inst.rs2_used <= rs2_used;
                   
                de_ex_inst.rd_used <=    ir[11:7] != 0
                                            && OPCODE != BRANCH
                                            && OPCODE != STORE;
                de_ex_inst.mem_type <= ir[14:12];
                de_ex_inst.J_TYPE <= J_TYPE;
                de_ex_inst.B_TYPE <= B_TYPE;
                de_ex_inst.I_TYPE <= I_TYPE;
                de_ex_inst.U_TYPE <= U_TYPE;
                de_ex_inst.S_TYPE <= S_TYPE;
            end      
        end          
    end
     
    
	
	
//==== Execute ======================================================

     instr_t ex_mem_inst;
     
     mux_4t1_nb #(.n(32)) rs1mux(
        .SEL   (fselA),  //fsel
        .D0    (de_ex_inst.rs1), //register 1
        .D1    (ex_mem_inst.ALU_result), // forwared alu result
        .D2    (w_data),   //CHECK THIS IF ERROR ___________________________
        .D3    (0),   //data memory out
        .D_OUT (rs1muxout));     //output        
        
    mux_4t1_nb #(.n(32)) rs2mux(
        .SEL   (fselB),  //fsel
        .D0    (de_ex_inst.rs2), //register 1
        .D1    (ex_mem_inst.ALU_result), // forwared alu result
        .D2    (w_data),   //CHECK THIS IF ERROR ___________________________
        .D3    (0),   //data memory out
        .D_OUT (rs2muxout));     //output           
     
     mux_4t1_nb  #(.n(32))SRCA(
        .SEL   (de_ex_inst.srcA_SEL),  //sel
        .D0    (rs1muxout), //register 1
        .D1    (de_ex_inst.U_TYPE), // utype imm
        .D2    (~rs1muxout),   // HEYYYYYYYYYYYYYYYYYYYYYYYYYY LUKKKEEEEEEEEEEEEEEE
        .D3    (32'b0),   //grounded
        .D_OUT (ALU_srcA));     //output   
        
    mux_4t1_nb  #(.n(32)) SRCB  (
        .SEL   (de_ex_inst.srcB_SEL), //sel
        .D0    (rs2muxout), //register 2
        .D1    (de_ex_inst.I_TYPE),  //itype imm
        .D2    (de_ex_inst.S_TYPE),  //stype imm
        .D3    (de_ex_inst.pc), //pc out
        .D_OUT (ALU_srcB)); //output
     
     ALU myALU  (
        .OP_1  (ALU_srcA), //operand 1
        .OP_2  (ALU_srcB), //operand 2
        .ALU_FUN (de_ex_inst.alu_fun), //operation choice
        .RESULT  (ALU_result)); //result
        
    //BRANCH ADDRESS GENERATOR
        assign jal = (de_ex_inst.pc) + de_ex_inst.J_TYPE;
        assign branch = (de_ex_inst.pc) + de_ex_inst.B_TYPE;
        assign jalr = rs1muxout + de_ex_inst.I_TYPE;
    
    //BRANCH_COND_GEN
    BRANCHCOND branchcomp (
        .int_taken (1'b0),
        .flushed (de_ex_inst.flushed),
        .rs1     (rs1muxout),
        .rs2     (rs2muxout),
        .opcode  (de_ex_inst.opcode),
        .func3   (de_ex_inst.func3),
        .PC_SEL  (PC_SEL));

    always_ff @(posedge CLK) begin
        if (miss_stall) begin
            ex_mem_inst <= ex_mem_inst;
        end else begin
            ex_mem_inst <= de_ex_inst; 
            ex_mem_inst.rs1 <= rs1muxout;
            ex_mem_inst.rs2 <= rs2muxout;
            ex_mem_inst.control_haz <= control_haz;
            ex_mem_inst.load_hazard <= load_hazard;          
            ex_mem_inst.ALU_result <= ALU_result;
            ex_mem_inst.MEM_ADDR2 <= ALU_result;
        end
    end


//==== Memory ======================================================
     
    instr_t mem_wb_inst;
    assign IOBUS_ADDR = ex_mem_inst.ALU_result;
    assign IOBUS_OUT = ex_mem_inst.rs2;
    
       D_MEM my_d_mem(
        .MEM_CLK        (CLK),
        .MEM_READ2      (dmem_RE),
        .MEM_WRITE2     (dmem_WE),
        .MEM_ADDR2      (dmem_addr),
        .IO_IN          (IOBUS_IN),
        .IO_WR          (IOBUS_WR),
        .IO_RD          (IO_RD),
        .w0_in          (dmem_w0_in),
        .w1_in          (cache_w1_out),
        .w2_in          (cache_w2_out),
        .w3_in          (cache_w3_out),
        .w0_out         (dmem_w0_out),
        .w1_out         (dmem_w1_out),
        .w2_out         (dmem_w2_out),
        .w3_out         (dmem_w3_out),
        .MEM_DOUT2      (MEM_DOUT2));  
        
    CacheFSM my_cache_fsm(
        .hit        (hit),
        .miss       (miss),
        .evict      (evict),
        .addr_is_io (addr_is_io),
        .MEM_READ2  (ex_mem_inst.memRead2),
        .MEM_WRITE2 (ex_mem_inst.memWrite),
        .CLK        (CLK),
        .RST        (RESET),
        .update     (update),
        .miss_stall (miss_stall),
        .dmem_WE_sel(dmem_WE_sel),
        .dmem_RE_sel(dmem_RE_sel)
    );
    
    assign cache_w0_in = update ? dmem_w0_out : ex_mem_inst.rs2;
    
    assign dmem_w0_in = evict ? cache_w0_out : ex_mem_inst.rs2;
    assign dmem_addr = evict ? evicted_addr : ex_mem_inst.MEM_ADDR2;
    assign write_block = evict;
   
   mux_4t1_nb #(.n(1)) dmem_WE_mux(
        .SEL   (dmem_WE_sel),    
        .D0    (1'b0),                  // WE = 0
        .D1    (1'b1),                  // WE = 1
        .D2    (ex_mem_inst.memWrite),  // WE = WE from pipeline
        .D3    (),
        .D_OUT (dmem_WE));          
   
   mux_4t1_nb #(.n(1)) dmem_RE_mux(
        .SEL   (dmem_RE_sel),    
        .D0    (1'b0),                  // RE = 0
        .D1    (1'b1),                  // RE = 1
        .D2    (ex_mem_inst.memRead2),  // RE = RE from pipeline
        .D3    (),
        .D_OUT (dmem_RE));          
    
    Cache my_cache (
    .addr       (ex_mem_inst.MEM_ADDR2),
    .CLK        (CLK),
    .RESET      (RESET),
    .update     (update),
    .MEM_WRITE2 (ex_mem_inst.memWrite),
    .MEM_READ2  (ex_mem_inst.memRead2),
    .w0_in      (cache_w0_in),
    .w1_in      (dmem_w1_out),
    .w2_in      (dmem_w2_out),
    .w3_in      (dmem_w3_out),
    .w0_out     (cache_w0_out),
    .w1_out     (cache_w1_out),
    .w2_out     (cache_w2_out),
    .w3_out     (cache_w3_out),
    .dout       (),
    .hit        (hit),
    .miss       (miss),
    .evict      (evict),
    .addr_is_io         (addr_is_io),
    .evicted_addr (evicted_addr)
);
    
    // get data from MM if there was a cache miss
    assign DOUT2 = update || IO_RD ? dmem_w0_out : cache_w0_out;

    always_ff @(posedge CLK) begin
        mem_wb_inst <= ex_mem_inst;
    end
     
//==== Write Back ==================================================

     mux_4t1_nb  #(.n(32)) REGMUX  (
        .SEL   (mem_wb_inst.RF_SEL),  //reg file mux
        .D0    (mem_wb_inst.pc + 3'b100), //pc + 4
        .D1    (32'b0), //csr RD
        .D2    (DOUT2), //memory out 2
        .D3    (mem_wb_inst.ALU_result),// result of ALU
        .D_OUT (w_data)); //data out of mux
       
     RegFile REGFILE (
        .w_data (w_data), //data in to regfile
        .clk    (CLK),  
        .en     (mem_wb_inst.RF_WE), //reg file write
        .adr1   (ir[19:15]), //addr 1
        .adr2   (ir[24:20]), //addr 2
        .w_adr  (mem_wb_inst.rd_addr), //w_adr
        .rs1    (rs1),  //register 1
        .rs2    (rs2));   //register 2
            
endmodule
