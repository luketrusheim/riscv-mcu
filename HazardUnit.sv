`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/03/2024 11:24:08 PM
// Design Name: 
// Module Name: HazardUnit
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

module HazardUnit(
    input logic [4:0] de_rs1,
    input logic [4:0] de_rs2,
    input logic de_rs1_used,
    input logic de_rs2_used,
    
    input logic [4:0] de_ex_rs1, 
    input logic [4:0] de_ex_rs2,
    input logic de_ex_rs1_used,
    input logic de_ex_rs2_used,
    input logic [4:0] de_ex_rd, 
    input logic de_ex_memRead2,
    
    input logic [4:0] ex_mem_rd,
    input logic ex_mem_RF_WE,
    input logic ex_mem_control_haz,
    
    input logic mem_wb_RF_WE,
    input logic [4:0] mem_wb_rd,
    input logic [2:0] PC_SEL,
    
    output logic [1:0] fselA,
    output logic [1:0] fselB,
    output logic control_haz,
    output logic load_hazard,
    output logic flush,
    output logic hazard_stall
//    output logic load_hazard_inputA_sel,
//    output logic load_hazard_inputB_sel
    );
    
    
    always_comb begin
        fselA = 2'b0;
        fselB = 2'b0;
        control_haz = 1'b0;
        load_hazard = 1'b0;
        flush = 1'b0;
        hazard_stall = 1'b0;

        if(ex_mem_RF_WE && (ex_mem_rd != 0) && (ex_mem_rd == de_ex_rs1) && de_ex_rs1_used)
            fselA = 2'b01;
            
        if(ex_mem_RF_WE && (ex_mem_rd != 0) && (ex_mem_rd == de_ex_rs2) && de_ex_rs2_used)
            fselB = 2'b01;
        
        if(mem_wb_RF_WE && (mem_wb_rd != 0) && !(ex_mem_RF_WE && (ex_mem_rd != 0) && (ex_mem_rd == de_ex_rs1) && de_ex_rs1_used) && (mem_wb_rd == de_ex_rs1) && de_ex_rs1_used)
            fselA = 2'b10;
            
        if(mem_wb_RF_WE && (mem_wb_rd != 0) && !(ex_mem_RF_WE && (ex_mem_rd != 0) && (ex_mem_rd == de_ex_rs2) && de_ex_rs2_used) && (mem_wb_rd == de_ex_rs2) && de_ex_rs2_used)
            fselB = 2'b10;
          
          //detecting load use hazard
          // if de_ex instr. is reading from mem, rd is not 0, and rd is the same as rs1 or rs2 of following instr.
        if (
        de_ex_memRead2 && (de_ex_rd != 0) && ((de_ex_rd == de_rs1 && de_rs1_used) || (de_ex_rd == de_rs2 && de_rs2_used)))
        begin
            load_hazard = 1;
        end
        
         // if load_hazard (RAW 1 instr. with a load), flush instruction in decode and stall 
        if (load_hazard) begin
            hazard_stall = 1;
            flush = 1;
        end
        
        // detecting control hazard
        if(PC_SEL != 0) begin
            control_haz = 1'b1;
            end

        // flushing
        if(control_haz || ex_mem_control_haz) begin
            flush = 1'b1;
            end
      end
            
        
            
        
        
endmodule
