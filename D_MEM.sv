`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/02/2024 07:26:44 PM
// Design Name: 
// Module Name: D_MEM
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


module D_MEM(
    input MEM_CLK,
    input logic [31:0] MEM_ADDR2,     //Data Memory Port
    input MEM_WRITE2,
    input MEM_READ2,
    input write_block,
    
    input [31:0] IO_IN,
    output logic IO_WR,
    output logic IO_RD,
    
    input logic [31:0] w0_in,
    input logic [31:0] w1_in,
    input logic [31:0] w2_in,
    input logic [31:0] w3_in,
    
    output logic [31:0] MEM_DOUT2,
    output logic [31:0] w0_out,
    output logic [31:0] w1_out,
    output logic [31:0] w2_out,
    output logic [31:0] w3_out
    
    
    );
    
    parameter ACTUAL_WIDTH=14;  //32KB     16K x 32
    
    wire [ACTUAL_WIDTH-1:0] memAddr2;
    logic memWrite2;    // change this name
    logic [31:0] memOut2;
    logic [31:0] temp_w0_out;
    logic [31:0] ioIn_buffer;
    logic [31:0] saved_mem_addr2;
    
    assign memAddr2 = MEM_ADDR2 [ACTUAL_WIDTH+1:2];
    
    
    (* ram_style="block" *) logic [31:0] memory[0:2**ACTUAL_WIDTH-1];
    
//    initial $readmemh("test_all_lab1.mem", memory, 0, 2**ACTUAL_WIDTH-1);
    
    always_ff @(posedge MEM_CLK) begin
        //PORT 2  //Data
        if(memWrite2)
            if (write_block) begin
                memory[{memAddr2[13:2],2'b00}] <= w0_in;
                memory[{memAddr2[13:2],2'b01}] <= w1_in;
                memory[{memAddr2[13:2],2'b10}] <= w2_in;
                memory[{memAddr2[13:2],2'b11}] <= w3_in;
            end else begin
                memory[memAddr2] <= w0_in;
            end
        if(MEM_READ2) begin
            temp_w0_out <= memory[{memAddr2[13:2],2'b00}];
            w1_out <= memory[{memAddr2[13:2],2'b01}];
            w2_out <= memory[{memAddr2[13:2],2'b10}];
            w3_out <= memory[{memAddr2[13:2],2'b11}];
        end
        
        saved_mem_addr2 <= MEM_ADDR2;
    end
    
    always_ff @(posedge MEM_CLK) begin
        if(MEM_READ2) begin
            ioIn_buffer<=IO_IN; 
        end
    end
   
    always_comb begin
        IO_RD = 0;
        if(saved_mem_addr2 >= 32'h11000000) begin      
            w0_out = ioIn_buffer;  
            IO_RD = 1;
        end else 
            w0_out = temp_w0_out;   
    end 

    always_comb begin
        IO_WR=0;
        if(MEM_ADDR2 >= 32'h11000000)
        begin       
            if(MEM_WRITE2) IO_WR = 1;
            memWrite2=0; 
        end
        else begin 
            memWrite2=MEM_WRITE2;
        end    
    end 
    
endmodule
