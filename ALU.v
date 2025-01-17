`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Jaxon Sprogis and Luke Trusheim
// 
// Create Date: 01/29/2024 12:42:18 PM
// Design Name: Arithmetic Logic Unit
// Module Name: ALU
// Project Name: Experiment 3
// Target Devices: 
// Tool Versions: 
// Description: The ALU does all the bit crunching for the
// for the main 11 instructions. Takes two operands and a
// selection for with instruction
// 
// Revision 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////

module ALU(OP_1, OP_2, ALU_FUN, RESULT);
    input [31:0] OP_1;
    input [31:0] OP_2;
    input [3:0] ALU_FUN;
    output reg [31:0] RESULT;
    
    
    parameter [3:0] ADD = 4'b0000, 
    SUB = 4'b1000, 
    OR = 4'b0110, 
    AND = 4'b0111,
    XOR = 4'b0100, 
    SRL = 4'b0101,
    SLL = 4'b0001,
    SRA = 4'b1101,
    SLT = 4'b0010,
    SLTU = 4'b0011,
    LUI = 4'b1001;
    
    always @ (*)
    begin
        case(ALU_FUN)
            ADD: 
                RESULT = OP_1 + OP_2; //adds OP1 to OP2
   
            SUB:  
                RESULT = OP_1 - OP_2; //subtracts OP2 from OP1
            
            OR:
                RESULT = OP_1 | OP_2;  //ORS OP1 and OP2
            
            AND:
                RESULT = OP_1 & OP_2; //ANDS OP1 and OP2
            
            XOR:
                RESULT = OP_1 ^ OP_2; //XOR OP1 and OP2
            
            SRL:
                RESULT = OP_1 >> OP_2[4:0]; // logic shift right OP1 by OP2[4:0]
            
            SLL:
                RESULT = OP_1 << OP_2[4:0];  // logic shift left OP1 by OP2[4:0]
            
            SRA:
                RESULT = $signed(OP_1) >>> OP_2[4:0]; // artimetic shift right OP1 by OP2[4:0]
            
            SLT:
                RESULT = $signed(OP_1) < $signed(OP_2); //set if OP1 is less than OP2
            
            SLTU:
                RESULT = OP_1 < OP_2; //set if OP1 is less than OP2 unsigned
            
            LUI:
                RESULT = OP_1; //copies OP1
            
            
            
    default: RESULT = 32'hDEADBEEF;   
    endcase
    end
endmodule
