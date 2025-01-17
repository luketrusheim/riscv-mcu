`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/22/2024 03:03:16 PM
// Design Name: 
// Module Name: BRANCHCOND
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


module BRANCHCOND(
input int_taken,
input flushed,
input [31:0]rs1,
input [31:0]rs2,
input [6:0]opcode,
input [2:0]func3,
output logic [2:0] PC_SEL);

logic br_eq;
logic br_ltu;
logic br_lt;

typedef enum logic [6:0] {
        LUI    = 7'b0110111,
        AUIPC  = 7'b0010111,
        JAL    = 7'b1101111,
        JALR   = 7'b1100111,
        BRANCH = 7'b1100011,
        LOAD   = 7'b0000011,
        STORE  = 7'b0100011,
        OP_IMM = 7'b0010011,
        OP_RG3 = 7'b0110011,
        SYS = 7'b1110011
   } opcode_t;
   opcode_t OPCODE; //- define variable of new opcode type
    
   assign OPCODE = opcode_t'(opcode); //- Cast input enum 

typedef enum logic [2:0] {        
        BEQ = 3'b000,
        BNE = 3'b001,
        BLT = 3'b100,
        BGE = 3'b101,
        BLTU = 3'b110,
        BGEU = 3'b111
        } func3_t;    
   func3_t FUNC3branch; //- define variable of new opcode type
   assign FUNC3branch = func3_t'(func3); //- Cast input enum 

always_comb begin
    PC_SEL = 3'b000; // schedule value to avoid latch
    
    if(rs1 == rs2) //if equal
        br_eq = 1;  
    else begin
        br_eq = 0;
        end   
        
    if (rs1 < rs2) //if less than unsigned
        br_ltu = 1;
    else begin
        br_ltu = 0;
        end   

    if ($signed(rs1) < $signed(rs2)) //if less than signed
        br_lt = 1;
    else begin
        br_lt = 0;
    end
    
    case(OPCODE)
         LUI:
         begin     //LUI
            if (int_taken)
                PC_SEL = 3'b100;
            else
                PC_SEL = 3'b000;
         end
         
         AUIPC:
         begin     //AUIPC
            if (int_taken)
                PC_SEL = 3'b100;
            else
                PC_SEL = 3'b000;
         end
			
         JAL:
         begin    //JAL
		    if (int_taken)
                PC_SEL = 3'b100;
            else if (!flushed)
                PC_SEL = 3'b011;
            else
                PC_SEL = 3'b000;
		 end
		 
		 JALR:
         begin     //jalr
            if (int_taken)
                PC_SEL = 3'b100;
            else if (!flushed)
                PC_SEL = 3'b001;
            else
                PC_SEL = 3'b000;
            
		 end

			
         LOAD: 
         begin     //load
            if (int_taken)
                PC_SEL = 3'b100;
            else
                PC_SEL = 3'b000;
         end
			
         STORE:
         begin      //store
            if (int_taken)
                PC_SEL = 3'b100;
            else
                PC_SEL = 3'b000;
         end
         
         OP_IMM:
         begin
         if (int_taken)
                PC_SEL = 3'b100;
            else
                PC_SEL = 3'b000;
         end
         
         BRANCH:
         begin
         case(FUNC3branch)
               BEQ:   //branch equals
               begin
               if (int_taken)
                PC_SEL = 3'b100;
               else if (br_eq && !flushed)   //if equal
                  PC_SEL = 3'b010;
               else
                  PC_SEL = 3'b000;
               end

               
               BNE:    //branch not equals
               begin
               if (int_taken)
                  PC_SEL = 3'b100;
               else if (br_eq == 0 && !flushed)    //if not equal
                  PC_SEL = 3'b010;
               else
                  PC_SEL = 3'b000;
               end
               
               BLT:      //branch less than
               begin
               if (int_taken)
                  PC_SEL = 3'b100;
               else if (br_lt && !flushed)     //if less than
                  PC_SEL = 3'b010;
               else
                  PC_SEL = 3'b000;
               end
               
               BGE:      //branch greater than
               begin
               if (int_taken)
                  PC_SEL = 3'b100;
               else if (br_lt == 0 && !flushed)    //if greater than
                  PC_SEL = 3'b010;
               else
                  PC_SEL = 3'b000;
               end
               
               BLTU:     //branch less than unsigned
               begin
               if (int_taken)
                  PC_SEL = 3'b100;
               else if (br_ltu && !flushed)   //if less than unsigned
                  PC_SEL = 3'b010;
               else
                  PC_SEL = 3'b000;
               end
               
               BGEU:     //branch greater than unsigned
               begin
               if (int_taken)
                  PC_SEL = 3'b100;
               else if (br_ltu == 0 && !flushed)    //if greater than unsigned
                  PC_SEL = 3'b010;
               else
                  PC_SEL = 3'b000;
               end
               
               default: 
               begin
                  PC_SEL = 3'b000; 
               end
         endcase
         end
         
         OP_RG3:
         begin
         if (int_taken)
                PC_SEL = 3'b100;
            else
                PC_SEL = 3'b000;
         end
         
          default:
         begin
             PC_SEL = 3'b000; 
         end
      endcase
end


        
endmodule
