`timescale 1ns / 1ps
///////////////////////////////////////////////////////////////////////////
// Company: Ratner Surf Designs
// Engineer: James Ratner + Jaxon Sprogis + Luke Trusheim 
// 
// Create Date: 01/29/2019 04:56:13 PM
// Design Name: 
// Module Name: CU_DCDR
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Control decoder for RISCV MCU
// 
// Dependencies:
// 
// Instantiation Template:
//
// CU_DCDR my_cu_dcdr(
//   .br_eq     (xxxx), 
//   .br_lt     (xxxx), 
//   .br_ltu    (xxxx),
//   .opcode    (xxxx),    
//   .func7     (xxxx),    
//   .func3     (xxxx),    
//   .ALU_FUN   (xxxx),
//   .PC_SEL    (xxxx),
//   .srcA_SEL  (xxxx),
//   .srcB_SEL  (xxxx), 
//   .RF_SEL    (xxxx)   );
//
// 
// Revision:
// Revision 1.00 - Created (02-01-2020) - from Paul, Joseph, & Celina
//          1.01 - (02-08-2020) - removed  else's; fixed assignments
//          1.02 - (02-25-2020) - made all assignments blocking
//          1.03 - (05-12-2020) - reduced func7 to one bit
//          1.04 - (05-31-2020) - removed misleading code
//          1.05 - (12-10-2020) - added comments
//          1.06 - (02-11-2021) - fixed formatting issues
//          1.07 - (12-26-2023) - changed signal names
//
// Additional Comments:
// 
///////////////////////////////////////////////////////////////////////////

module CU_DCDR(
   input [6:0] opcode,   //-  ir[6:0]
   input func7,          //-  ir[30]
   input [2:0] func3,    //-  ir[14:12] 
   input int_taken,
   input flush,
   output logic [3:0] ALU_FUN,
   output logic [1:0]srcA_SEL,
   output logic [1:0] srcB_SEL, 
   output logic [1:0] RF_SEL,
   output logic RF_WE,
   output logic memRDEN2,
   output logic memWE2
  );
   
   //- datatypes for RISC-V opcode types
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

   //- datatype for func3Symbols tied to values
// branch enum
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
   
   
typedef enum logic [2:0] {         
        //R-TYPE labels
        ADDSUB = 3'b000,
        SLL = 3'b001,
        SLT = 3'b010,
        SLTU = 3'b011,
        XORfunc = 3'b100,
        SRLSRA = 3'b101,
        ORfunc = 3'b110,
        ANDfunc = 3'b111} func3r_t;
 func3r_t FUNC3rtype; //- define variable of new opcode type
 assign FUNC3rtype = func3r_t'(func3); //- Cast input enum 
 
typedef enum logic [2:0] {         
        //I-TYPE labels
        ADDI = 3'b000,
        SLTI = 3'b010,
        SLTIU = 3'b011,
        ORI = 3'b110,
        XORI = 3'b100,
        ANDI = 3'b111,
        SLLI = 3'b001,
        SRLISRAI = 3'b101} func3i_t;
 func3i_t FUNC3itype; //- define variable of new opcode type
 assign FUNC3itype = func3i_t'(func3); //- Cast input enum 
 
 typedef enum logic [2:0] {        
        MRET = 3'b000,
        CSRRW = 3'b001,
        CSRRS = 3'b010,
        CSRRC = 3'b011
        } func3s_t;    
   func3s_t FUNC3sys; //- define variable of new opcode type
   assign FUNC3sys = func3s_t'(func3); //- Cast input enum 
                     
   always_comb
   begin 
      //- schedule all values to avoid latch
        srcB_SEL = 3'b000;     RF_SEL = 2'b00; 
      srcA_SEL = 2'b0;   ALU_FUN  = 4'b0000; memRDEN2 = 1'b0; RF_WE = 1'b0; memWE2 = 1'b0;
      
      
		
      case(OPCODE)
         LUI:
         begin     //LUI
            ALU_FUN = 4'b1001; 
            srcA_SEL = 2'b01; 
            RF_SEL = 2'b11; 
            RF_WE = 1'b1;
            memWE2 = 1'b0;
            memRDEN2 = 1'b0;
         end
         
         AUIPC:
         begin     //AUIPC
            ALU_FUN = 4'b0000; 
            srcA_SEL = 2'b01; 
            srcB_SEL = 3'b011;
            RF_SEL = 2'b11; 
            RF_WE = 1'b1;
            memWE2 = 1'b0;
            memRDEN2 = 1'b0;
         end
			
         JAL:
         begin    //JAL
		    RF_SEL = 2'b00; 
		    RF_WE = 1'b1;
            memWE2 = 1'b0;
		    memRDEN2 = 1'b0;
		 end
		 
		 JALR:
         begin     //jalr
            RF_SEL = 2'b00; 
            RF_WE = 1'b1;
            memWE2 = 1'b0;
            memRDEN2 = 1'b0;
            
		 end

			
         LOAD: 
         begin     //load
            ALU_FUN = 4'b0000; 
            srcA_SEL = 2'b00; 
            srcB_SEL = 3'b001; 
            RF_SEL = 2'b10; 
            RF_WE = 1'b1;
            memWE2 = 1'b0;
            memRDEN2 = 1'b1;
         end
			
         STORE:
         begin      //store
            ALU_FUN = 4'b0000; 
            srcA_SEL = 2'b00; 
            srcB_SEL = 3'b010; 
            RF_WE = 1'b0;
            memWE2 = 1'b1;
            memRDEN2 = 1'b0;
         end
		
         OP_IMM:
         begin
         srcA_SEL = 2'b00;    //stays same
         srcB_SEL = 3'b001;    //stays same
         RF_WE = 1'b1;
         memWE2 = 1'b0;
         memRDEN2 = 1'b0;
         RF_SEL = 2'b11;    //stays same
            case(FUNC3itype)
               ADDI: // instr: ADDI
               begin
                  ALU_FUN = 4'b0000;
               end
               
               SLTI:   //slti
               begin
                  ALU_FUN = 4'b0010;
               end
               
               SLTIU: //sltiu
               begin
                  ALU_FUN = 4'b0011;
               end
               
               ORI:  //ori
               begin
                  ALU_FUN = 4'b0110;
               end
               
               XORI: //xori
               begin
                  ALU_FUN = 4'b0100;
               end
               
               ANDI:   //andi
               begin
                  ALU_FUN = 4'b0111;
               end
               
               SLLI:   //slli
               begin
                  ALU_FUN = 4'b0001;
               end
               
               SRLISRAI:
               begin
               if(func7)     //SRAI
                  ALU_FUN = 4'b1101;
               else          //SRLI
                  ALU_FUN = 4'b0101;
               end
					
               default: 
               begin
                  ALU_FUN = 4'b0000;
                  srcA_SEL = 2'b00; 
                  srcB_SEL = 3'b000; 
                  RF_SEL = 2'b00; 
                  RF_WE = 1'b0;
                  memWE2 = 1'b0;
                  memRDEN2 = 1'b0;
               end
            endcase
         end
         
         BRANCH:
         begin
         RF_WE = 1'b0;
         memWE2 = 1'b0;
         memRDEN2 = 1'b0;
         end
         
         OP_RG3:
         begin
         srcA_SEL = 2'b00;   //same for all
         srcB_SEL = 3'b000;   //same for all
         RF_WE = 1'b1;
		 memWE2 = 1'b0;
	     memRDEN2 = 1'b0;
         RF_SEL = 2'b11;   //same for all
         ALU_FUN = 4'b0000;   //varies 
         case(FUNC3rtype)
         
               ADDSUB: // instr: ADD, SUB
               begin
               if (func7) begin //func7 1
                  ALU_FUN = 4'b1000;    //subtract
               end
               else begin //func7 0
                  ALU_FUN = 4'b0000;   //add
               end
               end
               
               SLL:   //SLL
               begin
                  ALU_FUN = 4'b0001;
               end
               
               SLT:    //SLT
               begin
                  ALU_FUN = 4'b0010;
               end 
               
               SLTU:    //SLTU
               begin
                  ALU_FUN = 4'b0011;
               end
               
               XORfunc:    //XOR
               begin
                  ALU_FUN = 4'b0100;
               end  
               
               SRLSRA: // instr: SRA , SRL
               begin
               if (func7) begin //func7 1
                  ALU_FUN = 4'b1101;    //sra
               end
               else begin //func7 0
                  ALU_FUN = 4'b0101;   //srl
               end
               end
                  
               ORfunc:     //OR
               begin
                  ALU_FUN = 4'b0110;
               end  
               
               ANDfunc:      //AND
               begin
                  ALU_FUN = 4'b0111;
               end   
               
               default: 
               begin
                  ALU_FUN = 4'b0000;
                  srcA_SEL = 2'b00; 
                  srcB_SEL = 3'b000; 
                  RF_SEL = 2'b00; 
                  RF_WE = 1'b0;
                  memWE2 = 1'b0;
                  memRDEN2 = 1'b0;
               end
            endcase
         end
         
         SYS:
         begin
         case(FUNC3sys)
               MRET:
               begin
//               if (int_taken)
//                  PC_SEL = 3'b100;
//               else
//                  PC_SEL = 3'b101;
               end 
               
               CSRRW:
               begin
               ALU_FUN = 4'b1001;
               srcA_SEL = 2'b00;
               RF_SEL = 2'b01;
               end
               
               CSRRC:
               begin
               ALU_FUN = 4'b0111;
               srcA_SEL = 2'b10;
               srcB_SEL = 3'b100;
               RF_SEL = 2'b01;
               end
               
               CSRRS:
               begin
               ALU_FUN = 4'b0110;
               srcA_SEL = 2'b00;
               srcB_SEL = 3'b100;
               RF_SEL = 2'b01;
               end
               
         default:
         begin
             srcB_SEL = 3'b000; 
             RF_SEL = 2'b00; 
             srcA_SEL = 2'b00; 
             ALU_FUN = 4'b0000;
             RF_WE = 1'b0;
             memWE2 = 1'b0;
             memRDEN2 = 1'b0;
         end
               
         endcase
         end

         default:
         begin
             srcB_SEL = 3'b000; 
             RF_SEL = 2'b00; 
             srcA_SEL = 2'b00; 
             ALU_FUN = 4'b0000;
             RF_WE = 1'b0;
             memWE2 = 1'b0;
             memRDEN2 = 1'b0;
             
         end
      endcase
       
   end
   

endmodule