`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/02/2024 07:14:29 PM
// Design Name: 
// Module Name: CacheFSM
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


module CacheFSM(
    input hit, input miss, input evict, input addr_is_io, input MEM_READ2, input MEM_WRITE2, input CLK, input RST, 
    output logic [1:0] dmem_WE_sel, output logic [1:0] dmem_RE_sel,
    output logic update, output logic miss_stall
    );
    typedef enum logic [1:0] {
    ST_READ_CACHE,
    ST_UPDATE_CACHE,
    ST_WRITE_MISS
    } state_type;
    
    state_type PS, NS;
    
    always_ff @(posedge CLK) begin
        if(RST == 1)
            PS <= ST_READ_CACHE;
        else
            PS <= NS;
    end
        
    always_comb begin
        update = 1'b0;
        miss_stall = 0;
        dmem_WE_sel = 2'b00;
        dmem_RE_sel = 2'b00;
        
        case (PS)
            ST_READ_CACHE: begin
                update = 1'b0;
                if (MEM_READ2 || MEM_WRITE2) begin
                    if (hit) begin
                        NS = ST_READ_CACHE;
                    end else if (miss) begin    // stall and allow memory to read or write according to instruction
                        dmem_WE_sel = 2'b10;
                        dmem_RE_sel = 2'b10;
                        miss_stall = 1'b1;
                        if (MEM_WRITE2) begin
                            NS = ST_WRITE_MISS;
                        end else begin
                            NS = ST_UPDATE_CACHE;
                        end
                    end else begin
                        if (addr_is_io) begin
                            dmem_WE_sel = 2'b10;
                            dmem_RE_sel = 2'b10;
                        end
                        NS = ST_READ_CACHE;
                    end
                end else begin
                    NS = ST_READ_CACHE;
                end
            end
            
            ST_UPDATE_CACHE: begin
                update = 1'b1;
                if (evict) begin    // stall and force MM to write
                    miss_stall = 1'b1;
                    dmem_WE_sel = 1;
                    dmem_RE_sel = 0;
                end
                NS = ST_READ_CACHE;
            end
            
            ST_WRITE_MISS: begin    // stall and force MM to read newly written data
                miss_stall = 1;
                dmem_WE_sel = 0;
                dmem_RE_sel = 1;
                NS = ST_UPDATE_CACHE;
            end
            
            default: NS = ST_READ_CACHE;
        endcase
   end
endmodule
