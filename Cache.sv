`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/02/2024 07:14:29 PM
// Design Name: 
// Module Name: Cache
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


module Cache(
    input [31:0] addr, input CLK, input RESET, input update, input MEM_WRITE2, input MEM_READ2,
    input logic [31:0] w0_in,input logic [31:0] w1_in, input logic [31:0] w2_in, input logic [31:0] w3_in,
    output logic [31:0] w0_out, output logic [31:0] w1_out, output logic [31:0] w2_out, output logic [31:0] w3_out,
    output logic [31:0] dout, output logic hit, output logic miss, output logic evict, output logic addr_is_io, output logic [31:0] evicted_addr
    );
    
    parameter NUM_SETS = 4;
    parameter NUM_BLOCKS = 4;
    parameter BLOCK_SIZE = 4;
    parameter SET_INDEX_SIZE = 2;
    parameter BLOCK_INDEX_SIZE = 2;
    parameter WORD_OFFSET_SIZE = 2;
    parameter BYTE_OFFSET_SIZE = 2;
    parameter TAG_SIZE = 32 - SET_INDEX_SIZE - WORD_OFFSET_SIZE - BYTE_OFFSET_SIZE;
    
    typedef struct {
        logic valid;
        logic dirty;
        logic [1:0] LRU;
        logic [TAG_SIZE-1:0] tag;
        logic [31:0] data[BLOCK_SIZE-1:0];
    } block_t;
    
    block_t cache[NUM_SETS][NUM_BLOCKS-1:0];
    logic [1:0] set_index;
    logic [TAG_SIZE-1:0]tag;
    logic [2:0]word_offset;
    logic [1:0] hit_block_index;
    
    logic [BLOCK_INDEX_SIZE - 1:0] miss_block_index;
    logic no_empty_block;
    logic [BLOCK_INDEX_SIZE - 1:0] random_block_to_replace;
    
    logic [31:0] saved_addr;
    logic [31:0] used_addr;
    
    assign used_addr = update ? saved_addr : addr;
    
    initial begin
        int i; int j; int k;
        for(i = 0; i < NUM_SETS; i = i + 1) begin //initializing RAM to 0
            for(j=0; j < NUM_BLOCKS; j = j + 1) begin
                cache[i][j].valid = 1'b0;
                cache[i][j].dirty = 1'b0;
                cache[i][j].LRU = 2'b0;
                cache[i][j].tag = 26'b0; //hardcoded in TAG_SIZE
                for (k = 0; k < BLOCK_SIZE; k = k + 1) begin
                    cache[i][j].data[k] = 32'b0;
                end
            end
        end
    end
    
    assign tag = addr[31:(SET_INDEX_SIZE + WORD_OFFSET_SIZE + BYTE_OFFSET_SIZE)];
    assign set_index = addr[(SET_INDEX_SIZE + WORD_OFFSET_SIZE + BYTE_OFFSET_SIZE - 1):(WORD_OFFSET_SIZE + BYTE_OFFSET_SIZE)];
    assign word_offset = addr[(WORD_OFFSET_SIZE + BYTE_OFFSET_SIZE - 1):(BYTE_OFFSET_SIZE)];

    // determine if data (to be read or written over) is in cache
    // if so, save the correct block
    // otherwise, determine which block will be written over
    always_comb begin
        hit = 1; miss = 0; addr_is_io = 0; hit_block_index = 2'b00; no_empty_block = 0;
        if (addr >= 32'h11000000) begin     // memory-mapped I/O => neither hit nor miss
            hit = 0;
            miss = 0;
            addr_is_io = 1;
        end else if (cache[set_index][2'b00].valid && cache[set_index][2'b00].tag == tag) begin
            hit_block_index = 2'b00;
        end else if (cache[set_index][2'b01].valid && cache[set_index][2'b01].tag == tag) begin
            hit_block_index = 2'b01;
        end else if (cache[set_index][2'b10].valid && cache[set_index][2'b10].tag == tag) begin
            hit_block_index = 2'b10;
        end else if (cache[set_index][2'b11].valid && cache[set_index][2'b11].tag == tag) begin
            hit_block_index = 2'b11;
        end else begin
            hit = 0;
            miss = 1;
        end
        
        if (miss) begin             // if missed, need to find a spot for data coming in from MM
            no_empty_block = 1; miss_block_index = 0;
            
            for (int i = 0; i < NUM_BLOCKS && no_empty_block; i = i + 1) begin        // find an empty block and save its index
                if (!cache[set_index][i].valid) begin
                    miss_block_index = i;
                    no_empty_block = 0;
                end
            end
            
            if (no_empty_block) begin                               // if no empty block found, pick "random" block to evict
                miss_block_index = random_block_to_replace;
            end
        end
    end
    
    always_ff @(posedge CLK or posedge RESET) begin
        evict <= 0;
        if (hit && MEM_READ2) begin                             // read hit
            w0_out <= cache[set_index][hit_block_index].data[word_offset];
        end else if (miss && (MEM_WRITE2 || MEM_READ2)) begin                   // read miss
            if (cache[set_index][miss_block_index].dirty) begin   // if block we are replacing is dirty, evict block
                evict <= 1;
                evicted_addr <= addr;
                w0_out <= cache[set_index][miss_block_index].data[2'b00];
                w1_out <= cache[set_index][miss_block_index].data[2'b01];
                w2_out <= cache[set_index][miss_block_index].data[2'b10];
                w3_out <= cache[set_index][miss_block_index].data[2'b11];
            end
        end
        
        // pseudo-random replacement policy
        if (RESET) begin                        // intialize values
            random_block_to_replace <= 0; evict <= 0;
        end else if (random_block_to_replace == 2'b11) begin
            random_block_to_replace <= 0;
        end else begin
            random_block_to_replace <= random_block_to_replace + 1;             // PUT IN OWN ALWAYS_FF
        end
    end
        
    always_ff @(negedge CLK) begin
        if (hit && MEM_WRITE2) begin
            cache[set_index][hit_block_index].data[word_offset] <= w0_in;
            cache[set_index][hit_block_index].dirty <= 1'b1;
        end
        
        if(update) begin
            cache[set_index][miss_block_index].data[2'b00] <= w0_in;
            cache[set_index][miss_block_index].data[2'b01] <= w1_in;
            cache[set_index][miss_block_index].data[2'b10] <= w2_in;
            cache[set_index][miss_block_index].data[2'b11] <= w3_in;
            cache[set_index][miss_block_index].valid <= 1'b1;
            cache[set_index][miss_block_index].dirty <= 1'b0;
            cache[set_index][miss_block_index].LRU <= 1'b0;
            cache[set_index][miss_block_index].tag <= tag;
        end
    end
endmodule

