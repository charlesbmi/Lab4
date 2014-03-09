`include "mips_defines.v"

module cache (
    input clk,
    input memclk,
    input [31:0] addr, // byte address
    input we,
    input re,
    input rst,
    input [31:0] din,

    output wire [31:0] dout,
    output wire complete
);

    wire [1:0] offset = addr[3:2];
    wire [5:0] index = addr[9:4];
    wire [21:0] tag = addr[31:10];
    wire [29:0] word = addr[31:2];

    // when implementing your cache FSM logic
    // you will write and read from these structures
    // containing the valid bits, tag, and data, for each cache
    // block, indexed from 0 to 63
    // you may modify these structures to implement more complicated caches,
    // but their total size must not increase

    // this will support 64 blocks of 4 words each, for a total of 1KB of
    // cache data storage
    reg [63:0] valid_bits;
    reg [21:0] tag_bits [63:0];
    reg [127:0] data_blocks [63:0];

    // write to cache on hit
    wire hit = valid_bits[index] && (tag_bits[index] == tag);
    always @(posedge clk) begin
        if (we && hit) begin
            case (offset)
                2'b00: data_blocks[index] <= {data_blocks[index][127:32], din};
                2'b01: data_blocks[index] <= {data_blocks[index][127:64], din, data_blocks[index][31:0]};
                2'b10: data_blocks[index] <= {data_blocks[index][127:96], din, data_blocks[index][63:0]};
                2'b11: data_blocks[index] <= {din, data_blocks[index][95:0]};
            endcase
        end
    end

    // outputs from DRAM
    wire [127:0] dram_out;
    wire dram_complete;

    wire read = (hit && re);

    always @(posedge clk) begin
        casex({read,offset, dram_complete})
            4'b100x: {cache_dout, data_blocks[index]} = {data_blocks[index][31:0], data_blocks[index]};
            4'b101x: {cache_dout, data_blocks[index]} = {data_blocks[index][63:32], data_blocks[index]};
            4'b110x: {cache_dout, data_blocks[index]} = {data_blocks[index][95:64], data_blocks[index]};
            4'b111x: {cache_dout, data_blocks[index]} = {data_blocks[index][127:96], data_blocks[index]};
            4'b0001: {cache_dout, data_blocks[index]} = {dram_out[index][31:0], dram_out};
            4'b0011: {cache_dout, data_blocks[index]} = {dram_out[index][63:32], dram_out};
            4'b0101: {cache_dout, data_blocks[index]} = {dram_out[index][95:64], dram_out};
            4'b0111: {cache_dout, data_blocks[index]} = {dram_out[index][127:96], dram_out};
            default: {cache_dout, data_blocks[index]} = {32'b0, 128'b0};
        endcase
    end

    assign valid_bits[index] = (re && read)? 1'b1:(dram_complete ? 1'b1: 1'b0);
    assign dout = cache_dout;
    assign complete = (~re && ~we)? 1'b1: (read ? 1'b1: dram_complete);

    // USE THIS SYNCHRONOUS BLOCK TO ASSIGN THE INPUTS TO DRAM
    // inputs to dram should be regs when assigned in a state machine
    reg dram_re;
    reg [`MEM_DEPTH-3:0] dram_addr;
    reg [31:0] dram_in;
    reg [31:0] cache_dout;

    always @(posedge clk) begin
        case(read)
            1'b1: {dram_re, dram_in, dram_addr} = {1'b0, 32'b0, 32'b0};
            1'b0: {dram_re, dram_in, dram_addr} = {1'b1, din, addr[`MEM_DEPTH+1:4]};
        endcase
    end

    always@(*)begin
        case(re && dram_complete) 
            1'b1: {valid_bits[index], tag_bits[index]} = {1'b1, tag};
            1'b0: {valid_bits[index], tag_bits[index]} = {valid_bits[index], tag_bits[index]};
        endcase
    end

    dataram dram (.clk(clk),
                  .memclk(memclk),
                  .rst(rst),
                  .we(we),
                  .re(dram_re),
                  .addr(dram_addr),
                  .offset(offset),
                  .din(dram_in),
                  .dout(dram_out),
                  .complete(dram_complete));

endmodule
