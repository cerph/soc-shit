module RAM 
(
    input        clk_i,
    input [23:0] ram_addr_i,
    input        ram_rstb_i,
    input [31:0] ram_data_i,
    input [3:0]  ram_mask_i,


    output reg [31:0] ram_data_o
);

reg [31:0] ram [0:16777216];
    
wire [29:0] word_addr = ram_addr_i[23:2];

always @(posedge clk_i) begin
    if (ram_rstb_i) begin
        ram_data_o <= ram[word_addr];
    end
    if (ram_mask_i[0]) ram[word_addr][7:0]   <= ram_data_i[7:0];
    if (ram_mask_i[1]) ram[word_addr][15:8]  <= ram_data_i[15:8];
    if (ram_mask_i[2]) ram[word_addr][23:16] <= ram_data_i[23:16];
    if (ram_mask_i[3]) ram[word_addr][31:24] <= ram_data_i[31:24];
end

/* 
    ADDI x20, x0, 69
    ADDI x5, x0, 20
    SW x20, 0(x5)
    LW x21, 20(x0)
    ADD x20, x20, x21 
    EBREAK
*/
initial begin 
    // $readmemh("E:/Documents/ceRV32/src/program.hex", ram);
end

// `ifdef BENCH
//     initial begin
//         // add x0, x0, x0
//         //                   rs2   rs1  add  rd   ALUREG
//         ram[0] = 32'b0000000_00000_00000_000_00000_0110011;
//         // add x1, x0, x0
//         //                    rs2   rs1  add  rd   ALUREG
//         ram[1] = 32'b0000000_00000_00000_000_00001_0110011;
//         // addi x1, x1, 1
//         //             imm         rs1  add  rd   ALUIMM
//         ram[2] = 32'b000000000001_00001_000_00001_0010011;
//         // addi x1, x1, 1
//         //             imm         rs1  add  rd   ALUIMM
//         ram[3] = 32'b000000000001_00001_000_00001_0010011;
//         // addi x1, x1, 1
//         //             imm         rs1  add  rd   ALUIMM
//         ram[4] = 32'b000000000001_00001_000_00001_0010011;
//         // addi x1, x1, 1
//         //             imm         rs1  add  rd   ALUIMM
//         ram[5] = 32'b000000000001_00001_000_00001_0010011;

//         // ebreak
//         //                                        SYSTEM
//         ram[6] = 32'b000000000001_00000_000_00000_1110011;
//     end
// `endif

endmodule