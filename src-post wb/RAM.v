module RAM 
#(
    parameter ADDR_WIDTH = 24
)
(
    input                  RST_I,
    input                  CLK_I,
    input                  CYC_I,    
    // input                  LOCK_I,   //TODO? transfer modes
    input                  STB_I,
    input [ADDR_WIDTH-1:0] ADR_I,  
    input [31:0]           DAT_I,
    input [3:0]            SEL_I,
    input                  WE_I,

    output                 ACK_O,
    output reg [31:0]      DAT_O
);

assign ACK_O = (STB_I & CYC_I);

reg [31:0] ram [0:16777216];
wire [29:0] word_addr = ADR_I[ADDR_WIDTH-1:2]; 

always @(posedge CLK_I) begin
    if (STB_I & !WE_I) begin
        DAT_O <= ram[word_addr]; // SEL_I not compliant here, read sel is instead done in cpu 
    end
end

always @(posedge CLK_I) begin
    if (STB_I & WE_I) begin
        if (SEL_I[0]) ram[word_addr][7:0]   <= DAT_I[7:0];
        if (SEL_I[1]) ram[word_addr][15:8]  <= DAT_I[15:8];
        if (SEL_I[2]) ram[word_addr][23:16] <= DAT_I[23:16];
        if (SEL_I[3]) ram[word_addr][31:24] <= DAT_I[31:24];
    end
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
// 
//         // ebreak
//         //                                        SYSTEM
//         ram[6] = 32'b000000000001_00000_000_00000_1110011;
//     end
// `endif

endmodule