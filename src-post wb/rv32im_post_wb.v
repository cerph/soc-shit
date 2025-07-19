// +FHDR------------------------------------------------------------------------
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -----------------------------------------------------------------------------
// DESC : 
// -FHDR------------------------------------------------------------------------

`default_nettype none
`define BENCH 1

`define RESET_VECTOR 31'h08000000 
`define TRAP_BASE_VECTOR 29'h08001000 // 256 byte aligned --> [7:2]=6'b0

module rv32im_core
(
    input           CLK_I,
    input           RST_I,
    input           ACK_I,
    input [31:0]    DAT_I,

    output          CYC_O,
    output          STB_O,
    output [31:0]   ADR_O,
    output [31:0]   DAT_O,
    output [3:0]    SEL_O,
    output          WE_O,

    // NON WISHBONE
    input [7:0]     ext_irq_bus_i
);


localparam ADDR_WIDTH = 32-1;

parameter CSR_MSTATUS = 12'h300;
parameter CSR_MIE     = 12'h304;
parameter CSR_MIP     = 12'h344;
parameter CSR_MTVEC   = 12'h305;
parameter CSR_MEPC    = 12'h341;
parameter CSR_MCAUSE  = 12'h342;

parameter MIE_MEIE_BIT = 11;    // Machine External Interrupt Enable
parameter MIE_MTIE_BIT = 7;     // Machine Timer Interrupt Enable
parameter MIE_MSIE_BIT = 3;     // Machine Software Interrupt Enable
parameter MSTATUS_MIE_BIT = 3;
parameter MSTATUS_MPIE_BIT = 7;

// system
reg [ADDR_WIDTH:0] pc_r = 0;
reg [31:0] instr_r;

reg [31:0] csr_mstatus = 0;
reg [31:0] csr_mie = 0;     
reg [31:0] csr_mip = 0;
reg [31:0] csr_mtvec = 0; // [1:0]=00 ==> direct mode
reg [31:0] csr_mepc = 0;
reg [31:0] csr_mcause = 0;

// interrupt
wire machine_irq_en = csr_mstatus[MSTATUS_MIE_BIT];
wire irq_pending = (|ext_irq_bus_i) & machine_irq_en;

wire [1:0] mtvec_mode = csr_mtvec[1:0];
wire [31:2] mtvec_base = csr_mtvec[31:2];
wire [31:0] trap_vector = ((mtvec_mode == 2'b01) && irq_pending) ? {mtvec_base, 2'b00} + (csr_mcause[30:0] << 2) :
                                            {mtvec_base, 2'b00};

// opcode
wire is_LUI     =  (instr_r[6:2] == 5'b01101); // rd <- Uimm   
wire is_ALUIPC  =  (instr_r[6:2] == 5'b00101); // rd <- PC + Uimm
wire is_JALR    =  (instr_r[6:2] == 5'b11001); // rd <- PC+4; PC<-rs1+Iimm
wire is_JAL     =  (instr_r[6:2] == 5'b11011); // rd <- PC+4; PC<-PC+Jimm
wire is_branch  =  (instr_r[6:2] == 5'b11000); // if(rs1 OP rs2) PC<-PC+Bimm
wire is_load    =  (instr_r[6:2] == 5'b00000); // rd <- mem[rs1+Iimm]
wire is_store   =  (instr_r[6:2] == 5'b01000); // mem[rs1+Simm] <- rs2
wire is_alu_imm =  (instr_r[6:2] == 5'b00100); // rd <- rs1 OP Iimm
wire is_alu_reg =  (instr_r[6:2] == 5'b01100); // rd <- rs1 OP rs2   
wire is_system  =  (instr_r[6:2] == 5'b11100); // system

wire is_csrrw   = is_system & (funct3 == 3'h1);
wire is_csrrs   = is_system & (funct3 == 3'h2);
wire is_csrrc   = is_system & (funct3 == 3'h3);
wire is_csrrwi  = is_system & (funct3 == 3'h5);
wire is_csrrsi  = is_system & (funct3 == 3'h6);
wire is_csrrci  = is_system & (funct3 == 3'h7);

wire is_ecall   = is_system & (funct3 == 3'b000 && instr_r[31:20] == 12'h0);
wire is_ebreak  = is_system & (funct3 == 3'b000 && instr_r[31:20] == 12'h1);

wire is_mret    = is_system & (funct3 == 3'b000 && instr_r[31:20] == 12'h302);
wire is_wfi     = is_system & (funct3 == 3'b000 && instr_r[31:20] == 12'h105);


// fields

wire [31:0] J_imm = {{12{instr_r[31]}}, instr_r[19:12], instr_r[20],    instr_r[30:21], 1'b0};
wire [31:0] B_imm = {{20{instr_r[31]}}, instr_r[7],     instr_r[30:25], instr_r[11:8],  1'b0};
wire [31:0] S_imm = {{21{instr_r[31]}}, instr_r[30:25], instr_r[11:7]};
wire [31:0] I_imm = {{21{instr_r[31]}}, instr_r[30:20]};
wire [31:0] U_imm = {instr_r[31],       instr_r[30:12], {12{1'b0}}};
wire [31:0] Z_imm = {27'b0, rs1_id};
    
wire [4:0] rs1_id = instr_r[19:15];
wire [4:0] rs2_id = instr_r[24:20];
wire [4:0] rd_id = instr_r[11:7];

wire [2:0] funct3 = instr_r[14:12];
wire [6:0] funct7 = instr_r[31:25];


// CSRs

wire [11:0] csr_addr = instr_r[31:20];
wire csr_op = is_csrrw | is_csrrs | is_csrrc | is_csrrwi | is_csrrsi | is_csrrci;

wire csr_read_en = csr_op && ~((is_csrrw || is_csrrwi) && rd_is_zero);
reg [31:0] csr_rdata;
always @(*) begin
    if (csr_read_en) begin      // do not affect a read when rd is x0
        case (csr_addr)
            CSR_MSTATUS: csr_rdata = csr_mstatus;
            CSR_MIE:     csr_rdata = csr_mie;     
            CSR_MIP:     csr_rdata = csr_mip;     
            CSR_MTVEC:   csr_rdata = csr_mtvec;
            CSR_MEPC:    csr_rdata = csr_mepc;
            CSR_MCAUSE:  csr_rdata = csr_mcause;
            default:     csr_rdata = 32'b0;
        endcase
    end
end


wire rs1_is_zero = (rs1_id == 5'd0);
wire rd_is_zero = (rd_id == 5'd0);
wire Zimm_is_zero = (Z_imm == 32'b0);

wire [31:0] csr_wdata = is_csrrw ? (rs1) :
                        is_csrrs ? (csr_rdata | rs1) :
                        is_csrrc ? (csr_rdata & ~rs1) : 
                        is_csrrwi ? Z_imm : 
                        is_csrrsi ? (csr_rdata | Z_imm) :
                        is_csrrci ? (csr_rdata & ~Z_imm) :
                        32'b0;

wire csr_write_en = (state == EXECUTE) && csr_op && ~(~(is_csrrw || is_csrrwi) && rs1_is_zero);
always @(posedge CLK_I) begin
    if (csr_write_en) begin
        case (csr_addr) //TODO: write masks
            CSR_MSTATUS: csr_mstatus <= csr_wdata;
            CSR_MIE:     csr_mie     <= (32'hFFFF0888 & csr_wdata);
            CSR_MTVEC:   csr_mtvec   <= csr_wdata;
            CSR_MEPC:    csr_mepc    <= csr_wdata;
            CSR_MCAUSE:  csr_mcause  <= csr_wdata;
        endcase
        // $display("csr_addr: %h = %h @ PC=%h",csr_addr,csr_wdata,pc_r);
    end
end

//TODO: clear all csr's on reset
always @(posedge CLK_I) begin
    if (!RST_I) begin
        csr_mip <= 32'b0;
    end
    else begin  
        csr_mip <= {8'b0, ext_irq_bus_i, 16'b0} & csr_mie;
    end
end

// register file

//TODO: clear on reset
reg [31:0] reg_file [0:31];
wire [31:0] rs1 = reg_file[rs1_id];
wire [31:0] rs2 = reg_file[rs2_id];

wire [31:0] rd_wb_data = (is_JAL || is_JALR) ? pc_add4    : 
                                      is_LUI ? U_imm      :
                                   is_ALUIPC ? pc_add_imm : 
                                     is_load ? load_data  : 
                                      csr_op ? csr_rdata  : alu_out;

wire rd_wb_en = (state == EXECUTE && !is_branch && !is_store && !is_load && !rd_is_zero) 
                // || (state == WAIT_DATA) 
                || (state == EXECUTE && csr_op && !rd_is_zero);

`ifdef BENCH
    integer i;
    initial begin
        for (i=0; i<32; ++i) begin
            reg_file[i] = 0;
        end
    end
`endif

// alu

wire [31:0] alu_s1 = rs1;
wire [31:0] alu_s2 = (is_alu_reg | is_branch) ? rs2 : I_imm;

wire [31:0] alu_add = alu_s1 + alu_s2;
wire [32:0] alu_sub = {1'b0, alu_s1} - {1'b0, alu_s2};

wire alu_EQ = (alu_sub[31:0] == 0);
wire alu_LT = (alu_s1[31] ^ alu_s2[31]) ? alu_s1[31] : alu_sub[32];
wire alu_LTU = alu_sub[32];

function [31:0] flip32;
    input [31:0] x;
    flip32 = {x[ 0], x[ 1], x[ 2], x[ 3], x[ 4], x[ 5], x[ 6], x[ 7], 
              x[ 8], x[ 9], x[10], x[11], x[12], x[13], x[14], x[15], 
              x[16], x[17], x[18], x[19], x[20], x[21], x[22], x[23],
              x[24], x[25], x[26], x[27], x[28], x[29], x[30], x[31]};
endfunction

wire [4:0] shamt = (is_alu_reg) ? rs2[4:0] : I_imm[4:0];
wire [31:0] right_shift = $signed({instr_r[30] & alu_s1[31], (funct3 == 3'b001) ? flip32(alu_s1) : alu_s1}) >>> shamt;
wire [31:0] left_shift = flip32(right_shift);

reg [31:0] alu_out;
always @(*) begin
    case (funct3)
        /*ADD*/  3'b000 : alu_out = (funct7[5] & instr_r[5]) ? alu_sub[31:0] : alu_add;
        /*SL*/   3'b001 : alu_out = left_shift;
        /*SLT*/  3'b010 : alu_out = {31'b0, alu_LT};
        /*SLTU*/ 3'b011 : alu_out = {31'b0, alu_LTU};
        /*XOR*/  3'b100 : alu_out = alu_s1 ^ alu_s2;
        /*SR*/   3'b101 : alu_out = right_shift; 
        /*OR*/   3'b110 : alu_out = alu_s1 | alu_s2;
        /*AND*/  3'b111 : alu_out = alu_s1 & alu_s2;
        default: alu_out = 32'bx;
    endcase
end

reg do_branch;
always @(*) begin
    case (funct3) 
        3'b000 : do_branch = alu_EQ;
        3'b001 : do_branch = !alu_EQ;
        3'b100 : do_branch = alu_LT;
        3'b101 : do_branch = !alu_LT;
        3'b110 : do_branch = alu_LTU;
        3'b111 : do_branch = !alu_LTU;
        default : do_branch = 1'b0;
    endcase
end

wire [31:0] pc_add4 = pc_r + 4;

wire [31:0] pc_add_imm = pc_r + (instr_r[3] ? J_imm :
                                 instr_r[4] ? U_imm : B_imm);

// TODO: why would i do {alu_add[31:1],1'b0} ?
wire [31:0] pc_next = ((is_branch && do_branch) || is_JAL) ? {pc_add_imm[31:2],2'b0}
                                                   : is_JALR ? {alu_add[31:2],2'b0} 
                                                   : pc_add4;

// state machine

localparam FETCH = 0;
localparam DECODE = 1;
localparam EXECUTE = 2;
localparam WAIT_DATA = 3;
localparam TRAP = 4;
localparam MRET = 5;

reg [3:0] state = FETCH;

always @(posedge CLK_I) begin
    if (!RST_I) begin
        pc_r <= `RESET_VECTOR;
        state <= FETCH;
    end 
    else begin
        if (rd_wb_en) begin
            reg_file[rd_id] <= rd_wb_data;
            // `ifdef BENCH $display("x%0d <= %h @ PC=%h",rd_id,rd_wb_data,pc_r); `endif	
        end

        case (state)
            FETCH : begin
                state <= DECODE;
            end

            DECODE : begin
                instr_r <= DAT_I;

                if (irq_pending) begin
                    csr_mepc <= pc_r;
                    csr_mcause <= {1'b1, 31'd11}; // 1,EXCCODE=11 -> Machine external interrupt 
                    csr_mstatus[MSTATUS_MPIE_BIT] <= csr_mstatus[MSTATUS_MIE_BIT];
                    csr_mstatus[MSTATUS_MIE_BIT] <= 0;
                    state <= TRAP;
                end
                else if (is_ecall) begin
                    csr_mepc <= pc_r;
                    csr_mcause <= 32'd11; // 0,EXCCODE=11 -> Environment call from M-mode
                    state <= TRAP;
                end
                else if (is_mret) begin
                    state <= MRET;
                end
                else if (is_wfi) begin
                    state <= FETCH;
                end
                else begin
                    state <= EXECUTE;
                end
            end

            EXECUTE : begin
                if (!is_ebreak) begin
                    pc_r <= pc_next;
                end
                `ifdef BENCH if (is_ebreak) $finish(); `endif
                state <= (is_load)  ? WAIT_DATA : FETCH;
            end

            WAIT_DATA : state <= FETCH;

            TRAP: begin
                pc_r <= trap_vector;
                state <= FETCH;
            end

            MRET: begin
                pc_r <= csr_mepc;
                csr_mstatus[MSTATUS_MIE_BIT] <= csr_mstatus[MSTATUS_MPIE_BIT];
                state <= FETCH;
            end
        endcase
    end
end

// have some logic to interface with wb, it routes the data etc.
// in execute and fetch, check for ack_i, loop if low, next stage if high

// memory

assign STB_O = (state == FETCH) || ((state == EXECUTE) && (is_load || is_store));
assign CYC_O = STB_O; //temp before block writes etc. whcih will require another state machine
assign ADR_O = (state == FETCH || state == DECODE) ? pc_r : loadstore_addr;
assign SEL_O = {4{(state == EXECUTE) & is_store}} & store_mask;

wire [31:0] loadstore_addr = reg_file[rs1_id] + (is_store ? S_imm : I_imm);

// load read alignments
wire [15:0] load_halfw = loadstore_addr[1] ? DAT_I[31:16] : DAT_I[15:0];
wire [15:0] load_byte = loadstore_addr[0] ? load_halfw[15:8] : load_halfw[7:0];

wire is_mem_halfw = funct3[1:0] == 2'b00;
wire is_mem_byte = funct3[1:0] == 2'b01;

wire load_data_sign = !funct3[2] & (is_mem_byte ? load_byte[7] : load_halfw[15]);

wire [31:0] load_data = is_mem_byte ? {{24{load_data_sign}}, load_byte}  :
                       is_mem_halfw ? {{16{load_data_sign}}, load_halfw} : DAT_I;

// store write alignment
assign DAT_O[7:0]   = rs2[7:0];
assign DAT_O[15:8]  = loadstore_addr[0] ? rs2[7:0]  : rs2[15:8];
assign DAT_O[23:16] = loadstore_addr[1] ? rs2[7:0]  : rs2[23:16];
assign DAT_O[31:24] = loadstore_addr[0] ? rs2[7:0]  :
                      loadstore_addr[1] ? rs2[15:8] : rs2[31:24];

wire [3:0] store_mask =
    is_mem_byte ? (
        loadstore_addr[1] ?
            (loadstore_addr[0] ? 4'b1000 : 4'b0100)
        :
            (loadstore_addr[0] ? 4'b0010 : 4'b0001)
    ) 
    : is_mem_halfw ? 
        (loadstore_addr[1] ? 4'b1100 : 4'b0011)
    :
        4'b1111;



endmodule