module spi_master
(
    // WISHBONE
    input CLK_I,
    input RST_I,            // active high
    input [26:0] ADR_I,
    input [31:0] DAT_I,
    input [3:0] SEL_I,
    input PAR_I,            // TGD_I
    input WE_I,
    input CYC_I,
    input STB_I,

    output [31:0] DAT_O,
    output reg ACK_O,
    output ERR_O,
    output RTY_O,

    // SPI
    output cs_b_o,
    output sclk_o,
    inout IO_0,
    inout IO_1,
    inout IO_2,
    inout IO_3
);

// TODO:  later todo
// make QSPI to talk with off chip flash memory      CPU -> WB -> WB -> QSPI -> QSPI -> flash


reg [7:0] spi_data;
reg [2:0] bit_count;
reg spi_active;
reg [31:0] wb_reg_data;

wire wb_write = CYC_I && STB_I && WE_I;
wire wb_read  = CYC_I && STB_I && ~WE_I;

assign ERR_O = 1'b0;
assign RTY_O = 1'b0;

// wishbone handshake
always @(posedge CLK_I or posedge RST_I) begin
    if (RST_I) begin
        ACK_O <= 1'b0;
    end 
    else begin
        ACK_O <= CYC_I && STB_I && !ACK_O;
    end
end

always @(posedge CLK_I or posedge RST_I) begin
    if (RST_I) begin
        spi_data <= 8'h00;
        wb_reg_data <= 32'h00000000;
        cs_b_o <= 1'b1;
        sclk_o <= 1'b0;
        mosi_o <= 1'b0;
        spi_active <= 1'b0;
        bit_cnt <= 3'd0;
    end 
    else begin
        if (wb_write && ADR_I[3:2] == 2'b00) begin
            // Write to SPI transmit register
            spi_data <= DAT_I[7:0];
            cs_b_o <= 1'b0;
            spi_active <= 1'b1;
            bit_cnt <= 3'd7;
        end
        else if (wb_read && ADR_I[3:2] == 2'b01) begin
            // Read from SPI receive register
            DAT_O <= {24'b0, spi_data};
        end

        // SPI bit shifting logic
        if (spi_active) begin
            sclk_o <= ~sclk_o;
            if (sclk_o) begin
                mosi_o <= spi_data[bit_cnt];
                bit_cnt <= bit_cnt - 1;
                if (bit_cnt == 0) begin
                    spi_active <= 1'b0;
                    cs_b_o <= 1'b1;
                end
            end else begin
                spi_data[bit_cnt] <= miso_o;
            end
        end
    end
end

endmodule