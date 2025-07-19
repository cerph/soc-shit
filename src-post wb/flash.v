module flash 
#(
    parameter ADDR_WIDTH = 27
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

    output reg             ACK_O,
    output reg [31:0]      DAT_O
);

// assign ACK_O = (STB_I & CYC_I);

reg [31:0] flash [0:134217728];
wire [29:0] word_addr = ADR_I[ADDR_WIDTH-1:2]; 

always @(posedge CLK_I) begin
    if (STB_I & !WE_I) begin
        DAT_O <= flash[word_addr]; // SEL_I not compliant here, read sel is instead done in cpu 
    end
    else if (STB_I & WE_I) begin
        if (SEL_I[0]) flash[word_addr][7:0]   <= DAT_I[7:0];
        if (SEL_I[1]) flash[word_addr][15:8]  <= DAT_I[15:8];
        if (SEL_I[2]) flash[word_addr][23:16] <= DAT_I[23:16];
        if (SEL_I[3]) flash[word_addr][31:24] <= DAT_I[31:24];
    end

    ACK_O <= STB_I;
end

initial begin 
    $readmemh("E:/Documents/ceRV32/src/firmware/boot_test/flash.memh", flash);
end

endmodule
