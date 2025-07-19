module intcontrol 
(
    input               RST_I,
    input               CLK_I,
    input               CYC_I,    
    input               STB_I,

    output reg          ACK_O,
    output reg [7:0]    DAT_O,

    // NON WISHBONE
    input [7:0]         ext_irq_bus_i
);

always @(posedge CLK_I) begin
    ACK_O <= STB_I;
end

always @(posedge CLK_I) begin
    if (STB_I) begin
        DAT_O <= IRR;
    end
end

reg [2:0] irq_id;
always @(*) begin
    casex (ext_irq_bus_i)
        8'b1xxxxxxx: irq_id = 3'd7;
        8'b01xxxxxx: irq_id = 3'd6;
        8'b001xxxxx: irq_id = 3'd5;
        8'b0001xxxx: irq_id = 3'd4;
        8'b00001xxx: irq_id = 3'd3;
        8'b000001xx: irq_id = 3'd2;
        8'b0000001x: irq_id = 3'd1;
        8'b00000001: irq_id = 3'd0;
        default: irq_id = 3'd0;
    endcase
end

wire [7:0] IRR = {5'b0, irq_id};

endmodule