`ifndef DEFINE_ME
    `include "uart_tx.v"
    `include "uart_rx.v"
`endif


module uart_top
(
    input           RST_I,
    input           CLK_I,
    input           CYC_I,
    input           STB_I,
    input [3:0]     ADR_I,
    input [7:0]     DAT_I,
    input           WE_I,

    output reg      ACK_O,
    output reg [7:0]    DAT_O,

    input           uart_rx_i,

    output          uart_tx_o,
    output          uart_irq_o
);

assign uart_irq_o = uart_rx_valid;

always @(posedge CLK_I) begin
    ACK_O <= STB_I;
    if (!WE_I & uart_is_SR) begin
        DAT_O <= uart_SR;
    end
    else if (!WE_I & uart_is_RBR) begin
        DAT_O <= uart_RBR;
    end
end

wire [7:0] uart_SR = {6'b0, uart_rx_valid, uart_tx_busy};
wire [7:0] uart_RBR;

wire uart_tx_done;
wire uart_tx_busy;
wire uart_rx_valid;

wire uart_is_RBR = STB_I & (ADR_I == 4'h0);
wire uart_is_THR = STB_I & (ADR_I == 4'h4);
wire uart_is_SR = STB_I & (ADR_I == 4'h8);

uart_tx TX (
    .clk_i(CLK_I),
    .stb_i(uart_is_THR & WE_I),
    .data_i(DAT_I),

    .done_o(uart_tx_done),
    .busy_o(uart_tx_busy),
    .tx_o(uart_tx_o)
);

uart_rx RX (
    .clk_i(CLK_I),
    .rx_i(uart_rx_i),
    .stb_i(uart_is_RBR & !WE_I),

    .valid_data_o(uart_rx_valid),
    .data_o(uart_RBR)
);

endmodule