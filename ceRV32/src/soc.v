`ifndef DEFINE_ME
    `include "rv32im_core.v"
    `include "RAM.v"
    `include "uart_tx.v"
    `include "uart_rx.v"
`endif



//     [27]=Flash     ADDR[26:0]
//      ├──────────────────────────────────┤
//      │
//      │ [24]=RAM       ADDR[23:0]               
//      │  ├───────────────────────────────┤
//      │  │                                
//      │  │   [20]=VGA    ADDR[19:0]            
//      │  │    ├──────────────────────────┤
//      │  │    │                           
// 31   │  │    │   16 15  12 11           0
// ├────┼──┼────┼────┤  ├──┤  ├────────────┤
// 0000 0000 0000 0000  0000  0000 0000 0000
//       HOT BIT         ID        ADDR     
// 
// 
// |------------------|-------|--------------------|------------|----------------------------------------|
// | Cold Peripheral  | ID    | Base Address       | Size       | Description                            |
// |------------------|-------|--------------------|------------|----------------------------------------|
// | System           | 0x0   | `0x0000_0000`      | 4 KiB      | Reserved system space                  |
// | Timer            | 0x1   | `0x0000_1000`      | 4 KiB      | 32-bit timer                           |
// | Interrupt Ctrl   | 0x2   | `0x0000_2000`      | 4 KiB      | Interrupt controller registers         |
// | UART             | 0x3   | `0x0000_3000`      | 4 KiB      | UART with TX/RX                        |
// | SPI              | 0x4   | `0x0000_4000`      | 4 KiB      | SPI registers                          |
// | GPIO             | 0x5   | `0x0000_5000`      | 4 KiB      | 32-bit GPIO register space             |
// |------------------|----------------------------|------------|----------------------------------------|
// | Hot Peripheral   | Base Address               | Size       | Description                            |
// |------------------|----------------------------|------------|----------------------------------------|
// | VGA [20]         | `0x0010_0000`              | 1 MiB      | VGA frame buffer                       |
// | RAM [24]         | `0x0100_0000`              | 16 MiB     | System RAM                             |
// | Flash [27]       | `0x0800_0000`              | 128 MiB    | SPI Flash Memory                       |
// |------------------|----------------------------|------------|----------------------------------------|
  

module soc 
(
    input clk,
    input rst_b,
    input UART_RX_i,

    output UART_TX_o,
    output [31:0] GPIO_o
);

// MEMORY / INTERCONN

wire mem_rstb_i;
wire mem_wstb_i;
wire [31:0] mem_addr_i;
wire [31:0] mem_data_i;
wire [3:0] mem_mask_i;
wire [31:0] mem_data_o;

assign mem_wstb_i = |mem_mask_i;

assign mem_data_o = 
      is_RAM ? RAM_data_o
    : intc_is_IRR ? intc_irr
    : uart_is_SR ? uart_SR
    : uart_is_RBR ? uart_RBR
    : is_GPIO ? GPIO_o 
    : 32'bx; 



// CPU
// ---------------------------------------------------------

wire [31:0] csr_mip;
wire [31:0] csr_mie;

rv32im_core core (
    .clk_i(clk),
    .rst_b_i(rst_b),
    .mem_data_i(mem_data_o),
    .irq_master_i(intc_irq_master),
    .irq_bus_i(intc_irq_bus),

    .mem_rstb_o(mem_rstb_i),
    .mem_addr_o(mem_addr_i),
    .mem_data_o(mem_data_i),
    .mem_mask_o(mem_mask_i),
    .csr_mip_o(csr_mip), // since cpu (can) write mip for timers etc.
    .csr_mie_o(csr_mie) 
);



// RAM
// ---------------------------------------------------------

wire [31:0] RAM_data_o;

RAM RAM (
    .clk_i(clk),
    .ram_addr_i(mem_addr_i[23:0]),
    .ram_rstb_i(is_RAM & mem_rstb_i),
    .ram_data_i(mem_data_i),
    .ram_mask_i({4{is_RAM}} & mem_mask_i),

    .ram_data_o(RAM_data_o)
);



// INTERRUPT CONTROLLER
// ---------------------------------------------------------
// 0x2000   IRR[2:0] : Interrupt Response Register   [RO]
//                     [2:0] = IRQ ID (active priority)

wire [7:0] intc_irq_bus = {
    7'b0,
    // uart_tx_done,
    uart_rx_valid
};

reg intc_irq_master;
reg [2:0] intc_irq_id;

wire intc_is_IRR = is_INTC & (cold_addr[3:0] == 4'h0);
wire [7:0] intc_irr = {5'b0, intc_irq_id};

// priority encoder (lowest index = highest priority)
always @(*) begin
    casex (intc_irq_bus & csr_mie)
        8'b1xxxxxxx: begin intc_irq_master = 1; intc_irq_id = 3'd7; end
        8'b01xxxxxx: begin intc_irq_master = 1; intc_irq_id = 3'd6; end
        8'b001xxxxx: begin intc_irq_master = 1; intc_irq_id = 3'd5; end
        8'b0001xxxx: begin intc_irq_master = 1; intc_irq_id = 3'd4; end
        8'b00001xxx: begin intc_irq_master = 1; intc_irq_id = 3'd3; end
        8'b000001xx: begin intc_irq_master = 1; intc_irq_id = 3'd2; end
        8'b0000001x: begin intc_irq_master = 1; intc_irq_id = 3'd1; end
        8'b00000001: begin intc_irq_master = 1; intc_irq_id = 3'd0; end
        default: begin intc_irq_master = 0; intc_irq_id = 3'd0; end
    endcase
end



// UART
// ---------------------------------------------------------
// 0x3000   RBR[7:0] : Receiver buffer register        [RO]
// 0x3004   THR[7:0] : Transmitter holding register    [WO]
// 0x3008   SR[7:0]  : Status register                 [RO] 
//                     [0] = busy flag
//                     [1] = rx valid

wire [7:0] uart_SR = {6'b0, uart_rx_valid, uart_tx_busy};
wire [7:0] uart_RBR;

wire uart_tx_done;
wire uart_tx_busy;
wire uart_rx_valid;

wire uart_is_RBR = is_UART & (cold_addr[3:0] == 4'h0);
wire uart_is_THR = is_UART & (cold_addr[3:0] == 4'h4);
wire uart_is_SR = is_UART & (cold_addr[3:0] == 4'h8);

uart_tx TX (
    .clk_i(clk),
    .stb_i(uart_is_THR & mem_wstb_i),
    .data_i(mem_data_i[7:0]),

    .done_o(uart_tx_done),
    .busy_o(uart_tx_busy),
    .tx_o(UART_TX_o)
);

uart_rx RX (
    .clk_i(clk),
    .rx_i(UART_RX_i),
    .stb_i(uart_is_RBR & mem_rstb_i),

    .valid_data_o(uart_rx_valid),
    .data_o(uart_RBR)
);



// GPIO
// ---------------------------------------------------------

reg [7:0] GPIO [3:0];
assign GPIO_o = {GPIO[3], GPIO[2], GPIO[1], GPIO[0]};

always @(posedge clk) begin
    if (is_GPIO) begin
        if (mem_mask_i[0]) GPIO[0] <= mem_data_i[7:0];
        if (mem_mask_i[1]) GPIO[1] <= mem_data_i[15:8];
        if (mem_mask_i[2]) GPIO[2] <= mem_data_i[23:16];
        if (mem_mask_i[3]) GPIO[3] <= mem_data_i[31:24];
    end
end

`ifdef BENCH
    always @(*) $display("%b", GPIO_o);
`endif



// DECODE
// ---------------------------------------------------------

localparam IO_HOT_RAM_BIT = 8;
localparam IO_HOT_VGA_BIT = 4;

// hot
wire [15:0] hot_field = mem_addr_i[31:16];

wire is_RAM = hot_field[IO_HOT_RAM_BIT];
wire is_VGA = (!is_RAM & hot_field[IO_HOT_VGA_BIT]);

// cold
wire [11:0] cold_addr = mem_addr_i[11:0];
wire [3:0] cold_id = {4{~|hot_field}} & mem_addr_i[15:12];

wire is_system    = (cold_id == 4'h0);
wire is_timer     = (cold_id == 4'h1);
wire is_INTC      = (cold_id == 4'h2);
wire is_UART      = (cold_id == 4'h3);
wire is_SPI       = (cold_id == 4'h4);
wire is_GPIO      = (cold_id == 4'h5);

wire is_valid_peripheral = (|cold_id) | is_RAM | is_VGA;

endmodule