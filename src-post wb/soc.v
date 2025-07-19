`ifndef DEFINE_ME
    `include "rv32im_core.v"
    `include "RAM.v"
    `include "flash.v"
    `include "uart_top.v"
    `include "intcontrol.v"
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
    input CLK,
    input RST_b,
    input UART_RX_i,

    output UART_TX_o
);



// INTERCONNECT
// ---------------------------------------------------------

wire inc_ack_o = ram_ack_o | flash_ack_o | intc_ack_o | uart_ack_o; // OR all ack inputs

wire [31:0] inc_dat_miso = 
      is_ram ? ram_dat_o
    : is_flash ? flash_dat_o
    : is_intc ? intc_dat_o
    : is_uart ? uart_dat_o
    : 32'bx; 

// these are for the future master arbiter,
// will need to compare each to master grant lines:
// MOSI
wire [31:0] inc_adr_o = cpu_adr_o;
wire [31:0] inc_dat_mosi = cpu_dat_o;
wire [3:0] inc_sel_o = cpu_sel_o;
wire inc_we_o = cpu_we_o;
wire inc_stb_o = cpu_stb_o;
wire inc_cyc_o = cpu_cyc_o;
// MISO
assign cpu_ack_i = inc_ack_o; 



// CPU
// ---------------------------------------------------------

wire cpu_ack_i;
wire cpu_cyc_o;
wire cpu_stb_o;
wire [31:0] cpu_adr_o;
wire [31:0] cpu_dat_o;
wire [3:0] cpu_sel_o;
wire cpu_we_o;

rv32im_core core 
(
    .CLK_I(CLK),
    .RST_I(RST_b),
    .ACK_I(cpu_ack_i),
    .DAT_I(inc_dat_miso),

    .CYC_O(cpu_cyc_o),
    .STB_O(cpu_stb_o),
    .ADR_O(cpu_adr_o),
    .DAT_O(cpu_dat_o),
    .SEL_O(cpu_sel_o),
    .WE_O(cpu_we_o),

    .ext_irq_bus_i(ext_irq_bus)
);



// RAM
// ---------------------------------------------------------

wire ram_ack_o;
wire [31:0] ram_dat_o;

RAM ram 
(
    .RST_I(RST_b),
    .CLK_I(CLK),
    .CYC_I(inc_cyc_o), 
    .STB_I(is_ram & inc_stb_o & inc_cyc_o),
    .ADR_I(inc_adr_o[23:0]),
    .DAT_I(inc_dat_mosi),
    .SEL_I(inc_sel_o),
    .WE_I(inc_we_o),

    .ACK_O(ram_ack_o),
    .DAT_O(ram_dat_o)
);



// FAKE FLASH
// ---------------------------------------------------------

wire flash_ack_o;
wire [31:0] flash_dat_o;

flash flash 
(
    .RST_I(RST_b),
    .CLK_I(CLK),
    .CYC_I(inc_cyc_o), 
    .STB_I(is_flash & inc_stb_o & inc_cyc_o),
    .ADR_I(inc_adr_o[26:0]),
    .DAT_I(inc_dat_mosi),
    .SEL_I(inc_sel_o),
    .WE_I(inc_we_o),

    .ACK_O(flash_ack_o),
    .DAT_O(flash_dat_o)
);



// INTERRUPT CONTROLLER
// ---------------------------------------------------------
// 0x2000   IRR[2:0] : Interrupt Response Register   [RO]
//                     [2:0] = IRQ ID (active priority)

wire [7:0] ext_irq_bus = {
    uart_irq,
    7'b0
};

wire intc_ack_o;
wire [7:0] intc_dat_o;
wire intc_is_IRR = is_intc & (cold_addr[3:0] == 4'h0);

intcontrol intc 
(
    .RST_I(RST_b),
    .CLK_I(CLK),
    .CYC_I(inc_cyc_o),
    .STB_I(intc_is_IRR & inc_stb_o & inc_cyc_o),
    .ACK_O(intc_ack_o),
    .DAT_O(intc_dat_o),

    .ext_irq_bus_i(ext_irq_bus)
);



// UART
// ---------------------------------------------------------
// 0x3000   RBR[7:0] : Receiver buffer register        [RO]
// 0x3004   THR[7:0] : Transmitter holding register    [WO]
// 0x3008   SR[7:0]  : Status register                 [RO] 
//                     [0] = busy flag
//                     [1] = rx valid

wire uart_ack_o;
wire [7:0] uart_dat_o;
wire uart_irq;

uart_top uart 
(
    .RST_I(RST_b),
    .CLK_I(CLK),
    .CYC_I(inc_cyc_o),
    .STB_I(is_uart & inc_stb_o & inc_cyc_o),
    .ADR_I(cold_addr[3:0]),
    .DAT_I(inc_dat_mosi[7:0]), //TODO: SEL_I()
    .WE_I(inc_we_o),

    .ACK_O(uart_ack_o),
    .DAT_O(uart_dat_o),

    .uart_rx_i(UART_RX_i),

    .uart_tx_o(UART_TX_o),
    .uart_irq_o(uart_irq)
);



// DECODE
// ---------------------------------------------------------

localparam IO_HOT_VGA_BIT = 4;
localparam IO_HOT_RAM_BIT = 8;
localparam IO_HOT_flash_BIT = 11;

// hot
wire [15:0] hot_field = inc_adr_o[31:16];

wire is_flash = hot_field[IO_HOT_flash_BIT];
wire is_ram = (!is_flash & hot_field[IO_HOT_RAM_BIT]);
wire is_VGA = (!is_flash & !hot_field[IO_HOT_RAM_BIT] & hot_field[IO_HOT_VGA_BIT]);

// cold
wire [11:0] cold_addr = inc_adr_o[11:0];
wire [3:0] cold_id = {4{~|hot_field}} & inc_adr_o[15:12];

wire is_system    = (cold_id == 4'h0);
wire is_timer     = (cold_id == 4'h1);
wire is_intc      = (cold_id == 4'h2);
wire is_uart      = (cold_id == 4'h3);
wire is_SPI       = (cold_id == 4'h4);
wire is_GPIO      = (cold_id == 4'h5);

//TODO: if not valid, raise exception
wire is_valid_peripheral = is_system | is_intc | is_uart | is_GPIO | is_ram | is_flash;

endmodule