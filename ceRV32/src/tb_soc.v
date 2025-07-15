`ifndef DEFINE_ME
    `include "soc.v"
`endif

`timescale 1ns/1ps

module soc_tb;
    reg clk = 0;
    reg rst_b = 0;
    reg UART_RX_i = 1;  // idle high
    wire UART_TX_o;
    wire [31:0] GPIO_o;

    soc uut (
        .clk(clk),
        .rst_b(rst_b),
        .UART_RX_i(UART_RX_i),
        .UART_TX_o(UART_TX_o),
        .GPIO_o(GPIO_o)
    );

    // Clock generation: 10ns period => 100MHz
    always #5 clk = ~clk;

    // UART TX Task (from testbench into SoC RX)
    task uart_send_byte;
        input [7:0] data;
        integer i;
        begin
            UART_RX_i = 0; // Start bit
            #(32*10);

            for (i = 0; i < 8; i = i + 1) begin
                UART_RX_i = data[i];
                #(32*10);
            end

            UART_RX_i = 1; // Stop bit
            #(32*10);
        end
    endtask

    initial begin
        $dumpfile("E:/Documents/ceRV32/dump/soc_dump.vhd");
        $dumpvars;

        // Initialize
        #100;
        rst_b = 1;
        #100;

        // Send byte 0xA5 = 8'b10100101
        uart_send_byte(8'hA5);

        // Wait for system to process and re-transmit
        #50000;

        $finish;
    end

endmodule
