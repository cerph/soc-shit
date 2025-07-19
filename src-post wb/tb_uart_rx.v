`ifndef DEFINE_ME
    `include "uart_rx.v"
`endif

`timescale 1ns/1ps

module uart_rx_tb;

    reg clk = 0;
    reg rx = 1; // Idle state for UART is high
    reg stb = 0;

    wire valid;
    wire [7:0] data;

    // Instantiate the UART RX module
    uart_rx #(
        .TICKS_PER_BIT(32)
    ) uart_rx (
        .clk_i(clk),
        .rx_i(rx),
        .stb_i(stb),
        .valid_data_o(valid),
        .data_o(data)
    );

    // Clock generator
    always #5 clk = ~clk; // 100MHz clock (10ns period)

    // UART transmit task (LSB first)
    task uart_send_byte(input [7:0] byte);
        integer i;
        begin
            // Start bit (low)
            rx <= 0;
            #(32*10);

            // Data bits
            for (i = 0; i < 8; i = i + 1) begin
                rx <= byte[i];
                #(32*10);
            end

            // Stop bit (high)
            rx <= 1;
            #(32*10);
        end
    endtask

    initial begin
        $dumpfile("E:/Documents/ceRV32/dump/uart_rx.vhd");
        $dumpvars(0, uart_rx_tb);

        // Give some time for reset
        #(32*10);

        // Send a byte
        uart_send_byte(8'hA5); // 10100101

        // Wait for DONE state to latch output
        #(32*20);

        // Trigger stb to reset the RX module
        stb <= 1;
        #(10);
        stb <= 0;

        // Observe result
        $display("Received: %02x, Valid: %b", data, valid);

        #(100);
        $finish;
    end

endmodule