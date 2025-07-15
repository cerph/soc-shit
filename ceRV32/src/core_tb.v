`ifndef DEFINE_ME
    `include "soc.v"
`endif

module core_tb;
    reg clk;
    reg rst;

    wire TX;
    wire [31:0] GPIO;
    soc soc (
        .clk(clk),
        .rst_b(rst),

        .UART_TX_o(TX),
        .GPIO_o(GPIO)
    );

    always #10 clk = ~clk;

    initial begin
        $dumpfile("E:/Documents/ceRV32/dump/core_tb_dump.vhd");
        $dumpvars;

        rst <= 0;
        clk <= 0;
        @ (posedge clk);
        rst <= 1;
    end

endmodule