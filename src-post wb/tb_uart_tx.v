`ifndef DEFINE_ME
    `include "uart_tx.v"
`endif

module UART_TB;
    reg clk;
    reg stb;
    reg [7:0] data;
    
    wire done;
    wire busy;
    wire tx;

    uart_tx TX (
        .clk_i(clk),
        .stb_i(stb),
        .data_i(data),
        .done_o(done),
        .busy_o(busy),
        .tx_o(tx)
    );

    always #10 clk = ~clk;

    // always @(*) begin
    //     if (done) $finish();
    // end

    initial begin
        $dumpfile("E:/Documents/ceRV32/dump/uart_tx.vhd");
        $dumpvars;

        clk <= 0;
        stb <= 0;
        @ (posedge clk);
        stb <= 1;
        data <= 8'b00110010;
        @ (posedge clk);
        stb <= 0;
        #8000
        $finish();
    end

endmodule
