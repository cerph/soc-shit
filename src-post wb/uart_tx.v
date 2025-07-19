module uart_tx 
#(
    parameter TICKS_PER_BIT = 32
) 
(
    input clk_i,
    input stb_i,
    input [7:0] data_i,
    output wire done_o,
    output wire busy_o,
    output wire tx_o
);

assign tx_o = tx_output;
assign done_o = done_flag;
assign busy_o = busy_flag;

reg [7:0] tx_data;
reg [3:0] tx_bit_counter;
reg [$clog2(TICKS_PER_BIT)-1:0] ticks_counter;

wire full_ticks = (ticks_counter == TICKS_PER_BIT-1);
wire full_tx_bits = (tx_bit_counter[3]); // >= 8

reg done_flag;
reg busy_flag;
reg tx_output;


reg [4:0] state;
reg [4:0] next_state;

localparam IDLE    	  = 5'b00001,
           SEND_START = 5'b00010,
           SEND_BITS  = 5'b00100,
           SEND_STOP  = 5'b01000,
           DONE		  = 5'b10000;

always @(*) begin
    case (state)
        default: begin 
            done_flag = 0;
            busy_flag = 0;
            tx_output = 1;
            next_state = IDLE;
        end

        IDLE: begin 
            done_flag = 0;
            busy_flag = 0;
            tx_output = 1; 
            next_state = (stb_i) ? SEND_START : IDLE;
        end 
        SEND_START: begin 
            done_flag = 0;
            busy_flag = 1;
            tx_output = 0; 
            next_state = (full_ticks) ? SEND_BITS : SEND_START;
        end 
        SEND_BITS: begin 
            done_flag = 0;
            busy_flag = 1;
            tx_output = full_tx_bits ? 1'b1 : tx_data[0];
            next_state = (full_tx_bits) ? SEND_STOP : SEND_BITS;
        end
        SEND_STOP: begin 
            done_flag = 0;
            busy_flag = 1;
            tx_output = 1; 
            next_state = (full_ticks) ? DONE : SEND_STOP;
        end 
        DONE: begin 
            done_flag = 1;
            busy_flag = 1;
            tx_output = 1; 
            next_state = IDLE;
        end 
    endcase
end

always @(posedge clk_i) begin 
    state <= next_state;

    if (state == SEND_START || 
        state == SEND_BITS  || 
        state == SEND_STOP) begin
        
        if (full_ticks) 
            ticks_counter <= 0;
        else 
            ticks_counter <= ticks_counter + 1;
    end
    
    if (state == SEND_BITS) begin 
        if (full_ticks) begin
            tx_bit_counter <= tx_bit_counter + 1;
            tx_data <= tx_data >> 1; 
        end
    end

    if (state == IDLE) begin 
        if (stb_i) begin 
            tx_bit_counter <= 0;
            tx_data <= data_i;
        end
    end
end

`ifdef BENCH
    initial begin
        state = IDLE;
        tx_data = 0;
        tx_bit_counter = 0;
        ticks_counter = 0;
    end
`endif

endmodule 