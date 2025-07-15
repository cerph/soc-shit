module uart_rx 
#(
    parameter TICKS_PER_BIT = 32
) 
(
    input clk_i,
    input rx_i,
    input stb_i,
    output wire valid_data_o,
    output wire [7:0] data_o
);

assign data_o = rx_data;
assign valid_data_o = rx_done;

reg [7:0] rx_data = 0;
reg [3:0] rx_bit_counter = 0;
reg [$clog2(TICKS_PER_BIT)-1:0] ticks_counter;

wire full_ticks = (ticks_counter == TICKS_PER_BIT-1);
wire half_ticks = (ticks_counter == TICKS_PER_BIT/2-1);
wire full_rx_bits = rx_bit_counter == 3'h7; // 8 bits done

reg rx_done;
reg rx_sample;

reg [4:0] state;
reg [4:0] next_state;

localparam IDLE     = 5'b00001,
           START    = 5'b00010,
           RECEIVE  = 5'b00100,
           STOP     = 5'b01000,
           DONE     = 5'b10000;

always @(*) begin
    case (state)
        default: begin
            rx_done = 0;
            next_state = IDLE;
        end

        IDLE: begin
            rx_done = 0;
            next_state = (~rx_i) ? START : IDLE;
        end
        START: begin
            rx_done = 0;
            if (half_ticks) begin
                next_state = RECEIVE;
                ticks_counter = 0;
            end
            else begin
                next_state = START;
            end
        end
        RECEIVE: begin
            rx_done = 0;
            next_state = (full_ticks & full_rx_bits) ? STOP : RECEIVE;
        end
        STOP: begin
            rx_done = 0;
            next_state = (full_ticks) ? DONE : STOP;
        end
        DONE: begin
            rx_done = 1;
            next_state = (stb_i) ? IDLE : DONE;
        end
    endcase
end

always @(posedge clk_i) begin
    case (state)
        IDLE: begin
            rx_bit_counter <= 0;
            ticks_counter <= 0;
        end

        START: begin
            ticks_counter <= ticks_counter + 1;
        end

        RECEIVE: begin
            if (full_ticks) begin
                rx_bit_counter <= rx_bit_counter + 1;
                rx_data <= {rx_i, rx_data[7:1]};
                ticks_counter <= 0;
            end 
            else begin
                ticks_counter <= ticks_counter + 1;
            end
        end

        STOP: begin
            ticks_counter <= ticks_counter + 1;
        end

        DONE: begin
            ticks_counter <= 0;
        end
    endcase
end

always @(posedge clk_i) begin
    state <= next_state;
end

endmodule
