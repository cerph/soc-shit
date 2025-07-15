module test
(
    input [3:0] func3
);

reg [31:0] rs1_r;
reg [31:0] rs2_r;



reg branch_flag;
always @(*) begin
    // case (func3) 
    //     3'b000 : branch_flag = (rs1_r == rs2_r);
    //     3'b001 : branch_flag = (rs1_r != rs2_r);
    //     3'b100 : branch_flag = ($signed(rs1_r) < $signed(rs2_r));
    //     3'b101 : branch_flag = ($signed(rs1_r) >= $signed(rs2_r));
    //     3'b110 : branch_flag = (rs1_r < rs2_r);
    //     3'b111 : branch_flag = (rs1_r >= rs2_r);
    //     default : branch_flag = 1'b0;
    // endcase
    case (func3) 
        3'b000 : branch_flag = (rs1_r == rs2_r);
        3'b001 : branch_flag = (rs1_r != rs2_r);
        3'b100 : branch_flag = ($signed(rs1_r) < $signed(rs2_r));
        3'b101 : branch_flag = ($signed(rs1_r) >= $signed(rs2_r));
        3'b110 : branch_flag = (rs1_r < rs2_r);
        3'b111 : branch_flag = (rs1_r >= rs2_r);
    endcase
end

endmodule
