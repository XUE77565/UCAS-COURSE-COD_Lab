`timescale 10 ns / 1 ns

`define DATA_WIDTH 32
`define ADDR_WIDTH 5

module reg_file(
	input                       clk,
	input  [`ADDR_WIDTH - 1:0]  waddr,
	input  [`ADDR_WIDTH - 1:0]  raddr1,
	input  [`ADDR_WIDTH - 1:0]  raddr2,
	input                       wen,
	input  [`DATA_WIDTH - 1:0]  wdata,
	output [`DATA_WIDTH - 1:0]  rdata1,
	output [`DATA_WIDTH - 1:0]  rdata2
);

	// TODO: Please add your logic design here
	// define a register 32*32
	reg [`DATA_WIDTH - 1:0] myreg [`DATA_WIDTH - 1:0];

	//write
	always @(posedge clk) begin
		//check wen and waddr
		if(wen && waddr)begin
			myreg[waddr] <= wdata;
		end
	end

	//read1, set 0 if address = 0
	assign rdata1 = (raddr1 == 0)? 32'h00000000 : myreg[raddr1];
	//read2, set 0 if address = 0
	assign rdata2 = (raddr2 == 0)? 32'h00000000 : myreg[raddr2];

endmodule
