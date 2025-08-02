`timescale 10ns / 1ns
`include "define.v"

module simple_cpu(
	input             clk,
	input             rst,

	output [31:0]     PC,
	input  [31:0]     Instruction,

	output [31:0]     Address,		//对内存访存的地址
	output            MemWrite,
	output [31:0]     Write_data,
	output [ 3:0]     Write_strb,

	input  [31:0]     Read_data,
	output            MemRead
);

	// THESE THREE SIGNALS ARE USED IN OUR TESTBENCH
	// PLEASE DO NOT MODIFY SIGNAL NAMES
	// AND PLEASE USE THEM TO CONNECT PORTS
	// OF YOUR INSTANTIATION OF THE REGISTER FILE MODULE
	wire			RF_wen;
	wire [4:0]		RF_waddr;
	wire [31:0]		RF_wdata;


	//将指令分层
	wire [5:0] opcode = 	Instruction[31:26];
	wire [5:0] func   =	Instruction[ 5: 0];

	//用于存储译码阶段中读出来的两个数据
	wire [31:0]		rdata1;
	wire [31:0]		rdata2;

	//进行控制信号的声明
	wire		Regdst;
	wire 		Branch;
	wire 		MemtoReg;
	wire		ALUSrc;
	wire   [2:0]    ALUop;
	wire   [1:0]	Shiftop;
	wire		RegWrite;
	wire		Jump;
	wire		AluShi_sel;

	wire 		Ext;
	wire   [1:0]	memLen;

	wire   [15:0]   offset;
	wire   [25:0]	instr_index;
	wire   [31:0]	Jump_addr;
	wire   [31:0]	Branch_addr;

	//定义两个器件的操作数
	wire	[31:0]	ALUop_A;
	wire	[31:0]	ALUop_B;
	wire	[31:0]	Shiftop_A;
	wire	[4:0]	Shiftop_B;

	wire	[4:0]	raddr1;
	wire	[4:0]	raddr2;


	//首先进行id译码
	id 	id_ex(
		.clk		(clk),
		.PC		(PC),
		.Instruction	(Instruction),

		.waddr		(RF_waddr),
		.raddr1		(raddr1),
		.raddr2		(raddr2),

		.Branch		(Branch),
		.MemRead	(MemRead),
		.MemWrite	(MemWrite),
		.MemtoReg	(MemtoReg),
		.ALUop		(ALUop),
		.Shiftop	(Shiftop),
		.RegWrite	(RF_wen),
		.Jump		(Jump),
		
		.AluShi_sel	(AluShi_sel),

		.ALUop_A	(ALUop_A),
		.ALUop_B	(ALUop_B),

		.Shiftop_A	(Shiftop_A),
		.Shiftop_B	(Shiftop_B),

		.Ext		(Ext),
		.memLen		(memLen),

		.offset		(offset),
		.instr_index	(instr_index),

		.Jump_addr	(Jump_addr),
		.Branch_addr	(Branch_addr),

		.rdata1		(rdata1),
		.rdata2		(rdata2)
	);

	wire [31:0]	ex_Result;
	wire 		ALU_Branch;

	//进行寄存器堆的实例化, 要注意整个程序中只能有一个寄存器堆, 否则会产生多个寄存器
	//导致产生并不希望的结果
	reg_file  reg_file_ex(
		.clk		(clk),
		.waddr		(RF_waddr),
		.raddr1		(raddr1),
		.raddr2		(raddr2),
		.wen		(RF_wen),
		.wdata		(RF_wdata),
		.rdata1		(rdata1),
		.rdata2		(rdata2)
	);

	//之后进行ex执行指令阶段
	ex	ex_ex(
		.clk		(clk),
		.PC		(PC),
		.Instruction	(Instruction),

		.ALUop_A	(ALUop_A),
		.ALUop_B	(ALUop_B),
		.ALUop		(ALUop),

		.Shiftop_A	(Shiftop_A),
		.Shiftop_B	(Shiftop_B),
		.Shiftop	(Shiftop),

		.offset		(offset),

		.AluShi_sel	(AluShi_sel),

		.Result		(ex_Result),
		
		.ALU_Branch	(ALU_Branch)
	);

	//对几个端口赋值
	assign Address	  =	ex_Result & 32'hfffffffc;	//对内存访存的地址

	//对于非对其的读和写,其读,写的位置和长度根据所产生的偏移量(也即alu的计算结果)产生
	//产生的结果为读或者写的地址,将这个得到的地址的低两位作为控制信号, 来决定写入的字节位置
	//本次实验中是小尾端
	wire [1:0] effaddr_ctrl;
	assign effaddr_ctrl = ex_Result[1:0];
	
	//用于选择左非对齐读入和右非对齐写入,0为L,1为R
	wire rl_sel;		
	assign rl_sel = opcode[2];

	//将从内存读出来的4个字节数据分成4个字节,用于小尾端和非对齐写入
	wire [7:0] read_byte_3;
	wire [7:0] read_byte_2;
	wire [7:0] read_byte_1;
	wire [7:0] read_byte_0;

	assign read_byte_3 = Read_data [31:24];
	assign read_byte_2 = Read_data [23:16];
	assign read_byte_1 = Read_data [15: 8];
	assign read_byte_0 = Read_data [ 7: 0];

	//用于记录写回寄存器原本的数据, 用于生成RF_wdata
	wire [31:0] wdata_i;
	assign  wdata_i   =	rdata2;
	
	assign RF_wdata  =	(opcode==`JAL || (opcode==`R_TYPE && func==`JALR))?			PC+8:		//对JAL和JALR
				(opcode==`R_TYPE || opcode[5:3]==3'b001)?		ex_Result:			//对于JALR指令和I型计算指令, 将PC+8写回							
				(memLen==2'b00)?		((effaddr_ctrl==2'b00)? 	((Ext==1)?	{{24{read_byte_0[7]}},read_byte_0}:		{{24{1'b0}},read_byte_0}):	//对于读字节的操作, 其中第二个信号用于控制读入的字节在4个字节中位于哪个位置
								 (effaddr_ctrl==2'b01)? 	((Ext==1)?	{{24{read_byte_1[7]}},read_byte_1}:		{{24{1'b0}},read_byte_1}):
								 (effaddr_ctrl==2'b10)? 	((Ext==1)?	{{24{read_byte_2[7]}},read_byte_2}:		{{24{1'b0}},read_byte_2}):
								 			 	((Ext==1)?	{{24{read_byte_3[7]}},read_byte_3}:		{{24{1'b0}},read_byte_3})			
								 															):			
				(memLen==2'b01)?		((effaddr_ctrl==2'b00)?		((Ext==1)? 	{{16{read_byte_1[7]}},read_byte_1,read_byte_0}:	{{16{1'b0}},read_byte_1,read_byte_0}):
												((Ext==1)? 	{{16{read_byte_3[7]}},read_byte_3,read_byte_2}:	{{16{1'b0}},read_byte_3,read_byte_2})
																							):	//对于对齐读半字的操作												
				({rl_sel,memLen}==3'b010)?	((effaddr_ctrl==2'b00)?		{read_byte_0,wdata_i[23:0]}:				//表示010, 非对齐左读入的情况, 一下为四种非对齐写入的情况
								 (effaddr_ctrl==2'b01)?		{read_byte_1,read_byte_0,wdata_i[15:0]}:
								 (effaddr_ctrl==2'b10)?		{read_byte_2,read_byte_1,read_byte_0,wdata_i[7:0]}:
								 				{read_byte_3,read_byte_2,read_byte_1,read_byte_0}
								):
				({rl_sel,memLen}==3'b110)?	((effaddr_ctrl==2'b00)?		{read_byte_3,read_byte_2,read_byte_1,read_byte_0}:	//表示110, 非对齐右读入的情况, 一下为四种非对齐写入的情况
								 (effaddr_ctrl==2'b01)?		{wdata_i[31:24],read_byte_3,read_byte_2,read_byte_1}:
								 (effaddr_ctrl==2'b10)?		{wdata_i[31:16],read_byte_3,read_byte_2}:
								 				{wdata_i[31:8],read_byte_3}):
												Read_data;	//表示读整个字的操作

	//对写内存进行strb的赋值
	assign  Write_strb     =	(memLen==2'b00)?	{(effaddr_ctrl==2'b11),(effaddr_ctrl==2'b10),(effaddr_ctrl==2'b01),(effaddr_ctrl==2'b00)}:	//写字节, 用effaddr的最后两位来控制写的位置
					(memLen==2'b01)?	{effaddr_ctrl[1],effaddr_ctrl[1],~effaddr_ctrl[1],~effaddr_ctrl[1]}:				//写半字,10时为1100,01时为0011
					({rl_sel,memLen}==3'b010)?	((effaddr_ctrl==2'b00)?	4'b0001:
									 (effaddr_ctrl==2'b01)?	4'b0011:
									 (effaddr_ctrl==2'b10)?	4'b0111:
									 			4'b1111):
					({rl_sel,memLen}==3'b110)?	((effaddr_ctrl==2'b00)?	4'b1111:
									 (effaddr_ctrl==2'b01)?	4'b1110:
									 (effaddr_ctrl==2'b10)?	4'b1100:
									 			4'b1000):
												4'b1111;							//表示写整个字
	
	//由于非对齐和对齐中写入的字节不同,于是更改Write_data;
	wire [5:0] Write_data_sel;
	assign Write_data_sel   =	{rl_sel,memLen,effaddr_ctrl};

	//将从寄存器读出来的4个字节数据分成4个字节,用于小尾端和非对齐写入
	wire [7:0] wb3;
	wire [7:0] wb2;
	wire [7:0] wb1;
	wire [7:0] wb0;

	assign wb3 = rdata2 [31:24];
	assign wb2 = rdata2 [23:16];
	assign wb1 = rdata2 [15: 8];
	assign wb0 = rdata2 [ 7: 0];

	assign Write_data   =	(memLen==2'b00)?		((effaddr_ctrl==2'b00)?		{{24{1'b0}},wb0}:		//针对写入字节的情况
								 (effaddr_ctrl==2'b01)?		{{16{1'b0}},wb0,{8{1'b0}}}:
								 (effaddr_ctrl==2'b10)?		{{8{1'b0}},wb0,{16{1'b0}}}:
								 				{wb0,{24{1'b0}}}):
				(memLen==2'b01)?		((effaddr_ctrl[1]==0)?		{{16{1'b0}},wb1,wb0}:		
												{wb1,wb0,{16{1'b0}}}):
				(Write_data_sel==5'b01000)?	{{24{1'b0}},wb3}:
				(Write_data_sel==5'b01001)?	{{16{1'b0}},wb3,wb2}:
				(Write_data_sel==5'b01010)?	{{8{1'b0}},wb3,wb2,wb1}:
				(Write_data_sel==5'b11001)?	{wb2,wb1,wb0,{8{1'b0}}}:
				(Write_data_sel==5'b11010)?	{wb1,wb0,{16{1'b0}}}:
				(Write_data_sel==5'b11011)?	{wb0,{24{1'b0}}}:
				rdata2;

	pc	pc_ex(
		.clk		(clk),
		.rst		(rst),
		
		.Branch		(Branch),
		.ALU_Branch	(ALU_Branch),
		.Jump		(Jump),
		.Branch_addr	(Branch_addr),
		.Jump_addr	(Jump_addr),

		.PC		(PC)
	);
endmodule
