`timescale 10ns / 1ns
`include "define.v"

module custom_cpu(
	input         clk,
	input         rst,

	//Instruction request channel
	output [31:0] PC,
	output        Inst_Req_Valid,
	input         Inst_Req_Ready,

	//Instruction response channel
	input  [31:0] Instruction,
	input         Inst_Valid,
	output        Inst_Ready,

	//Memory request channel
	output [31:0] Address,
	output        MemWrite,
	output [31:0] Write_data,
	output [ 3:0] Write_strb,
	output        MemRead,
	input         Mem_Req_Ready,

	//Memory data response channel
	input  [31:0] Read_data,
	input         Read_data_Valid,
	output        Read_data_Ready,

	input         intr,

	output [31:0] cpu_perf_cnt_0,
	output [31:0] cpu_perf_cnt_1,
	output [31:0] cpu_perf_cnt_2,
	output [31:0] cpu_perf_cnt_3,
	output [31:0] cpu_perf_cnt_4,
	output [31:0] cpu_perf_cnt_5,
	output [31:0] cpu_perf_cnt_6,
	output [31:0] cpu_perf_cnt_7,
	output [31:0] cpu_perf_cnt_8,
	output [31:0] cpu_perf_cnt_9,
	output [31:0] cpu_perf_cnt_10,
	output [31:0] cpu_perf_cnt_11,
	output [31:0] cpu_perf_cnt_12,
	output [31:0] cpu_perf_cnt_13,
	output [31:0] cpu_perf_cnt_14,
	output [31:0] cpu_perf_cnt_15,

	output [69:0] inst_retire
);

/* The following signal is leveraged for behavioral simulation, 
* which is delivered to testbench.
*
* STUDENTS MUST CONTROL LOGICAL BEHAVIORS of THIS SIGNAL.
*
* inst_retired (70-bit): detailed information of the retired instruction,
* mainly including (in order) 
* { 
*   reg_file write-back enable  (69:69,  1-bit),
*   reg_file write-back address (68:64,  5-bit), 
*   reg_file write-back data    (63:32, 32-bit),  
*   retired PC                  (31: 0, 32-bit)
* }
*
*/

//wire [69:0] inst_retire;




//=====================================================
//=====================================================
//===================状态机FSM部分======================
//=====================================================
//=====================================================

//定义寄存器状态
reg	[8:0]	current_state;
reg	[8:0]	next_state;
//定义状态
localparam 	INIT	=	9'b000000001,
		IF	=	9'b000000010,
		IW	=	9'b000000100,
		ID	=	9'b000001000,
		EX	=	9'b000010000,
		ST	=	9'b000100000,
		LD	=	9'b001000000,
		RDW	=	9'b010000000,
		WB	=	9'b100000000;


//定义next_state, 利用连续赋值always @(*), 此处主要的问题是比较复杂的EX状态
always @(*) begin
	case(current_state)
		INIT:	next_state	=	IF;
		IF: begin
			if(Inst_Req_Ready) begin
				next_state	=	IW;
			end
			else begin
				next_state	=	IF;
			end
		end
		IW: begin
			if(Inst_Valid) begin
				next_state	=	ID;
			end
			else begin
				next_state	=	IW;
			end
		end
		ID: begin
			if(Instruction_current[31:0] == 32'b0) begin
				next_state	=	IF;
			end
			else begin
				next_state	=	EX;
			end
		end
		EX: begin	
			if(opcode==`REGIMM || opcode[5:2]==4'b0001 || opcode==`J) begin //表示直接返回取指阶段的三种指令类型, REGIMM, I跳转和J
				next_state	=	IF;
			end
			else if (opcode[5:3]==3'b101) begin
				next_state	=	ST;
			end
			else if (opcode[5:3]==3'b100) begin
				next_state	=	LD;
			end
			else begin
				next_state	=	WB;
			end
		end
		ST: begin
			if(Mem_Req_Ready) begin
				next_state	=	IF;
			end
			else begin
				next_state	=	ST;	
			end
		end
		LD: begin
			if(Mem_Req_Ready) begin
				next_state	=	RDW;
			end
			else begin
				next_state	=	LD;
			end
		end
		RDW: begin
			if(Read_data_Valid) begin
				next_state	=	WB;
			end
			else begin
				next_state	=	RDW;
			end
		end
		WB: begin
			next_state	=	IF;
		end
		default : begin
			next_state	=	INIT;
		end
	endcase
end

//定义状态转移
always @(posedge clk) begin
	if(rst) begin
		current_state 	<=	INIT;
	end
	else begin
		current_state	<=	next_state;
	end
end



//第三段状态机
assign Inst_Req_Valid 	= (current_state == IF);
assign Inst_Ready     	= (current_state == INIT || current_state == IW);
assign MemRead	      	= (current_state == LD);
assign MemWrite	     	= (current_state == ST);
assign Read_data_Ready 	= (current_state == RDW || current_state == INIT);
//=====================================================
//=====================================================
//=======================取指部分=======================
//=====================================================
//=====================================================

//取指阶段的握手机制, 只有在握手成功的时候, 才更改当前的指令
reg  [31:0] Instruction_current;
always @(posedge clk) begin
	if(rst) begin
		Instruction_current   <=	32'b0;
	end
	else if(Inst_Ready && Inst_Valid) begin
		Instruction_current	<=	Instruction;
	end
	else begin
		Instruction_current	<=	Instruction_current;
	end
end

//指令分段
wire [5:0] opcode 	=	Instruction_current [31:26];
wire [4:0] rs	  	= 	Instruction_current [25:21];
wire [4:0] rt	  	= 	Instruction_current [20:16];
wire [4:0] rd     	= 	Instruction_current [15:11];
wire [4:0] shamt  	= 	Instruction_current [10:6];
wire [5:0] func   	= 	Instruction_current [5:0];
//add I-type branch wire
wire [15:0] offset   	= 	Instruction_current [15:0];
//add I-type calculate wire
wire [15:0] imm	        = 	Instruction_current [15:0];
//add I-type memory wire
wire [4:0] base	        = 	Instruction_current [25:21];	//和rs在位置上等价
//add REGIMM wire
 wire [4:0] REG = Instruction_current [20:16];
//add J-type wire
wire [25:0] instr_index;
 assign instr_index = Instruction_current [25:0];

//=====================================================
//=====================================================
//=======================译码部分=======================
//=====================================================
//=====================================================
//进行端口定义
wire [4:0]		RF_waddr;
wire [31:0]		RF_wdata;

//用于存储译码阶段中读出来的两个数据
wire [31:0]		rdata1;
wire [31:0]		rdata2;

//进行控制信号的声明
wire 		Branch;
wire   [2:0]    ALUop;
wire   [1:0]	Shiftop;
wire		Jump;
wire		AluShi_sel;

wire 		Ext;
wire   [1:0]	memLen;

wire   [31:0]	Jump_addr;
wire   [31:0]	Branch_addr;

//定义两个器件的操作数
reg	[31:0]	ALUop_A;
reg	[31:0]	ALUop_B;
reg	[31:0]	Shiftop_A;
reg	[4:0]	Shiftop_B;

wire	[4:0]	raddr1;
wire	[4:0]	raddr2;

//寄存器输入控制信号
assign RegDst = (opcode==`R_TYPE)? 1:0;

//读寄存器地址
assign raddr1 = rs;
assign raddr2 = rt;

//写寄存器地址
assign  RF_waddr  =	(RegDst==1)?		rd:
			(opcode==`JAL)?		5'b11111:
					rt;

//对立即数进行拓展(零拓展和符号位拓展),这个拓展也包含了offset的拓展,因为offset都是符号位拓展
wire [31:0]  op_imm;
assign op_imm	=	(opcode[5:2]==4'b0011)?		{{16{1'b0}},imm}:	//零扩展
							{{16{imm[15]}},imm};	//符号扩展
//运算器功能选择信号

assign ALUop =  ((opcode[5]==1) || (opcode==`ADDIU) || (opcode==`R_TYPE && (func==`ADDU || func[5:3]==3'b001)))? 	`ADD :	//用到add的情况
	        ((opcode==`R_TYPE && func==`SUBU) || opcode[5:1]==5'b00010)?						`SUB :  //用到sub的情况
		((opcode==`R_TYPE && func==`AND_) || opcode==`ANDI)?							`AND :  //用到and的情况
		((opcode==`R_TYPE && func==`OR_) || opcode==`ORI)?							`OR  :	//用到or的情况
		((opcode==`R_TYPE && func==`XOR_) || opcode==`XORI)?							`XOR :  //用到xor的情况
		(opcode==`R_TYPE && func==`NOR_)?									`NOR :  //用到NOR的情况
		((opcode==`R_TYPE && func==`SLT_) || opcode==`SLTI || opcode==`REGIMM || opcode[5:1]==5'b00011)?	`SLT : 	//用到SLT的情况
		((opcode==`R_TYPE && func==`SLTU_) || opcode==`SLTIU)?							`SLTU:	//用到SLTU的情况
															`OR;	//有的情况不需要ALU,默认赋值为or

//运算器操作数A选择信号
always @(posedge clk) begin
	if(rst) begin
		ALUop_A   <=	32'b0;
	end
	else if(current_state==ID) begin 
		ALUop_A   <=	(opcode==`BGTZ)?			32'b0:		//0<rdata1时为1,表示rdata大于0时分支
				(opcode==`REGIMM && REG[0]==1)?		{32{1'b1}}:	//-1<rdata1时为1,表示rdata大于等于0时分支		
									rdata1;
	end
	else begin
		ALUop_A <= ALUop_A;
	end
end

//运算器输入选择控制信号, rdata or imm 如果是Rtype或者beq和bne则输入rdata2
assign ALUSrc = (opcode==`R_TYPE || opcode[5:1]==5'b00010)? 0:1;

//运算器操作数B选择信号
always @(posedge clk) begin
	if(rst) begin
		ALUop_B   <=	32'b0;
	end
	else if(current_state==ID) begin 
		ALUop_B	  <=	(opcode==`R_TYPE && (func==`MOVZ || func==`MOVN))?		32'b0:		//对于R中的两个移动指令,ALUop为add,因此rdata1+0
				(opcode==`REGIMM && REG[0]==0)?					32'b0:		//rdata1小于0时分支
				(opcode==`BLEZ)?						32'b1:		//rdata1小于1时分支,也就是小于等于0时分支
				(opcode==`BGTZ || (opcode==`REGIMM && REG[0]==1))?		rdata1:		//对于上面的两种情况,B应该是rdata1
				(ALUSrc)?							op_imm:
												rdata2;
	end
	else begin
		ALUop_B <= ALUop_B;
	end
end


//移位器选择信号
assign Shiftop = ((opcode==`R_TYPE && func[5:3]==3'b000))? 	func[1:0] :			//Rtype算数移位指令
		 ((opcode==`LUI))? 			   	2'b00     :			//加载到高位
		 						2'b01;				//其他情况下不移位	

//移位器操作数A选择信号
always @(posedge clk) begin
	if(rst) begin
		Shiftop_A   <=	32'b0;
	end
	else if(current_state==ID) begin
		Shiftop_A  <=	(opcode==`LUI)?		op_imm:		//LUI指令对立即数做移位
							rdata2;		//rt移动
	end
	else begin
		Shiftop_A <= Shiftop_A;
	end
end

//移位器操作数B选择信号
always @(posedge clk) begin
	if(rst) begin
		Shiftop_B   <=	5'b0;
	end
	else if(current_state==ID) begin
		Shiftop_B  <=   (opcode==`R_TYPE && rs==5'b0)?		shamt:
				(opcode==`LUI)? 			5'b10000:	//LUI指令将立即数左移16位
									rdata1[4:0];	//可变目标左右移量为rs[4:0]
	end
	else begin
		Shiftop_B <= Shiftop_B;
	end
end

//定义一个中间控制信号,用于控制movn和movz
wire mov_con;
assign mov_con = (rdata2 == 32'b0);

wire    RF_wen_i;
assign  RF_wen_i  = 	(opcode==`R_TYPE && func==`JR)?										0:		//JR时不写入
			((opcode == `R_TYPE && {func[5:3],func[1]}!=4'b0011) || opcode[5:3]==3'b001 || opcode[5:3]==3'b100)? 	1:
			(opcode==`R_TYPE && func==`MOVZ)?									mov_con:	//rt==0则写入
			(opcode==`R_TYPE && func==`MOVN)?									~mov_con:	//rt!=0则写入
			(opcode==`JAL)?												1:		//JAL的情况, 拉高wen
																0;		//其他情况下不写入

//移位器和运算器结果二选一信号
assign AluShi_sel = ((opcode==`R_TYPE && func[5:3]==3'b000) || opcode==`LUI);			//结果为1,选择shifter,否则选择alu

//分支控制信号
assign Branch = (opcode[5:2]==4'b0001 || opcode==`REGIMM);

//跳转控制信号
assign Jump   = (opcode[5:1]==5'b00001 || (opcode==`R_TYPE && func[3:1]==3'b100) || opcode==`JAL);

//内存读出时的拓展控制信号
assign Ext =	(opcode[5:4]==2'b10 && (opcode[2]==0))? 	1 : 	//符号扩展
								0;  	//零扩展

assign memLen = ({opcode[5:4],opcode[1:0]}==4'b1000)?		2'b00:	//字节
		({opcode[5:4],opcode[1:0]}==4'b1001)?		2'b01:	//半字
		({opcode[5:4],opcode[1:0]}==4'b1010)?		2'b10:	//低两位为10的时候是非对齐, 用10来标记非对齐
		({opcode[5:4],opcode[1:0]}==4'b1011)?		2'b11:	//字
								2'b00;	//默认	

//计算出分支目标地址
wire [15:0]	offset_temp;
assign offset_temp  =  offset << 2;
assign Branch_addr  =   {{16{offset_temp[15]}},offset_temp};

//计算出跳转目标地址
assign Jump_addr    =	(opcode[5:1]==5'b00001)?	{PC_reg[31:28],instr_index,2'b00} :	//J型指令
							rdata1;				//R型跳转指令

//=====================================================
//=====================实例化regfile====================
//=====================================================
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

//=====================================================
//=====================================================
//=======================执行部分=======================
//===================ALU和Shifter不用改=================
//=====================================================
wire Zero;
wire [31:0] ALU_result;
wire [31:0] Shifter_result;


reg  [31:0] Result;

alu alu_ex(
	.A              (ALUop_A),
	.B              (ALUop_B),
	.ALUop          (ALUop),
	.Overflow       (),
	.CarryOut       (),
	.Zero           (Zero),
	.Result         (ALU_result)     
);

//实例化Shifter

shifter shifter_ex(
	.A              (Shiftop_A),
	.B              (Shiftop_B),
	.Shiftop        (Shiftop),
	.Result         (Shifter_result)
);

//在Shifter和ALU的结果中二选一,输出一个结果
always @(posedge clk) begin
	if(rst) begin
		Result   <=	32'b0;
	end
	else if(current_state==EX) begin
		Result <= (AluShi_sel)?   Shifter_result: ALU_result;
	end
	else begin
		Result <= Result;
	end
end

//对于分支指令,将其分支控制信号赋值
wire   ALU_Branch_temp;
assign ALU_Branch_temp = (opcode[5:1]==5'b00011 || opcode==`REGIMM)?    	ALU_result :    //对于比大小的分支指令
				(opcode==`BEQ)?                                 Zero    :       //对于beq,  两个相等, 跳转
				(opcode==`BNE)?                                 ~Zero   :       //对于bne, 两个不相等, 跳转
										0;              //其他一般情况下, 不跳转
assign ALU_Branch      = ALU_Branch_temp; 
//=====================================================
//=====================================================
//=======================PC部分========================
//=====================================================
//=====================================================
wire Branch_f;
assign Branch_f = Branch && ALU_Branch;

reg [31:0] PC_reg;
always @(posedge clk) begin
	if (rst) begin
		PC_reg	<=	32'b0;
	end
	else if(Instruction_current==32'b0 && current_state==ID) begin //表示NOP指令, 在译码后直接回到取指阶段
		PC_reg	<=	PC_reg + 4;
	end
	else if(current_state==EX) begin	//对于其他类型的指令, 根据指令类型来判断跳转
		PC_reg	<=	(Jump? 	   Jump_addr:
				 Branch_f? PC_reg + Branch_addr + 4:
				 	   PC_reg + 4);
	end
	else begin
		PC_reg	<=	PC_reg;		//对于其他状态, PC不改变
	end
end

assign PC = PC_reg;
//要注意的是, 我们在对指令JAL以及JALR的阶段写回时, 用的是未更新的PC, 所以需要一个寄存器用来存储改变前的PC
reg [31:0] old_PC;

always @(posedge clk) begin
	if(rst) begin
		old_PC   <=	32'b0;
	end
	else if (current_state==IF) begin
		old_PC	<=  PC_reg;
	end
	else begin
		old_PC	<=  old_PC;
	end
end

//=====================================================
//=====================================================
//=======================写回部分=======================
//=====================================================
//=====================================================
wire   RF_wen;
assign RF_wen 	=  RF_wen_i && (current_state==WB);	//写回只可能在WB状态进行写回
assign Address  =  Result & 32'hfffffffc;		//对内存访存的地址

//对于非对其的读和写,其读,写的位置和长度根据所产生的偏移量(也即alu的计算结果)产生
//产生的结果为读或者写的地址,将这个得到的地址的低两位作为控制信号, 来决定写入的字节位置
//本次实验中是小尾端
wire [1:0] effaddr_ctrl;
assign effaddr_ctrl = Result[1:0];

//用于选择左非对齐读入和右非对齐写入,0为L,1为R
wire rl_sel;		
assign rl_sel = opcode[2];

//将从内存读出来的4个字节数据分成4个字节,用于小尾端和非对齐写入
wire [7:0] read_byte_3;
wire [7:0] read_byte_2;
wire [7:0] read_byte_1;
wire [7:0] read_byte_0;

reg [31:0] Read_data_current;

always @(posedge clk) begin
	if(rst) begin
		Read_data_current  <=	32'b0;
	end
	else if(Read_data_Ready && Read_data_Valid) begin
		Read_data_current <= Read_data;
	end
	else begin
		Read_data_current <= Read_data_current;
	end
end

assign read_byte_3 = Read_data_current [31:24];
assign read_byte_2 = Read_data_current [23:16];
assign read_byte_1 = Read_data_current [15: 8];
assign read_byte_0 = Read_data_current [ 7: 0];

//用于记录写回寄存器原本的数据, 用于生成RF_wdata
wire [31:0] wdata_i;
assign      wdata_i   =	rdata2;

assign RF_wdata  =	(opcode==`JAL || (opcode==`R_TYPE && func==`JALR))?			old_PC+8:	//对JAL和JALR
			(opcode==`R_TYPE || opcode[5:3]==3'b001)?				Result:		//对于JALR指令和I型计算指令, 将PC+8写回							
			(memLen==2'b00)?		((effaddr_ctrl==2'b00)? 	((Ext==1)?	{{24{read_byte_0[7]}},read_byte_0}:		{{24{1'b0}},read_byte_0}):	//对于读字节的操作, 其中第二个信号用于控制读入的字节在4个字节中位于哪个位置
							(effaddr_ctrl==2'b01)? 		((Ext==1)?	{{24{read_byte_1[7]}},read_byte_1}:		{{24{1'b0}},read_byte_1}):
							(effaddr_ctrl==2'b10)? 		((Ext==1)?	{{24{read_byte_2[7]}},read_byte_2}:		{{24{1'b0}},read_byte_2}):
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
											Read_data_current;	//表示读整个字的操作

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
wire [4:0] Write_data_sel;
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

			
//=====================================================
//=====================================================
//=====================计数器部分=======================
//=====================================================
//=====================================================

reg [31:0] cycle_cnt;

always @(posedge clk) begin
	if(rst) begin
		cycle_cnt <= 32'd0;
	end
	else begin
		cycle_cnt <= cycle_cnt + 32'd1;
	end
end

assign cpu_perf_cnt_0 = cycle_cnt;

reg [31:0] ins_cnt;

always @(posedge clk) begin
	if(rst) begin
		ins_cnt <= 32'd0;
	end
	else if (Inst_Ready && Inst_Valid )begin
		ins_cnt <= ins_cnt + 32'd1;
	end
	else begin
		ins_cnt <= ins_cnt;
	end
end

assign cpu_perf_cnt_1 = ins_cnt;

reg [31:0] mem_cnt;

always @(posedge clk) begin
	if(rst) begin
		mem_cnt <= 32'd0;
	end
	else if (Read_data_Ready && Read_data_Valid)begin
		mem_cnt <= mem_cnt + 32'd1;
	end
	else begin
		mem_cnt <= mem_cnt;
	end
end

assign cpu_perf_cnt_2 = mem_cnt;


endmodule
