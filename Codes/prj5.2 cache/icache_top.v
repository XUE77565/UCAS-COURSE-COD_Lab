`timescale 10ns / 1ns

`define CACHE_SET	8
`define CACHE_WAY	4
`define TAG_LEN		24
`define LINE_LEN	256

module icache_top (
	input	      clk,
	input	      rst,
	
	//CPU interface
	/** CPU instruction fetch request to Cache: valid signal */
	input         from_cpu_inst_req_valid,
	/** CPU instruction fetch request to Cache: address (4 byte alignment) */
	input  [31:0] from_cpu_inst_req_addr,
	/** Acknowledgement from Cache: ready to receive CPU instruction fetch request */
	output        to_cpu_inst_req_ready,
	
	/** Cache responses to CPU: valid signal */
	output        to_cpu_cache_rsp_valid,
	/** Cache responses to CPU: 32-bit Instruction value */
	output [31:0] to_cpu_cache_rsp_data,
	/** Acknowledgement from CPU: Ready to receive Instruction */
	input	      from_cpu_cache_rsp_ready,

	//Memory interface (32 byte aligned address)
	/** Cache sending memory read request: valid signal */
	output        to_mem_rd_req_valid,
	/** Cache sending memory read request: address (32 byte alignment) */
	output [31:0] to_mem_rd_req_addr,	//32bit对齐, 低5位一定是20
	/** Acknowledgement from memory: ready to receive memory read request */
	input         from_mem_rd_req_ready,

	/** Memory return read data: valid signal of one data beat */
	input         from_mem_rd_rsp_valid,
	/** Memory return read data: 32-bit one data beat */
	input  [31:0] from_mem_rd_rsp_data,
	/** Memory return read data: if current data beat is the last in this burst data transmission */
	input         from_mem_rd_rsp_last,
	/** Acknowledgement from cache: ready to receive current data beat */
	output        to_mem_rd_rsp_ready
);

//TODO: Please add your I-Cache code here

//cache状态机部分
localparam	WAIT	  =	8'b00000001,
		TAG_RD	  =	8'b00000010,
		EVICT	  =	8'b00000100,
		MEM_RD	  =	8'b00001000,
		CACHE_RD  =	8'b00010000,
		RECV	  =	8'b00100000,
		REFILL	  =	8'b01000000,
		RESP	  =	8'b10000000;

reg  [7:0]  icache_current_state;
reg  [7:0]  icache_next_state;


always @(posedge clk)begin
	if(rst) begin
		icache_current_state <=	WAIT;
	end
	else begin
		icache_current_state <= icache_next_state;
	end
end


always @(*) begin
	case (icache_current_state) 
		WAIT:begin
			if(from_cpu_inst_req_valid) begin
				icache_next_state = TAG_RD;
			end
			else begin
				icache_next_state = WAIT;
			end
		end
		TAG_RD:begin
			if(~Read_hit) begin
				icache_next_state = EVICT;
			end
			else if(Read_hit) begin
				icache_next_state = CACHE_RD;
			end
			else begin
				icache_next_state = TAG_RD;
			end
		end
		EVICT:begin
			icache_next_state = MEM_RD;
		end
		MEM_RD:begin
			if(from_mem_rd_req_ready) begin
				icache_next_state = RECV;
			end
			else begin
				icache_next_state = MEM_RD;
			end
		end
		RECV:begin
			if(from_mem_rd_rsp_valid && from_mem_rd_rsp_last) begin //每次valid读入4个字节, last标记最后一个4byte已经被接受
				icache_next_state = REFILL;
			end
			else begin
				icache_next_state = RECV;
			end
		end
		REFILL: begin
			icache_next_state = RESP;
		end
		CACHE_RD:begin
			icache_next_state = RESP;
		end
		RESP: begin
			if(from_cpu_cache_rsp_ready) begin
				icache_next_state = WAIT;
			end
			else begin
				icache_next_state = RESP;//防止锁存
			end
		end
		default: begin
			icache_next_state = WAIT;
		end
	endcase
end
//对于发出的控制信号进行赋值
assign 	to_cpu_inst_req_ready 	=	(icache_current_state==WAIT);
assign	to_cpu_cache_rsp_valid	=	(icache_current_state==RESP);
assign	to_mem_rd_req_valid	=	(icache_current_state==MEM_RD);
assign	to_mem_rd_rsp_ready	=	(icache_current_state==RECV) || (icache_current_state == WAIT);

reg	[31:0]	cpu_inst_reg;

always @(posedge clk) begin
	if(rst) begin
		cpu_inst_reg <= 32'b0;
	end
	else if(from_cpu_inst_req_valid && to_cpu_inst_req_ready) begin
		cpu_inst_reg <= from_cpu_inst_req_addr;
	end
	else begin
		cpu_inst_reg <= cpu_inst_reg;

	end
end

//分段cpu请求
wire	[`TAG_LEN-1:0]	tag	=	cpu_inst_reg[31:8];
wire	[2:0]		index	=	cpu_inst_reg[7:5];	//index标记了在一路cache块中的地址
wire	[4:0]		offset  =	cpu_inst_reg[4:0];


//对于每一路内的valid
reg	[`CACHE_SET-1:0]   valid_way0;
reg	[`CACHE_SET-1:0]   valid_way1;
reg	[`CACHE_SET-1:0]   valid_way2;
reg	[`CACHE_SET-1:0]   valid_way3;


//对于命中的逻辑
wire	Read_hit;

//4路都有可能命中, index对应的valid以及tag和指令本身的tag相等
wire	hit_way0;
wire	hit_way1;
wire	hit_way2;
wire	hit_way3;

wire	[23:0]  tag_way0;
wire	[23:0]  tag_way1;
wire	[23:0]  tag_way2;
wire	[23:0]  tag_way3;

assign	hit_way0 = valid_way0[index] && (tag_way0==tag);
assign	hit_way1 = valid_way1[index] && (tag_way1==tag);
assign	hit_way2 = valid_way2[index] && (tag_way2==tag);
assign	hit_way3 = valid_way3[index] && (tag_way3==tag);

assign  Read_hit = hit_way0 || hit_way1 || hit_way2 || hit_way3;

//利用PLRU来决定要替换的数据, 因为一个cache块中有8个set, 所以对于每一个set都要有一条PLRU树
reg [2:0] plru_tree [7:0];

//如果一个set的四个都被用了, 才需要进行替换
wire   	valid_all;
assign 	valid_all = valid_way0[index] && valid_way1[index] && valid_way2[index] && valid_way3[index];

wire	replace_way0;
wire	replace_way1;
wire	replace_way2;
wire	replace_way3;

//每次的PLRU树都指向最不被常访问的通道, 改写或者直接填入
assign	replace_way0 = (valid_all && ~plru_tree[index][0] && ~plru_tree[index][1]) || ~valid_way0[index];
assign	replace_way1 = (valid_all && ~plru_tree[index][0] && plru_tree[index][1]) || ~valid_way1[index];
assign	replace_way2 = (valid_all && plru_tree[index][0] && ~plru_tree[index][2]) || ~valid_way2[index];
assign	replace_way3 = (valid_all && plru_tree[index][0] && plru_tree[index][2]) || ~valid_way3[index];

//对PLRU树进行更新, visit表示击中或者替换
reg	visit_way0;
reg	visit_way1;
reg	visit_way2;
reg	visit_way3;

always @(posedge clk) begin
	if(rst) begin
		plru_tree[0]=3'b0; plru_tree[1]=3'b0; plru_tree[2]=3'b0; plru_tree[3]=3'b0;
		plru_tree[4]=3'b0; plru_tree[5]=3'b0; plru_tree[6]=3'b0; plru_tree[7]=3'b0;
	end
	else if(icache_current_state==RESP || icache_current_state == CACHE_RD) begin	//每次应答的时候更新plru树
		if(visit_way0 || visit_way1) begin
			plru_tree[index][0] <= 1;
			plru_tree[index][1] <= visit_way0;//如果访问了0, 就指向右边的1
		end
		else if(visit_way2 || visit_way3)begin
			plru_tree[index][0] <= 0;
			plru_tree[index][2] <= visit_way2;
		end
	end
end


//下面对于读和replace等部分更新4路的visit值, 最好一个always块只对一个赋值, 不过这里貌似不影响
reg	Read_hit_reg;

always @(posedge clk) begin
	if(rst) begin
		Read_hit_reg <= 0;
		visit_way0   <= 0;
		visit_way1   <= 0;
		visit_way2   <= 0;
		visit_way3   <= 0;
	end
	else if(icache_current_state == TAG_RD) begin//对于命中的部分
		Read_hit_reg <= Read_hit;
		visit_way0   <= hit_way0;
		visit_way1   <= hit_way1;
		visit_way2   <= hit_way2;
		visit_way3   <= hit_way3;
	end
	else if(icache_current_state == EVICT) begin//对于更新的部分
		visit_way0   <= replace_way0;
		visit_way1   <= replace_way1;
		visit_way2   <= replace_way2;
		visit_way3   <= replace_way3;
	end
	else begin
		Read_hit_reg <= Read_hit_reg;
		visit_way0   <= visit_way0;
		visit_way1   <= visit_way1;
		visit_way2   <= visit_way2;
		visit_way3   <= visit_way3;
	end
end


//对于每一片cache块的valid内容进行更新
always @(posedge clk) begin
	if(rst) begin
		valid_way0 <= 8'b0; valid_way1 <= 8'b0;
		valid_way2 <= 8'b0; valid_way3 <= 8'b0;
	end
	else if (icache_current_state == REFILL) begin//在REFILL阶段更新
		if(replace_way0)
			valid_way0[index] <= 1;
		else if(replace_way1) 
			valid_way1[index] <= 1;
		else if(replace_way2) 
			valid_way2[index] <= 1;
		else if(replace_way3) 
			valid_way3[index] <= 1;
	end
end

//根据取出来的index来更新原本cache片中的tag
wire	tag_wen0;
wire	tag_wen1;
wire	tag_wen2;
wire	tag_wen3;

assign	tag_wen0 = (icache_current_state == REFILL) && replace_way0;
assign	tag_wen1 = (icache_current_state == REFILL) && replace_way1;
assign	tag_wen2 = (icache_current_state == REFILL) && replace_way2;
assign	tag_wen3 = (icache_current_state == REFILL) && replace_way3;

// assign	tag_way0 = (current_state == REFILL) && visit_way0;
// assign	tag_way1 = (current_state == REFILL) && visit_way1;
// assign	tag_way2 = (current_state == REFILL) && visit_way2;
// assign	tag_way3 = (current_state == REFILL) && visit_way3;

//对于数据的写使能
wire	data_wen0;
wire	data_wen1;
wire	data_wen2;
wire	data_wen3;

assign	data_wen0 = (icache_current_state == REFILL) && replace_way0;
assign	data_wen1 = (icache_current_state == REFILL) && replace_way1;
assign	data_wen2 = (icache_current_state == REFILL) && replace_way2;
assign	data_wen3 = (icache_current_state == REFILL) && replace_way3;

// assign	data_way0 = (current_state == REFILL) && visit_way0;
// assign	data_way1 = (current_state == REFILL) && visit_way1;
// assign	data_way2 = (current_state == REFILL) && visit_way2;
// assign	data_way3 = (current_state == REFILL) && visit_way3;

//对于输出数据部分
wire [255:0] read_data0;
wire [255:0] read_data1;
wire [255:0] read_data2;
wire [255:0] read_data3;
wire [255:0] cache_data_hit_raw;

assign cache_data_hit_raw  = 	(visit_way0)?	read_data0:
				(visit_way1)?	read_data1:
				(visit_way2)?	read_data2:
						read_data3;

wire [31:0] cache_data_hit_out;
//位只能是常量, 不能是变量!!!
//assign cache_data_hit_out  =	cache_data_hit_raw[(offset << 3) + 31 : (offset << 3)];
assign cache_data_hit_out  =	cache_data_hit_raw[{offset,3'b0} +: 32];

//和内存进行数据交互
reg  [255:0] mem_data_raw;
//对于突发, 一直读入直到last
always@(posedge clk) begin
	if(rst) begin
		mem_data_raw <= 256'b0;
	end
	else if(icache_current_state==RECV && from_mem_rd_rsp_valid) begin
		//每次读入四字节, 要让高地址放在高位, 这样的偏移才是正确的
		mem_data_raw <= {from_mem_rd_rsp_data,mem_data_raw[255:32]};
	end
	else begin
		mem_data_raw <= mem_data_raw;
	end
end

assign to_mem_rd_req_addr = {cpu_inst_reg[31:5],5'b0};//32字节对齐

wire [31:0] mem_data_out;
assign mem_data_out	=	mem_data_raw[{offset,3'b0} +: 32];

//对于最终的输出
assign to_cpu_cache_rsp_data = (Read_hit_reg)? 	cache_data_hit_out:
						mem_data_out;

//接下来是对data_array和tag_array分别的实例化
tag_array way0_tag(
	.clk    (clk),
	.waddr  (index),
	.raddr  (index),
	.wen    (tag_wen0),
	.wdata  (tag),
	.rdata  (tag_way0)
);

tag_array way1_tag(
	.clk    (clk),
	.waddr  (index),
	.raddr  (index),
	.wen    (tag_wen1),
	.wdata  (tag),
	.rdata  (tag_way1)
);

tag_array way2_tag(
	.clk    (clk),
	.waddr  (index),
	.raddr  (index),
	.wen    (tag_wen2),
	.wdata  (tag),
	.rdata  (tag_way2)
);

tag_array way3_tag(
	.clk    (clk),
	.waddr  (index),
	.raddr  (index),
	.wen    (tag_wen3),
	.wdata  (tag),
	.rdata  (tag_way3)
);


//对data_array的实例化
data_array way0_data(
        .clk    (clk),
        .waddr  (index),
        .raddr  (index),
        .wen    (data_wen0),
        .wdata  (mem_data_raw),
        .rdata  (read_data0)
);

data_array way1_data(
        .clk    (clk),
        .waddr  (index),
        .raddr  (index),
        .wen    (data_wen1),
        .wdata  (mem_data_raw),
        .rdata  (read_data1)
);

data_array way2_data(
        .clk    (clk),
        .waddr  (index),
        .raddr  (index),
        .wen    (data_wen2),
        .wdata  (mem_data_raw),
        .rdata  (read_data2)
);

data_array way3_data(
        .clk    (clk),
        .waddr  (index),
        .raddr  (index),
        .wen    (data_wen3),
        .wdata  (mem_data_raw),
        .rdata  (read_data3)
);
endmodule

