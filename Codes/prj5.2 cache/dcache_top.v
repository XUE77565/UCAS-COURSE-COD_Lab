`timescale 10ns / 1ns

`define CACHE_SET	8
`define CACHE_WAY	4
`define TAG_LEN		24
`define LINE_LEN	256

module dcache_top (
	input	      clk,
	input	      rst,
  
	//CPU interface
	/** CPU memory/IO access request to Cache: valid signal */
	input         from_cpu_mem_req_valid,
	/** CPU memory/IO access request to Cache: 0 for read; 1 for write (when req_valid is high) */
	input         from_cpu_mem_req,
	/** CPU memory/IO access request to Cache: address (4 byte alignment) */
	input  [31:0] from_cpu_mem_req_addr,
	/** CPU memory/IO access request to Cache: 32-bit write data */
	input  [31:0] from_cpu_mem_req_wdata,
	/** CPU memory/IO access request to Cache: 4-bit write strobe */
	input  [ 3:0] from_cpu_mem_req_wstrb,
	/** Acknowledgement from Cache: ready to receive CPU memory access request */
	output        to_cpu_mem_req_ready,
		
	/** Cache responses to CPU: valid signal */
	output        to_cpu_cache_rsp_valid,
	/** Cache responses to CPU: 32-bit read data */
	output [31:0] to_cpu_cache_rsp_data,
	/** Acknowledgement from CPU: Ready to receive read data */
	input         from_cpu_cache_rsp_ready,
		
	//Memory/IO read interface
	/** Cache sending memory/IO read request: valid signal */
	output        to_mem_rd_req_valid,
	/** Cache sending memory read request: address
	  * 4 byte alignment for I/O read 
	  * 32 byte alignment for cache read miss */
	output [31:0] to_mem_rd_req_addr,
        /** Cache sending memory read request: burst length
	  * 0 for I/O read (read only one data beat), 旁路
	  * 7 for cache read miss (read eight data beats) */
	output [ 7:0] to_mem_rd_req_len,
        /** Acknowledgement from memory: ready to receive memory read request */
	input	      from_mem_rd_req_ready,

	/** Memory return read data: valid signal of one data beat */
	input	      from_mem_rd_rsp_valid,
	/** Memory return read data: 32-bit one data beat */
	input  [31:0] from_mem_rd_rsp_data,
	/** Memory return read data: if current data beat is the last in this burst data transmission */
	input	      from_mem_rd_rsp_last,
	/** Acknowledgement from cache: ready to receive current data beat */
	output        to_mem_rd_rsp_ready,

	//Memory/IO write interface
	/** Cache sending memory/IO write request: valid signal */
	output        to_mem_wr_req_valid,
	/** Cache sending memory write request: address
	  * 4 byte alignment for I/O write 
	  * 4 byte alignment for cache write miss
          * 32 byte alignment for cache write-back */
	output [31:0] to_mem_wr_req_addr,
        /** Cache sending memory write request: burst length
          * 0 for I/O write (write only one data beat)
          * 0 for cache write miss (write only one data beat)
          * 7 for cache write-back (write eight data beats) */
	output [ 7:0] to_mem_wr_req_len,
        /** Acknowledgement from memory: ready to receive memory write request */
	input         from_mem_wr_req_ready,

	/** Cache sending memory/IO write data: valid signal for current data beat */
	output        to_mem_wr_data_valid,
	/** Cache sending memory/IO write data: current data beat */
	output [31:0] to_mem_wr_data,
	/** Cache sending memory/IO write data: write strobe
	  * 4'b1111 for cache write-back 
	  * other values for I/O write and cache write miss according to the original CPU request*/ 
	output [ 3:0] to_mem_wr_data_strb,
	/** Cache sending memory/IO write data: if current data beat is the last in this burst data transmission */
	output        to_mem_wr_data_last,
	/** Acknowledgement from memory/IO: ready to receive current data beat */
	input	      from_mem_wr_data_ready
);

  //TODO: Please add your D-Cache code here
//定义状态机
localparam	WAIT	    =	14'b00000000000001,//1
		TAG_RD	    =   14'b00000000000010,//2握手后的初始状态
		CACHE_WR    =	14'b00000000000100,//4cache写
		CACHE_RD    =   14'b00000000001000,//8cache读
		EVICT	    =   14'b00000000010000,//10
		MEM_WR	    =	14'b00000000100000,//20在miss后遇到了要替换dirty, 要有一个写回的阶段
		TRANS	    =   14'b00000001000000,//40向内存写
		MEM_RD	    =	14'b00000010000000,//80从内存读出数据存到cache中
		RECV	    =	14'b00000100000000,//100
		REFILL      =	14'b00001000000000,//200
		RESP	    =   14'b00010000000000,//400CPU应答
		BY_WR	    =   14'b00100000000000,//800检测到旁路请求
		BY_RD	    =   14'b01000000000000,//1000
		BY_WAIT	    =   14'b10000000000000;//2000

reg  [13:0]	dcache_current_state;

reg  [13:0]	dcache_next_state;

always @(posedge clk) begin
	if(rst) begin
		dcache_current_state <= WAIT;
	end
	else begin
		dcache_current_state <= dcache_next_state;
	end
end

always @(*) begin
	case (dcache_current_state) 
		WAIT: begin
			if(from_cpu_mem_req_valid) begin
				dcache_next_state = TAG_RD;
			end
			else begin
				dcache_next_state = WAIT;
			end
		end
		TAG_RD: begin
			if(bypath) begin//表示旁路读
				dcache_next_state = BY_WAIT;
			end
			else if(~cpu_req && Read_hit) begin	//表示读命中
				dcache_next_state = CACHE_RD;
			end
			else if(cpu_req && Read_hit) begin	//表示写命中
				dcache_next_state = CACHE_WR;
			end
			else if(~Read_hit) begin
				dcache_next_state = EVICT;
			end
			else begin
				dcache_next_state = TAG_RD;
			end
		end
		BY_WAIT: begin
			if(~cpu_req && from_mem_rd_req_ready) begin//表示旁路读
				dcache_next_state = BY_RD;
			end
			else if(cpu_req && from_mem_wr_req_ready) begin //表示旁路写
				dcache_next_state = BY_WR;
			end
			else begin
				dcache_next_state = BY_WAIT;
			end
		end
		EVICT: begin
			if(dirty) begin
				dcache_next_state = MEM_WR;//需要写来进行替换
			end
			else begin
				dcache_next_state = MEM_RD;
			end
		end
		MEM_WR: begin
			if(from_mem_wr_req_ready) begin
				dcache_next_state = TRANS;
			end
			else begin
				dcache_next_state = MEM_WR;
			end
		end
		TRANS: begin
			if(from_mem_wr_data_ready && to_mem_wr_data_last) begin
				dcache_next_state = MEM_RD;//由于最终的结果是write_strb和内存32位共同组成的结果, 所以有读的过程
			end
			else begin
				dcache_next_state = TRANS;
			end
		end
		MEM_RD: begin
			if(from_mem_rd_req_ready) begin
				dcache_next_state = RECV;
			end
			else begin
				dcache_next_state = MEM_RD;
			end
		end
		RECV:	begin
			if(from_mem_rd_rsp_valid && from_mem_rd_rsp_last) begin
				dcache_next_state = REFILL;//接受数据一直到RECV位置
			end
			else begin
				dcache_next_state = RECV;
			end
		end
		REFILL:	begin
			if(cpu_req) begin
				dcache_next_state = CACHE_WR;
			end
			else begin
				dcache_next_state = CACHE_RD;
			end
		end
		CACHE_RD:begin
			dcache_next_state = RESP;
		end
		CACHE_WR:begin
			dcache_next_state = WAIT;
			//dcache_next_state = RESP;写不需要CPU应答
		end
		BY_WR:begin
			if(from_mem_wr_data_ready && to_mem_wr_data_last) begin
				dcache_next_state = WAIT;
			end
			else begin
				dcache_next_state = BY_WR;
			end
		end
		BY_RD:begin
			if(from_mem_rd_rsp_last && from_mem_rd_rsp_valid) begin
				dcache_next_state = RESP;
			end
			else begin
				dcache_next_state = BY_RD;
			end
		end
		RESP:begin
			dcache_next_state = WAIT;
		end
		default: begin
			dcache_next_state = WAIT;
		end
	endcase
end

//对于握手的控制信号
assign	to_cpu_mem_req_ready 	= (dcache_current_state == WAIT);
assign  to_cpu_cache_rsp_valid 	= (dcache_current_state == RESP);

assign 	to_mem_rd_req_valid 	= (dcache_current_state == MEM_RD) || (~cpu_req && bypath && dcache_current_state == TAG_RD);
assign 	to_mem_rd_rsp_ready 	= (dcache_current_state == RECV)   || (dcache_current_state == BY_RD) || (dcache_current_state == WAIT);

assign 	to_mem_wr_req_valid	= (dcache_current_state == MEM_WR) || (cpu_req && bypath && dcache_current_state == TAG_RD);
assign  to_mem_wr_data_valid	= (dcache_current_state == TRANS)  || (dcache_current_state == BY_WR);


//要注意数据旁路, 就是0x00和0x1F的内存访问请求
reg  [31:0]	cpu_req_addr;
reg  [31:0]	cpu_write_data;
reg  [3:0]	cpu_write_strb;
reg		cpu_req;//0表示读,1表示写


always @(posedge clk) begin
	if(rst) begin
		cpu_req_addr 	<= 32'b0;
		cpu_write_data  <= 32'b0;
		cpu_write_strb  <= 4'b0;
	end
	else if(to_cpu_mem_req_ready && from_cpu_mem_req_valid) begin
		cpu_req		<= from_cpu_mem_req;
		cpu_req_addr	<= from_cpu_mem_req_addr;
		cpu_write_data  <= from_cpu_mem_req_wdata;
		cpu_write_strb  <= from_cpu_mem_req_wstrb;
	end
	else begin
		cpu_req		<= cpu_req;
		cpu_req_addr	<= cpu_req_addr;
		cpu_write_data  <= cpu_write_data;
		cpu_write_strb  <= cpu_write_strb;
	end
end

wire bypath;

//在cache中有不可缓存的空间, 用bypath来标记
//0x00~0x1F 0x4000 0000
assign bypath = (cpu_req_addr[31] || cpu_req_addr[30]) || (cpu_req_addr[31:5]==27'b0);

//cpu发出请求的分段
//分段cpu请求
wire	[`TAG_LEN-1:0]	tag	=	cpu_req_addr[31:8];
wire	[2:0]		index	=	cpu_req_addr[7:5];	//index标记了在一路cache块中的地址
wire	[4:0]		offset  =	cpu_req_addr[4:0];


//对于每片cache内信号, 相比icache多了dirty
reg	[`CACHE_SET-1:0]   valid_way0;
reg	[`CACHE_SET-1:0]   valid_way1;
reg	[`CACHE_SET-1:0]   valid_way2;
reg	[`CACHE_SET-1:0]   valid_way3;

reg	[`CACHE_SET-1:0]   dirty_way0;
reg	[`CACHE_SET-1:0]   dirty_way1;
reg	[`CACHE_SET-1:0]   dirty_way2;
reg	[`CACHE_SET-1:0]   dirty_way3;


//对于命中信号
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
//对于PLRU算法, 在这里和icache是一样的
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
	//每次应答的时候更新plru树, dache中由于有写操作, 修改一下
	//而且旁路不需要访问cache中的存储
	else if (dcache_current_state==CACHE_RD || dcache_current_state==CACHE_WR || dcache_current_state == REFILL) begin	
		if(visit_way0 || visit_way1) begin
			plru_tree[index][0] <= 1;
			plru_tree[index][1] <= visit_way0;//如果访问了0, 就指向右边的1
		end
		else begin
			plru_tree[index][0] <= 0;
			plru_tree[index][2] <= visit_way2;
		end
	end
end

//检查要替换的是否是dirty的数据, 来决定是否要写回
wire   dirty;
assign dirty = 	(replace_way0 && valid_way0[index] && dirty_way0[index]) ||	//写回的地方valid有效, 并且dirty
		(replace_way1 && valid_way1[index] && dirty_way1[index]) ||
		(replace_way2 && valid_way2[index] && dirty_way2[index]) ||
		(replace_way3 && valid_way3[index] && dirty_way3[index]);


//更新选择信号
reg	Read_hit_reg;

always @(posedge clk) begin
	if(rst) begin
		Read_hit_reg <= 0;
		visit_way0   <= 0;
		visit_way1   <= 0;
		visit_way2   <= 0;
		visit_way3   <= 0;
	end
	else if(dcache_current_state == TAG_RD) begin//对于命中的部分
		Read_hit_reg <= Read_hit;
		visit_way0   <= hit_way0;
		visit_way1   <= hit_way1;
		visit_way2   <= hit_way2;
		visit_way3   <= hit_way3;
	end
	else if(dcache_current_state == EVICT) begin//对于更新的部分
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

//更新valid信号
always @(posedge clk) begin
	if(rst) begin
		valid_way0 <= 8'b0; valid_way1 <= 8'b0;
		valid_way2 <= 8'b0; valid_way3 <= 8'b0;
	end
	else if (dcache_current_state == REFILL) begin//在REFILL阶段更新
		if(visit_way0)
			valid_way0[index] <= 1;
		else if(visit_way1) 
			valid_way1[index] <= 1;
		else if(visit_way2) 
			valid_way2[index] <= 1;
		else if(visit_way3) 
			valid_way3[index] <= 1;
	end
end

//更新dirty信号
always @(posedge clk) begin
	if(rst) begin
		dirty_way0[7:0] <= 8'b0; dirty_way1[7:0] <= 8'b0;
		dirty_way2[7:0] <= 8'b0; dirty_way3[7:0] <= 8'b0;
	end
	else begin
		//在WR阶段拉高对应的位置的dirty值
		if(dcache_current_state == CACHE_WR) begin
			if(visit_way0)
				dirty_way0[index] <= 1;
			else if(visit_way1) 
				dirty_way1[index] <= 1;
			else if(visit_way2) 
				dirty_way2[index] <= 1;
			else if(visit_way3) 
				dirty_way3[index] <= 1;
		end
		//在REFILL阶段, 重置dirty值
		else if(dcache_current_state == REFILL) begin
			if(visit_way0)
				dirty_way0[index] <= 0;
			else if(visit_way1) 
				dirty_way1[index] <= 0;
			else if(visit_way2) 
				dirty_way2[index] <= 0;
			else if(visit_way3) 
				dirty_way3[index] <= 0;
		end
	end
end


//对于各cache片的写使能信号
wire	tag_wen0;
wire	tag_wen1;
wire	tag_wen2;
wire	tag_wen3;

assign	tag_wen0 = (dcache_current_state == REFILL) && replace_way0;
assign	tag_wen1 = (dcache_current_state == REFILL) && replace_way1;
assign	tag_wen2 = (dcache_current_state == REFILL) && replace_way2;
assign	tag_wen3 = (dcache_current_state == REFILL) && replace_way3;

//在dcache中, 写逻辑的时候也更新data, 因为在refill后需要用cpu传入的wdata程序来部分改变cache中的内容 
// assign	data_wen0 = (dcache_current_state == REFILL || dcache_current_state == CACHE_WR) && replace_way0;
// assign	data_wen1 = (dcache_current_state == REFILL || dcache_current_state == CACHE_WR) && replace_way1;
// assign	data_wen2 = (dcache_current_state == REFILL || dcache_current_state == CACHE_WR) && replace_way2;
// assign	data_wen3 = (dcache_current_state == REFILL || dcache_current_state == CACHE_WR) && replace_way3;

assign	data_wen0 = (dcache_current_state == REFILL && replace_way0)|| (dcache_current_state == CACHE_WR && hit_way0);
assign	data_wen1 = (dcache_current_state == REFILL && replace_way1)|| (dcache_current_state == CACHE_WR && hit_way1);
assign	data_wen2 = (dcache_current_state == REFILL && replace_way2)|| (dcache_current_state == CACHE_WR && hit_way2);
assign	data_wen3 = (dcache_current_state == REFILL && replace_way3)|| (dcache_current_state == CACHE_WR && hit_way3);

//决定cache中读出来的数据
wire [255:0] read_data0;
wire [255:0] read_data1;
wire [255:0] read_data2;
wire [255:0] read_data3;
wire [255:0] cache_data_raw;
wire [31: 0] cache_data_out;

//refill之后回到cache_read过程, 这时候已经被替换, 是最终的数据
assign cache_data_raw  = 	(visit_way0)?	read_data0:
				(visit_way1)?	read_data1:
				(visit_way2)?	read_data2:
				(visit_way3)?	read_data3:
						256'b0;

// assign cache_data_raw = 	( {`LINE_LEN{visit_way0}} & read_data0 ) |
// 				( {`LINE_LEN{visit_way1}} & read_data1 ) |
// 				( {`LINE_LEN{visit_way2}} & read_data2 ) |
// 				( {`LINE_LEN{visit_way3}} & read_data3 ) ;

assign cache_data_out  =	cache_data_raw[{offset,3'b0} +: 32];



//和内存交互的部分
//miss后产生的读请求, 长度旁路 0 非旁路7
assign to_mem_rd_req_len  =	(bypath)?	8'b00000000:
						8'b00000111;
//和内存进行数据交互
reg  [255:0] mem_data_raw;
//对于突发, 一直读入直到last
always@(posedge clk) begin
	if(rst) begin
		mem_data_raw <= 256'b0;
	end
	else if((dcache_current_state == BY_RD || dcache_current_state==RECV) && from_mem_rd_rsp_valid) begin
		//每次读入四字节, 要让高地址放在高位, 这样的偏移才是正确的
		mem_data_raw <= {from_mem_rd_rsp_data,mem_data_raw[255:32]};
	end
	else begin
		mem_data_raw <= mem_data_raw;
	end
end

wire [31:0] mem_data_out;
assign mem_data_out	=	mem_data_raw[{offset,3'b0} +: 32];

assign to_mem_rd_req_addr  =  (bypath)?		cpu_req_addr:
						{cpu_req_addr[31:5],5'b0};

//由于cache旁路的内存部分返回的32bit数据, 所以只取最新鲜的32位
wire [31:0] bypath_data_out;
assign bypath_data_out	=	mem_data_raw[255:224];

//最终选出返回给cpu的数据, 因为在这个dcache中的状态机, refill之后回到cache_read过程, 在握手的时候已经被替换
assign to_cpu_cache_rsp_data =  (bypath)?	bypath_data_out:
						cache_data_out;


//对于写回内存的部分
wire	[31:0]  cpu_write_data_out;
wire    [7:0]	write_byte_0;
wire    [7:0]	write_byte_1;
wire    [7:0]	write_byte_2;
wire    [7:0]	write_byte_3;

assign  write_byte_0 = (cpu_write_strb[0])?	cpu_write_data[7:0]   :	cache_data_out[7:0];
assign  write_byte_1 = (cpu_write_strb[1])?	cpu_write_data[15:8]  :	cache_data_out[15:8];
assign  write_byte_2 = (cpu_write_strb[2])?	cpu_write_data[23:16] :	cache_data_out[23:16];
assign  write_byte_3 = (cpu_write_strb[3])?	cpu_write_data[31:24] :	cache_data_out[31:24];

assign  cpu_write_data_out = {write_byte_3,write_byte_2,write_byte_1,write_byte_0};

//根据offset的相对位置来对写回的data进行赋值, offset低两位对齐, 直接取出来高三位
wire	[255:0] cache_data_write;
wire    [2:0]	eff;
assign	eff = offset[4:2];
assign	cache_data_write   =  (eff==3'b000)?	{cache_data_raw[255:32],cpu_write_data_out}:
			      (eff==3'b001)?	{cache_data_raw[255:64],cpu_write_data_out,cache_data_raw[31:0]}:
			      (eff==3'b010)?	{cache_data_raw[255:96],cpu_write_data_out,cache_data_raw[63:0]}:
			      (eff==3'b011)?	{cache_data_raw[255:128],cpu_write_data_out,cache_data_raw[95:0]}:
			      (eff==3'b100)?	{cache_data_raw[255:160],cpu_write_data_out,cache_data_raw[127:0]}:
			      (eff==3'b101)?	{cache_data_raw[255:192],cpu_write_data_out,cache_data_raw[159:0]}:
			      (eff==3'b110)?	{cache_data_raw[255:224],cpu_write_data_out,cache_data_raw[191:0]}:
			      			{cpu_write_data_out,cache_data_raw[223:0]};

wire    [255:0]	final_write_data;
assign  final_write_data   =    (dcache_current_state==REFILL)? 	mem_data_raw:  
				(dcache_current_state==CACHE_WR)?	cache_data_write://分别是REFILL和CACHE_WR
									256'b0;

//miss后产生的写请求, 长度旁路 0 非旁路7
assign  to_mem_wr_req_len  =	(bypath)?	8'b00000000:
						8'b00000111;
						

//写地址32字节对齐,要注意有可能是cache中的dirty数据写回的过程
assign  to_mem_wr_req_addr  =	(bypath)?	cpu_req_addr:
				(visit_way0)?	{tag_way0,index,5'b0}:
				(visit_way1)?	{tag_way1,index,5'b0}:
				(visit_way2)?	{tag_way2,index,5'b0}:
						{tag_way3,index,5'b0};

//下面构建last信号, 在IO设备中等第一个就传回, 所以设为1, 在其他时候要传8个字节, 所以第八位设为1
reg	[7:0]   last_string;
always @(posedge clk) begin
	if(dcache_current_state == MEM_WR || dcache_current_state == BY_WAIT) begin
		if(bypath) begin
			last_string <= 8'b00000001;
		end
		else begin
			last_string <= 8'b10000000;
		end
	end
	else if(dcache_current_state == TRANS && from_mem_wr_data_ready) begin//在传输的时候,每传输32位右移一位
		last_string <= {1'b0,last_string[7:1]};
	end
end

//对于写回的数据
reg [255:0] write_data_raw;
always @(posedge clk) begin
	if(rst) begin
		write_data_raw <= 256'b0;
	end
	else if(dcache_current_state == MEM_WR) begin//在dirty写回的时候, 将cache_data_raw的数据取出来
		write_data_raw <= cache_data_raw;
	end
	else if(dcache_current_state == TRANS && from_mem_wr_data_ready) begin
		//每次写32位
		write_data_raw <= {32'b0,write_data_raw[255:32]};
	end
	else begin
		write_data_raw <= write_data_raw;
	end
end

//标记写的最后一个自己
assign  to_mem_wr_data_last = last_string[0] && from_mem_wr_data_ready && to_mem_wr_data_valid;
assign  to_mem_wr_data      =   (bypath)? 	cpu_write_data:
						write_data_raw[31:0];

//对于返回的strb赋值, 在非旁路的时候设成1111
assign  to_mem_wr_data_strb =	(bypath)?	cpu_write_strb:
						4'b1111;

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
        .wdata  (final_write_data),
        .rdata  (read_data0)
);

data_array way1_data(
        .clk    (clk),
        .waddr  (index),
        .raddr  (index),
        .wen    (data_wen1),
        .wdata  (final_write_data),
        .rdata  (read_data1)
);

data_array way2_data(
        .clk    (clk),
        .waddr  (index),
        .raddr  (index),
        .wen    (data_wen2),
        .wdata  (final_write_data),
        .rdata  (read_data2)
);

data_array way3_data(
        .clk    (clk),
        .waddr  (index),
        .raddr  (index),
        .wen    (data_wen3),
        .wdata  (final_write_data),
        .rdata  (read_data3)
);
endmodule
