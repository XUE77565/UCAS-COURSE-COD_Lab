#include "printf.h"
#include "trap.h"
#include "mul.h"
#include "div.h"

#define FRAC_BIT 10

#define RD_ADDR 135106448
#define RD_SIZE_D0 1
#define RD_SIZE_D1 1
//每行56字节
#define RD_SIZE_D2 28
#define RD_SIZE_D3 28

//对于kernel
#define WEIGHT_ADDR 134217728
//有20组, 有几组卷积核就有几个输出
#define WEIGHT_SIZE_D0 20
//每个卷积核中有一个权重
#define WEIGHT_SIZE_D1 1
//一组权重值的大小为5x5
#define WEIGHT_SIZE_D2 5
#define WEIGHT_SIZE_D3 5

//输出图象
#define WR_ADDR 135108240
#define WR_SIZE_D0 1
#define WR_SIZE_D1 20
#define WR_SIZE_D2 12
#define WR_SIZE_D3 12

#define KERN_ATTR_CONV_PAD 0
//步长为1
#define KERN_ATTR_CONV_STRIDE 1
//池化的宏定义
#define KERN_ATTR_POOL_PAD 0
#define KERN_ATTR_POOL_KERN_SIZE 2
#define KERN_ATTR_POOL_STRIDE 2

//MMIO register address of DNN accelerator
#define GPIO_START_ADDR    0x60030000
#define GPIO_DONE_ADDR     0x60030008

struct size_vec4
{
	unsigned d0;
	unsigned d1;
	unsigned d2;
	unsigned d3;
};

struct mem_addr
{
	unsigned rd_addr;
	unsigned weight_addr;
	unsigned wr_addr;
};

int mul(short a, short b)
{
#ifndef USE_MUL
	int ans = mul_ll(a, b);
#else
	int ans = a * b;
#endif
	return ans;
}

struct mem_addr addr = {RD_ADDR, WEIGHT_ADDR, WR_ADDR};
struct size_vec4 rd_size = {RD_SIZE_D0, RD_SIZE_D1, RD_SIZE_D2, RD_SIZE_D3};
struct size_vec4 wr_size = {WR_SIZE_D0, WR_SIZE_D1, WR_SIZE_D2, WR_SIZE_D3};
struct size_vec4 weight_size = {WEIGHT_SIZE_D0, WEIGHT_SIZE_D1, WEIGHT_SIZE_D2, WEIGHT_SIZE_D3};

struct size_vec4 conv_size;

extern char _binary_data_result_bin_start[];
extern char _binary_data_result_bin_size[];

void convolution()
{
	//输入,权重和输出
	short *in = (short *)addr.rd_addr;
	short *weight = (short *)addr.weight_addr;
	short *out = (short *)addr.wr_addr;

	//unsigned output_offset = 0;
	//unsigned input_offset = 0;

	unsigned input_fm_w = rd_size.d3;
	unsigned input_fm_h = rd_size.d2;

	unsigned pad = KERN_ATTR_CONV_PAD;
	unsigned pad_len = pad << 1;

	unsigned conv_out_w = rd_size.d3 - weight_size.d2 + pad_len;
	unsigned conv_out_h = rd_size.d2 - weight_size.d2 + pad_len;

	unsigned stride = KERN_ATTR_CONV_STRIDE;

	conv_out_w = div(conv_out_w, stride);
	conv_out_h = div(conv_out_h, stride);

	conv_out_w++;
	conv_out_h++;

	conv_size.d0 = wr_size.d0;
	conv_size.d1 = wr_size.d1;
	conv_size.d2 = conv_out_h;
	conv_size.d3 = conv_out_w;

	//=========================================
	//TODO: Please add your implementation here
	//=========================================	

	//由于所有的输入,kernel和输出都是在内存中连续的排列, 所以直接用线性地址
	unsigned input_size = mul(input_fm_h,input_fm_w);			//输入的大小
	unsigned filter_size = 1 + mul(weight_size.d2, weight_size.d3);	//每组kernel的大小, 每个kernel的第一个元素是bias
									//每个kernel都有一个输出

	unsigned num_out;		//输出个数
	unsigned num_in;		//输入个数
	unsigned bias;

	short	input_address;
	short	filter_outer_address;
	
	//一些即将用到的的线性坐标
	short	filter_line_addr;
	short	input_line_addr;

	//x*S, y*S
	short 	x_stride;
	short	y_stride;

	//用于标记输出相对于基址的偏移
	short	position = 0;

	//输入的位置
	short   iw;
	short	ih;

	//用于暂存结果, 用int来防止溢出
	int 	value_temp;
	short 	value_true;

	for(num_out = 0; num_out < conv_size.d1; num_out++){
		filter_outer_address = mul(num_out,filter_size);	//调整到对应的filter
		bias = weight[filter_outer_address];
		//当前输出所在的位置
		for(int y = 0; y < conv_out_h ; y++){
			y_stride = mul(y,stride);
			for(int x = 0; x < conv_out_w; x++){
				value_temp = 0;
				x_stride = mul(x,stride);
				for(num_in = 0; num_in < rd_size.d1; num_in++){
					input_address = mul(num_in,input_size);
					for(int ky = 0; ky < weight_size.d2; ky++){
						ih = ky + y_stride - pad;	//找到对应的输入的位置, 放在外面进行可以减少乘法的计算次数,  要考虑pad
						if(ih < 0 || ih >= input_fm_h){
							continue; //考虑pad的情况, pad在最周围一圈, 放在之前来减少乘法的计算次数
						}
						filter_line_addr = mul(ky,weight_size.d3);//注意是宽度
						input_line_addr = mul(ih,input_fm_w);
						for(int kx = 0; kx < weight_size.d3; kx++){
							iw = kx + x_stride - pad;
							if(iw < 0 || iw >= input_fm_w){
								continue;
							}
							//weight里的i用于跳过bias
							value_temp = value_temp + mul(in[input_address + input_line_addr + iw], weight[1 + filter_outer_address + filter_line_addr + kx]);
						}//kx
					}//ky
				}//in
				value_true = (value_temp >> FRAC_BIT);
				out[position] = (short)(bias + value_true);
				position++;	//计算之后的值
				//out[position] = (short)(bias + value_temp >> FRAC_BIT);
			}//x
		}//y
	}//no
}

void pooling()
{
	short *out = (short *)addr.wr_addr;

	//unsigned output_offset = 0;
	//unsigned input_offset = 0;

	unsigned input_fm_w = conv_size.d3;
	unsigned input_fm_h = conv_size.d2;

	unsigned pad = KERN_ATTR_POOL_PAD;
	unsigned pad_len = pad << 1;

	unsigned pad_w_test = conv_size.d3 - KERN_ATTR_POOL_KERN_SIZE;
	unsigned pad_h_test = conv_size.d2 - KERN_ATTR_POOL_KERN_SIZE;

	unsigned pool_out_w = pad_w_test + pad_len;
	unsigned pool_out_h = pad_h_test + pad_len;

	unsigned stride = KERN_ATTR_POOL_STRIDE;

	unsigned pad_w_test_remain = pad_w_test - mul(div(pad_w_test, stride), stride);
	unsigned pad_h_test_remain = pad_h_test - mul(div(pad_h_test, stride), stride);

	pool_out_w = div(pool_out_w, stride);
	pool_out_h = div(pool_out_h, stride);
	pool_out_w++;
	pool_out_h++;

	if ((!pad) && (pad_w_test_remain || pad_h_test_remain))
	{
		pool_out_w++;
		pool_out_h++;
	}

	//=========================================
	//TODO: Please add your implementation here
	//=========================================

	short input_size = mul(input_fm_w,input_fm_h);
	short input_address;

	short num_out;

	short y_stride;
	short x_stride;

	short ky;
	short kx;

	//输入的位置
	short iw;
	short ih;

	//一些即将用到的的线性坐标
	short	input_line_addr;

	short position = 0;

	//要比较的value的大小
	short value;

	short pool_size = KERN_ATTR_POOL_KERN_SIZE;

	for(num_out = 0; num_out < conv_size.d1; num_out++){
		input_address = mul(num_out, input_size);
		for(short y = 0; y < pool_out_h; y++){
			y_stride = mul(y, stride);
			for(short x = 0; x < pool_out_w; x++){
				x_stride = mul(x,stride);
				short max = 0x8000;//初始化最大值, 来实现池化的功能
				for(ky = 0; ky < pool_size; ky++){
					ih = ky + y_stride - pad;
					if(ih < 0 || ih >= input_fm_h){
						continue; //考虑pad的情况, pad在最周围一圈, 放在之前来减少乘法的计算次数
					}
					input_line_addr = mul(ih, input_fm_w);
					for(kx = 0; kx < pool_size; kx++){
						iw = kx + x_stride - pad;
						if(iw < 0 || iw >= input_fm_w){
							continue; //考虑pad的情况, pad在最周围一圈, 放在之前来减少乘法的计算次数
						}
						value = out[input_address + input_line_addr + iw];
						max = (max > value)? max: value;
					}//kx
				}//ky
				out[position] = max;
				position++;
			}//x
		}//y

	}//no
}

#ifdef USE_HW_ACCEL
void launch_hw_accel()
{
	volatile int* gpio_start = (void*)(GPIO_START_ADDR);
	volatile int* gpio_done = (void*)(GPIO_DONE_ADDR);

	//TODO: Please add your implementation here

	//1启用 0:不启用
	*gpio_start = 1;

	while(*gpio_done);

	*gpio_start = 0;
}
#endif

int comparing()
{
	char *out = (char *)addr.wr_addr;
	char *result = (char *)_binary_data_result_bin_start;

#ifdef USE_HW_ACCEL
	int count = (int)_binary_data_result_bin_size + 
		    (16 - WR_SIZE_D3) * 2 * WR_SIZE_D2 * WR_SIZE_D1;
#else
	int count = (int)_binary_data_result_bin_size;
#endif

	for (int i = 0, j = 0; i < count; i++)
	{
#ifdef USE_HW_ACCEL
		int alignment = i & 0x0000001f;
		if (alignment >= (WR_SIZE_D3 << 1))
			continue;
#endif
		if (*(out + i) != *(result + j))
		{
			printf("Failed! at address %x and %x with data %x and %x\n", out + i, result + j, *(out + i), *(result + j));
			return 1;
		}
		j++;
	}

	printf("Passed!\n");
	return 0;
}

int main()
{

#ifdef USE_HW_ACCEL
	printf("Launching task...\n");
	launch_hw_accel();
#else
	printf("starting convolution\n");
	convolution();
	printf("starting pooling\n");
	pooling();
#endif

	int result = comparing();
	printf("benchmark finished\n");

	if (result == 0) {
		hit_good_trap();
	} else {
		nemu_assert(0);
	}

	return 0;
}
