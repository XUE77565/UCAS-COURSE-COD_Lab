`timescale 10ns / 1ns
`include "define.v"

module ex(
        input           clk,

        input [31:0]    PC,
        input [31:0]    Instruction,

        //ALU需要用到的数据以及控制信号
        input [31:0]    ALUop_A,
        input [31:0]    ALUop_B,
        input [2:0]     ALUop,

        //Shifter需要用到的数据以及控制信号
        input [31:0]    Shiftop_A,
        input [ 4:0]    Shiftop_B,
        input [1:0]     Shiftop,

        //id阶段读入的offset偏移
        input [15:0]    offset,

        //ALU和Shifter结果二选一信号
        input           AluShi_sel,

        //定义ALU和Shifter输出信号
        output [31:0]   Result,
        
        //有关ALU结果的寄存器信号控制
        output          ALU_Branch
);
        //实例化ALU
        wire Zero;
        wire [31:0] ALU_result;
        wire [5:0] opcode;
        assign opcode = Instruction[31:26];

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
        wire [31:0] Shifter_result;

        shifter shifter_ex(
                .A              (Shiftop_A),
                .B              (Shiftop_B),
                .Shiftop        (Shiftop),
                .Result         (Shifter_result)
        );

        //在Shifter和ALU的结果中二选一,输出一个结果
        assign Result = (AluShi_sel)?   Shifter_result:
                                        ALU_result;

        //对于分支指令,将其分支控制信号赋值
        wire   ALU_Branch_temp;
        assign ALU_Branch_temp = (opcode[5:1]==5'b00011 || opcode==`REGIMM)?    ALU_result :    //对于比大小的分支指令
                                 (opcode==`BEQ)?                                Zero    :       //对于beq,  两个相等, 跳转
                                 (opcode==`BNE)?                                ~Zero   :       //对于bne, 两个不相等, 跳转
                                                                                0;              //其他一般情况下, 不跳转
        assign ALU_Branch      = ALU_Branch_temp; 

endmodule
