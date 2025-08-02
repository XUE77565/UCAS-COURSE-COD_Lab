`timescale 10ns / 1ns
`include "define.v"

module pc(
        input           clk,
        input           rst,

        //输入的有关分支和跳转的控制信号
        input           Branch,
        input           ALU_Branch,
        input           Jump,

        //输入的有关分支和跳转的目标地址
        input  [31:0]   Jump_addr,
        input  [31:0]   Branch_addr,
        
        output reg [31:0]   PC
);
        wire Branch_f;
        assign Branch_f = Branch && ALU_Branch;

        always @(posedge clk) begin
                if(rst) begin
                        PC <= 32'd0;
                end
                else if(Branch_f)begin          //对于分支
                        PC <= PC + Branch_addr + 4; 
                end
                else if(Jump)begin              //对于跳转
                        PC <= Jump_addr;
                end
                else begin                      //其他一般情况
                        PC <= PC + 4;
                end
        end

endmodule