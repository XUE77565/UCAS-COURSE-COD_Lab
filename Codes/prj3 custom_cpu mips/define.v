//define the operation
`define DATA_WIDTH 32
`define AND  3'b000
`define OR   3'b001
`define XOR  3'b100
`define NOR  3'b101
`define ADD  3'b010
`define SUB  3'b110 
`define SLT  3'b111
`define SLTU 3'b011


//define R-type inst
//define the opcode
`define R_TYPE  6'b000000  
//define the func
`define ADDU    6'b100001
`define SUBU    6'b100011
`define AND_    6'b100100
`define OR_     6'b100101
`define XOR_    6'b100110
`define NOR_    6'b100111
`define SLT_    6'b101010
`define SLTU_   6'b101011

`define SLL     6'b000000
`define SRA     6'b000011
`define SRL     6'b000010
`define SLLV    6'b000100
`define SRAV    6'b000111
`define SRLV    6'b000110

`define JR      6'b001000
`define JALR    6'b001001

`define MOVZ    6'b001010
`define MOVN    6'b001011



//define REGIMM inst
//define the opcode
`define REGIMM  6'b000001



//define J-TYPE inst
//define the opcode
`define J       6'b000010
`define JAL     6'b000011



//define I-type inst
//define the opcode

//I-type branch
`define BEQ     6'b000100 
`define BNE     6'b000101
`define BLEZ    6'b000110
`define BGTZ    6'b000111

//I-type calculate
`define ADDIU   6'b001001
`define LUI     6'b001111
`define ANDI    6'b001100
`define ORI     6'b001101
`define XORI    6'b001110
`define SLTI    6'b001010
`define SLTIU   6'b001011

//I-type mem read
`define LB      6'b100000
`define LH      6'b100001
`define LW      6'b100011
`define LBU     6'b100100
`define LHU     6'b100101
`define LWL     6'b100010
`define LWR     6'b100110

//I-tyoe mem write
`define SB      6'b101000
`define SH      6'b101001
`define SW      6'b101011
`define SWL     6'b101010
`define SWR     6'b101110