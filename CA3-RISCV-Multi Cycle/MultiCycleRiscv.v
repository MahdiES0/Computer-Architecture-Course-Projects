`define lw   7'b0000011 
`define sw   7'b0100011 
`define RT   7'b0110011 
`define BT   7'b1100011 
`define IT   7'b0010011 
`define jalr 7'b1100111 
`define jal  7'b1101111 
`define lui  7'b0110111 
 
`define S0  5'b00000 
`define S1  5'b00001 
`define S2  5'b00010 
`define S3  5'b00011 
`define S4  5'b00100 
`define S5  5'b00101 
`define S6  5'b00110 
`define S7  5'b00111 
`define S8  5'b01000 
`define S9  5'b01001 
`define S10 5'b01010 
`define S11 5'b01011 
`define S12 5'b01100 
`define S13 5'b01101 
`define S14 5'b01110 
`define S15 5'b01111 
`define S16 5'b10000 
`define S17 5'b10001 
`define S18 5'b10010 
`define S19 5'b10011 
`define S20 5'b10100 

module MultiCycleCPU (clk, rst);
input clk, rst;
wire pcwrite, adrsrc, memwrite, irwrite, regwrite;
wire [1:0] resultsrc, alusrcb, alusrca;
wire [2:0] aluctrl, immsrc;
wire [6:0] op;
wire [2:0] func3;
wire [6:0] func7;
wire zero, blt, bge, lt, done;

datapath mydp(.clk(clk), .rst(rst), .pcwrite(pcwrite), .adrsrc(adrsrc), .memwrite(memwrite), .irwrite(irwrite), .resultsrc(resultsrc), 
    .aluctrl(aluctrl), .alusrcb(alusrcb), .alusrca(alusrca), .immsrc(immsrc) , .regwrite(regwrite), .op(op), .func3(func3), .func7(func7), .zero(zero), .bge(bge), .blt(blt));
controller myctrl(.clk(clk), .rst(rst), .op(op), .func3(func3), .func7(func7), .zero(zero), .lt(lt), .PCWrite(pcwrite), 
    .AdrSrc(adrsrc), .MemWrite(memwrite), .IRWrite(irwrite), .ResultSrc(resultsrc), .AluControl(aluctrl), .AluSrcA(alusrca), 
    .AluSrcB(alusrcb), .ImmSrc(immsrc), .RegWrite(regwrite), .done(done)); 
  
endmodule

module controller(clk, rst, op, func3, func7, zero, lt, PCWrite, AdrSrc, MemWrite, IRWrite, 
    ResultSrc, AluControl, AluSrcB, AluSrcA, ImmSrc, RegWrite, done); 
input clk, rst, zero, lt; 
input[6:0] op, func7; 
input [2:0] func3; 
output reg PCWrite, AdrSrc, MemWrite, IRWrite, RegWrite, done; 
output reg[1:0] ResultSrc, AluSrcB, AluSrcA; 
output reg[2:0] ImmSrc; 
output[2:0] AluControl; 
reg [1:0]AluOp; 
reg branch; 
reg[4:0] ns, ps = `S0; 
wire beq, bne, blt, bge; 
assign beq = branch & (func3 == 3'b000); 
assign bne = branch & (func3 == 3'b001); 
assign blt = branch & (func3 == 3'b100); 
assign bge = branch & (func3 == 3'b101); 
assign AluControl = (AluOp == 2'b00)? 3'b000: 
                    (AluOp == 2'b01)? 3'b001: 
                    (AluOp == 2'b11)? 3'b100: 
                    (AluOp == 2'b10)? (func3 == 3'b000)? ((op == `RT & func7 == 7'b0100000)? 3'b001: 3'b000): 
                                      (func3 == 3'b111)? 3'b010: 
                                      (func3 == 3'b100)? 3'b111: 
                                      (func3 == 3'b110)? 3'b011: 
                                      (func3 == 3'b010)? 3'b101: 3'b000: 3'b000; 
 
always@(ps, beq, bne, blt, bge, zero, lt) begin 
 ns = `S0; 
 {PCWrite, AdrSrc, MemWrite, IRWrite, RegWrite, branch, done} = 7'b0; 
 {ResultSrc, AluSrcB, AluSrcA, AluOp} = 8'b0; 
 ImmSrc = 3'b0; 
 
 case(ps) 
  `S0:  begin IRWrite = 1; AluSrcB = 2'b10; ResultSrc = 2'b10; PCWrite = 1; ns = `S1; end 
  `S1:  begin AluSrcB = 2'b01; AluSrcA = 2'b01; ImmSrc = 3'b010;  
         ns = (op == `lw)? `S3: 
       (op == `sw)?   `S6: 
       (op == `RT)?   `S8: 
       (op == `BT)?   `S2: 
       (op == `IT)?   `S10: 
       (op == `jalr)? `S12: 
       (op == `jal)?  `S15: 
       (op == `lui)?  `S18: `S20; 
        end 
     `S2:  begin AluSrcA = 2'b10; AluOp = 2'b01; branch = 1; PCWrite = (beq & zero)? 1: 
           (bne & ~zero)? 1: 
           (blt & lt)? 1: 
           (bge & ~lt)? 1:0; 
      ns = `S0; end 
        `S3:  begin AluSrcA = 2'b10; AluSrcB = 2'b01; ns = `S4; end 
        `S4:  begin AdrSrc = 1; ns = `S5; end 
        `S5:  begin ResultSrc = 2'b01; RegWrite = 1; ns = `S0; end 
        `S6:  begin ImmSrc = 3'b001; AluSrcA = 2'b10; AluSrcB = 2'b01; ns = `S7; end 
        `S7:  begin AdrSrc = 1; MemWrite = 1; ns = `S0; end 
        `S8:  begin AluSrcA = 2'b10; AluOp = 2'b10; ns = `S9; end 
        `S9:  begin RegWrite = 1; ns = `S0; end 
        `S10: begin AluSrcA = 2'b10; AluSrcB = 2'b01; AluOp = 2'b10; ns = `S11; end 
        `S11: begin RegWrite = 1; ns = `S0; end 
        `S12: begin AluSrcA = 2'b10; AluSrcB = 2'b01; ns = `S13; end 
        `S13: begin PCWrite = 1; AluSrcA = 2'b01; AluSrcB = 2'b10; ns = `S14; end 
        `S14: begin RegWrite = 1; ns = `S0; end 
        `S15: begin AluSrcA = 2'b01; AluSrcB = 2'b01; ImmSrc = 3'b011; ns = `S16; end 
        `S16: begin PCWrite = 1; AluSrcA = 2'b01; AluSrcB = 2'b10; ns = `S17; end 
        `S17: begin RegWrite = 0; ns = `S0; end 
        `S18: begin ImmSrc = 3'b100; AluSrcB = 2'b01; AluOp = 2'b11; ns = `S19; end 
        `S19: begin RegWrite = 1; ns = `S0; end 
   `S20: begin done = 1; ns = `S20; end 
 endcase 
end 
 
always@(posedge clk) begin 
 ps <= ns; 
end 
endmodule



module datapath(clk, rst, pcwrite, adrsrc, memwrite, irwrite, resultsrc, aluctrl, alusrcb, alusrca, immsrc, regwrite, op, func3, func7, zero, bge, blt); 

    input pcwrite, adrsrc, memwrite, irwrite, regwrite, clk, rst;
    input [1:0] resultsrc, alusrcb, alusrca;
    input [2:0] aluctrl, immsrc;
    output [6:0] op;
    output [2:0] func3;
    output [6:0] func7;
    output zero, blt, bge;

    wire [31:0] pc, adr, oldpc, instr, readdata, data, immext, rd1, rd2, A, B, srca, srcb, aluresult, aluout, result; 
    pc mypc(result, pc, clk, pcwrite, rst);
    MUX2 mymux2(pc, result, adrsrc, adr);
    InstrDataMemory myIDM(adr, readdata, B, memwrite, clk, adrsrc);
    Instrreg myinstrreg(pc, readdata, oldpc, instr, irwrite, clk);
    RegisterFile myRF(instr[19:15], instr[24:20], instr[11:7], result, rd1, rd2, regwrite, clk);
    AandBreg myAB(rd1, rd2, A, B, clk);
    MUX3 mymuxA(pc, oldpc, A, alusrca, srca);
    MUX3 mymuxB(B, immext, 4, alusrcb, srcb);
    ALU myALU(srca, srcb, aluresult, aluctrl, zero, blt, bge);
    AluOutReg myALUout(aluresult, aluout, clk);
    MUX4 muMUXresult(aluout, data, aluresult, immext, resultsrc, result);
    Extend myex(immsrc, instr[31:7], immext);
    MDReg myMDR(readdata, data, clk);

    assign op = instr[6:0];
    assign func3 = instr[14:12];
    assign func7 = instr[31:25];

endmodule

module InstrDataMemory (Adr, ReadData, writeData, we, clk, adrsrc);
    input clk, we, adrsrc;
    input [31:0] Adr;
    input [31:0] writeData;
    output reg [31:0] ReadData;

    reg [31:0] datamem [0:15999];

    initial begin
        $readmemb("instructions.txt", datamem, 0, 7999);
        $readmemb("your_file.txt", datamem, 8000, 15999);
    end
     
    always @(*)begin
        if(adrsrc)
            ReadData = datamem[Adr/4 + 8000];
        else
            ReadData = datamem[Adr/4];
    end

    always @(posedge clk) begin 
        if(we)
        begin
            if(adrsrc)
                datamem[Adr + 8000] <= writeData;
            else 
                datamem[Adr] <= writeData;
        end 
    end

endmodule

module Instrreg(pc, ReadData, oldpc, instr, IRwrite, clk);
    input [31:0] pc, ReadData;
    output reg [31:0] oldpc, instr;
    input IRwrite, clk;

    always @(posedge clk) begin 
        oldpc <= pc;
        if(IRwrite)
            instr <= ReadData;
    end

endmodule

module RegisterFile (A1, A2, A3, wd, Rd1, Rd2, we,clk);
    input [4:0]A1, A2, A3;
    input we, clk;
    input [31:0] wd;
    output reg [31:0] Rd1, Rd2;

    reg [31:0] regmem [0:31];
    assign regmem[0] = 32'b0;

    assign Rd1 = regmem[A1];
    assign Rd2 = regmem[A2];

    always @(posedge clk) begin 
        if(we)
            regmem[A3] <= wd;
    end
    
endmodule

module AandBreg(rd1, rd2, A, B, clk);

    input clk;
    input [31:0] rd1, rd2;
    output reg [31:0] A, B;

    always @(posedge clk) begin 
        A <= rd1;
        B <= rd2;
    end

endmodule

module Extend (immsrc, instr, immExt);
    input [2:0]immsrc;
    input [31:7]instr;
    output [31:0]immExt;

    assign immExt = (immsrc == 3'b000) ? {{21{instr[31]}}, instr[30:20]}: //I Type
                    (immsrc == 3'b001) ? {{21{instr[31]}}, instr[30:25], instr[11:7]}: //S Type
                    (immsrc == 3'b010) ? {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}: //B Type
                    (immsrc == 3'b100) ? {instr[31], instr[30:20], instr[19:12], 12'b0}: //U Type
                    (immsrc == 3'b011) ? {{12{instr[31]}}, instr[19:12], instr[20], instr[30:25], instr[24:21], 1'b0}: //J Type
                    32'bx;
endmodule

module MUX3 (data1, data2, data3, sel, out);
    input [31:0] data1, data2, data3;
    input [1:0] sel;
    output reg [31:0] out;

    assign out = (sel == 2'b00) ? data1 : 
                (sel == 2'b01) ? data2 :
                (sel == 2'b10) ? data3 :
                32'bx;

endmodule

module ALU (srcA, srcB, AluResult, AluControl, zero, blt, bge);
    input [31:0]srcA, srcB;
    input [2:0]AluControl;
    output reg [31:0]AluResult;
    output reg zero, blt, bge;

    parameter [3:0]  
    ADD = 3'b000,
    SUB = 3'b001,
    AND = 3'b010,
    OR = 3'b011,
    BUFFB = 3'b100,
    COMPARE = 3'b101,
    XOR = 3'b111
    ;

    assign zero = ~(|AluResult);
    assign blt = (AluResult[31] == 1) ? 1'b1 : 1'b0;
    assign bge = (AluResult[31] == 0) ? 1'b1 : 1'b0;

    always @(*) begin
        case(AluControl) 
            ADD :  begin AluResult = srcA + srcB; $display("dakhele add miay? srcA: %b, srcB : %b", srcA, srcB); end
            SUB : begin AluResult = srcA - srcB; if(AluResult == 0) zero = 0; end
            AND : begin AluResult = (srcA & srcB); end
            OR : begin AluResult = (srcA | srcB); end
            BUFFB : begin zero = (srcA < srcB) ? 1'b1 : 1'b0;
                          AluResult = (srcA < srcB) ? 32'b1 : 32'b0; end
            COMPARE : begin AluResult = (srcA < srcB); end  
            XOR : begin AluResult = (srcA ^ srcB); end            
        endcase
    end
endmodule


module AluOutReg(AluResult, AluOut, clk);

    input clk;
    input [31:0] AluResult;
    output reg [31:0] AluOut;

    always @(posedge clk) begin 
        AluOut <= AluResult;
    end

endmodule


module MDReg(ReadData, Data, clk);

    input [31:0] ReadData;
    output reg [31:0] Data;
    input clk;

    always @(posedge clk) begin 
        Data <= ReadData;
    end

endmodule


module pc(result, pc, clk, pcwrite, rst);

        input clk, pcwrite, rst;
        input [31:0] result;
        output reg [31:0] pc;

        always @(posedge clk, posedge rst) begin 
            if(rst)
                pc <= 32'b0;
            else if(pcwrite)
                pc <= result;
        end

endmodule

module MUX2 (data1, data2, sel, out);
    input [31:0] data1, data2;
    input sel;
    output reg [31:0] out;

    assign out = (sel == 0) ? data1 : data2;

endmodule

module MUX4 (data1, data2, data3, data4, sel, out);
    input [31:0] data1, data2, data3, data4;
    input [1:0] sel;
    output reg [31:0] out;   

assign out = (sel == 2'b00) ? data1 : 
             (sel == 2'b01) ? data2 :
             (sel == 2'b10) ? data3 :
             (sel == 2'b11) ? data4 :
             32'bx;

endmodule

module CPUtbM ();

    reg cc = 1'b1;
    reg rr = 1'b1;

    MultiCycleCPU mycpu(.clk(cc), .rst(rr));

    always #20 cc = ~cc;
    initial begin
    #20 rr = 1'b0;
    #10000 $stop;
    end

endmodule


