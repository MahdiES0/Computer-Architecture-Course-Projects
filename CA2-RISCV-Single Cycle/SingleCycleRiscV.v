`timescale 1ns/1ns

module theCPU(clk, reset);
input clk, reset;
wire memwrite, alusrc, regwrite, zero, blt, bge;
wire [2:0] aluctrl, immsrc, func3;
wire [6:0] op, func7;
wire [1:0] pcsrc, resultsrc;
DataPath mydp(.pcsrc(pcsrc), .resultsrc(resultsrc), .memwrite(memwrite), .aluctrl(aluctrl), .alusrc(alusrc), .immsrc(immsrc), .regwrite(regwrite), .zero(zero), .op(op), .func3(func3), .func7(func7), .clk(clk), .reset(reset), .blt(blt), .bge(bge));
Controller myctrl(.blt(blt), .bge(bge), .pcsrc(pcsrc), .resultsrc(resultsrc), .memwrite(memwrite), .aluctrl(aluctrl), .alusrc(alusrc), .immsrc(immsrc), .regwrite(regwrite), .zero(zero), .op(op), .func3(func3), .func7(func7));

initial $display("Inja miay?????");
initial $display("reset in TopLevel : %d", reset);
initial $display("clk in TopLevel : %d", clk);
endmodule


module CPUtb(); 

reg cc = 1'b1;
reg rr = 1'b1;

theCPU mycpuu(.clk(cc), .reset(rr));

always #20 cc = ~cc;
initial begin
    #20 rr = 1'b0;
    #5000 $stop;
end

endmodule



module DataPath (pcsrc, resultsrc, memwrite, aluctrl, alusrc, immsrc, regwrite, zero, op, func3, func7, clk, reset, blt, bge);
    input memwrite, alusrc, regwrite, clk, reset;
    input [1:0] resultsrc, pcsrc;
    input [2:0] aluctrl;
    input [2:0] immsrc;
    output [6:0] op;
    output [2:0] func3;
    output [6:0] func7;
    output zero, blt, bge;

    wire [31:0]pcnxt, pc, instr, srca, srcb, rd1, rd2, immext, alurslt, wdata, rdata, rslt, pcplus4, pctrgt; 

    assign op = instr[6:0];
    assign func3 = instr[14:12];
    assign func7 = instr[31:25];

    InstructionMemory myIM(pc, instr);
    RegisterFile myRF(instr[19:15], instr[24:20], instr[11:7], rslt, rd1, rd2, regwrite ,clk);
    Extend myex(immsrc, instr[31:7], immext);

    ALU myalu(rd1, srcb, alurslt, aluctrl, zero, blt, bge);
    PC mypc(clk, reset, pcsrc, immext, rslt, pc);
    DataMemory myDM(alurslt, rdata, wdata, memwrite, clk);
    MUX2 mymux2(rd2, immext, alusrc, srcb);
    MUX4 mymux4(alurslt, rdata, pcplus4, immext, resultsrc, rslt);

    initial $display("datapath miay?????");
    initial $display("reset in dapath : %d", reset);
    initial $display("instr : %b", instr);

endmodule

module Controller (blt, bge, pcsrc, resultsrc, memwrite, aluctrl, alusrc, immsrc, regwrite, zero, op, func3, func7);
output reg memwrite, alusrc, regwrite;
output reg [1:0] resultsrc, pcsrc;
output reg [2:0] aluctrl;
output reg [2:0] immsrc;
input [6:0] op;
input [2:0] func3;
input [6:0] func7;
input zero, blt, bge;

initial $display("controller miay?????");

assign regwrite = 1'b0;
assign memwrite = 1'b0;
assign alusrc = 1'b0;
assign resultsrc = 2'b00;
assign aluctrl = 3'b000;
assign immsrc = 3'b000;
assign pcsrc = 2'b00;
assign jump = 1'b0;

parameter [6:0]  
    RT = 7'b0110011,
    LW = 7'b0000011, 
    IT = 7'b0010011,
    JALr = 7'b1100111,
    SW = 7'b0100111,
    JAL = 7'b1101111,
    BT = 7'b1100011,
    LUi = 7'b0110111;

parameter [2:0]
    ADD = 3'b000,
    SUB = 3'b000,
    SLT = 3'b010,
    OR = 3'b110,
    AND = 3'b111,
    ADDI = 3'b000,
    SLTI = 3'b010,
    XORI = 3'b100,
    ORI = 3'b110,
    BEQ = 3'b000,
    BNE = 3'b001,
    BLT = 3'b100,
    BGE = 3'b101;


parameter [6:0]
    ADD7 = 7'b0000000,
    SUB7 = 7'b0100000,
    SLT7 = 7'b0000000,
    OR7 = 7'b0000000,
    AND7 = 7'b0000000;

always @(op, func3, func7) begin 
    {pcsrc, resultsrc, memwrite, aluctrl, alusrc, immsrc, regwrite} = 12'b0;
    case(op)
        RT : begin
            pcsrc = 2'b00;
            regwrite = 1'b1;
            alusrc = 1'b0;
            memwrite = 1'b0;
            resultsrc = 2'b00;
            immsrc = 3'b000;
            if((func3 == ADD) & (func7 == ADD7)) aluctrl = 3'b000;
            else if((func3 == SUB) & (func7 == SUB7)) aluctrl = 3'b001;
            else if((func3 == SLT) & (func7 == SLT7)) aluctrl = 3'b101;
            else if((func3 == OR) & (func7 == OR7)) aluctrl = 3'b011;
            else if((func3 == AND) & (func7 == AND7)) aluctrl = 3'b010;
            else aluctrl = 3'bx;
        end

       
        LW : begin
            pcsrc = 2'b00;
            regwrite = 1'b1;
            immsrc = 3'b000;
            alusrc = 1'b1;
            memwrite = 1'b0;
            resultsrc = 2'b01;
            aluctrl = 3'b000;
        end

        IT : begin 
            regwrite = 1'b1;
            immsrc = 3'b000;
            alusrc = 1'b1;
            memwrite = 1'b0;
            resultsrc = 1'b0;
            if((func3 == ADDI)) aluctrl = 3'b000;
            else if((func3 == SLTI)) aluctrl = 3'b101;
            else if((func3 == XORI)) aluctrl = 3'b111;
            else if((func3 == ORI)) aluctrl = 3'b001;
        end

        JALr : begin
            pcsrc = 2'b10;
            regwrite = 1'b1;
            immsrc = 3'b000;
            memwrite = 1'b0;
            resultsrc = 2'b11;
            alusrc = 1'b1;
            aluctrl = 3'b000;

        end

        SW : begin 
            pcsrc = 2'b00;
            regwrite = 1'b0;
            immsrc = 3'b001;
            alusrc = 1'b1;
            memwrite = 1'b1;
            aluctrl = 3'b000;
            resultsrc = 2'b10;
        end

        JAL : begin
            pcsrc = 2'b01;
            alusrc = 1'b1;
            resultsrc = 2'b11;
            regwrite = 1'b0;
            immsrc = 3'b011;
            memwrite = 1'b0;
            aluctrl = 3'b000;
            if(func3 == 3'b010) aluctrl = 3'b000;
        end

        BT : begin 
            regwrite = 1'b0;
            immsrc = 3'b010;
            alusrc = 1'b0;
            memwrite = 1'b0;
            aluctrl = 3'b001;
            pcsrc = (func3 == 3'b000 && zero == 1'b1) ? 2'b01:
                    (func3 == 3'b001 && zero == 1'b0) ? 2'b01:
                    (func3 == 3'b100 && bge == 1'b1) ? 2'b01:
                    (func3 == 3'b101 && blt == 1'b1) ? 2'b01:
                    2'b00;
                    
        end

        LUi : begin 
            regwrite = 1'b1;
            immsrc = 3'b011;
            memwrite = 1'b0;
            resultsrc = 2'b11;
            pcsrc = 2'b00;
        end



endcase
          

      

end


    
endmodule

module InstructionMemory (pc, instr);
    input [31:0]pc;
    output [31:0] instr;
    reg [31:0] im [0:99];


    initial
        $readmemb("instructions.txt", im);
    initial
        $display ("%b", im[0]);
    assign instr = im[pc/4];
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



module DataMemory (AluResult, ReadData, writeData, memwrite, clk);
    input clk, memwrite;
    input [31:0] AluResult;
    input [31:0] writeData;
    output reg [31:0] ReadData;

    reg [31:0] datamem [0:1599];

    initial 
        $readmemb("your_file.txt", datamem);
     
    assign ReadData = datamem[AluResult/4];

    always @(posedge clk, posedge memwrite) begin 
        if(memwrite)
            datamem[AluResult] <= writeData;
        
    end

endmodule

module Adder(d1, d2, dataout);
    input [31:0] d1, d2;
    output [31:0] dataout;

    assign dataout = d1 + d2;

endmodule

module MUX2 (data1, data2, sel, out);
    input [31:0] data1, data2;
    input sel;
    output reg [31:0] out;

    assign out = (sel == 0) ? data1 : data2;

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

module PC(clk, reset, pcsrc, immext, result, pc);

input clk, reset;
input [1:0]pcsrc;
input [31:0] immext, result;
output reg [31:0] pc;
wire [31:0] pcplus4, pctarget;
reg pcnext;

adder add1(pc, 4, pcplus4);
adder add2(pc, immext, pctarget);

assign pcnext = (pcsrc == 2'b00) ? pcplus4 : 
                (pcsrc == 2'b01) ? pctarget :
                (pcsrc == 2'b10) ? result :
                pcplus4;

always @(posedge clk, posedge reset) begin 
    if(reset) begin
        pc <= 32'b0;
        pcnext <= 32'b0;
    end
    else begin
        case(pcsrc) 
            2'b00 : pc <= pcplus4;
            2'b01 : pc <= pctarget;
            2'b10 : pc <= result;
            default : pc <= pcplus4;
        endcase
    end
end

endmodule

module adder(data1, data2, out);
    input [31:0] data1, data2;
    output [31:0] out;
    assign out = data1 + data2;
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