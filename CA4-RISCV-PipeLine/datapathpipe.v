//Be Name Khoda

'`timescale 1ns/1ns

// module Datapath(clk, RegWriteD, ,ResultSrcD, MemWriteD, JumpD, BranchD, ALUControlD, ALUSrcD,  ImmSrcD, JumpE, PCSrcE, ZeroE ,RegWriteD, RegWriteE , ResultSrcE1, MemWriteE, BranchE, ALUControlE,
// RegWriteM, RegWriteW, ResultSrcM, MemWriteM); 


// endmodule

module DataPath(rst, clk, RegWriteW, ImmSrcD, ALUSrcE, ALUControlE, MemWriteM, ResultSrcW, op, func3, func7, zero, bge, blt, StallF, StallD, FlushD, FlushE, ForwardAE, ForwardBE, Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW); 


    input rst, clk, ALUSrcE, MemWriteM, StallF, StallD, FlushD, FlushE, RegWriteW;
    input [1:0] ImmSrcD, ResultSrcW, ForwardAE, ForwardBE;
    input [2:0] ALUControlE;
    output [6:0] op;
    output [2:0] func3;
    output [6:0] func7;
    output zero, blt, bge;
    output [4:0] Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW;

    wire [4:0] RdD;
    wire [31:0] PCin, PCout, PCPlus4F, instr, instrD, PCD, ExtImmD, PCPlus4D, rd1, rd2, RD1E, RD2E, PCE, ExtImmE, PCPlus4E, PCTargetE, SrcAE, SrcBE, WriteDataE, PCPlus4M, WriteDataM, AluResultM, AluResultE, PCPlus4W, ReadDataW, ResultW;

    //module MUX2 (data1, data2, sel, out);
    MUX2 myPCMUX(PCPlus4F, PCTargetE, PCSrcE, PCin);
    //module PCF(PCFin, PCFout, clk, StallF); 
    PCF myPCF(PCin, PCout, clk, StallF, rst);
    //module InstructionMemory (pc, instr);
    InstructionMemory myIM(PCout, instr);
    //module Adder(d1, d2, dataout);
    Adder addpc4(PCout, 4, PCPlus4F);
    //module Register1(clk, FlushD, StallD, instr, PCF, PCPlus4F, instrD, PCD, PCPlus4D);
    Register1 reg1(rst, clk, FlushD, StallD, instr, PCout, PCPlus4F, instrD, PCD, PCPlus4D);
    //module RegisterFile (A1, A2, A3, wd, Rd1, Rd2, we, clk);
    RegisterFile myRF(instrD[19:15], instrD[24:20], RdW, ResultW, rd1, rd2, RegWriteW, clk);
    //module Extend (immsrc, instr, immExt);
    Extend myex(ImmSrcD, instrD[31:7], ExtImmD);
    //module Register2(rd1, rd2, RD1E, RD2E, PCD, Rs1D, Rs2D, RdD, ExtImmD, PCPlus4D, PCE, RS1E, Rs2E, RdE, ExtImmE, PCPlus4E, FlushE, clk); 
    Register2 reg2(rst, rd1, rd2, RD1E, RD2E, PCD, Rs1D, Rs2D, RdD, ExtImmD, PCPlus4D, PCE, Rs1E, Rs2E, RdE, ExtImmE, PCPlus4E, FlushE, clk);
    //module MUX3 (data1, data2, data3, sel, out);
    MUX3 myMUXA(RD1E, ResultW, AluResultM, ForwardAE, SrcAE);
    //module MUX3 (data1, data2, data3, sel, out);
    MUX3 myMUXWD(RD2E, ResultW, AluResultM, ForwardBE, WriteDataE);
    //module MUX2 (data1, data2, sel, out);
    MUX2 myMUXB(WriteDataE, ExtImmE, ALUSrcE, SrcBE);
    //module Adder(d1, d2, dataout);
    Adder AddPCtarget(PCE, ExtImmE, PCTargetE);
    //module Register3(clk, ALUResultE, WriteDataE, RdE, PCPlus4E, ALUResultM, WriteDataM, RdM, PCPlus4M);
    Register3 reg3(rst, clk, ALUResultE, WriteDataE, RdE, PCPlus4E, ALUResultM, WriteDataM, RdM, PCPlus4M);
    //module DataMemory (AluResult, ReadData, writeData, memwrite, clk);
    DataMemory myDM(ALUResultM, ReadDataM, WriteDataM, MemWriteM, clk);
    //module Register4(clk, ALUResultM, ReadDataM, RdM, PCPlus4M, ALUResultW, ReadDataW, RdW, PCPlus4W);
    Register4 reg4(rst, clk, ALUResultM, ReadDataM, RdM, PCPlus4M, ALUResultW, ReadDataW, RdW, PCPlus4W);
    //module MUX3 (data1, data2, data3, sel, out);
    MUX3 MUXresult(ALUResultW, ReadDataW, PCPlus4W, ResultSrcW, ResultW);



endmodule

module InstructionMemory (pc, instr);
    input [31:0]pc;
    output [31:0] instr;
    reg [31:0] im [0:99];


    initial
        $readmemb("instructions.txt", im);
    
    assign instr = im[pc/4];

endmodule

module Register1(rst, clk, FlushD, StallD, instr, PCF, PCPlus4F, instrD, PCD, PCPlus4D);
    
    input clk, FlushD, StallD;
    input [31:0] instr, PCF, PCPlus4F, instrD, PCD, PCPlus4D;

    always @(posedge clk, posedge rst) begin 

        if(FlushD | rst) begin 
            {instrD, PCD, PCPlus4D} = 96'b0;
        end

        if(!StallD) begin
            instrD <= instr;
            PCD <= PCF;
            PCPlus4D <= PCPlus4F;
        end
    end

endmodule


module RegisterFile (A1, A2, A3, wd, Rd1, Rd2, we, clk);
    input [4:0]A1, A2, A3;
    input we, clk;
    input [31:0] wd;
    output reg [31:0] Rd1, Rd2;

    reg [31:0] regmem [0:31];
    assign regmem[0] = 32'b0;

    assign Rd1 = regmem[A1];
    assign Rd2 = regmem[A2];

    always @(negedge clk) begin 
        if(we)
            regmem[A3] <= wd;
        // else begin
        // Rd1 <= regmem[A1];
        // Rd2 <= regmem[A2];
        // end    
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
    // ??
endmodule


module Register2(rst, rd1, rd2, RD1E, RD2E, PCD, Rs1D, Rs2D, RdD, ExtImmD, PCPlus4D, PCE, Rs1E, Rs2E, RdE, ExtImmE, PCPlus4E, FlushE, clk); 

    input clk, FlushE;
    input [31:0] rd1, rd2, PCD, ExtImmD, PCPlus4D;
    input [4:0] Rs1D, Rs2D, RdD;
    output reg [31:0] RD1E, RD2E, PCE, ExtImmE, PCPlus4E; 
    output reg [4:0] Rs1E, Rs2E, RdE;

    always @(posedge clk, posedge rst) begin 
        if(FlushE | rst) begin 
            {RD1E, RD2E, PCE, Rs1E, Rs2E, RdE, ExtImmE, PCPlus4E} <= 175'b0;
        end
        else begin 
            RD1E <= rd1;
            RD2E <= rd2;
            PCE <= PCD;
            Rs1E <= Rs1D;
            Rs2E <= Rs2D;
            RdE <= RdD;
            ExtImmE <= ExtImmD;
            PCPlus4E <= PCPlus4D;
            
        end
    end


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
    SLT = 3'b100,
    COMPARE = 3'b101,
    XOR = 3'b111,
    BUFFB = 3'b110;
    ;

    assign zero = ~(|AluResult);
    assign blt = (AluResult[31] == 1) ? 1'b1 : 1'b0;
    assign bge = (AluResult[31] == 0) ? 1'b1 : 1'b0;

    always @(*) begin // == or <=
        case(AluControl) 
            ADD :  begin AluResult = srcA + srcB; $display("dakhele add miay? srcA: %b, srcB : %b", srcA, srcB); end
            SUB : begin AluResult = srcA - srcB; if(AluResult == 0) zero = 0; end
            AND : begin AluResult = (srcA & srcB); end
            OR : begin AluResult = (srcA | srcB); end
            SLT : begin zero = (srcA < srcB) ? 1'b1 : 1'b0;
                          AluResult = (srcA < srcB) ? 32'b1 : 32'b0; end
            COMPARE : begin AluResult = (srcA < srcB); end  
            XOR : begin AluResult = (srcA ^ srcB); end   
            BUFFB : begin AluResult = srcB; end         
        endcase
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


module Register3(rst, clk, ALUResultE, WriteDataE, RdE, PCPlus4E, ALUResultM, WriteDataM, RdM, PCPlus4M);

    input clk;
    input [31:0] ALUResultE, WriteDataE, PCPlus4E;
    input [4:0] RdE;
    output reg [31:0] ALUResultM, WriteDataM, PCPlus4M;
    output reg [4:0] RdM;

    always @(posedge clk, posedge rst) begin 
        if(rst) begin 
            {ALUResultM, WriteDataM, PCPlus4M, RdM} = 101'b0;
        end
        else begin
            ALUResultM <= ALUResultE;
            WriteDataM <= WriteDataE;
            RdM <= RdE;
            PCPlus4M <= PCPlus4E;
        end
    end


endmodule

module DataMemory (AluResult, ReadData, writeData, memwrite, clk);
    input clk, memwrite;
    input [31:0] AluResult;
    input [31:0] writeData;
    output reg [31:0] ReadData;

    reg [31:0] datamem [0:99];

    initial 
        $readmemb("your_file.txt", datamem);
     

    always @(posedge clk, posedge memwrite) begin 
        if(memwrite)
            datamem[AluResult] <= writeData;
        else begin 
            ReadData <= datamem[AluResult];
        end 
    end

    // register 32*2^14
endmodule


module Register4(rst, clk, ALUResultM, ReadDataM, RdM, PCPlus4M, ALUResultW, ReadDataW, RdW, PCPlus4W);

    input clk;
    input [31:0] ALUResultM, ReadDataM, PCPlus4M;
    input [4:0] RdM;
    output reg [31:0] ALUResultW, ReadDataW, PCPlus4W;
    output reg [4:0] RdW;

    always @(posedge clk, posedge rst) begin 
        if(rst) begin 
            {ALUResultW, ReadDataW, PCPlus4W, RdW} = 101'b0;
        end
        else begin
            ALUResultW <= ALUResultM;
            ReadDataW <= ReadDataM;
            RdW <= RdM;
            PCPlus4W <= PCPlus4M;
        end
    end


endmodule

module PCF(PCFin, PCFout, clk, StallF, rst); 

    input clk, StallF;
    input [31:0] PCFin;
    output reg [31:0] PCFout;

    always @(posedge clk, posedge rst) begin 
        if(rst) begin 
            PCFout <= 32'b0;
        end
        if(!StallF) begin 
            PCFout <= PCFin;
        end
    end


endmodule