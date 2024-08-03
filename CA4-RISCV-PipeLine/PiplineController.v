module Controller (clk, rst, zeroE, op, func3, func7, regwriteW, resultsrcW, memwriteM, aluctrlE, alusrcE, immsrcD, pcsrcE);
    input clk, rst, zeroE;
    input [6:0] op;
    input [2:0] func3;
    input [6:0] func7;
    output reg regwriteW, memwriteM, alusrcE, pcsrcE;
    output reg [1:0] resultsrcW, immsrcD;
    output reg [2:0] alucntrlE;

    reg jumpD, branchD;
    reg memwriteD, alusrcD, regwriteD;
    reg [1:0] resultsrcD;
    reg [2:0] aluctrlD;

    wire regwriteE, memwriteE, regwriteM, branchE, jumpE;
    wire [1:0] resultsrcE, resultsrcM;

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
    {jumpD, branchD, resultsrcD, memwriteD, aluctrlD, alusrcD, immsrcD, regwriteD} = 12'b0;
    case(op)
        RT : begin
            // pcsrcE = 2'b00;
            branchD = 1'b0
            jumpD = 1'b0;
            regwriteD = 1'b1;
            alusrcD = 1'b0;
            memwriteD = 1'b0;
            resultsrcD = 2'b00;
            immsrcD = 3'b000;
            if((func3 == ADD) & (func7 == ADD7)) aluctrlD = 3'b000;
            else if((func3 == SUB) & (func7 == SUB7)) aluctrlD = 3'b001;
            else if((func3 == SLT) & (func7 == SLT7)) aluctrlD = 3'b101;
            else if((func3 == OR) & (func7 == OR7)) aluctrlD = 3'b011;
            else if((func3 == AND) & (func7 == AND7)) aluctrlD = 3'b010;
            else aluctrlD = 3'bx;
        end

       
        LW : begin
            // pcsrcE = 2'b00;
            branchD = 1'b0;
            jumpD = 1'b0;
            regwriteD = 1'b1;
            immsrcD = 3'b000;
            alusrcD = 1'b1;
            memwriteD = 1'b0;
            resultsrcD = 2'b01;
            aluctrlD = 3'b000;
        end

        IT : begin 
            branchD = 1'b0;
            jumpD = 1'b0;
            regwriteD = 1'b1;
            immsrcD = 3'b000;
            alusrcD = 1'b1;
            memwriteD = 1'b0;
            resultsrcD = 1'b0;
            if((func3 == ADDI)) aluctrlD = 3'b000;
            else if((func3 == SLTI)) aluctrlD = 3'b101;
            else if((func3 == XORI)) aluctrlD = 3'b111;
            else if((func3 == ORI)) aluctrlD = 3'b001;
        end

        JALr : begin     
            // pcsrcE = 2'b10;
            branchD = 1'b0;
            jumpD = 1'b1;
            regwriteD = 1'b1;
            immsrcD = 3'b000;
            memwriteD = 1'b0;
            resultsrcD = 2'b11;
            alusrcD = 1'b1;
            aluctrlD = 3'b000;

        end

        SW : begin 
            // pcsrcE = 2'b00;
            branchD = 1'b0;
            jumpD = 1'b0;
            regwriteD = 1'b0;
            immsrcD = 3'b001;
            alusrcD = 1'b1;
            memwriteD = 1'b1;
            aluctrlD = 3'b000;
            resultsrcD = 2'b10;
        end

        JAL : begin     
            // pcsrcE = 2'b01;
            branchD = 1'b0;
            jumpD = 1'b1;
            alusrcD = 1'b1;
            resultsrcD = 2'b11;
            regwriteD = 1'b0;
            immsrcD = 3'b011;
            memwriteD = 1'b0;
            //jump = 1'b1;
            aluctrlD = 3'b000;
            if(func3 == 3'b010) aluctrlD = 3'b000;
        end

        BT : begin 
            branchD = 1'b1;
            jumpD = 1'b0;
            regwriteD = 1'b0;
            immsrcD = 3'b010;
            alusrcD = 1'b0;
            memwriteD = 1'b0;
            aluctrlD = 3'b001;
            // pcsrcE = (func3 == 3'b000 && zero == 1'b1) ? 2'b01:
            //         (func3 == 3'b001 && zero == 1'b0) ? 2'b01:
            //         (func3 == 3'b100 && bge == 1'b1) ? 2'b01:
            //         (func3 == 3'b101 && blt == 1'b1) ? 2'b01:
            //         2'b00;
        end

        LUi : begin 
            branchD = 1'b0;
            jumpD = 1'b0;
            regwriteD = 1'b1;
            immsrcD = 3'b011;
            memwriteD = 1'b0;
            resultsrcD = 2'b11;
            aluctrlD = 3'b110;

            // pcsrcE = 2'b00;
        end
    endcase
end

    ERegister er(clk, regwriteD, resultsrcD, memwriteD, jumpD, branchD, aluctrlD, alusrcD, regwriteE, resultsrcE, 
        memwriteE, jumpE, branchE, alucntrlE, alusrcE);
    MRegister mr(clk, regwriteE, resultsrcE, memwriteE, regwriteM, memwriteM,resultsrcM);
    WRegister wr(clk, regwriteM, resultsrcM, regwriteW, resultsrcW);

    assign pcsrcE = (zeroE & branchE) | jumpE;

endmodule

module ERegister (clk, regwriteD, resultsrcD, memwriteD, jumpD, branchD, aluctrlD, alusrcD, regwriteE, resultsrcE, 
    memwriteE, jumpE, branchE, alucntrlE, alusrcE);
    input clk, regwriteD, memwriteD, jumpD, branchD, alusrcD;
    input [1:0] resultsrcD, immsrcD;
    input [2:0] aluctrlD;
    output reg regwriteE, memwriteE, jumpE, branchE, alusrcE;
    output reg [1:0] resultsrcE, immsrcE;
    output reg [2:0] aluctrlE;
    
    always @(posedge clk) begin
        regwriteE <= regwriteD;
        memwriteE <= memwriteD;
        jumpE <= jumpD;
        branchE <= branchD;
        alusrcE <= alusrcD;
        resultsrcE <= resultsrcD;
        immsrcE <= immsrcD;
        aluctrlE <= aluctrlD;
    end

endmodule

module MRegister (clk, regwriteE, resultsrcE, memwriteE, regwriteM, memwriteM,resultsrcM);
    input clk, regwriteE, memwriteE;
    input [1:0] resultsrcE;
    output reg regwriteM, memwriteM;
    output reg [1:0] resultsrcM;

    always @(posedge clk) begin
        regwriteM <= regwriteE;
        memwriteM <= memwriteE;
        resultsrcM <= resultsrcE;
    end

endmodule

module WRegister (clk, regwriteM, resultsrcM, regwriteW, resultsrcW);
    input clk, regwriteM;
    input [1:0] resultsrcM;
    output reg regwriteW;
    output reg [1:0] resultsrcW;

    always @(posedge clk) begin
        regwriteW <= regwriteM;
        resultsrcW <= resultsrcM;
    end

endmodule