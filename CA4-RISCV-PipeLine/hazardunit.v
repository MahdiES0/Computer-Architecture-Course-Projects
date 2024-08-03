module HazardUnit(Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW, PCSrcE, ResultSrcE, RegWriteM, RegWriteW, StallF, StallD, FlushD, FlushE, ForwardAE, ForwardBE);

    input [4:0] Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW;
    input PCSrcE, RegWriteW, RegWriteM;
    input [1:0] ResultSrcE;

    output StallF, StallD, FlushD, FlushE;
    output [1:0] ForwardAE, ForwardBE;

    wire lwStall;

    assign lwStall = ((Rs1D == RdE) or (Rs2D == RdE)) and ResultSrcE;
    assign StallF = lwStall;
    assign StallD = lwStall;

    assign FlushD = PCSrcE;
    assign FlushE = PCSrcE or lwStall;
    assign ForwardAE = ((Rs1E == RdM) and RegWriteM) and (Rs1E != 1'b0) ? 2'b10:
                       ((Rs1E == RdW) and RegWriteW) and (Rs1E != 1'b0) ? 2'b01:
                       2'b00;

    assign ForwardBE = ((Rs2E == RdM) and RegWriteM) and (Rs2E != 1'b0) ? 2'b10:
                       ((Rs2E == RdW) and RegWriteW) and (Rs2E != 1'b0) ? 2'b01:
                       2'b00;

endmodule