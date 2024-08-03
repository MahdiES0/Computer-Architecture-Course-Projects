module myPipe(rst, clk); 

input rst, clk;
wire [4:0] Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW;
wire PCSrcE, RegWriteW, RegWriteM;
wire [1:0] ResultSrcE;
wire StallF, StallD, FlushD, FlushE;
wire [1:0] ForwardAE, ForwardBE;
wire [6:0] op;
wire [2:0] func3;
wire [6:0] func7;
//output reg regwriteW, memwriteM, alusrcE, pcsrcE;
//output reg [1:0] resultsrcW, immsrcD;
//output reg [2:0] alucntrlE;
wire ALUSrcE, MemWriteM;
wire [1:0] ImmSrcD, ResultSrcW;
wire [2:0] ALUControlE;
wire zero, blt, bge;


DataPath mydp(rst, clk, RegWriteW, ImmSrcD, ALUSrcE, ALUControlE, MemWriteM, ResultSrcW, op, func3, func7, zero, bge, blt, StallF, StallD, FlushD, FlushE, ForwardAE, ForwardBE, Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW); 
Controller myController(clk, rst, zeroE, op, func3, func7, RegWriteW, ResultSrcW, MemWriteM, ALUControlE, ALUSrcE, ImmSrcD, PCSrcE);
HazardUnit myHZ(Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW, PCSrcE, ResultSrcE, RegWriteM, RegWriteW, StallF, StallD, FlushD, FlushE, ForwardAE, ForwardBE);

endmodule


module CPUPiPEtb(); 

reg cc = 1'b1;
reg rr = 1'b1;

theCPU myPipe(.clk(cc), .reset(rr));

always #20 cc = ~cc;
initial begin
    #20 rr = 1'b0;
    #5000 $stop;
end

endmodule