`timescale 1ns/1ns

module Controller(input clk, rst, start, directionsignal, dout, load, done, run, backtrack, stackempty, fail, waitt, output reg rd, wr, push, ldx, ldy, pop, popit, move); 

    parameter [3:0]
        ST0 = 4'b0000,
        ST1 = 4'b0001,
        ST2 = 4'b0010,
        ST3 = 4'b0011,
        ST4 = 4'b0100,
        ST5 = 4'b0101,
        ST6 = 4'b0110,
        ST7 = 4'b0111,
        ST8 = 4'b1000,
        ST9 = 4'b1001,
        ST10 = 4'b1010,
        ST11 = 4'b1011,
        ST12 = 4'b1100;

    reg [3:0] ps = ST0;
    reg [3:0] ns = ST0;

    always @(waitt, ps, start, directionsignal, dout, load, done, run, move, backtrack, stackempty, fail) begin
        {rd, wr, push, ldx, ldy, pop, popit, move} = 8'b00000000;
        case(ps) 
            ST0 : begin ns = start ? ST1 : ST0; end
            ST1 : begin ns = directionsignal ? ST2 : ST1; end
            ST2 : begin ns = !dout ? ST3 : done ? ST5 : backtrack ? ST11 : ST2; {rd} = 1'b1; end
            ST3 : begin ns = load ? ST4 : ST3; {push, wr} = 2'b11; end
            ST4 : begin ns = done ? ST5 : ST2; {ldx, ldy} = 2'b11; end
            ST5 : begin ns = run ? ST6 : ST5 ; end
            ST6 : begin ns = ST0; move = 1'b1; end
            ST7 : begin ns = stackempty ? ST9 : ST12; pop = 1'b1; end
            ST8 : begin ns = ST2; {ldx, ldy} = 2'b11; end
            ST9 : begin ns = fail ? ST10 : ST9; end 
            ST10 : begin ns = directionsignal ? ST2 : ST10; end
            ST11 : begin ns = ST7; {wr} = 1'b1; end
            ST12 : begin ns = load ? ST8 : ST12; popit = 1'b1; end
        endcase
    end
    
    always @(posedge clk, posedge rst) begin
        if(rst) begin
            ps <= ST0;
            ns <= ST0;
        end
        else
            ps <= ns;
    end  
endmodule


module Datapath (popit, rd, wr, push, ldx, ldy, pop, clk, rst, run, start, dout, directionsignal, load, done, run, move, backtrack, stackempty, fail, waitt);

    input move, rd, wr, push, ldx, ldy, pop, clk, rst, run, start, popit;
    output directionsignal, load, done, backtrack, stackempty, fail, dout, waitt; 

    wire [1:0] dirin;
    wire [1:0] dirout;

    Stack MyStack(clk, move, rst, pop, push, dirout, dirin, stackempty); 
    IntRat MyRat(popit, waitt, clk, rst, pop, dirin, directionsignal, backtrack, dout, rd, wr, ldx, ldy, dirout, load, done, fail); //ta dout input


endmodule

module Memory(popit, x, y, clk, pop, dirin, rd, wr, load, backtrack, dout, newx, newy, dirout, load, directionsignal, waitt, fail); 

    input [3:0] x, y;
    input clk, rd, wr, pop, popit;
    input [1:0] dirin;

    output reg load, dout, directionsignal;
    output reg [3:0] newx = 1'b0;
    output reg [3:0] newy = 1'b0;
    output reg [1:0] dirout;
    reg [0:15] memory [0:15];
    output reg backtrack = 1'b0;
    output reg waitt = 1'b0;
    output reg fail = 1'b0;

    parameter [1:0] UP = 2'b00, RIGHT = 2'b01, LEFT = 2'b10, DOWN = 2'b11;

    initial
        $readmemh("mymaze.dat", memory); 
        
    always @(posedge clk) begin
        directionsignal <= 1'b1;
        dout <= 1'b1;
        if(wr) 
        begin
            memory[x][y] <= 1'b1;
        end
        else if(rd)
        begin
           dout <= 1'b0; 
           load <= 1'b0;
           directionsignal <= 1'b1;
            if(((x - 1) >= 0) && (memory[x - 1][y] == 1'b0)) begin
                dout <= 1'b0;
                newx <= x - 1;
                load <= 1'b1;
                dirout <= UP;
            end
            else if(((y + 1) <= 15) && (memory[x][y + 1]) == 1'b0) begin
                dout <= 1'b0;
                newy <= y + 1; 
                load <= 1'b1;
                dirout <= RIGHT;
            end
            else if(((y - 1) >= 0) && (memory[x][y - 1]) == 1'b0) begin
                dout <= 1'b0;
                newy <= y - 1;
                load <= 1'b1;
                dirout <= LEFT;
            end
            else if(((x + 1) <= 15) && (memory[x + 1][y]) == 1'b0) begin
                dout <= 1'b0;
                newx <= x + 1;
                load <= 1'b1;
                dirout <= DOWN;
            end
            else begin 
                directionsignal <= 1'b1;
                backtrack <= 1'b1;
                dout <= 1'b1;
                load <= 1'b1; 
            end
        end

        if(popit) begin
            backtrack <= 1'b0;
            load <= 1'b0;
            if(dirin == UP) begin
                newx <= x - 1;
                load <= 1'b1;
            end
            else if(dirin == RIGHT) begin
                newy <= y + 1; 
                load <= 1'b1; 
            end
            else if(dirin == LEFT) begin
                newy <= y - 1;
                load <= 1'b1;
            end
            else if(dirin == DOWN) begin
                newx <= x + 1;
                load <= 1'b1;
            end
            else begin 
                fail <= 1'b1;
                load <= 1'b0;
            end
        end
    end
    
endmodule

module IntRat (popit, waitt, clk, rst, pop, dirin, directionsignal, backtrack, dout, rd, wr, ldx, ldy, dirout, load, done ,fail); 

    input  clk, rst, pop, ldx, ldy, rd, wr, popit;
    output backtrack, load, directionsignal, dout, waitt;
    output reg done = 1'b0;
    output fail;
    
    input [1:0] dirin;
    output [1:0] dirout;

    wire [3:0] newx;
    wire [3:0] newy;

    reg[3:0] x; 
    reg[3:0] y;
    
    parameter [1:0] UP = 2'b00, RIGHT = 2'b01, LEFT = 2'b10, DOWN = 2'b11;

    Memory m1(popit, x, y, clk, pop, dirin, rd, wr, load, backtrack, dout, newx, newy, dirout, load, directionsignal, waitt, fail);
    always @(posedge clk, posedge rst) begin
        if(rst)begin
            x <= 4'b0000;
            y <= 4'b0000;
        end 

        if (ldx) begin
            x <= newx;
        end

        if(ldy)begin 
            y <= newy;
        end
        
        if((x == 15) && (y == 15))begin
            done <= 1;
        end
    end

endmodule

module Stack(clk, move, rst, pop, push, dirout, dirin, stackempty);

    input [1:0] dirout;
    input push, pop, clk, rst, move;
    output reg [1:0] dirin;
    output reg stackempty = 1'b0;
   
    reg [1:0] stack [255:0]; 

    integer counter = 0;
    integer i = 0;
    integer j;

    always @(posedge clk, posedge rst) begin
        if(rst) begin
            counter <= 0;
        end
        else if(push)begin
            stack[counter] <= dirout;
            counter <= counter + 1;
        end
        else if(pop)begin
            dirin = ~(stack[counter - 1]);
            counter <= counter - 1;  
            if(!counter)
                stackempty <= 1;
        end
        else if(move)begin
            for(i = 0; i < counter; i = i + 1)
            begin
                if(stack[i] == 2'b00)
                    $display("UP ");
                
                if(stack[i] == 2'b01)
                    $display("RIGHT ");
                
                if(stack[i] == 2'b10)
                    $display("LeFT ");
                
                if(stack[i] == 2'b11)
                    $display("DOWN ");
            end
            $display("---------------------------------------------------------FINISHED---------------------------------------------------------");
        end
    end

endmodule

module MyMaze(start, clk, rst, run, fail, done, move);

    input start, clk, rst, run;
    output fail, done, move;

    wire rd, wr;
    wire dout, dir, backtrack, ldx, ldy, stackempty;
    wire push, pop, qsignal, load, waitt;


    Datapath dp(popit, rd, wr, push, ldx, ldy, pop, clk, rst, run, start, dout, directionsignal, load, done, run, move, backtrack, stackempty, fail, waitt); ////.////
    Controller cu(.clk(clk), .rst(rst), .start(start), .directionsignal(directionsignal),
     .dout(dout), .load(load), .done(done), .run(run), .backtrack(backtrack), 
     .stackempty(stackempty), .fail(fail),
      .rd(rd), .wr(wr), .push(push), .ldx(ldx), .ldy(ldy), .pop(pop), .waitt(waitt), .popit(popit), .move(move));
endmodule

module tb4();
    reg ss;
    reg cc = 1'b0;
    reg rst;
    reg run = 1'b1;

    wire fail;
    wire done;
    wire move;

    MyMaze mymaze(ss, cc, rst, run, fail, done, move);

    always #10 cc = ~cc;
    initial begin
        #40 rst = 1'b1;
        #25 rst = 1'b0;
        #30 ss = 1'b1;
        #100000 $stop;
    end

endmodule