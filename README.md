# Computer-Architecture-Course-Projects
# Computer Architecture Course Projects

## Intro
Projects for the computer architecture course at Tehran University.

## CA 1: Maze

### Maze Intro
In this project, we read maze data from a file and then attempt to find a path from our mouse to the cheese. The movement priority is up, right, left, down, ensuring the route we find is unique.

### Maze Asset
- **maps**: List of maps: in each file, if you convert the 4-digit hex string to binary format, each 1 represents a wall and each 0 indicates empty cells through which we can move. There are 16 lines, each with a 4-digit hex, forming a 16 x 16 map.

### Maze Code
Verilog files that you can use to simulate the project using ModelSim.

## CA 2-4: RISC-V

### RISC-V Intro

In these projects, we designed a RISC-V processor using Verilog with different approaches. The first approach employs a simple Single-Cycle model, which is not very efficient in terms of hardware utilization. The second approach uses a shorter cycle but multiple cycles, allowing each command to use as many cycles as needed, resulting in better performance. The final approach is the Pipeline model, which combines some instructions to achieve the best performance.

Supported commands:
- **R-Type**: add, sub, and, or, slt
- **I-Type**: lw, addi, xori, slti, jalr
- **S-Type**: sw
- **J-Type**: jal
- **B-Type**: beq, bneq, blt, beg
- **U-Type**: lui

### RISC-V Assets
- **assembly**: Assembly code of RISC-V.
- **controller and datapath**: General design of RISC-V.
