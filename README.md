# EE511 Project 2: 5-Stage Pipelined Cortex-M0 Processor

## Project Overview
In this project, you will design a 5-stage pipelined processor compatible with Cortex-M0. The processor will operate exclusively in thread mode, with the following specifications:
- Excluded instructions: Memory barrier, 'DMB', 'DSB', 'ISB'
- Focus on thread mode registers: R0-R12, MSP (R13), LR (R14), PC (R15), and APSR

## Provided Files
1. **Processor Files**
   - `CortexM0.v` - 5-stage pipelined processor (to be modified and submitted)
   - `MemModel.v` - Memory functional model for main memory
   - `RegisterFileModel.v` - Register file functional model in the processor

2. **Test Files**
   - `tb.v` - Testbench file
   - `tb.f` - File list for testing
   - `test.hex` - Simple program (initializes registers and performs '5+7')
   - `test.dis` - Corresponding disassembly

3. **Sample Test Case**
   - `ALU_TEST.v`
   - `ALU_TEST.f`
   - `ALU_TEST_inst.hex`

To run ALU test, 
iverilog -c .\ALU_TEST.f -o .\ALU_TEST
vvp .\ALU_TEST
generate ALU_TEST_data.hex for each op_test ?????????

## Project Requirements

### 1. Code Organization
- Place all relevant code in `CortexM0.v`
- Do not create additional source files

### 2. Code Modifications
- Only modify sections marked with 'your code here*' in `CortexM0.v`
- Do not alter input/output ports of `CortexM0.v`
- Use provided `RegisterFileModel.v` for register file
- Use provided `MemModel.v` for main memory

### 3. Code Restrictions
- Do not include Verilog System Tasks and Functions (e.g., `$display`) in `CortexM0.v`

### 4. Documentation
Submit a report file named `{Student_ID}_{Name}_report.pdf` containing:
- Hardware block diagram of the data-path
- Test results
- Explanation of test program design decisions
