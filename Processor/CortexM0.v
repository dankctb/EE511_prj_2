module CortexM0 (
	input	wire		      CLK,
	input	wire	        RESET_N, // reset when negative
	
	// For instruction memory
	output wire	    		IREQ,
	output wire [31:0]	IADDR,
	input	wire  [31:0]	INSTR,

	// For data memory
	output wire	    		DREQ,
	output wire	[31:0]	DADDR,
	output wire	  			DRW,
	output wire	[ 1:0]	DSIZE,
	input	wire	[31:0]	DIN,
	output wire	[31:0]	DOUT
);

// add input ports for register file
REGFILE REGFILE (
  .CLK(CLK),
  .nRST(RESET_N),
  .WEN1(reg_write_en),
  .WA1(reg_write_addr), 
  .DI1(reg_write_data), 
  .WEN2(1'b0),  // Second write port not used
  .WA2(4'h0),
  .DI2(32'h0),
  .RA0(reg_read_addr1),
  .RA1(reg_read_addr2),
  .RA2(4'h0),  // Third read port not used
  .DOUT0(reg_read_data1),
  .DOUT1(reg_read_data2),
  .DOUT2()
);

// your code here
// Basic signals and control registers
reg [31:0] PC;              // Program Counter
reg [31:0] PC_next;         // Next Program Counter

// Register file interface signals
reg [3:0] reg_read_addr1;   // First source register address
reg [3:0] reg_read_addr2;   // Second source register address
wire [31:0] reg_read_data1; // First source register data
wire [31:0] reg_read_data2; // Second source register data
reg reg_write_en;           // Register write enable
reg [3:0] reg_write_addr;   // Register write address
reg [31:0] reg_write_data;  // Register write data

// Hazard detection signals
wire data_hazard;           // Data hazard detected
reg stall_pipeline;         // Pipeline stall signal
reg flush_pipeline;         // Pipeline flush signal

// Forwarding signals
reg [1:0] forward_a;        // Forwarding control for ALU input A
reg [1:0] forward_b;        // Forwarding control for ALU input B

// Instruction fields
wire [15:0] instr_16bit;    // 16-bit instruction
wire [31:0] instr_32bit;    // 32-bit instruction
wire [3:0] opcode;          // Instruction opcode
wire [3:0] rd;              // Destination register
wire [3:0] rn;              // First source register
wire [3:0] rm;              // Second source register
wire [11:0] imm12;          // 12-bit immediate

// ALU signals
reg [31:0] alu_a;           // ALU input A
reg [31:0] alu_b;           // ALU input B
reg [31:0] alu_result;      // ALU result
reg alu_carry;              // ALU carry flag
reg alu_zero;               // ALU zero flag
reg alu_negative;           // ALU negative flag
reg alu_overflow;           // ALU overflow flag

// Control signals
reg mem_to_reg;             // Memory to register flag
reg alu_src;                // ALU source flag
reg reg_write;              // Register write flag
reg mem_read;               // Memory read flag
reg mem_write;              // Memory write flag
reg [1:0] mem_size;         // Memory access size
reg [3:0] alu_op;           // ALU operation
reg branch;                 // Branch flag
reg [31:0] branch_target;   // Branch target address

// Status register (APSR)
reg apsr_n;                 // Negative flag
reg apsr_z;                 // Zero flag
reg apsr_c;                 // Carry flag
reg apsr_v;                 // Overflow flag

// Pipeline registers
// IF/ID Pipeline Registers
reg [31:0] IF_ID_PC;        // PC value in ID stage
reg [31:0] IF_ID_INSTR;     // Instruction in ID stage

// ID/EX Pipeline Registers
reg [31:0] ID_EX_PC;
reg [31:0] ID_EX_REG_A;
reg [31:0] ID_EX_REG_B;
reg [31:0] ID_EX_IMM;
reg [3:0]  ID_EX_DEST_REG;
reg        ID_EX_REG_WE;
reg        ID_EX_MEM_RE;
reg        ID_EX_MEM_WE;
reg [1:0]  ID_EX_MEM_SIZE;
reg [3:0]  ID_EX_ALU_CTRL;
reg        ID_EX_ALU_SRC;
reg        ID_EX_MEM_TO_REG;

// EX/MEM Pipeline Registers
reg [31:0] EX_MEM_ALU_RESULT;
reg [31:0] EX_MEM_REG_B;
reg [3:0]  EX_MEM_DEST_REG;
reg        EX_MEM_REG_WE;
reg        EX_MEM_MEM_RE;
reg        EX_MEM_MEM_WE;
reg [1:0]  EX_MEM_MEM_SIZE;
reg        EX_MEM_MEM_TO_REG;

// MEM/WB Pipeline Registers
reg [31:0] MEM_WB_MEM_DATA;
reg [31:0] MEM_WB_ALU_RESULT;
reg [3:0]  MEM_WB_DEST_REG;
reg        MEM_WB_REG_WE;
reg        MEM_WB_MEM_TO_REG;

// Memory interface connections
assign IREQ = 1'b1;               // Always requesting instructions
assign IADDR = PC;                // PC is the address for instruction fetch
assign DREQ = EX_MEM_MEM_RE || EX_MEM_MEM_WE;  // Data request when reading or writing
assign DADDR = EX_MEM_ALU_RESULT; // ALU result is the address for data access
assign DRW = EX_MEM_MEM_WE;       // Read=0, Write=1
assign DSIZE = EX_MEM_MEM_SIZE;   // Access size (byte, halfword, word)
assign DOUT = EX_MEM_REG_B;       // Data to be written to memory

// Instruction fetch stage (IF)
always @(posedge CLK or negedge RESET_N) begin
    if (~RESET_N) begin
        PC <= 32'h0;  // Reset PC to 0
    end else begin
        PC <= PC_next; // Update PC
    end
end

// PC update logic
always @(*) begin
    // Default sequential execution
    PC_next = PC + 4;
    
    // Stall if data hazard detected
    if (stall_pipeline) begin
        PC_next = PC;  // Keep PC the same to stall
    end
    // Branch handling (in ID stage)
    else if (branch) begin
        PC_next = branch_target;
    end
end

// Instruction fields extraction
assign instr_16bit = IF_ID_INSTR[15:0];
assign instr_32bit = IF_ID_INSTR;

// Detailed instruction field extraction for various formats
// These are simplified examples - actual Cortex-M0 instruction decoding is more complex
wire [3:0] rn_field = instr_16bit[7:4];    // Rn field in many formats
wire [3:0] rm_field = instr_16bit[3:0];    // Rm field in many formats  
wire [3:0] rd_field = instr_16bit[11:8];   // Rd field in most formats
wire [7:0] imm8 = instr_16bit[7:0];        // 8-bit immediate
wire [10:0] imm11 = instr_16bit[10:0];     // 11-bit immediate for branch
wire [2:0] imm3 = instr_16bit[8:6];        // 3-bit immediate for shift
wire [1:0] imm2 = instr_16bit[1:0];        // 2-bit immediate

// T1 encoding - main opcode field
wire [5:0] t1_opcode = instr_16bit[15:10]; 

// Determine instruction type
wire is_data_proc = (t1_opcode[5:2] == 4'b0100);                     // Data Processing Instructions
wire is_add_sub_imm = (t1_opcode == 6'b000111) || (t1_opcode == 6'b000110); // ADD/SUB immediate
wire is_mov_imm = (t1_opcode == 6'b001000);                          // MOV immediate
wire is_alu_op = (t1_opcode == 6'b010000);                           // ALU operations
wire is_branch = (t1_opcode[5:3] == 3'b101);                         // Branch instructions
wire is_load_store = (t1_opcode[5:4] == 2'b01 && t1_opcode[3:2] != 2'b00); // Load/Store

// For ALU operations in 0100 00xx xxxx xxxx format
wire [3:0] alu_operation = instr_16bit[9:6]; // ALU operation bits

// Simplified ALU operation decoding
function [3:0] decode_alu_op;
    input [3:0] op_bits;
    begin
        case(op_bits)
            4'b0000: decode_alu_op = 4'h2;  // AND -> ALU op 2
            4'b0001: decode_alu_op = 4'h4;  // EOR/XOR -> ALU op 4
            4'b0010: decode_alu_op = 4'h7;  // LSL -> ALU op 7
            4'b0011: decode_alu_op = 4'h8;  // LSR -> ALU op 8
            4'b0100: decode_alu_op = 4'h0;  // ADD -> ALU op 0
            4'b0101: decode_alu_op = 4'h1;  // SUB -> ALU op 1
            4'b1000: decode_alu_op = 4'h6;  // MUL -> ALU op 6
            4'b1111: decode_alu_op = 4'h5;  // MVN/NOT -> ALU op 5
            4'b1100: decode_alu_op = 4'h3;  // ORR -> ALU op 3
            default: decode_alu_op = 4'h0;  // Default to ADD
        endcase
    end
endfunction

// Extract opcode and operands for different instruction formats
assign opcode = instr_16bit[15:12];  // Simplified opcode extraction
assign rd = (is_alu_op) ? instr_16bit[2:0] : rd_field;  // Destination register depends on format
assign rn = (is_alu_op) ? instr_16bit[5:3] : rn_field;  // First source reg depends on format
assign rm = (is_alu_op) ? instr_16bit[8:6] : rm_field;  // Second source reg depends on format
assign imm12 = instr_32bit[11:0];    // 12-bit immediate (for some 32-bit instructions)

// Hazard detection logic
assign data_hazard = 
    // Load-use hazard
    (ID_EX_MEM_RE && 
     ((ID_EX_DEST_REG == reg_read_addr1) || 
      (ID_EX_DEST_REG == reg_read_addr2))) ? 1'b1 : 1'b0;

// Pipeline control
always @(*) begin
    stall_pipeline = data_hazard;
    flush_pipeline = branch; // Flush on branch taken
end

// Forwarding logic
always @(*) begin
    // Default: no forwarding
    forward_a = 2'b00;
    forward_b = 2'b00;
    
    // EX hazard
    if (EX_MEM_REG_WE && 
        (EX_MEM_DEST_REG != 4'h0) && 
        (EX_MEM_DEST_REG == ID_EX_DEST_REG)) begin
        forward_a = 2'b10;
    end
    // MEM hazard
    else if (MEM_WB_REG_WE && 
             (MEM_WB_DEST_REG != 4'h0) && 
             (MEM_WB_DEST_REG == ID_EX_DEST_REG)) begin
        forward_a = 2'b01;
    end
    
    // EX hazard
    if (EX_MEM_REG_WE && 
        (EX_MEM_DEST_REG != 4'h0) && 
        (EX_MEM_DEST_REG == reg_read_addr2)) begin
        forward_b = 2'b10;
    end
    // MEM hazard
    else if (MEM_WB_REG_WE && 
             (MEM_WB_DEST_REG != 4'h0) && 
             (MEM_WB_DEST_REG == reg_read_addr2)) begin
        forward_b = 2'b01;
    end
end

// ALU input selection with forwarding
always @(*) begin
    // ALU input A with forwarding
    case (forward_a)
        2'b00: alu_a = ID_EX_REG_A;                 // No forwarding
        2'b01: alu_a = reg_write_data;              // Forward from WB stage
        2'b10: alu_a = EX_MEM_ALU_RESULT;           // Forward from MEM stage
        default: alu_a = ID_EX_REG_A;
    endcase
    
    // ALU input B with forwarding and immediate selection
    if (ID_EX_ALU_SRC) begin
        alu_b = ID_EX_IMM; // Use immediate
    end else begin
        case (forward_b)
            2'b00: alu_b = ID_EX_REG_B;             // No forwarding
            2'b01: alu_b = reg_write_data;          // Forward from WB stage
            2'b10: alu_b = EX_MEM_ALU_RESULT;       // Forward from MEM stage
            default: alu_b = ID_EX_REG_B;
        endcase
    end
end

// Instruction decode stage (ID)
// Update pipeline registers
always @(posedge CLK or negedge RESET_N) begin
    if (~RESET_N) begin
        IF_ID_PC <= 32'h0;
        IF_ID_INSTR <= 32'h0;
    end else if (flush_pipeline) begin
        // Flush the pipeline when a branch is taken
        IF_ID_PC <= 32'h0;
        IF_ID_INSTR <= 32'h0; // Insert a NOP
    end else if (!stall_pipeline) begin
        // Regular update if not stalled
        IF_ID_PC <= PC;
        IF_ID_INSTR <= INSTR;
    end
    // If stalled, keep the current values
end

// Control signal generation
always @(*) begin
    // Default values
    alu_src = 1'b0;       // Use register by default
    mem_to_reg = 1'b0;    // Use ALU result by default
    reg_write = 1'b0;     // Don't write to register by default
    mem_read = 1'b0;      // Don't read from memory by default
    mem_write = 1'b0;     // Don't write to memory by default
    mem_size = 2'b10;     // Word size by default
    alu_op = 4'h0;        // ADD by default
    branch = 1'b0;        // No branch by default
    branch_target = 32'h0; // Default branch target
    
    // Detailed control logic based on instruction type
    if (is_alu_op) begin
        // ALU operations (AND, EOR, LSL, LSR, ASR, ADC, SBC, ROR, TST, NEG, CMP, CMN, ORR, MUL, BIC, MVN)
        reg_write = 1'b1; // Most ALU ops write to a register
        alu_op = decode_alu_op(alu_operation);
        
        // Exception for compare instructions that don't write to a register
        if (alu_operation == 4'b1010) begin // CMP
            reg_write = 1'b0; // Compare doesn't write back
        end
    end
    else if (is_add_sub_imm) begin
        // ADD/SUB immediate
        alu_src = 1'b1;  // Use immediate
        reg_write = 1'b1;
        
        // Check if it's ADD or SUB
        if (t1_opcode[0]) 
            alu_op = 4'h1; // SUB
        else
            alu_op = 4'h0; // ADD
    end
    else if (is_mov_imm) begin
        // MOV immediate - we can implement this as ADD with 0
        alu_src = 1'b1;  // Use immediate
        reg_write = 1'b1;
        alu_op = 4'h0;   // Treat as ADD with first operand 0
    end
    else if (is_load_store) begin
        // Common to both load and store: use immediate for address calculation
        alu_src = 1'b1;
        alu_op = 4'h0;   // Use ADD for address calculation
        
        // Determine if it's a load or store
        if (t1_opcode[3]) begin
            // Load instruction
            mem_to_reg = 1'b1;
            reg_write = 1'b1;
            mem_read = 1'b1;
        end else begin
            // Store instruction
            mem_write = 1'b1;
        end
        
        // Determine access size
        if (t1_opcode[1:0] == 2'b00)
            mem_size = 2'b00; // Byte access
        else if (t1_opcode[1:0] == 2'b01)
            mem_size = 2'b01; // Halfword access
        else
            mem_size = 2'b10; // Word access
    end
    else if (is_branch) begin
        // Branch instruction
        branch = 1'b1;
        
        // Different branch types
        if (t1_opcode[2]) begin // Conditional branch
            // Get condition code
            case (instr_16bit[11:8])
                4'b0000: branch = apsr_z;                     // EQ: Z=1
                4'b0001: branch = !apsr_z;                    // NE: Z=0
                4'b0010: branch = apsr_c;                     // CS/HS: C=1
                4'b0011: branch = !apsr_c;                    // CC/LO: C=0
                4'b0100: branch = apsr_n;                     // MI: N=1
                4'b0101: branch = !apsr_n;                    // PL: N=0
                4'b0110: branch = apsr_v;                     // VS: V=1
                4'b0111: branch = !apsr_v;                    // VC: V=0
                4'b1000: branch = (apsr_c && !apsr_z);        // HI: C=1 & Z=0
                4'b1001: branch = (!apsr_c || apsr_z);        // LS: C=0 or Z=1
                4'b1010: branch = (apsr_n == apsr_v);         // GE: N=V
                4'b1011: branch = (apsr_n != apsr_v);         // LT: N!=V
                4'b1100: branch = (!apsr_z && (apsr_n == apsr_v)); // GT: Z=0 & N=V
                4'b1101: branch = (apsr_z || (apsr_n != apsr_v));  // LE: Z=1 or N!=V
                4'b1110: branch = 1'b1;                       // AL: Always
                default: branch = 1'b0;
            endcase
            
            // Calculate branch target - note that PC is 4 bytes ahead in the pipeline
            if (branch)
                branch_target = IF_ID_PC + {{23{imm8[7]}}, imm8, 1'b0};
        end else if (t1_opcode[1:0] == 2'b00) begin // Unconditional branch B
            // PC-relative branch - note the different immediate format
            branch_target = IF_ID_PC + {{20{imm11[10]}}, imm11, 1'b0};
        end
    end
    else begin
        // Default - we'll treat unrecognized instructions as NOPs
        // Set default values as appropriate
    end
end

// Register file connections
always @(*) begin
    // Register read addresses
    reg_read_addr1 = rn;
    reg_read_addr2 = rm;
    
    // Register write connections - from WB stage
    reg_write_en = MEM_WB_REG_WE;
    reg_write_addr = MEM_WB_DEST_REG;
    reg_write_data = MEM_WB_MEM_TO_REG ? MEM_WB_MEM_DATA : MEM_WB_ALU_RESULT;
end

// Execute stage (EX)
// Update pipeline registers
always @(posedge CLK or negedge RESET_N) begin
    if (~RESET_N) begin
        ID_EX_PC <= 32'h0;
        ID_EX_REG_A <= 32'h0;
        ID_EX_REG_B <= 32'h0;
        ID_EX_IMM <= 32'h0;
        ID_EX_DEST_REG <= 4'h0;
        ID_EX_REG_WE <= 1'b0;
        ID_EX_MEM_RE <= 1'b0;
        ID_EX_MEM_WE <= 1'b0;
        ID_EX_MEM_SIZE <= 2'b0;
        ID_EX_ALU_CTRL <= 4'h0;
        ID_EX_ALU_SRC <= 1'b0;
        ID_EX_MEM_TO_REG <= 1'b0;
    end else begin
        ID_EX_PC <= IF_ID_PC;
        ID_EX_REG_A <= reg_read_data1;
        ID_EX_REG_B <= reg_read_data2;
        ID_EX_IMM <= {{20{imm12[11]}}, imm12}; // Sign extend immediate
        ID_EX_DEST_REG <= rd;
        ID_EX_REG_WE <= reg_write;
        ID_EX_MEM_RE <= mem_read;
        ID_EX_MEM_WE <= mem_write;
        ID_EX_MEM_SIZE <= mem_size;
        ID_EX_ALU_CTRL <= alu_op;
        ID_EX_ALU_SRC <= alu_src;
        ID_EX_MEM_TO_REG <= mem_to_reg;
    end
end

// ALU operation
always @(*) begin
    // Default values
    alu_result = 32'h0;
    alu_carry = 1'b0;
    alu_zero = 1'b0;
    alu_negative = 1'b0;
    alu_overflow = 1'b0;
    
    case(ID_EX_ALU_CTRL)
        4'h0: begin // ADD
            {alu_carry, alu_result} = alu_a + alu_b;
        end
        4'h1: begin // SUB
            {alu_carry, alu_result} = alu_a - alu_b;
        end
        4'h2: begin // AND
            alu_result = alu_a & alu_b;
        end
        4'h3: begin // OR (ORR)
            alu_result = alu_a | alu_b;
        end
        4'h4: begin // XOR (EOR)
            alu_result = alu_a ^ alu_b;
        end
        4'h5: begin // NOT (MVN)
            alu_result = ~alu_a;
        end
        4'h6: begin // MUL
            alu_result = alu_a * alu_b;
        end
        4'h7: begin // LSL (logical shift left)
            alu_result = alu_a << alu_b[4:0]; // Shift by the amount in lower 5 bits of alu_b
            if (|alu_b[4:0]) begin
                alu_carry = alu_a[32 - alu_b[4:0]]; // Carry out is the last bit shifted out
            end
        end
        4'h8: begin // LSR (logical shift right)
            alu_result = alu_a >> alu_b[4:0]; // Shift by the amount in lower 5 bits of alu_b
            if (|alu_b[4:0]) begin
                alu_carry = alu_a[alu_b[4:0] - 1]; // Carry out is the last bit shifted out
            end
        end
        default: begin
            // Default to pass A (MOVA)
            alu_result = alu_a;
        end
    endcase
    
    // Set flags - these are important for conditional execution
    alu_zero = (alu_result == 32'h0);
    alu_negative = alu_result[31];
    
    // Overflow is only valid for ADD and SUB
    if (ID_EX_ALU_CTRL == 4'h0) begin // ADD
        // Overflow occurs when two positives give a negative or two negatives give a positive
        alu_overflow = (alu_a[31] == alu_b[31]) && (alu_result[31] != alu_a[31]);
    end else if (ID_EX_ALU_CTRL == 4'h1) begin // SUB
        // For subtraction, we're effectively adding A and -B, so the signs are different
        // Overflow occurs when A and -B have the same sign but result has a different sign
        alu_overflow = (alu_a[31] != alu_b[31]) && (alu_result[31] != alu_a[31]);
    end
end

// Memory stage (MEM)
// Update pipeline registers
always @(posedge CLK or negedge RESET_N) begin
    if (~RESET_N) begin
        EX_MEM_ALU_RESULT <= 32'h0;
        EX_MEM_REG_B <= 32'h0;
        EX_MEM_DEST_REG <= 4'h0;
        EX_MEM_REG_WE <= 1'b0;
        EX_MEM_MEM_RE <= 1'b0;
        EX_MEM_MEM_WE <= 1'b0;
        EX_MEM_MEM_SIZE <= 2'b0;
        EX_MEM_MEM_TO_REG <= 1'b0;
    end else begin
        EX_MEM_ALU_RESULT <= alu_result;
        EX_MEM_REG_B <= ID_EX_REG_B;
        EX_MEM_DEST_REG <= ID_EX_DEST_REG;
        EX_MEM_REG_WE <= ID_EX_REG_WE;
        EX_MEM_MEM_RE <= ID_EX_MEM_RE;
        EX_MEM_MEM_WE <= ID_EX_MEM_WE;
        EX_MEM_MEM_SIZE <= ID_EX_MEM_SIZE;
        EX_MEM_MEM_TO_REG <= ID_EX_MEM_TO_REG;
    end
end

// Update APSR flags
always @(posedge CLK or negedge RESET_N) begin
    if (~RESET_N) begin
        apsr_n <= 1'b0;
        apsr_z <= 1'b0;
        apsr_c <= 1'b0;
        apsr_v <= 1'b0;
    end else begin
        // Update flags only for ALU operations that affect flags
        if (ID_EX_REG_WE && !ID_EX_MEM_TO_REG) begin
            apsr_n <= alu_negative;
            apsr_z <= alu_zero;
            apsr_c <= alu_carry;
            apsr_v <= alu_overflow;
        end
    end
end

// Write-back stage (WB)
// Update pipeline registers
always @(posedge CLK or negedge RESET_N) begin
    if (~RESET_N) begin
        MEM_WB_MEM_DATA <= 32'h0;
        MEM_WB_ALU_RESULT <= 32'h0;
        MEM_WB_DEST_REG <= 4'h0;
        MEM_WB_REG_WE <= 1'b0;
        MEM_WB_MEM_TO_REG <= 1'b0;
    end else begin
        MEM_WB_MEM_DATA <= DIN;  // Read data from memory
        MEM_WB_ALU_RESULT <= EX_MEM_ALU_RESULT;
        MEM_WB_DEST_REG <= EX_MEM_DEST_REG;
        MEM_WB_REG_WE <= EX_MEM_REG_WE;
        MEM_WB_MEM_TO_REG <= EX_MEM_MEM_TO_REG;
    end
end

endmodule

// your code here (for other modules)


