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

REGFILE REGFILE (
  .CLK(CLK),
  .nRST(RESET_N),
  .WEN1(reg_write_en),
  .WA1(reg_write_addr), 
  .DI1(reg_write_data), 
  .WEN2(1'b0),             // Only using one write port
  .WA2(4'b0),
  .DI2(32'b0),
  .RA0(rs1_addr),
  .RA1(rs2_addr),
  .RA2(4'b0),              // Only using two read ports
  .DOUT0(rs1_data),
  .DOUT1(rs2_data),
  .DOUT2()
);

// your code here

// Pipeline registers and control signals
// IF stage registers
reg [31:0] pc;
wire [31:0] pc_next;
reg [31:0] pc_plus4;

// ID stage registers
reg [31:0] id_pc;
reg [31:0] id_instr;

// EX stage registers
reg [31:0] ex_pc;
reg [31:0] ex_rs1_data;
reg [31:0] ex_rs2_data;
reg [31:0] ex_imm;
reg [4:0] ex_alu_control;
reg ex_reg_write;
reg ex_mem_read;
reg ex_mem_write;
reg [1:0] ex_mem_size;
reg [1:0] ex_wb_sel;
reg [3:0] ex_rd_addr;
reg ex_branch;
reg ex_jump;
reg ex_alu_src;

// MEM stage registers
reg [31:0] mem_alu_result;
reg [31:0] mem_rs2_data;
reg mem_reg_write;
reg mem_mem_read;
reg mem_mem_write;
reg [1:0] mem_mem_size;
reg [1:0] mem_wb_sel;
reg [3:0] mem_rd_addr;

// WB stage registers
reg [31:0] wb_alu_result;
reg [31:0] wb_mem_data;
reg [31:0] wb_pc_plus4;
reg wb_reg_write;
reg [1:0] wb_wb_sel;
reg [3:0] wb_rd_addr;

// Control signals
wire [3:0] rs1_addr;
wire [3:0] rs2_addr;
wire [3:0] rd_addr;
wire [31:0] rs1_data;
wire [31:0] rs2_data;
wire reg_write_en;
wire [3:0] reg_write_addr;
wire [31:0] reg_write_data;

// Immediate generation
wire [31:0] imm_i, imm_s, imm_b, imm_j, imm_u;
wire [31:0] imm_out;

// Hazard detection and forwarding
wire stall;
wire flush;
wire [1:0] forward_a, forward_b;

// Branch/Jump control
wire branch_taken;
wire [31:0] branch_target;
wire [31:0] jump_target;

// ALU signals
wire [31:0] alu_in1, alu_in2;
wire [31:0] alu_result;
wire alu_zero;

// Decode signals from instruction
wire [6:0] opcode;
wire [2:0] funct3;
wire [6:0] funct7;
wire [4:0] alu_control;
wire reg_write, mem_read, mem_write, branch, jump, alu_src;
wire [1:0] mem_size, wb_sel;

// Connect instruction memory interface
assign IREQ = RESET_N;  // Always request instruction when not in reset
assign IADDR = pc;

// Connect data memory interface
assign DREQ = mem_mem_read | mem_mem_write;
assign DADDR = mem_alu_result;
assign DRW = mem_mem_write;
assign DSIZE = mem_mem_size;
assign DOUT = mem_rs2_data;

// Extract instruction fields
assign opcode = id_instr[6:0];
assign rd_addr = id_instr[11:8];  // Assuming 4-bit register addresses for Cortex-M0
assign funct3 = id_instr[14:12];
assign rs1_addr = id_instr[19:16]; // Assuming 4-bit register addresses for Cortex-M0
assign rs2_addr = id_instr[24:21]; // Assuming 4-bit register addresses for Cortex-M0
assign funct7 = id_instr[31:25];

// Immediate generation
assign imm_i = {{20{id_instr[31]}}, id_instr[31:20]};
assign imm_s = {{20{id_instr[31]}}, id_instr[31:25], id_instr[11:7]};
assign imm_b = {{20{id_instr[31]}}, id_instr[7], id_instr[30:25], id_instr[11:8], 1'b0};
assign imm_j = {{12{id_instr[31]}}, id_instr[19:12], id_instr[20], id_instr[30:21], 1'b0};
assign imm_u = {id_instr[31:12], 12'b0};

// Control logic
always @(*) begin
    case (opcode)
        7'b0110011: begin // R-type (register-register)
            reg_write = 1'b1;
            mem_read = 1'b0;
            mem_write = 1'b0;
            branch = 1'b0;
            jump = 1'b0;
            alu_src = 1'b0;
            mem_size = 2'b10; // word
            wb_sel = 2'b01; // ALU result
            imm_out = 32'b0; // Not used for R-type
            
            case (funct3)
                3'b000: alu_control = (funct7[5]) ? 5'b00001 : 5'b00000; // SUB : ADD
                3'b001: alu_control = 5'b00010; // SLL
                3'b010: alu_control = 5'b00011; // SLT
                3'b011: alu_control = 5'b00100; // SLTU
                3'b100: alu_control = 5'b00101; // XOR
                3'b101: alu_control = (funct7[5]) ? 5'b00111 : 5'b00110; // SRA : SRL
                3'b110: alu_control = 5'b01000; // OR
                3'b111: alu_control = 5'b01001; // AND
                default: alu_control = 5'b00000;
            endcase
        end
        
        7'b0010011: begin // I-type (immediate)
            reg_write = 1'b1;
            mem_read = 1'b0;
            mem_write = 1'b0;
            branch = 1'b0;
            jump = 1'b0;
            alu_src = 1'b1;
            mem_size = 2'b10; // word
            wb_sel = 2'b01; // ALU result
            imm_out = imm_i;
            
            case (funct3)
                3'b000: alu_control = 5'b00000; // ADDI
                3'b001: alu_control = 5'b00010; // SLLI
                3'b010: alu_control = 5'b00011; // SLTI
                3'b011: alu_control = 5'b00100; // SLTUI
                3'b100: alu_control = 5'b00101; // XORI
                3'b101: alu_control = (funct7[5]) ? 5'b00111 : 5'b00110; // SRAI : SRLI
                3'b110: alu_control = 5'b01000; // ORI
                3'b111: alu_control = 5'b01001; // ANDI
                default: alu_control = 5'b00000;
            endcase
        end
        
        7'b0000011: begin // Load instructions
            reg_write = 1'b1;
            mem_read = 1'b1;
            mem_write = 1'b0;
            branch = 1'b0;
            jump = 1'b0;
            alu_src = 1'b1;
            wb_sel = 2'b00; // Memory load data
            imm_out = imm_i;
            alu_control = 5'b00000; // ADD for address calculation
            
            case (funct3)
                3'b000: mem_size = 2'b00; // LB
                3'b001: mem_size = 2'b01; // LH
                3'b010: mem_size = 2'b10; // LW
                3'b100: mem_size = 2'b00; // LBU
                3'b101: mem_size = 2'b01; // LHU
                default: mem_size = 2'b10;
            endcase
        end
        
        7'b0100011: begin // S-type (store)
            reg_write = 1'b0;
            mem_read = 1'b0;
            mem_write = 1'b1;
            branch = 1'b0;
            jump = 1'b0;
            alu_src = 1'b1;
            imm_out = imm_s;
            alu_control = 5'b00000; // ADD for address calculation
            wb_sel = 2'b00; // Not used
            
            case (funct3)
                3'b000: mem_size = 2'b00; // SB
                3'b001: mem_size = 2'b01; // SH
                3'b010: mem_size = 2'b10; // SW
                default: mem_size = 2'b10;
            endcase
        end
        
        7'b1100011: begin // B-type (branch)
            reg_write = 1'b0;
            mem_read = 1'b0;
            mem_write = 1'b0;
            branch = 1'b1;
            jump = 1'b0;
            alu_src = 1'b0;
            mem_size = 2'b10; // Not used
            wb_sel = 2'b00; // Not used
            imm_out = imm_b;
            
            case (funct3)
                3'b000: alu_control = 5'b01010; // BEQ
                3'b001: alu_control = 5'b01011; // BNE
                3'b100: alu_control = 5'b01100; // BLT
                3'b101: alu_control = 5'b01101; // BGE
                3'b110: alu_control = 5'b01110; // BLTU
                3'b111: alu_control = 5'b01111; // BGEU
                default: alu_control = 5'b00000;
            endcase
        end
        
        7'b1101111: begin // JAL
            reg_write = 1'b1;
            mem_read = 1'b0;
            mem_write = 1'b0;
            branch = 1'b0;
            jump = 1'b1;
            alu_src = 1'b1;
            mem_size = 2'b10; // Not used
            wb_sel = 2'b10; // PC+4
            imm_out = imm_j;
            alu_control = 5'b00000; // Not used directly
        end
        
        7'b1100111: begin // JALR
            reg_write = 1'b1;
            mem_read = 1'b0;
            mem_write = 1'b0;
            branch = 1'b0;
            jump = 1'b1;
            alu_src = 1'b1;
            mem_size = 2'b10; // Not used
            wb_sel = 2'b10; // PC+4
            imm_out = imm_i;
            alu_control = 5'b10000; // JALR operation
        end
        
        7'b0110111: begin // LUI
            reg_write = 1'b1;
            mem_read = 1'b0;
            mem_write = 1'b0;
            branch = 1'b0;
            jump = 1'b0;
            alu_src = 1'b1;
            mem_size = 2'b10; // Not used
            wb_sel = 2'b01; // ALU result
            imm_out = imm_u;
            alu_control = 5'b10001; // LUI operation (pass immediate)
        end
        
        7'b0010111: begin // AUIPC
            reg_write = 1'b1;
            mem_read = 1'b0;
            mem_write = 1'b0;
            branch = 1'b0;
            jump = 1'b0;
            alu_src = 1'b1;
            mem_size = 2'b10; // Not used
            wb_sel = 2'b01; // ALU result
            imm_out = imm_u;
            alu_control = 5'b10010; // AUIPC operation (PC + immediate)
        end
        
        default: begin // Default values for unknown opcodes
            reg_write = 1'b0;
            mem_read = 1'b0;
            mem_write = 1'b0;
            branch = 1'b0;
            jump = 1'b0;
            alu_src = 1'b0;
            mem_size = 2'b10;
            wb_sel = 2'b00;
            imm_out = 32'b0;
            alu_control = 5'b00000;
        end
    endcase
end

// ALU implementation
always @(*) begin
    case (ex_alu_control)
        5'b00000: alu_result = alu_in1 + alu_in2; // ADD
        5'b00001: alu_result = alu_in1 - alu_in2; // SUB
        5'b00010: alu_result = alu_in1 << alu_in2[4:0]; // SLL
        5'b00011: alu_result = ($signed(alu_in1) < $signed(alu_in2)) ? 32'b1 : 32'b0; // SLT
        5'b00100: alu_result = (alu_in1 < alu_in2) ? 32'b1 : 32'b0; // SLTU
        5'b00101: alu_result = alu_in1 ^ alu_in2; // XOR
        5'b00110: alu_result = alu_in1 >> alu_in2[4:0]; // SRL
        5'b00111: alu_result = $signed(alu_in1) >>> alu_in2[4:0]; // SRA
        5'b01000: alu_result = alu_in1 | alu_in2; // OR
        5'b01001: alu_result = alu_in1 & alu_in2; // AND
        5'b01010: alu_result = (alu_in1 == alu_in2) ? 32'b1 : 32'b0; // BEQ
        5'b01011: alu_result = (alu_in1 != alu_in2) ? 32'b1 : 32'b0; // BNE
        5'b01100: alu_result = ($signed(alu_in1) < $signed(alu_in2)) ? 32'b1 : 32'b0; // BLT
        5'b01101: alu_result = ($signed(alu_in1) >= $signed(alu_in2)) ? 32'b1 : 32'b0; // BGE
        5'b01110: alu_result = (alu_in1 < alu_in2) ? 32'b1 : 32'b0; // BLTU
        5'b01111: alu_result = (alu_in1 >= alu_in2) ? 32'b1 : 32'b0; // BGEU
        5'b10000: alu_result = (alu_in1 + alu_in2) & ~1; // JALR (clear LSB)
        5'b10001: alu_result = alu_in2; // LUI (pass immediate)
        5'b10010: alu_result = ex_pc + alu_in2; // AUIPC
        default: alu_result = 32'b0;
    endcase
end

assign alu_zero = (alu_result == 32'b0);

// Forwarding unit
always @(*) begin
    // Forward A - for RS1
    if (ex_reg_write && (mem_rd_addr != 4'b0) && (mem_rd_addr == rs1_addr))
        forward_a = 2'b10; // Forward from MEM stage
    else if (wb_reg_write && (wb_rd_addr != 4'b0) && (wb_rd_addr == rs1_addr))
        forward_a = 2'b01; // Forward from WB stage
    else
        forward_a = 2'b00; // No forwarding
    
    // Forward B - for RS2
    if (ex_reg_write && (mem_rd_addr != 4'b0) && (mem_rd_addr == rs2_addr))
        forward_b = 2'b10; // Forward from MEM stage
    else if (wb_reg_write && (wb_rd_addr != 4'b0) && (wb_rd_addr == rs2_addr))
        forward_b = 2'b01; // Forward from WB stage
    else
        forward_b = 2'b00; // No forwarding
end

// Hazard detection unit
always @(*) begin
    stall = 1'b0;
    
    // Load-use hazard
    if (ex_mem_read && ((ex_rd_addr == rs1_addr) || (ex_rd_addr == rs2_addr)))
        stall = 1'b1;
end

// Branch prediction (simple)
assign branch_taken = ex_branch && alu_zero;
assign branch_target = ex_pc + ex_imm;
assign jump_target = (ex_alu_control == 5'b10000) ? alu_result : (ex_pc + ex_imm);

// PC update logic
assign pc_next = (branch_taken || ex_jump) ? 
                   (ex_jump ? jump_target : branch_target) : 
                   (stall ? pc : pc + 4);

// Flush control
assign flush = branch_taken || ex_jump;

// ALU inputs with forwarding
always @(*) begin
    case (forward_a)
        2'b00: alu_in1 = ex_rs1_data;
        2'b01: alu_in1 = reg_write_data;
        2'b10: alu_in1 = mem_alu_result;
        default: alu_in1 = ex_rs1_data;
    endcase
    
    if (ex_alu_src)
        alu_in2 = ex_imm;
    else begin
        case (forward_b)
            2'b00: alu_in2 = ex_rs2_data;
            2'b01: alu_in2 = reg_write_data;
            2'b10: alu_in2 = mem_alu_result;
            default: alu_in2 = ex_rs2_data;
        endcase
    end
end

// Register write back
assign reg_write_en = wb_reg_write;
assign reg_write_addr = wb_rd_addr;

// Select write back data
always @(*) begin
    case (wb_wb_sel)
        2'b00: reg_write_data = wb_mem_data; // From memory
        2'b01: reg_write_data = wb_alu_result; // From ALU
        2'b10: reg_write_data = wb_pc_plus4; // PC+4 for JAL/JALR
        default: reg_write_data = wb_alu_result;
    endcase
end

// Pipeline registers
always @(posedge CLK or negedge RESET_N) begin
    if (~RESET_N) begin
        // Reset state
        pc <= 32'b0;
        
        // ID stage reset
        id_pc <= 32'b0;
        id_instr <= 32'b0;
        
        // EX stage reset
        ex_pc <= 32'b0;
        ex_rs1_data <= 32'b0;
        ex_rs2_data <= 32'b0;
        ex_imm <= 32'b0;
        ex_alu_control <= 5'b0;
        ex_reg_write <= 1'b0;
        ex_mem_read <= 1'b0;
        ex_mem_write <= 1'b0;
        ex_mem_size <= 2'b0;
        ex_wb_sel <= 2'b0;
        ex_rd_addr <= 4'b0;
        ex_branch <= 1'b0;
        ex_jump <= 1'b0;
        ex_alu_src <= 1'b0;
        
        // MEM stage reset
        mem_alu_result <= 32'b0;
        mem_rs2_data <= 32'b0;
        mem_reg_write <= 1'b0;
        mem_mem_read <= 1'b0;
        mem_mem_write <= 1'b0;
        mem_mem_size <= 2'b0;
        mem_wb_sel <= 2'b0;
        mem_rd_addr <= 4'b0;
        
        // WB stage reset
        wb_alu_result <= 32'b0;
        wb_mem_data <= 32'b0;
        wb_pc_plus4 <= 32'b0;
        wb_reg_write <= 1'b0;
        wb_wb_sel <= 2'b0;
        wb_rd_addr <= 4'b0;
    end
    else begin
        // PC update
        pc <= pc_next;
        pc_plus4 <= pc + 4;
        
        // IF/ID pipeline register
        if (!stall) begin
            id_pc <= pc;
            id_instr <= INSTR;
        end
        
        // ID/EX pipeline register
        if (flush) begin
            ex_reg_write <= 1'b0;
            ex_mem_read <= 1'b0;
            ex_mem_write <= 1'b0;
            ex_branch <= 1'b0;
            ex_jump <= 1'b0;
        end
        else begin
            ex_pc <= id_pc;
            ex_rs1_data <= rs1_data;
            ex_rs2_data <= rs2_data;
            ex_imm <= imm_out;
            ex_alu_control <= alu_control;
            ex_reg_write <= reg_write;
            ex_mem_read <= mem_read;
            ex_mem_write <= mem_write;
            ex_mem_size <= mem_size;
            ex_wb_sel <= wb_sel;
            ex_rd_addr <= rd_addr;
            ex_branch <= branch;
            ex_jump <= jump;
            ex_alu_src <= alu_src;
        end
        
        // EX/MEM pipeline register
        mem_alu_result <= alu_result;
        mem_rs2_data <= ex_rs2_data;
        mem_reg_write <= ex_reg_write;
        mem_mem_read <= ex_mem_read;
        mem_mem_write <= ex_mem_write;
        mem_mem_size <= ex_mem_size;
        mem_wb_sel <= ex_wb_sel;
        mem_rd_addr <= ex_rd_addr;
        
        // MEM/WB pipeline register
        wb_alu_result <= mem_alu_result;
        wb_mem_data <= DIN;
        wb_pc_plus4 <= id_pc + 4;
        wb_reg_write <= mem_reg_write;
        wb_wb_sel <= mem_wb_sel;
        wb_rd_addr <= mem_rd_addr;
    end
end

endmodule

// your code here (for other modules)
