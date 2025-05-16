//`timescale 1ns/1ps

module TB ();

	parameter CLK_PER = 4;
	parameter NUM_CLK = 10000;


	// --------------------------------------------
	// Wires and Regs
	// --------------------------------------------
	reg				CLK;
	reg				RESET_N;

	wire			IREQ;
	wire	[31:0]	IADDR;
	wire	[31:0]	INSTR;

	wire			DREQ;
	wire	[31:0]	DADDR;
	wire			DWE;
	wire	[1:0]	DSIZE;
	wire	[31:0]	DIN;
	wire	[31:0]	DOUT;

	reg		[3:0]	DBE;

	// --------------------------------------------
	// Reg for ALU_TEST
	// --------------------------------------------
	reg		[15:0]	ALU_CLK;
	reg 	[31:0]	ALU_TEST_MEM [0:145];
	
		// Add these declarations at the beginning of the module
	integer file_in, file_out, r;
	reg [8*256-1:0] line; // Buffer for reading lines, 256 characters max


	always #(CLK_PER/2) CLK = ~CLK;
	always #(CLK_PER) ALU_CLK = ALU_CLK+1;

	initial begin
		CLK = 1'b0;
		RESET_N = 1'b0;
		ALU_CLK = 1'b0;

		#(CLK_PER/4);
		
		#(CLK_PER*4);
			RESET_N = 1;
	end

	
	CortexM0 CortexM0 (
		.CLK(CLK),
		.RESET_N(RESET_N),
		
		// For instruction memory
		.IREQ(IREQ),
		.IADDR(IADDR),
		.INSTR(INSTR),

		// For data memory
		.DREQ(DREQ),
		.DADDR(DADDR),
		.DRW(DWE),		// read/write
		.DSIZE(DSIZE),	// Data memory access size 
		.DIN(DIN),
		.DOUT(DOUT)
	);

	SRAM MEM (
		.CLK (CLK),
		.CSN1 (1'b0),			// always chip select
		.ADDR1 (IADDR[13:2]),
		.WE1 (1'b0),			// only read operation
		.BE1 (4'b1111),			// word access
		.DI1 (),				// not used
		.DO1 (INSTR),			// read data

		.CSN2 (~DREQ),
		.ADDR2 (DADDR[13:2]),
		.WE2 (DWE),
		.BE2 (DBE),
		.DI2 (DOUT),
		.DO2 (DIN)
	);

	always @* begin
		casex( {DSIZE, DADDR[1:0]} )
			{2'b00, 2'b00}	:	DBE = 4'b0001;
			{2'b00, 2'b01}	:	DBE = 4'b0010;
			{2'b00, 2'b10}	:	DBE = 4'b0100;
			{2'b00, 2'b11}	:	DBE = 4'b1000;
			{2'b01, 2'b00}	:	DBE = 4'b0011;
			{2'b01, 2'b10}	:	DBE = 4'b1100;
			{2'b10, 2'b00}	:	DBE = 4'b1111;
		endcase
	
	// --------------------------------------------
	// for ALU_TEST (can be ignored)
	// --------------------------------------------
		if (ALU_CLK == 16'd9500) begin
			$display("Reading ALU_TEST_data.hex at clock=%d", ALU_CLK);
  
			// Save a copy of ALU_TEST_data.hex before reading it

			
			// file_in = $fopen("ALU_TEST_data.hex", "r");
			// file_out = $fopen("ALU_TEST_data_backup_9500.hex", "w");
			
			// if (file_in && file_out) begin
			// 	while (!$feof(file_in)) begin
			// 	r = $fgets(line, file_in);
			// 	if (r) $fwrite(file_out, "%s", line);
			// 	end
			// 	$fclose(file_in);
			// 	$fclose(file_out);
			// 	$display("Created backup of ALU_TEST_data.hex at clock 9500");
			// end else begin
			// 	$display("Error: Could not create backup of ALU_TEST_data.hex");
			// end


			$readmemh("ALU_TEST_data.hex", ALU_TEST_MEM); // This is from the variable ram in MemModel.v
			// Print some of the loaded values to verify
			$display("Memory[0]=%h", ALU_TEST_MEM[0]);
			$display("Memory[1]=%h", ALU_TEST_MEM[1]); 
			$display("Memory[2]=%h", ALU_TEST_MEM[2]);
			$display("Memory[3]=%h", ALU_TEST_MEM[3]);
			$display("Memory[4]=%h", ALU_TEST_MEM[4]);
			$display("Memory[5]=%h", ALU_TEST_MEM[5]);
			$display("Memory[6]=%h", ALU_TEST_MEM[6]);
			$display("Memory[7]=%h", ALU_TEST_MEM[7]);
			$display("Memory[8]=%h", ALU_TEST_MEM[8]);
			$display("Memory[9]=%h", ALU_TEST_MEM[9]);
			$display("Memory[10]=%h", ALU_TEST_MEM[10]);
			$display("Memory[11]=%h", ALU_TEST_MEM[11]);
			$display("Memory[12]=%h", ALU_TEST_MEM[12]);
			$display("Memory[13]=%h", ALU_TEST_MEM[13]);
			$display("Memory[14]=%h", ALU_TEST_MEM[14]);
			$display("Memory[15]=%h", ALU_TEST_MEM[15]);
			$display("Memory[16]=%h", ALU_TEST_MEM[16]);
			$display("Memory[17]=%h", ALU_TEST_MEM[17]);
			$display("Memory[18]=%h", ALU_TEST_MEM[18]);
			$display("Memory[19]=%h", ALU_TEST_MEM[19]);
			$display("Memory[20]=%h", ALU_TEST_MEM[20]);
			$display("Memory[21]=%h", ALU_TEST_MEM[21]);
			$display("Memory[22]=%h", ALU_TEST_MEM[22]);
			$display("Memory[23]=%h", ALU_TEST_MEM[23]);
			$display("Memory[24]=%h", ALU_TEST_MEM[24]);
			$display("Memory[25]=%h", ALU_TEST_MEM[25]);
			$display("Memory[26]=%h", ALU_TEST_MEM[26]);
			$display("Memory[27]=%h", ALU_TEST_MEM[27]);
			$display("Memory[28]=%h", ALU_TEST_MEM[28]);
			$display("Memory[29]=%h", ALU_TEST_MEM[29]);
			$display("Memory[30]=%h", ALU_TEST_MEM[30]);
			$display("Memory[31]=%h", ALU_TEST_MEM[31]);
			$display("Memory[32]=%h", ALU_TEST_MEM[32]);
			$display("Memory[33]=%h", ALU_TEST_MEM[33]);
			$display("Memory[34]=%h", ALU_TEST_MEM[34]);
			$display("Memory[35]=%h", ALU_TEST_MEM[35]);
			$display("Memory[36]=%h", ALU_TEST_MEM[36]);
			$display("Memory[37]=%h", ALU_TEST_MEM[37]);
			$display("Memory[38]=%h", ALU_TEST_MEM[38]);
			$display("Memory[39]=%h", ALU_TEST_MEM[39]);
			$display("Memory[40]=%h", ALU_TEST_MEM[40]);
			$display("Memory[41]=%h", ALU_TEST_MEM[41]);
			$display("Memory[42]=%h", ALU_TEST_MEM[42]);
			$display("Memory[43]=%h", ALU_TEST_MEM[43]);
			$display("Memory[44]=%h", ALU_TEST_MEM[44]);
			$display("Memory[45]=%h", ALU_TEST_MEM[45]);
			$display("Memory[46]=%h", ALU_TEST_MEM[46]);
			$display("Memory[47]=%h", ALU_TEST_MEM[47]);
			$display("Memory[48]=%h", ALU_TEST_MEM[48]);
			$display("Memory[49]=%h", ALU_TEST_MEM[49]);
			$display("Memory[50]=%h", ALU_TEST_MEM[50]);
			$display("Memory[51]=%h", ALU_TEST_MEM[51]);
			$display("Memory[52]=%h", ALU_TEST_MEM[52]);
			$display("Memory[53]=%h", ALU_TEST_MEM[53]);
			$display("Memory[54]=%h", ALU_TEST_MEM[54]);
			$display("Memory[55]=%h", ALU_TEST_MEM[55]);
			$display("Memory[56]=%h", ALU_TEST_MEM[56]);
			$display("Memory[57]=%h", ALU_TEST_MEM[57]);
			$display("Memory[58]=%h", ALU_TEST_MEM[58]);
			$display("Memory[59]=%h", ALU_TEST_MEM[59]);
			$display("Memory[60]=%h", ALU_TEST_MEM[60]);
			$display("Memory[61]=%h", ALU_TEST_MEM[61]);
			$display("Memory[62]=%h", ALU_TEST_MEM[62]);
			$display("Memory[63]=%h", ALU_TEST_MEM[63]);
			$display("Memory[64]=%h", ALU_TEST_MEM[64]);
			$display("Memory[65]=%h", ALU_TEST_MEM[65]);
			$display("Memory[66]=%h", ALU_TEST_MEM[66]);
			$display("Memory[67]=%h", ALU_TEST_MEM[67]);
			$display("Memory[68]=%h", ALU_TEST_MEM[68]);
			$display("Memory[69]=%h", ALU_TEST_MEM[69]);
			$display("Memory[70]=%h", ALU_TEST_MEM[70]);
			$display("Memory[71]=%h", ALU_TEST_MEM[71]);
			$display("Memory[72]=%h", ALU_TEST_MEM[72]);
			$display("Memory[73]=%h", ALU_TEST_MEM[73]);
			$display("Memory[74]=%h", ALU_TEST_MEM[74]);
			$display("Memory[75]=%h", ALU_TEST_MEM[75]);
			$display("Memory[76]=%h", ALU_TEST_MEM[76]);
			$display("Memory[77]=%h", ALU_TEST_MEM[77]);
			$display("Memory[78]=%h", ALU_TEST_MEM[78]);
			$display("Memory[79]=%h", ALU_TEST_MEM[79]);
			$display("Memory[80]=%h", ALU_TEST_MEM[80]);
			$display("Memory[81]=%h", ALU_TEST_MEM[81]);
			$display("Memory[82]=%h", ALU_TEST_MEM[82]);
			$display("Memory[83]=%h", ALU_TEST_MEM[83]);
			$display("Memory[84]=%h", ALU_TEST_MEM[84]);
			$display("Memory[85]=%h", ALU_TEST_MEM[85]);
			$display("Memory[86]=%h", ALU_TEST_MEM[86]);
			$display("Memory[87]=%h", ALU_TEST_MEM[87]);
			$display("Memory[88]=%h", ALU_TEST_MEM[88]);
			$display("Memory[89]=%h", ALU_TEST_MEM[89]);
			$display("Memory[90]=%h", ALU_TEST_MEM[90]);
			$display("Memory[91]=%h", ALU_TEST_MEM[91]);
			$display("Memory[92]=%h", ALU_TEST_MEM[92]);
			$display("Memory[93]=%h", ALU_TEST_MEM[93]);
			$display("Memory[94]=%h", ALU_TEST_MEM[94]);
			$display("Memory[95]=%h", ALU_TEST_MEM[95]);
			$display("Memory[96]=%h", ALU_TEST_MEM[96]);
			$display("Memory[97]=%h", ALU_TEST_MEM[97]);
			$display("Memory[98]=%h", ALU_TEST_MEM[98]);
			$display("Memory[99]=%h", ALU_TEST_MEM[99]);
			$display("Memory[100]=%h", ALU_TEST_MEM[100]);
			$display("Memory[101]=%h", ALU_TEST_MEM[101]);
			$display("Memory[102]=%h", ALU_TEST_MEM[102]);
			$display("Memory[103]=%h", ALU_TEST_MEM[103]);
			$display("Memory[104]=%h", ALU_TEST_MEM[104]);
			$display("Memory[105]=%h", ALU_TEST_MEM[105]);
			$display("Memory[106]=%h", ALU_TEST_MEM[106]);
			$display("Memory[107]=%h", ALU_TEST_MEM[107]);
			$display("Memory[108]=%h", ALU_TEST_MEM[108]);
			$display("Memory[109]=%h", ALU_TEST_MEM[109]);
			$display("Memory[110]=%h", ALU_TEST_MEM[110]);
			$display("Memory[111]=%h", ALU_TEST_MEM[111]);
			$display("Memory[112]=%h", ALU_TEST_MEM[112]);
			$display("Memory[113]=%h", ALU_TEST_MEM[113]);
			$display("Memory[114]=%h", ALU_TEST_MEM[114]);
			$display("Memory[115]=%h", ALU_TEST_MEM[115]);
			$display("Memory[116]=%h", ALU_TEST_MEM[116]);
			$display("Memory[117]=%h", ALU_TEST_MEM[117]);
			$display("Memory[118]=%h", ALU_TEST_MEM[118]);
			$display("Memory[119]=%h", ALU_TEST_MEM[119]);
			$display("Memory[120]=%h", ALU_TEST_MEM[120]);
			$display("Memory[121]=%h", ALU_TEST_MEM[121]);
			$display("Memory[122]=%h", ALU_TEST_MEM[122]);
			$display("Memory[123]=%h", ALU_TEST_MEM[123]);
			$display("Memory[124]=%h", ALU_TEST_MEM[124]);
			$display("Memory[125]=%h", ALU_TEST_MEM[125]);
			$display("Memory[126]=%h", ALU_TEST_MEM[126]);
			$display("Memory[127]=%h", ALU_TEST_MEM[127]);
			$display("Memory[128]=%h", ALU_TEST_MEM[128]);
			$display("Memory[129]=%h", ALU_TEST_MEM[129]);
			$display("Memory[130]=%h", ALU_TEST_MEM[130]);
			$display("Memory[131]=%h", ALU_TEST_MEM[131]);
			$display("Memory[132]=%h", ALU_TEST_MEM[132]);
			$display("Memory[133]=%h", ALU_TEST_MEM[133]);
			$display("Memory[134]=%h", ALU_TEST_MEM[134]);
			$display("Memory[135]=%h", ALU_TEST_MEM[135]);
			$display("Memory[136]=%h", ALU_TEST_MEM[136]);
			$display("Memory[137]=%h", ALU_TEST_MEM[137]);
			$display("Memory[138]=%h", ALU_TEST_MEM[138]);
			$display("Memory[139]=%h", ALU_TEST_MEM[139]);
			$display("Memory[140]=%h", ALU_TEST_MEM[140]);
			$display("Memory[141]=%h", ALU_TEST_MEM[141]);
			$display("Memory[142]=%h", ALU_TEST_MEM[142]);
			$display("Memory[143]=%h", ALU_TEST_MEM[143]);
			$display("Memory[144]=%h", ALU_TEST_MEM[144]);
			$display("Memory[145]=%h", ALU_TEST_MEM[145]);

		end
		if (ALU_CLK == 16'd9550) begin
			// Print both expected and actual values
			$display("ADD_REG_test: got=%h, expected=%h", ALU_TEST_MEM[132], 32'h0000_0001);
			if (ALU_TEST_MEM[132] == 32'h0000_0001)
				$display("ADD_REG_test passed");
			else
				$display("ADD_REG_test failed");
			
			$display("ADD_IMM_test: got=%h, expected=%h", ALU_TEST_MEM[133], 32'h0000_0001);
			if (ALU_TEST_MEM[133] == 32'h0000_0001)
				$display("ADD_IMM_test passed");
			else
				$display("ADD_IMM_test failed");

			$display("SUB_REG_test: got=%h, expected=%h", ALU_TEST_MEM[134], 32'hfffffffd);
			if (ALU_TEST_MEM[134] == 32'hfffffffd)
				$display("SUB_REG_test passed");
			else
				$display("SUB_REG_test failed");

			$display("SUB_IMM_test: got=%h, expected=%h", ALU_TEST_MEM[135], 32'hfffffffd);
			if (ALU_TEST_MEM[135] == 32'hfffffffd)
				$display("SUB_IMM_test passed");
			else
				$display("SUB_IMM_test failed");

			$display("MUL_test: got=%h, expected=%h", ALU_TEST_MEM[136], 32'hfffffffe);
			if (ALU_TEST_MEM[136] == 32'hfffffffe)
				$display("MUL_test passed");
			else
				$display("MUL_test failed");

			$display("LSL_IMM_test: got=%h, expected=%h", ALU_TEST_MEM[137], 32'hfffffffe);
			if (ALU_TEST_MEM[137] == 32'hffffffff)
				$display("LSL_IMM_test passed");
			else
				$display("LSL_IMM_test failed");

			$display("LSL_REG_test: got=%h, expected=%h", ALU_TEST_MEM[138], 32'hffffffff);
			if (ALU_TEST_MEM[138] == 32'hffffffff)
				$display("LSL_REG_test passed");
			else
				$display("LSL_REG_test failed");


			$display("LSR_REG_test: got=%h, expected=%h", ALU_TEST_MEM[139], 32'hfffffffc);
			if (ALU_TEST_MEM[139] == 32'hfffffffc)
				$display("LSR_REG_test passed");
			else
				$display("LSR_REG_test failed");

			$display("LSR_IMM_test: got=%h, expected=%h", ALU_TEST_MEM[140], 32'hfffffffc);
			if (ALU_TEST_MEM[140] == 32'hfffffffc)
				$display("LSR_IMM_test passed");
			else
				$display("LSR_IMM_test failed");

			$display("AND_test: got=%h, expected=%h", ALU_TEST_MEM[141], 32'h00000002);
			if (ALU_TEST_MEM[141] == 32'h00000002)
				$display("AND_test passed");
			else
				$display("AND_test failed");

			$display("XOR_test: got=%h, expected=%h", ALU_TEST_MEM[142], 32'hfffffffd);
			if (ALU_TEST_MEM[142] == 32'hfffffffd)
				$display("XOR_test passed");
			else
				$display("XOR_test failed");

			$display("OR_test: got=%h, expected=%h", ALU_TEST_MEM[143], 32'hffffffff);
			if (ALU_TEST_MEM[143] == 32'hffffffff)
				$display("OR_test passed");
			else
				$display("OR_test failed");

			$display("NOT_test: got=%h, expected=%h", ALU_TEST_MEM[144], 32'hfffffffd);
			if (ALU_TEST_MEM[144] == 32'hfffffffd)
				$display("NOT_test passed");
			else
				$display("NOT_test failed");

			$display("MOVA_test: got=%h, expected=%h", ALU_TEST_MEM[145], 32'h0000_0001);
			if (ALU_TEST_MEM[145] == 32'h0000_0001)
				$display("MOVA_test passed");
			else
				$display("NOT_test failed");
		end

		// // Add a continuous monitoring to see memory changes during execution
		// always @(posedge CLK) begin
		// 	if (ALU_CLK > 9000 && ALU_CLK < 9600) begin
		// 		// Print relevant memory values periodically
		// 		if (ALU_CLK % 100 == 0) begin
		// 			$display("Clock=%d, mem[132]=%h, mem[133]=%h", 
		// 					ALU_CLK, ALU_TEST_MEM[132], ALU_TEST_MEM[133]);
		// 		end
		// 	end
		// end

		// // Add debug to watch for memory write activity
		// if (DREQ && DWE) begin
		// 	$display("Memory write at clock=%d: addr=%h, data=%h, size=%b", 
		// 			 ALU_CLK, DADDR, DOUT, DSIZE);
		// end
	// --------------------------------------------
	end

	// --------------------------------------------
	// Load test vector to inst and data memory
	// --------------------------------------------
	// Caution : Assumption : input file has hex data like below. 
	//			 input file : M[0x03]M[0x02]M[0x01]M[0x00]
	//                        M[0x07]M[0x06]M[0x05]M[0x04]
	//									... 
	//           If the first 4 bytes in input file is 1234_5678
	//           then, the loaded value is mem[0x0000] = 0x1234_5678 (LSB)

	// // ROMDATA = initial memory data
	// defparam TB.MEM.ROMDATA = "ALU_TEST_inst.hex";
	// // MEMDATA = additional file for ALU_TEST (can be ignored)
  	// defparam TB.MEM.MEMDATA = "ALU_TEST_data.hex";


	// --------------------------------------------
	// For Dump variables
	// --------------------------------------------

	initial begin
		$dumpfile("myfile.dmp");
		$dumpvars;
	end

	initial begin
		#(CLK_PER * NUM_CLK); $finish;
	end


endmodule

