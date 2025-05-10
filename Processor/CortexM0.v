
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
  .WEN1(),
  .WA1(), 
  .DI1(), 
  .WEN2(),
  .WA2(),
  .DI2(),
  .RA0(),
  .RA1(),
  .RA2(),
  .DOUT0(),
  .DOUT1(),
  .DOUT2()
);

// your code here

endmodule

// your code here (for other modules)
