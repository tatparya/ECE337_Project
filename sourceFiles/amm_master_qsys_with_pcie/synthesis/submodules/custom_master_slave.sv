// custom_master_slave module : Acts as an avalon slave to receive input commands from PCIE IP

module custom_master_slave #(
	parameter MASTER_ADDRESSWIDTH = 26 ,  	// ADDRESSWIDTH specifies how many addresses the Master can address 
	parameter SLAVE_ADDRESSWIDTH = 3 ,  	// ADDRESSWIDTH specifies how many addresses the slave needs to be mapped to. log(NUMREGS)
	parameter DATAWIDTH = 32 ,    			// DATAWIDTH specifies the data width. Default 32 bits
	parameter NUMREGS = 8 ,       			// Number of Internal Registers for Custom Logic
	parameter REGWIDTH = 32       			// Data Width for the Internal Registers. Default 32 bits
)	
(	
	input logic  clk,
	input logic  reset_n,

	// Interface to Top Level
	input logic rdwr_cntl,			//SW[17]
	input logic n_action,			// Trigger the Read or WRITE. Additional control to avoid continuous transactions. Not a required signal. Can and should be removed for actual application.
	input logic add_data_sel,		//SW[16]
	input logic [MASTER_ADDRESSWIDTH-1:0] rdwr_address,//SW[15:0]
	output logic [DATAWIDTH-1:0] display_data,

	//17 enable
	//16 1=Write 0=read
	//15-0 unassigned

	// Bus Slave Interface
	input logic [SLAVE_ADDRESSWIDTH-1:0] slave_address,
	input logic [DATAWIDTH-1:0] slave_writedata,
	input logic  slave_write,
	input logic  slave_read,
	input logic  slave_chipselect,
	//      input logic  slave_readdatavalid, 			// These signals are for variable latency reads. 
	//	output logic slave_waitrequest,   			// See the Avalon Specifications for details  on how to use them.
	output logic [DATAWIDTH-1:0] slave_readdata,

	// Bus Master Interface
	output logic [MASTER_ADDRESSWIDTH-1:0] master_address,
	output logic [DATAWIDTH-1:0] master_writedata,
	output logic  master_write,
	output logic  master_read,
	input logic [DATAWIDTH-1:0] master_readdata,
	input logic  master_readdatavalid,
	input logic  master_waitrequest

);

//	----------	Address parameters	------------

parameter SDRAM_ADDR = 32'h08000000;

//	VGA base address
parameter VGA_START_ADDR = 32'h08000000;
//	VGA end address
parameter VGA_END_ADDR = 32'h0812C000;

//	Start byte address
parameter START_BYTE_ADDR = 32'h0812D000;
//	Start byte
parameter START_BYTE = 32'hF00BF00B;
//	Stop byte address
parameter STOP_BYTE_ADDR = 32'h0812E000;
//	Stop byte
parameter STOP_BYTE = 32'hDEADF00B;

//	Input image base address
parameter INP_IMG_START_ADDR = 32'h09000000;
//	Input image end address
parameter INP_IMG_END_ADDR = 32'h0912C000;

logic [MASTER_ADDRESSWIDTH-1:0] address, nextAddress;
logic [MASTER_ADDRESSWIDTH-1:0] writeAddress, nextWriteAddress;
logic [MASTER_ADDRESSWIDTH-1:0] readAddress, nextReadAddress;

logic [DATAWIDTH-1:0] nextRead_data, read_data;
logic [DATAWIDTH-1:0] nextData, wr_data;
logic [NUMREGS-1:0][REGWIDTH-1:0] csr_registers;  		// 	Command and Status Registers (CSR) for custom logic 
logic [NUMREGS-1:0] reg_index, nextRegIndex;
logic [NUMREGS-1:0][REGWIDTH-1:0] read_data_registers;  //	Store SDRAM read data for display
logic new_data_flag;
logic color;
logic [31:0]	writeColor;
logic [31:0]	readColor;
logic [31:0]	nextReadColor;
logic readDone;

//	----------	State Definitions	------------

typedef enum {	
				IDLE, 			//	000
				IMAGEDONE,		//	001
				CREATEIMAGE,	//	010
				WRITESTARTBYTE,//	011
				CHECKSTARTBYTE,//	100
				READCOLOR,		// 101
				WRITECOLOR		// 110
			} state_t;
state_t state, nextState;

custom_data_writer customWriter (.r(rdwr_address[2]), .g(rdwr_address[1]), .b(rdwr_address[0]), .data(writeColor));

assign display_data = 
	{
		8'hda,
		1'b0,1'b0,1'b0,rdwr_address[15],
		1'b0,1'b0,1'b0,rdwr_address[14],
		1'b0,1'b0,1'b0,rdwr_address[13],
		1'b0,1'b0,1'b0,state[2],
		1'b0,1'b0,1'b0,state[1],
		1'b0,1'b0,1'b0,state[0]
	};

// Slave side 
always_ff @ ( posedge clk ) begin 
  if(!reset_n)
  	begin
    		slave_readdata <= 32'h0;
 	      	csr_registers <= '0;
  	end
  else 
  	begin
  	  if(slave_write && slave_chipselect && (slave_address >= 0) && (slave_address < NUMREGS))
  	  	begin
  	  	   csr_registers[slave_address] <= slave_writedata;  // VGAPAINT a value to a CSR register
  	  	end
  	  else if(slave_read && slave_chipselect  && (slave_address >= 0) && (slave_address < NUMREGS)) // reading a CSR Register
  	    	begin
	       		// Send a value being requested by a master. 
	       		// If the computation is small you may compute directly and send it out to the master directly from here.
  	    	   slave_readdata <= csr_registers[slave_address];
  	    	end
  	 end
end

// Master Side 

//	State Register Logic
always_ff @ ( posedge clk ) begin 
	if (!reset_n) begin 
		address 		<= SDRAM_ADDR;
		state 			<= IDLE;
		readColor 		<= 32'h0;
		readAddress 	<= INP_IMG_START_ADDR;
		writeAddress 	<= VGA_START_ADDR;
	end else begin 
		state 			<= nextState;
		address 		<= nextAddress;
		readColor 		<= nextReadColor;
		readAddress 	<= nextReadAddress;
		writeAddress 	<= nextWriteAddress;
	end
end

// Next State Logic 
	// If user wants to input data and addresses using a state machine instead of signals/conditions,
	// the following code has commented lines for how this could be done.
always_comb begin 
	nextState 			=	state;
	nextAddress 		= 	address;
	nextReadColor 		= 	readColor;
	nextReadAddress 	= 	readAddress;
	nextWriteAddress 	= 	writeAddress;
	
	case( state ) 
		IDLE : 
		begin 
			if ( rdwr_address[15] ) 
			begin 
				nextState = CREATEIMAGE;
				nextAddress = INP_IMG_START_ADDR;
			end 
			else 
			begin
				nextState = IDLE;
			end
		end

		IMAGEDONE :
		begin
			if( rdwr_address[12] )
			begin
				nextState = IDLE;
			end
			else
			begin
				nextState = IMAGEDONE;
			end
		end

		CREATEIMAGE :
		begin
			if ( !master_waitrequest && rdwr_address[14] ) 
			begin 
				if (address < INP_IMG_END_ADDR )
				begin
					nextAddress =  address + 4;
					nextState = CREATEIMAGE;
				end
				else
				begin
					nextState = WRITESTARTBYTE;
				end
			end
		end

		WRITESTARTBYTE:
		begin
			if( !master_waitrequest && rdwr_address[13] )
			begin
				nextState = CHECKSTARTBYTE;
			end
		end

		CHECKSTARTBYTE : 
		begin 
			if( master_readdatavalid && master_readdata == START_BYTE )
			begin
				nextState = READCOLOR;
				nextReadAddress = INP_IMG_START_ADDR;
				nextWriteAddress = VGA_START_ADDR;
			end
		end 

		READCOLOR : 
		begin 
			if( master_readdatavalid )
			begin
				if(nextReadAddress < INP_IMG_END_ADDR)
				begin
					nextState = WRITECOLOR;
					nextReadAddress = readAddress + 4;
					nextReadColor = master_readdata;
				end
				else
				begin
					nextState = IMAGEDONE;
				end
			end
		end 

		WRITECOLOR : 
		begin 
			if( !master_waitrequest )
			begin
				nextState = READCOLOR;
				nextWriteAddress = writeAddress + 4;
			end
		end 

		default:
		begin	
			nextState 			=	state;
			nextAddress 		= 	address;
			nextReadColor 		= 	readColor;
			nextReadAddress 	= 	readAddress;
			nextWriteAddress 	= 	writeAddress;
		end 

	endcase
end

// Output Logic 
always_comb begin 
	master_write = 1'b0;
	master_read = 1'b0;
	master_writedata = 32'h0;
	master_address = 32'hbad1bad1;

	case(state) 		

		IDLE:
		begin
		end

		CHECKSTARTBYTE:
		begin
			master_read = 1;
			master_address = START_BYTE_ADDR;
		end

		READCOLOR:
		begin
			master_read = 1;
			master_address = readAddress;
		end

		WRITECOLOR:
		begin
			master_write = 1;
			master_address = writeAddress;
			master_writedata = readColor;
		end

		WRITESTARTBYTE:
		begin
			master_write = 1;
			master_address = START_BYTE_ADDR;
			master_writedata = START_BYTE;
		end

		CREATEIMAGE:
		begin
			master_write = 1;
			master_address = address;
			master_writedata = ( address < 32'h09096000) ? 32'h00FF0000 : 32'h0000FF00;
		end
		
	endcase

end

endmodule


