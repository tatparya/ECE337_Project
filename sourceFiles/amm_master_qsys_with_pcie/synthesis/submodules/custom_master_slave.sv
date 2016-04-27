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

//	------------	Address parameters	------------

parameter SDRAM_ADDR = 32'h08000000;

//	VGA base address
parameter VGA_START_ADDR 		= 	32'h08000000;
//	VGA end address
parameter VGA_END_ADDR 			= 	32'h0812C000;
// Last write address
parameter LAST_WRITE_ADDR 	=  32'h0812B0F1;


//	Start byte address
parameter START_BYTE_ADDR 		= 	32'h00000000;
//	Start byte
parameter START_BYTE 			= 	32'h00000053;
//	Stop byte address
parameter STOP_BYTE_ADDR 		= 	32'h00000000;
//	Stop byte
parameter STOP_BYTE 			= 	32'h00000012;
//	Copy Byte
parameter COPY_BYTE 			= 	32'h00000017;
//	Next Byte
parameter NEXT_BYTE 			= 	32'h00000013;
//	Last Pixel Byte
parameter LASTPIX_BYTE 			= 	32'h00000019;

//	Input image base address
parameter INP_IMG_START_ADDR 	= 	32'h09000000;
//	Input image end address
parameter INP_IMG_END_ADDR 		= 	32'h0912C000;

parameter IMG_SIZE				=	32'h4A196;

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
logic [9:0] columnCount;
logic [9:0] nextColumnCount;

logic [9:0] slantCount;
logic [9:0] nextslantCount;

logic [31:0] rPC;
logic [31:0] nRPC;
logic [31:0] wPC;
logic [31:0] nWPC;

logic readDone;
logic w_image_done;
logic w_pixel_written;

//	----------	State Definitions	------------

typedef enum {	
				IDLE, 				//	0000
				CHECKSTARTBYTE,	//	0001
				CHECKSTATUS,		// 0010
				COPYPIXEL,			// 0011
				ASSERTNEXTBYTE,	// 0100
				STARTPROCESS,  	// 0101

				CHECKENABLE,		//	0110: Check for read / write enable
				READPIXEL,			//	0111: Read a pixel and pass to custom logic
				READDONE,			// 	1000: 
				READDONE2,			// 	1001: 
				WRITEPIXEL,			//	1010: Write pixel from custom logic
				WRITEDONE,			// 	1011
				WRITEDONE2,			

				READCOLOR,			// 	1100
				WRITECOLOR,			// 	1101
				IMAGEDONE,			//	1110
				PREIMAGEDONE
				//GETCOLOR,			//	
				//VGAPAINT,			//	
				//VGAPAINTNORMAL	//	
			} state_t;
state_t state, nextState;

//	Sobel inputs
logic NReset;
logic nextNReset;
logic [ 31:0 ] startSignal_i;
logic [ 31:0 ] next_startSignal_i;
logic [ 31:0 ] inputData_i;
logic [ 31:0 ] next_inputData_i;
logic [ 31:0 ] rwError_i;
logic [ 31:0 ] next_rwError_i;
logic [ 31:0 ] readDone_i;
logic [ 31:0 ] next_readDone_i;
logic [ 31:0 ] writeDone_i;
logic [ 31:0 ] next_writeDone_i;

//	Sobel Outputs
logic [ 31:0 ] readAddress_i;
logic [ 31:0 ] writeAddress_i;
logic [ 31:0 ] writeAddress_i_calculated_by_sid;
logic [ 31:0 ] outputData_i;
logic [ 31:0 ] readEnable_i;
logic [ 31:0 ] writeEnable_i;
logic [ 31:0 ] outputError_i;

logic [ 31:0 ] numProcessedPixels;
logic [ 31:0 ] nextNumProcessedPixels;

//custom_data_writer customWriter (.r(rdwr_address[2]), .g(rdwr_address[1]), .b(rdwr_address[0]), .data(writeColor));

edgeDetection2 edgeDetectModule( 	.clk			( clk ),
									.n_rst			( NReset ),
									.startSignal	( startSignal_i ),
									.inputData		( inputData_i ),
									.rwError		( rwError_i ),
									.readDone		( readDone_i ),
									.writeDone		( writeDone_i ),

									.readAddress	( readAddress_i ),
									.writeAddress	( writeAddress_i ),
									.outputData		( outputData_i ),
									.readEnable		( readEnable_i ),
									.writeEnable	( writeEnable_i ),
									.outputError	( outputError_i )
									);

//	7 seg display
always_comb
begin

	if( rdwr_address[0] && rdwr_address[1] )
	begin
		display_data = { readEnable_i[15:0], writeEnable_i[15:0] };
	end
	else if( rdwr_address[0] && rdwr_address[2] )
	begin
		display_data = outputData_i;
	end

	if( rdwr_address[0] && rdwr_address[3] )
	begin
		display_data = rPC;
	end
	else if( rdwr_address[0] && rdwr_address[4] )
	begin
		display_data = wPC;
	end


	else if( rdwr_address[0] )
	begin
		display_data = 
			{
				1'b0,1'b0,1'b0,w_image_done,
				1'b0,1'b0,1'b0,rdwr_cntl,
				1'b0,1'b0,1'b0,rdwr_address[15],
				1'b0,1'b0,1'b0,rdwr_address[14],
				1'b0,1'b0,1'b0,state[3],
				1'b0,1'b0,1'b0,state[2],
				1'b0,1'b0,1'b0,state[1],
				1'b0,1'b0,1'b0,state[0]
			};
	end
	else if( rdwr_address[1] )
	begin
		display_data = csr_registers[0];
	end
	else if( rdwr_address[2] )
	begin
		display_data = csr_registers[1];
	end
	else if( rdwr_address[3] )
	begin
		display_data = csr_registers[2];
	end
	else if( rdwr_address[4] )
	begin
		display_data = readColor;
	end
	else if( rdwr_address[5] )
	begin
		display_data = readAddress;
	end
	else if( rdwr_address[6] )
	begin
		display_data = writeAddress;
	end

	else
	begin
		display_data = address;
	end
end

// Slave side 
always_ff @ ( posedge clk ) 
begin 
	if(!reset_n)
	begin
		slave_readdata <= 32'h0;
		csr_registers <= '0;
	end
	else 
	begin
		if(slave_write && slave_chipselect && (slave_address >= 0) && (slave_address < NUMREGS))
		begin
			csr_registers[slave_address] <= slave_writedata;  // Write a value to a CSR register
		end
		else if(slave_read && slave_chipselect  && (slave_address >= 0) && (slave_address < NUMREGS)) // reading a CSR Register
		begin
			// Send a value being requested by a master. 
			// If the computation is small you may compute directly and send it out to the master.
			slave_readdata <= csr_registers[slave_address];
		end
		else if(w_pixel_written)
		begin
			csr_registers[1]	<= NEXT_BYTE;
		end
		else if (w_image_done) // Whenever image_done flag is asserted, write STOPBYTE to signal Atom/C code.
		begin
			csr_registers[0]	<= STOP_BYTE;
		end
	end
end

//	Custom State Registers Logic
always_ff @ ( posedge clk ) begin 
	if (!reset_n) 
	begin
		startSignal_i	<=	32'd0;
		inputData_i		<=	32'd0;
		rwError_i		<=	32'd0;
		readDone_i		<=	32'd0;
		writeDone_i		<=	32'd0;
		numProcessedPixels <= 32'd0;
		slantCount		<=	0;
	end else 
	begin
		startSignal_i		<=	next_startSignal_i;
		inputData_i			<=	next_inputData_i;
		rwError_i			<=	next_rwError_i;
		readDone_i			<=	next_readDone_i;
		writeDone_i			<=	next_writeDone_i;
		numProcessedPixels 	<= 	nextNumProcessedPixels;
		nextslantCount		<=	nextslantCount;
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
		columnCount 	<= 0;
		NReset 			<= 0;
		rPC				<= 0;
		wPC				<= 0;

		end else begin 
		state 			<= nextState;
		address 			<= nextAddress;
		readColor 		<= nextReadColor;
		readAddress 	<= nextReadAddress;
		writeAddress 	<= nextWriteAddress;
		columnCount		<= nextColumnCount;
		NReset 		   <= nextNReset;
		
		rPC				<= nRPC;
		wPC				<= nWPC;
		
	end
end

// Next State Logic 
always_comb begin 
	nextState 			=	state;
	nextAddress 		= 	address;
	nextReadColor 		= 	readColor;
	nextReadAddress 	= 	readAddress;
	nextWriteAddress 	= 	writeAddress;

	nextNumProcessedPixels = numProcessedPixels;
	nextslantCount 		= slantCount;

	next_startSignal_i	= 	startSignal_i;
	next_inputData_i	= 	inputData_i;
	next_rwError_i		= 	rwError_i;
	next_readDone_i		= 	readDone_i;
	next_writeDone_i	= 	writeDone_i;
	nextNReset = NReset;
	nextColumnCount = columnCount;

	nRPC = rPC;
	nWPC = wPC;
	case( state ) 
		//	IDLE State
		IDLE : 
		begin 
			if ( rdwr_cntl ) 
			begin 
				nextState = CHECKSTARTBYTE;
				nextNReset = 0;
			end 
			else 
			begin
				nextState = IDLE;
			end
		end


		//	Check start byte to start image processing
		CHECKSTARTBYTE : 
		begin 
			//	RESET
			if( rdwr_address[9] )
			begin
				nextState = IDLE;
			end
			//	Check start byte
			if( ( csr_registers[0] == START_BYTE ) && add_data_sel )
			begin
				nextState = 	CHECKSTATUS;
				nextAddress = 	INP_IMG_START_ADDR;
				nextNReset = 1;
			end
		end 

		//	Check status on slave status register
		CHECKSTATUS:
		begin
			if( rdwr_address[15] )
			begin
				//	Copy next Pixel
				if( csr_registers[1] == COPY_BYTE )
				begin
					nextState = COPYPIXEL;
				end
				//	Copying done
				else if( csr_registers[1] == LASTPIX_BYTE )
				begin
					nextReadAddress = INP_IMG_START_ADDR;
					nextWriteAddress = VGA_START_ADDR;
					nextState = STARTPROCESS;
				end
				else
				begin
					nextState = CHECKSTATUS;
				end
			end
		end 

		//	Get data from slave register
		COPYPIXEL:
		begin 
			if( rdwr_address[14] )
			begin
				if( !master_waitrequest )
				begin
					nextState = ASSERTNEXTBYTE;
					nextAddress = address + 4;
				end
			end
		end 

		//	Write NEXT status to slave register
		ASSERTNEXTBYTE:
		begin
			nextState = CHECKSTATUS;
		end

		//	Start processing image
		STARTPROCESS:
		begin
			if(rdwr_address[13])
			begin
				if(!rdwr_address[1])	nextState = CHECKENABLE;
				else nextState = READCOLOR;
				nextReadAddress = INP_IMG_START_ADDR;
				nextWriteAddress = VGA_START_ADDR;
				
				//	Custom logic stuff
				next_startSignal_i	=	32'd1;
				next_inputData_i	=	32'd0;
				next_rwError_i		=	32'd0;
				next_readDone_i		=	32'd0;
				next_writeDone_i	=	32'd0;

				nextNumProcessedPixels = 32'd0;
				
				nextWriteAddress = VGA_START_ADDR;
				nextColumnCount  = 0;
				
				nRPC = 32'd0;
				nWPC = 32'd0;
			end
		end

		CHECKENABLE	:
		begin
			if(rdwr_address[12])
			begin

				if(writeAddress >= LAST_WRITE_ADDR || nextNumProcessedPixels > IMG_SIZE)	//	Exit condition
				begin
					//nextReadAddress = INP_IMG_START_ADDR;
					//nextWriteAddress = VGA_START_ADDR;
					nextState = PREIMAGEDONE;
				end

				else if( writeEnable_i == 1 )	//	Writing the pixel from custom logic
				begin
					//	Write pixel data to VGA output
					nextState = WRITEPIXEL;
					next_writeDone_i = 0;
					nextWriteAddress = writeAddress;
				end

				else if( readEnable_i == 1 )	//	Read a pixel to custom logic
				begin
					//	Pass data into custom logic
					nextState = READPIXEL;
					next_readDone_i = 0;
					nextReadAddress = INP_IMG_START_ADDR + (readAddress_i);
				end

				else
				begin
					nextState = CHECKENABLE;
				end
			end
		end

		READPIXEL	:
		begin

		if(rdwr_address[11])
		begin
			if( master_readdatavalid )
				begin
					nextState = READDONE;
					next_inputData_i = master_readdata;
					next_readDone_i = 1;
					nRPC = rPC + 1;
				end
			end
		end

		READDONE	:
		begin
			nextState = READDONE2;
			next_readDone_i = 1;
		end

		READDONE2	:
		begin
			nextState = CHECKENABLE;
			next_readDone_i = 0;
		end

		WRITEPIXEL	:
		begin
			if( !master_waitrequest )
			begin
				nextState = WRITEDONE;
				next_writeDone_i = 1;
				nWPC = wPC + 1;
			end
			else
			begin
				nextState = WRITEPIXEL;
			end
		end
		

		WRITEDONE	:
		begin
			if(rdwr_address[10])
			begin
				if(columnCount == 10'd637)
				begin
					nextColumnCount = 0;

					if( slantCount >= 10 )
						nextWriteAddress = writeAddress + 16;
					else
						nextWriteAddress = writeAddress + 12; // Accounting for padding

					nextslantCount = slantCount + 1;
				end
				else
				begin
					nextColumnCount = columnCount + 1;
					nextWriteAddress = writeAddress + 4; // Normal
				end
				next_writeDone_i = 1;
				nextState = WRITEDONE2;
			end
		end

		WRITEDONE2	:
		begin
			nextState = CHECKENABLE;
			nextNumProcessedPixels = numProcessedPixels + 1;
			next_writeDone_i = 0;
		end

		//	Read Color from SDRAM memory
		READCOLOR : 
		begin 
			if(rdwr_address[12])
			begin
				if( master_readdatavalid )
				begin
					if(nextReadAddress < INP_IMG_END_ADDR)
					begin
						nextState = WRITECOLOR;
						nextReadColor = master_readdata;
					end
					else
					begin
						nextState = PREIMAGEDONE;
					end
				end
			end
		end 

		//	Write color to display on VGA
		WRITECOLOR : 
		begin 
			if(rdwr_address[11])
			begin

				if( !master_waitrequest )
				begin
					nextState = READCOLOR;
					nextReadAddress = readAddress + 4;
					nextWriteAddress = writeAddress + 4;
					
				end
			end 
		end
		
		//	End process state
		PREIMAGEDONE :
		begin
			if( rdwr_address[9] )
			begin
				nextState = STARTPROCESS;
			end
			else if( rdwr_address[8] )
			begin
				nextState = IMAGEDONE;
			end
			else
			begin
				nextState = PREIMAGEDONE;
				//nextAddress = VGA_START_ADDR;
			end
		end
		IMAGEDONE :
		begin
			if( rdwr_address[7] )
			begin
				nextState = IDLE;
			end
			else
			begin
				nextState = IMAGEDONE;
				//nextAddress = VGA_START_ADDR;
			end
		end

		/*
		
		GETCOLOR:
		begin
			if( master_readdatavalid && rdwr_address[13])
			begin
				nextState = VGAPAINT;
				nextReadColor = master_readdata;
				nextAddress = VGA_START_ADDR;
			end
		end

		VGAPAINT: 
		begin
			if(rdwr_address[12])
			begin
				if (!master_waitrequest) 
				begin 
					if (address < VGA_END_ADDR )
					begin
						nextAddress =  address + 4;
						nextState = VGAPAINT;
					end
					else
					begin
						nextState = IMAGEDONE;
					end
				end
			end
		end
		VGAPAINTNORMAL: 
		begin
			if (!master_waitrequest) 
			begin 
				if (address < VGA_END_ADDR )
				begin
					nextAddress =  address + 4;
					nextState = VGAPAINT;
				end
				else
				begin
					nextState = IDLE;
				end
			end
		end

		*/

		//	Default state
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
	w_image_done = 1'b0;
	w_pixel_written = 1'b0;
	master_writedata = 32'h0;
	master_address = 32'h00000000;

	case(state) 		

		IDLE:
		begin
		end


		CHECKSTARTBYTE:
		begin
		end

		COPYPIXEL:
		begin
			master_write = 1;
			master_address = address;
			master_writedata = csr_registers[2];
		end

		ASSERTNEXTBYTE:
		begin
			w_pixel_written = 1;
		end

		STARTPROCESS:
		begin
		end

		CHECKENABLE	:
		begin
		end

		READPIXEL	:
		begin
			master_read = 1;
			master_address = readAddress;
		end

		READDONE	:
		begin
		end

		READDONE2	:
		begin
		end

		WRITEPIXEL	:
		begin
			master_write = 1;
			master_address = writeAddress;		
			master_writedata = ( outputData_i[7:0] > 100 ) ? 32'h00FFFFFF : 32'd0;
			// master_writedata = {8'b0, outputData_i[7:0], outputData_i[7:0], outputData_i[7:0]};
		end

		WRITEDONE	:
		begin
		end

		WRITEDONE2	:
		begin
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

		IMAGEDONE:
		begin
			w_image_done = 1;
		end


		/*
		GETCOLOR:
		begin
			master_read = 1;
			master_address = address;
		end

		VGAPAINT : 
		begin 
			master_write = 1;
			master_address =  address;
			master_writedata = readColor;
		end 
		VGAPAINTNORMAL : 
		begin 
			master_write = 1;
			master_address =  address;
			master_writedata = readColor;
		end 
		*/
	endcase

end
endmodule



