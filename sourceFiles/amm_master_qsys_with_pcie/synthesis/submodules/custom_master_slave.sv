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
logic w_image_done;
logic w_pixel_written;

//	----------	State Definitions	------------

typedef enum {	
				IDLE, 			//	0000
				CHECKSTARTBYTE,//	0001
				CHECKSTATUS,	// 0010
				COPYPIXEL,		// 0011
				ASSERTNEXTBYTE,// 0100
				STARTPROCESS,  // 0101
				READCOLOR,		// 0110
				WRITECOLOR,		// 0111
				IMAGEDONE		//	1000
				//GETCOLOR,		//	0101
				//VGAPAINT,			//	110
				//VGAPAINTNORMAL		//	110
			} state_t;
state_t state, nextState;

custom_data_writer customWriter (.r(rdwr_address[2]), .g(rdwr_address[1]), .b(rdwr_address[0]), .data(writeColor));

//	7 seg display
always_comb
begin
	if( rdwr_address[0] )
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

// Master Side 
logic pixelCount;
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
always_comb begin 
	nextState 			=	state;
	nextAddress 		= 	address;
	nextReadColor 		= 	readColor;
	nextReadAddress 	= 	readAddress;
	nextWriteAddress 	= 	writeAddress;
	
	case( state ) 
		//	IDLE State
		IDLE : 
		begin 
			if ( rdwr_cntl ) 
			begin 
				nextState = CHECKSTARTBYTE;
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
			if( rdwr_address[10] )
			begin
				nextState = IDLE;
			end
			//	Check start byte
			if( ( csr_registers[0] == START_BYTE ) && add_data_sel )
			begin
				nextState = CHECKSTATUS;
				nextAddress = INP_IMG_START_ADDR;
			end
		end 

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

		ASSERTNEXTBYTE:
		begin
			nextState = CHECKSTATUS;
		end


		STARTPROCESS:
		begin
			if(rdwr_address[13])
			begin
				nextState = READCOLOR;
				nextReadAddress = INP_IMG_START_ADDR;
				nextWriteAddress = VGA_START_ADDR;
			end
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
						nextState = IMAGEDONE;
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
		IMAGEDONE :
		begin
			if( rdwr_address[10] )
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
			pixelCount = 0;
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



