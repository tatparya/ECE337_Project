module custom_data_writer 
(	
	input logic  r,
	input logic  g,
	input logic  b, 
	output logic [31:0] data
);
assign data = {
				8'h00,
				r, r, r, r, r, r, r, r,
				g, g, g, g, g, g, g, g, 
				b, b, b, b, b, b, b, b
				};
endmodule


