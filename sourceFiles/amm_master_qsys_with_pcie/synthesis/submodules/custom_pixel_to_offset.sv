module custom_da 
(	
	input logic  [31:0] pixelCount,
	output logic [31:0] offset
);
assign offset = (pixelCount +1);
endmodule


