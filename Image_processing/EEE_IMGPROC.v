module EEE_IMGPROC(
	// global clock & reset
	clk,
	reset_n,
	
	// mm slave
	s_chipselect,
	s_read,
	s_write,
	s_readdata,
	s_writedata,
	s_address,

	// stream sink
	sink_data,
	sink_valid,
	sink_ready,
	sink_sop,
	sink_eop,
	
	// streaming source
	source_data,
	source_valid,
	source_ready,
	source_sop,
	source_eop,
	
	// conduit
	mode
	
);


// global clock & reset
input	clk;
input	reset_n;

// mm slave
input							s_chipselect;
input							s_read;
input							s_write;
output	reg	[31:0]	s_readdata;
input	[31:0]				s_writedata;
input	[2:0]					s_address;


// streaming sink
input	[23:0]            	sink_data;
input								sink_valid;
output							sink_ready;
input								sink_sop;
input								sink_eop;

// streaming source
output	[23:0]			  	   source_data;
output								source_valid;
input									source_ready;
output								source_sop;
output								source_eop;

// conduit export
input                         mode;

////////////////////////////////////////////////////////////////////////
//
parameter IMAGE_W = 11'd640;
parameter IMAGE_H = 11'd480;
parameter MESSAGE_BUF_MAX = 256;
parameter MSG_INTERVAL = 6;
parameter RBB_COL_DEFAULT = 24'hff0000;
parameter WBB_COL_DEFAULT = 24'hffffff;
parameter YBB_COL_DEFAULT = 24'hffff00;
parameter GBB_COL_DEFAULT = 24'h00ff00;
parameter BBB_COL_DEFAULT = 24'h0000ff;
parameter Y_SCAN = 195;
parameter Y_SCAN_1 = 420;
parameter W_UPPER_BOUND = 100;
parameter INITIAL_BEACON_SIZE = 1;
parameter MIDDLE_LEFT_BOUND = 310;
parameter MIDDLE_RIGHT_BOUND = 330;
parameter INITIAL_BEACON_MAX_F = 4;
parameter INITIAL_BEACON_MAX_S = 8;
parameter MAX_BEACON_SIZE = 130;


wire [7:0]   red, green, blue, grey;
wire [7:0]   red_out, green_out, blue_out;

wire         sop, eop, in_valid, out_ready;
////////////////////////////////////////////////////////////////////////

//conversion of RGB to HSV
//fixed point with 8 bits for fractional part
// wire [15:0] n_r, n_g, n_b;
// //wire [9:0] n_red, n_green, n_blue;
wire [7:0] max, min, diff;
reg [17:0] hue; //from 0 - 255 degree
reg [18:0] saturation; //from 0-255
reg [15:0] value; //from 0-255

// assign n_r = red*256;
// assign n_g = green << 8;
// assign n_b = blue << 8;

// assign n_red = n_r/255;
// assign n_green = n_g/255;
// assign n_blue = n_b/255;

assign max = (red > green) ? ((red > blue)? red : blue) : (green > blue ? green : blue);
assign min = (red < green) ? ((red < blue)? red : blue) : (green < blue ? green : blue);
assign diff = max - min;

calculate hue
always@(*) begin
	// if (max == red) begin
	// 	hue = 60*(green - blue)/diff + 360;
	// end
	// else if (max == green) begin
	// 	hue = 60*(blue - red)/diff + 120;
	// end
	// else begin
	// 	hue = 60*(red - green)/diff + 240;
	// end

	// if (diff==0) begin
	// 	hue = 0;
	// end
	// else if (max == n_r) begin
	// 	hue = ((n_g < n_b) ? (n_g - n_b + (256<<8)) : (n_g - n_b)) * 43 / (diff>>8);
	// end
	// else if (max == n_g) begin
	// 	hue = (((n_b < n_r) ? (n_b - n_r + (256<<8)) : (n_b - n_r))*43/(diff>>8)) + (85<<8);
	// end
	// else begin
	// 	hue = (((n_r < n_g) ? (n_r - n_g + (256<<8)) : (n_r - n_g))*43/(diff>>8)) + (171<<8);
	// end

	if (diff==0) begin
		hue = 0;
	end
	else if (max == red) begin
		hue = ((green < blue) ? ((green - blue + 256)<<8) : ((green - blue)<<8)) * 43 / (diff<<8);
	end
	else if (max == green) begin
		hue = (((blue < red) ? ((blue - red + 256)<<8) : ((blue - red)<<8))*43/(diff<<8)) + 85;
	end
	else begin
		hue = (((red < green) ? ((red - green + 256)<<8) : ((red - green)<<8))*43/(diff<<8)) + 171;
	end

 end

//calculate saturation
// always@(*) begin
// 	if (max==0) begin
// 		saturation <= 16'b0;
// 	end
// 	else begin
// 		//saturation <= 255*diff/max;
// 		saturation <= 255*diff;
// 	end

// 	//calculate value
//     value <= max;
// end

// //calculate value
// assign value = n_r;


// Detect red areas
wire red_detect, yellow_detect, green_detect, blue_detect, white_detect;

// assign white_detect = red[7] & green[7] & blue[7] & red[6] & green[6] & blue[6] ;
// assign yellow_detect = (red >224) & (green>224) & (blue<128);
// assign red_detect = (red >224) & (green<128) & (blue<128);
// assign blue_detect = (red<128) & (green<128) & (blue >224);

assign white_detect = (hue > (39<<8) & hue < (139<<8)) & (saturation > 8 & saturation <20) & (value > (253<<8));
assign red_detect = (hue < (21<<8)) & (saturation >180) & (value > (56<<8));
assign blue_detect = (hue > (124<<8) & hue < (160<<8)) & (saturation > 208) & (value > (239<<8));
assign yellow_detect = (hue > (22<<8) & hue < (53<<8)) & (saturation > 21) & (value > (111<<8));

// assign white_detect = value > 253;//(saturation > (8<<8)) & (saturation < (20<<8)) & (value > (240<<8));
// assign red_detect = saturation > 180;
// assign blue_detect = saturation > 208;
// assign yellow_detect = saturation > 21;

// Find boundary of cursor box

// Highlight detected areas
reg [23:0] color;
assign grey = green[7:1] + red[7:2] + blue[7:2]; //Grey = green/2 + red/4 + blue/4
//assign color = white_detect ? {8'hff, 8'hff, 8'hff} : {grey, grey, grey};

always@(*) begin
	if (red_detect) begin 
		color <= {8'hff, 8'h0, 8'h0};
	end
	else if (yellow_detect) begin 
		color <= {8'hff, 8'hff, 8'h0};
	end
	// else if (green_detect) begin 
	// 	color <= {8'h0, 8'hff, 8'h0};
	// end
	else if (blue_detect) begin
		color <= {8'h0, 8'h0, 8'hff};
	end
	else if (white_detect) begin 
		color <= {8'h0, 8'hff, 8'h0};
	end
	else begin
		color <=  {grey, grey, grey};
	end
end

// Show bounding box
reg [23:0] new_image;
wire rbb_active, ybb_active, gbb_active, bbb_active, wbb_active, wbb_active_1, wbb_active_2, wbb_active_3;
assign rbb_active = (x == r_left) | (x == r_right) | (y == r_top) | (y == r_bottom);
assign ybb_active = (x == y_left) | (x == y_right) | (y == y_top) | (y == y_bottom);
//assign gbb_active = (x == g_left) | (x == g_right) | (y == g_top) | (y == g_bottom);
assign bbb_active = (x == b_left) | (x == b_right) | (y == b_top) | (y == b_bottom);
assign wbb_active_1 = (x == w_left_1) | (x == w_right_1) | (y == w_top_1) | (y == w_bottom_1);
assign wbb_active_2 = (x == w_left_2) | (x == w_right_2) | (y == w_top_2) | (y == w_bottom_2);
assign wbb_active_3 = (x == w_left_3) | (x == w_right_3) | (y == w_top_3) | (y == w_bottom_3);
assign wbb_active = wbb_active_1 | wbb_active_2 | wbb_active_3;

//assign new_image = rbb_active ? rbb_col : red_high;
//determining color of bounding box
always @(*) begin
	if (y == Y_SCAN_1) begin
		new_image <= {8'h00, 8'hff, 8'h00};
	end
	else if (y==Y_SCAN+2 | y==Y_SCAN+3 | y==Y_SCAN+1) begin
		if (x > left_max && x < right_min) begin
			new_image <= {8'h8f, 8'h00, 8'hff};
		end
		else begin
			new_image <= color;
		end
	end
	else if (wbb_active) begin
		if ((wbb_active_1 && x<213) | (wbb_active_2 && (x>213) && (x<426)) | (wbb_active_3 && x > 426)) begin
		   new_image <= WBB_COL_DEFAULT;
		end
		else begin
			new_image <= color;
		end
	end
	else if (rbb_active) begin
		new_image <= RBB_COL_DEFAULT;
	end
	else if (ybb_active) begin
		new_image <= YBB_COL_DEFAULT;
	end
	// else if (gbb_active) begin
	// 	new_image <= GBB_COL_DEFAULT;
	// end
	else if (bbb_active) begin
		new_image <= BBB_COL_DEFAULT;
	end
	else begin
		new_image <= color;
	end
end

// Switch output pixels depending on mode switch
// Don't modify the start-of-packet word - it's a packet discriptor
// Don't modify data in non-video packets
assign {red_out, green_out, blue_out} = (mode & ~sop & packet_video) ? new_image : {red,green,blue};

//Count valid pixels to tget the image coordinates. Reset and detect packet type on Start of Packet.
reg [10:0] x, y;
reg packet_video;
always@(posedge clk) begin
	if (sop) begin
		x <= 11'h0;
		y <= 11'h0;
		packet_video <= (blue[3:0] == 3'h0);
	end
	else if (in_valid) begin
		if (x == IMAGE_W-1) begin
			x <= 11'h0;
			y <= y + 11'h1;
		end
		else begin
			x <= x + 11'h1;
		end
	end
end

//Find first and last red pixels
reg [10:0] rx_min, ry_min, rx_max, ry_max;
always@(posedge clk) begin
	if (red_detect & in_valid) begin	//Update bounds when the pixel is red
		if (x < rx_min) rx_min <= x;
		if (x > rx_max) rx_max <= x;
		if (y < ry_min) ry_min <= y;
		ry_max <= y;
	end
	if (sop & in_valid) begin	//Reset bounds on start of packet
		rx_min <= IMAGE_W-11'h1;
		rx_max <= 0;
		ry_min <= IMAGE_H-11'h1;
		ry_max <= 0;
	end
end

//find first and last green pixels
// reg [10:0] gx_min, gy_min, gx_max, gy_max;
// always@(posedge clk) begin
// 	if (green_detect & in_valid) begin	//Update bounds when the pixel is red
// 		if (x < gx_min) gx_min <= x;
// 		if (x > gx_max) gx_max <= x;
// 		if (y < gy_min) gy_min <= y;
// 		gy_max <= y;
// 	end
// 	if (sop & in_valid) begin	//Reset bounds on start of packet
// 		gx_min <= IMAGE_W-11'h1;
// 		gx_max <= 0;
// 		gy_min <= IMAGE_H-11'h1;
// 		gy_max <= 0;
// 	end
// end

//find first and last yellow pixels
reg [10:0] yx_min, yy_min, yx_max, yy_max;
//wire y_size;
always@(posedge clk) begin
	if (yellow_detect & in_valid) begin	//Update bounds when the pixel is red
		if (x < yx_min) yx_min <= x;
		if (x > yx_max) yx_max <= x;
		if (y < yy_min) yy_min <= y;
		yy_max <= y;
	end
	if (sop & in_valid) begin	//Reset bounds on start of packet
		yx_min <= IMAGE_W-11'h1;
		yx_max <= 0;
		yy_min <= IMAGE_H-11'h1;
		yy_max <= 0;
	end
end
//assign y_size = yx_max - yx_min;

//find first and last blue pixels
reg [10:0] bx_min, by_min, bx_max, by_max;
//wire b_size;
always@(posedge clk) begin
	if (blue_detect & in_valid) begin	//Update bounds when the pixel is red
		if (x < bx_min) bx_min <= x;
		if (x > bx_max) bx_max <= x;
		if (y < by_min) by_min <= y;
		by_max <= y;
	end
	if (sop & in_valid) begin	//Reset bounds on start of packet
		bx_min <= IMAGE_W-11'h1;
		bx_max <= 0;
		by_min <= IMAGE_H-11'h1;
		by_max <= 0;
	end
end
//assign b_size = bx_max - bx_min;

//find first and last white pixels
reg [10:0] wx_min_1, wy_min_1, wx_max_1, wy_max_1;
reg [10:0] wy_max_2;
reg [10:0] wx_min_3, wy_min_3, wx_max_3, wy_max_3;
always@(posedge clk) begin
	if (white_detect & in_valid) begin	//Update bounds when the pixel is red
	    if (x < 213 & y > W_UPPER_BOUND) begin
			if (x < wx_min_1) wx_min_1 <= x;
		    if (x > wx_max_1) wx_max_1 <= x;
		    if (y < wy_min_1) wy_min_1 <= y;
		    wy_max_1 <= y;
		end
		else if (x < 426) begin
			//if (x < wx_min_2) wx_min_2 <= x;
		    //if (x > wx_max_2) wx_max_2 <= x;
		    //if (y < wy_min_2) wy_min_2 <= y;
		    wy_max_2 <= y;
		end
		else if (x < 639 & y > W_UPPER_BOUND) begin
			if (x < wx_min_3) wx_min_3 <= x;
		    if (x > wx_max_3) wx_max_3 <= x;
		    if (y < wy_min_3) wy_min_3 <= y;
		    wy_max_3 <= y;
		end
	end
	if (sop & in_valid) begin	//Reset bounds on start of packet
	//for left plane
		wx_min_1 <= IMAGE_W-11'h1;
		wx_max_1 <= 0;
		wy_min_1 <= IMAGE_H-11'h1;
		wy_max_1 <= 0;

		//for middle plane
	    // wx_min_2 <= IMAGE_W-11'h1;
		// wx_max_2 <= 0;
		// wy_min_2 <= IMAGE_H-11'h1;
		wy_max_2 <= 0;

		//for right plane
		wx_min_3 <= IMAGE_W-11'h1;
		wx_max_3 <= 0;
		wy_min_3 <= IMAGE_H-11'h1;
		wy_max_3 <= 0;
	end
end

//count white pixels at y = scan
// reg counting, send_white;
// reg [10:0] start_w, end_w;
// always @ (*) begin
// 	if (y == Y_SCAN) begin
// 		if (counting) begin
// 			if (white_detect) begin
// 			    end_w = x;
// 				send_white = 0;
// 			end
// 			else begin
// 				send_white = 1;
// 				counting = 0;
// 				start_w = 0;
// 			    end_w = 0;
// 			end
// 		end
// 		else begin
// 			if (white_detect) begin
// 				counting = 1;
// 				start_w = x;
// 			    end_w = x;
// 			end
// 		end
// 	end
// 	else begin
// 		counting = 0;
// 		send_white = 0;
// 	end
// end

//find max on left plane and min on right plane
reg [10:0] left_max, right_min;
always@(*) begin
	if (sop && in_valid) begin
		left_max = 0;
		right_min = IMAGE_W;
	end
	else if (in_valid && white_detect && (y==Y_SCAN)) begin
		if (x < 320) begin
			left_max = x;
		end
		else begin
			if (x < right_min) begin
				right_min = x;
			end
		end
	end
end



//Process bounding box at the end of the frame.
//reg [1:0] msg_state;
reg [2:0] msg_state;
reg [10:0] r_left, r_right, r_top, r_bottom;
reg [10:0] g_left, g_right, g_top, g_bottom;
reg [10:0] b_left, b_right, b_top, b_bottom;
reg [10:0] y_left, y_right, y_top, y_bottom;
reg [10:0] w_left_1, w_right_1, w_top_1, w_bottom_1;
reg [10:0] w_left_2, w_right_2, w_top_2, w_bottom_2;
reg [10:0] w_left_3, w_right_3, w_top_3, w_bottom_3;
reg [10:0] left_coord, right_coord;
reg [10:0] r_size, y_size, b_size;
reg [10:0] r_x, r_y, y_x, y_y, b_x, b_y;
reg [9:0] frame_count;
always@(posedge clk) begin
	if (eop & in_valid & packet_video) begin  //Ignore non-video packets
		
		//Latch edges for display overlay on next frame
		//red
		r_left <= rx_min;
		r_right <= rx_max;
		r_top <= ry_min;
		r_bottom <= ry_max;
		r_size <= rx_max - rx_min;
		r_x <= (rx_max + rx_min) >> 1;
		r_y <= (ry_max + ry_min) >> 1;

		//green
		// g_left <= gx_min;
		// g_right <= gx_max;
		// g_top <= gy_min;
		// g_bottom <= gy_max;

		//yellow
		y_left <= yx_min;
		y_right <= yx_max;
		y_top <= yy_min;
		y_bottom <= yy_max;
		y_size <= yx_max - yx_min;
		y_x <= (yx_max + yx_min) >> 1;
		y_y <= (yy_max + yy_min) >> 1;

		//blue
		b_left <= bx_min;
		b_right <= bx_max;
		b_top <= by_min;
		b_bottom <= by_max;
		b_size <= bx_max - bx_min;
		b_x <= (bx_max + bx_min) >> 1;
		b_y <= (by_max + by_min) >> 1;

		//white
		w_left_1 <= wx_min_1;
		w_right_1 <= wx_max_1;
		w_top_1 <= wy_min_1;
		w_bottom_1 <= wy_max_1;
		
		// w_left_2 <= wx_min_2;
		// w_right_2 <= wx_max_2;
		// w_top_2 <= wy_min_2;
		w_bottom_2 <= wy_max_2;

		w_left_3 <= wx_min_3;
		w_right_3 <= wx_max_3;
		w_top_3 <= wy_min_3;
		w_bottom_3<= wy_max_3;

		//for left and right coordinates
		left_coord <= left_max;
		right_coord <= right_min;
		
		//Start message writer FSM once every MSG_INTERVAL frames, if there is room in the FIFO
		//frame_count <= frame_count - 3;
		frame_count <= frame_count - 1;
		
		if (frame_count == 0 && msg_buf_size < MESSAGE_BUF_MAX - 3) begin
			msg_state <= 3'b001;
			frame_count <= MSG_INTERVAL-1;
		end
	end
	
	//Cycle through message writer states once started
	if (msg_state != 3'b000) msg_state <= msg_state + 3'b001;
end

//check beacons
reg red_beacon, yellow_beacon;
reg blue_long, blue_short;
reg red_middle, yellow_middle, blue_middle;
//reg check_node;

always@(*) begin
	if (eop & in_valid & packet_video) begin 
		red_beacon <= red_detect & (r_size > INITIAL_BEACON_SIZE) & (r_size < MAX_BEACON_SIZE);
		yellow_beacon <= yellow_detect & (y_size > INITIAL_BEACON_SIZE) & (!red_detect) & (y_size < MAX_BEACON_SIZE);
		//blue_beacon <= blue_detect & (b_size > INITIAL_BEACON_SIZE);
		blue_long <= blue_detect & (b_size > INITIAL_BEACON_SIZE) & (b_size < INITIAL_BEACON_MAX_F);
		blue_short <= blue_detect & (b_size > INITIAL_BEACON_MAX_F) & (b_size < INITIAL_BEACON_MAX_S);

		red_middle <= red_detect & (r_x > MIDDLE_LEFT_BOUND) & (r_x < MIDDLE_RIGHT_BOUND);
		blue_middle <= blue_detect & (b_x > MIDDLE_LEFT_BOUND) & (b_x < MIDDLE_RIGHT_BOUND);
		yellow_middle <= yellow_detect & (y_x > MIDDLE_LEFT_BOUND) & (y_x < MIDDLE_RIGHT_BOUND);

	end
end



//identify whether there is intersection or not
wire intersect_detect;
assign intersect_detect = (y==Y_SCAN) & white_detect;
	
//Generate output messages for CPU
reg [31:0] msg_buf_in; 
wire [31:0] msg_buf_out;
reg msg_buf_wr;
wire msg_buf_rd, msg_buf_flush;
wire [7:0] msg_buf_size;
wire msg_buf_empty;

`define RED_BOX_MSG_ID "NEW";

// always @ (posedge clk) begin
// 	if (intersect_detect && (frame_count-x==0) && (msg_buf_size < MESSAGE_BUF_MAX - 3)) begin
// 		msg_buf_in = {21'b0, x};
// 		msg_buf_wr = 1'b1;
// 	end
// end

always@(*) begin	
	//Write words to FIFO as state machine advances
    // if (intersect_detect && (frame_count-x==0) && (msg_buf_size < MESSAGE_BUF_MAX - 3)) begin
	// 	msg_buf_in = {21'b0, x};
	// 	msg_buf_wr = 1'b1;
	// end
	// else begin
	// 	msg_buf_in <= 32'b0;
	// 	msg_buf_wr <= 1'b0;
	// end
	case(msg_state)
	3'b000: begin
		msg_buf_in = 32'b0;
		msg_buf_wr = 1'b0;
	end
	3'b001: begin
		msg_buf_in = {25'b0, red_beacon, yellow_beacon, blue_long, blue_short, red_middle, yellow_middle, blue_middle};	//Message ID
		msg_buf_wr = 1'b1;
	end
	3'b010: begin
		msg_buf_in = {5'b01100, left_max, 5'b0, right_min};	//points for mapping
		//msg_buf_in = {5'b01100, left_max, 5'b0, b_size};	//points for mapping
		msg_buf_wr = 1'b1;
	end
	3'b011: begin
		//msg_buf_in = {4'b1, 1'b0, r_x, 5'b0, r_y}; //r_bottom r_right coordinate
		msg_buf_in = {5'b00010, 16'b0, w_bottom_2}; //points for stop the rover
		msg_buf_wr = 1'b1;
	end
	3'b100: begin
		msg_buf_in = {4'b0010, 1'b0, w_top_1, 5'b0, w_bottom_1};	//for gradient on left section
		msg_buf_wr = 1'b1;
	end
	3'b101: begin
		msg_buf_in = {4'b0011, 1'b0, w_right_1, 5'b0, w_left_1};	//for gradien on left section
		msg_buf_wr = 1'b1;
	end
	3'b110: begin
		msg_buf_in = {4'b0100, 1'b0, w_top_3, 5'b0, w_bottom_3};	//for gradient on right section
		msg_buf_wr = 1'b1;
	end
	3'b111: begin
		msg_buf_in = {4'b0101, 1'b0, w_left_3, 5'b0, w_right_3};	//for gradient on right section
		msg_buf_wr = 1'b1;
	end
	endcase
	
end


//Output message FIFO
MSG_FIFO	MSG_FIFO_inst (
	.clock (clk),
	.data (msg_buf_in),
	.rdreq (msg_buf_rd),
	.sclr (~reset_n | msg_buf_flush),
	.wrreq (msg_buf_wr),
	.q (msg_buf_out),
	.usedw (msg_buf_size),
	.empty (msg_buf_empty)
	);


//Streaming registers to buffer video signal
STREAM_REG #(.DATA_WIDTH(26)) in_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(sink_ready),
	.valid_out(in_valid),
	.data_out({red,green,blue,sop,eop}),
	.ready_in(out_ready),
	.valid_in(sink_valid),
	.data_in({sink_data,sink_sop,sink_eop})
);

STREAM_REG #(.DATA_WIDTH(26)) out_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(out_ready),
	.valid_out(source_valid),
	.data_out({source_data,source_sop,source_eop}),
	.ready_in(source_ready),
	.valid_in(in_valid),
	.data_in({red_out, green_out, blue_out, sop, eop})
);


/////////////////////////////////
/// Memory-mapped port		 /////
/////////////////////////////////

// Addresses
`define REG_STATUS    			0
`define READ_MSG    				1
`define READ_ID    				2
`define REG_BBCOL					3

//Status register bits
// 31:16 - unimplemented
// 15:8 - number of words in message buffer (read only)
// 7:5 - unused
// 4 - flush message buffer (write only - read as 0)
// 3:0 - unused


// Process write

reg  [7:0]   reg_status;
reg	[23:0]	bb_col;

always @ (posedge clk)
begin
	if (~reset_n)
	begin
		reg_status <= 8'b0;
		bb_col <= RBB_COL_DEFAULT;
	end
	else begin
		if(s_chipselect & s_write) begin
		   if      (s_address == `REG_STATUS)	reg_status <= s_writedata[7:0];
		   if      (s_address == `REG_BBCOL)	bb_col <= s_writedata[23:0];
		end
	end
end


//Flush the message buffer if 1 is written to status register bit 4
assign msg_buf_flush = (s_chipselect & s_write & (s_address == `REG_STATUS) & s_writedata[4]);


// Process reads
reg read_d; //Store the read signal for correct updating of the message buffer

// Copy the requested word to the output port when there is a read.
always @ (posedge clk)
begin
   if (~reset_n) begin
	   s_readdata <= {32'b0};
		read_d <= 1'b0;
	end
	
	else if (s_chipselect & s_read) begin
		if   (s_address == `REG_STATUS) s_readdata <= {16'b0,msg_buf_size,reg_status};
		if   (s_address == `READ_MSG) s_readdata <= {msg_buf_out};
		if   (s_address == `READ_ID) s_readdata <= 32'h1234EEE2;
		if   (s_address == `REG_BBCOL) s_readdata <= {8'h0, bb_col};
	end
	
	read_d <= s_read;
end

//Fetch next word from message buffer after read from READ_MSG
assign msg_buf_rd = s_chipselect & s_read & ~read_d & ~msg_buf_empty & (s_address == `READ_MSG);
						


endmodule

