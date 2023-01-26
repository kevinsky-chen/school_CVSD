module ipdc (                       //Don't modify interface
	input         i_clk,
	input         i_rst_n,
	input         i_op_valid,
	input  [ 3:0] i_op_mode,
    output        o_op_ready,
	input         i_in_valid,
	input  [23:0] i_in_data,
	output        o_in_ready,
	output        o_out_valid,
	output [23:0] o_out_data
);

// ---------------------------------------------------------------------------
// Wires and Registers
// ---------------------------------------------------------------------------
// outputs
reg	[23:0]	o_out_data_w, o_out_data_r;
reg			o_op_ready_w, o_op_ready_r, 
			o_in_ready_w, o_in_ready_r, 
			o_out_valid_w, o_out_valid_r;
// vars
reg [23:0]	image [255:0];
reg [23:0]	filter [8:0]; // 3*3 filter
reg [23:0]  out [15:0]; // 4*4 output
reg [3:0]	count_out, count_out_next; // 0~15 for output
reg	[7:0]	count_load, count_load_next;
wire [7:0]	data_R, data_G, data_B;
reg	[3:0]	cur_state, next_state;
reg	[2:0]	step_size, step_size_next; // 1, 2, 4 
reg	[7:0]	origin, origin_next, cur_A, cur_A_next; // 0~255
reg			flag_OF, col_OF, row_OF;
reg	[3:0]	right_mv, down_mv;
reg	[23:0]	catch [35:0]; // 6*6 (max 4*4 + padding), each value 0~255
reg	[3:0]	count_filter, count_filter_next;
reg [23:0]	row1_max, row1_mid, row1_min, row2_max, row2_mid, row2_min, row3_max, row3_mid, row3_min;
reg [23:0]	col1_max, col1_mid, col1_min, col2_max, col2_mid, col2_min, col3_max, col3_mid, col3_min;
reg	[2:0]	median_step, median_step_next; // 0~2

wire [23:0]	sram_o_Q;
wire [7:0]	sram_i_A;
reg	[7:0]	sram_i_A_w, sram_i_A_r;
wire		sram_i_CEN, sram_i_WEN;
reg			sram_i_CEN_w, sram_i_CEN_r, sram_i_WEN_w, sram_i_WEN_r;

// paras
parameter S_IDLE			= 4'b0000; // Idle
parameter S_IF				= 4'b0001; // op fetch (o_op_ready=1)
parameter S_EX				= 4'b0010; // run opation
parameter S_LOAD_IMAGE		= 4'b0011; // load image for 256 cycle
parameter S_LOAD_CATCH		= 4'b0100; // get image into 3*3 filter
parameter S_FIND_MEDIAN		= 4'b0101; // get the median of the 3*3 filter
parameter S_OUTPUT_REGION	= 4'b0110; // get the outputs of the region
parameter S_OUTPUT_MEDIAN	= 4'b0111; // get the outputs of the median
parameter S_OUTPUT_YCBCR	= 4'b1000; // get the outputs of the YCbCr
parameter S_OUTPUT_CENSUS	= 4'b1001; // get the outputs of the Census transform
parameter S_OUT_DATA		= 4'b1110; // output the out[]
parameter S_END				= 4'b1111; // Process End

parameter OP_LOAD_IMAGE		= 4'b0000;
parameter OP_SHIFT_RIGHT	= 4'b0100;
parameter OP_SHIFT_LEFT		= 4'b0101;
parameter OP_SHIFT_UP		= 4'b0110;
parameter OP_SHIFT_DOWN		= 4'b0111;
parameter OP_SCALE_DOWN		= 4'b1000;
parameter OP_SCALE_UP		= 4'b1001;
parameter OP_MEDIAN_FILTER	= 4'b1100;
parameter OP_YCBCR_DISPLAY	= 4'b1101;
parameter OP_CENSUS_TRANS	= 4'b1110;

// ---------------------------------------------------------------------------
// Continuous Assignment
// ---------------------------------------------------------------------------
assign o_op_ready	= o_op_ready_r;
assign o_in_ready	= o_in_ready_r;
assign o_out_valid	= o_out_valid_r;
assign data_R		= i_in_data[23:16];
assign data_G       = i_in_data[15:8];
assign data_B       = i_in_data[7:0];

assign sram_i_A		= sram_i_A_r;
assign sram_i_CEN	= sram_i_CEN_r;
assign sram_i_WEN	= sram_i_WEN_r;

sram_256x8 sram_R(
	.Q(sram_o_Q[23:16]),
	.CLK(i_clk),
	.CEN(sram_i_CEN),
	.WEN(sram_i_WEN),
	.A(sram_i_A),
	.D(data_R)
);
sram_256x8 sram_G(
	.Q(sram_o_Q[15:8]),
	.CLK(i_clk),
	.CEN(sram_i_CEN),
	.WEN(sram_i_WEN),
	.A(sram_i_A),
	.D(data_G)
);
sram_256x8 sram_B(
	.Q(sram_o_Q[7:0]),
	.CLK(i_clk),
	.CEN(sram_i_CEN),
	.WEN(sram_i_WEN),
	.A(sram_i_A),
	.D(data_B)
);
// ---------------------------------------------------------------------------
// Combinational Blocks
// ---------------------------------------------------------------------------
always@(*)begin
	// init
	o_out_valid_w	= 0;
	sram_i_A_w		= 0;
	sram_i_CEN_w	= 0;
	sram_i_WEN_w	= 1;
	o_in_ready_w	= 1;
	o_op_ready_w	= 0;
	
	count_load_next	= 0;
	count_out_next	= 0;
	o_out_valid_w	= 0;
	median_step		= 0;
	next_state		= 0;
	step_size_next	= step_size;
	origin_next		= origin;
	cur_A_next		= cur_A;

	case(cur_state)
		S_IDLE: begin
			o_out_valid_w	= 0;
			sram_i_CEN_w	= 1;
			sram_i_WEN_w	= 0;
			sram_i_A_w		= 0;
			o_in_ready_w	= 1;
			next_state		= S_IF;
		end
		S_IF: begin
			o_out_valid_w	= 0;
			o_op_ready_w	= 1;
			sram_i_CEN_w	= 1;
			sram_i_WEN_w	= 0;
			sram_i_A_w		= 0;
			o_in_ready_w 	= 1;
			flag_OF			= 0;
			col_OF			= 0;
			row_OF			= 0;
			next_state		= S_EX;
		end
		S_EX: begin
			o_op_ready_w	= 0;
			o_in_ready_w	= 1;
			if(!i_op_valid) begin // wait until i_op_valid
				next_state = S_EX;
			end else begin
				case(i_op_mode)
					OP_LOAD_IMAGE: begin
						count_load_next	= 0;
						next_state	= S_LOAD_IMAGE;
					end
					OP_SHIFT_RIGHT: begin
						{col_OF, origin_next[3:0]} = origin[3:0] + step_size + 3;

						if(col_OF) begin
							origin_next = origin;
						end else begin
							origin_next[3:0] = origin[3:0] + step_size;
						end
						next_state	= S_OUTPUT_REGION;
					end
					OP_SHIFT_LEFT: begin
						{col_OF, origin_next[3:0]} = origin[3:0] - step_size;
						if(col_OF) begin
							origin_next[3:0] = origin[3:0];
						end
						next_state	= S_OUTPUT_REGION;
					end
					OP_SHIFT_UP: begin
						{row_OF, origin_next[7:4]} = origin[7:4] - step_size;
						if(row_OF) begin
							origin_next[7:4] = origin[7:4];
						end
						next_state	= S_OUTPUT_REGION;
					end
					OP_SHIFT_DOWN: begin
						{row_OF, origin_next[7:4]} = origin[7:4] + step_size + 3;
						if(row_OF) begin
							origin_next[7:4] = origin[7:4];
						end else begin
							origin_next[7:4] = origin[7:4] + step_size;
						end
						next_state	= S_OUTPUT_REGION;
					end
					OP_SCALE_DOWN: begin
						if(!step_size[2]) step_size_next = step_size << 1;
						next_state	= S_OUTPUT_REGION;
					end
					OP_SCALE_UP: begin
						if(!step_size[0]) step_size_next = step_size >> 1;
						next_state	= S_OUTPUT_REGION;
					end
					OP_MEDIAN_FILTER: begin
						//next_state		= S_LOAD_FILTER;
						count_out_next	= 0;
						//cur_A	= origin;
						next_state	= S_OUTPUT_MEDIAN;
					end
					OP_YCBCR_DISPLAY: begin
						count_out_next	= 0;
						//cur_A	= origin;
						next_state	= S_OUTPUT_YCBCR;
					end
					OP_CENSUS_TRANS: begin
						count_out_next	= 0;
						//cur_A	= origin;
						next_state	= S_OUTPUT_CENSUS;
					end
				endcase
			end
		end
		S_OUT_DATA: begin
			o_out_valid_w	= 1;
			o_out_data_w	= out[count_out];

			count_out_next = count_out + 1;
			case(step_size)
				3'd1: begin
					if(count_out == 4'd15) 
						next_state = S_IF;
					else
						next_state = S_OUT_DATA;
				end
				3'd2: begin
					if(count_out == 4'd3) 
						next_state = S_IF;
					else
						next_state = S_OUT_DATA;
				end
				3'd4: begin
					next_state = S_IF;
				end
				default: begin
					next_state = S_IF;
				end
			endcase
		end
		S_OUTPUT_REGION: begin
			if(step_size == 1) begin
				out[0] = image[origin]; out[1] = image[origin+1]; out[2] = image[origin+2]; out[3] = image[origin+3];
				out[4] = image[origin+16]; out[5] = image[origin+17]; out[6] = image[origin+18]; out[7] = image[origin+19];
				out[8] = image[origin+32]; out[9] = image[origin+33]; out[10] = image[origin+34]; out[11] = image[origin+35];
				out[12] = image[origin+48]; out[13] = image[origin+49]; out[14] = image[origin+50]; out[15] = image[origin+51];
			end else if(step_size == 2) begin
				out[0] = image[origin]; out[1] = image[origin+2]; 
				out[2] = image[origin+32]; out[3] = image[origin+34];
			end else begin
				out[0] = image[origin];
			end
			count_out_next = 0;
			next_state = S_OUT_DATA;
		end
		S_OUTPUT_MEDIAN: begin
			col_OF	= ((cur_A[3:0]+step_size)-origin[3:0] > 3) || ((cur_A[3:0]+step_size) < origin[3:0]);
			row_OF	= ((cur_A[7:4] + step_size)-origin[7:4] > 3) || ((cur_A[7:4]+step_size) < origin[7:4]);
			cur_A_next	= {cur_A[7:4], cur_A[3:0] + step_size}; // shift right
			if(col_OF > 3) begin
				cur_A_next	= {cur_A[7:4] + step_size, origin[3:0]}; // shift down
			end
			
			// fill the filter
			filter[0] = (cur_A[7:4]>=step_size && cur_A[3:0]>=step_size)? image[{cur_A[7:4]-step_size, cur_A[3:0]-step_size}] : 0;
			filter[1] = (cur_A[7:4]>=step_size)? image[{cur_A[7:4]-step_size, cur_A[3:0]}] : 0;
			filter[2] = (cur_A[7:4]>=step_size && (cur_A[3:0]+step_size)<=15)? image[{cur_A[7:4]-step_size, cur_A[3:0]+step_size}] : 0;
			filter[3] = (cur_A[3:0]>=step_size)? image[{cur_A[7:4], cur_A[3:0]-step_size}] : 0;
			filter[4] = image[cur_A];
			filter[5] = ((cur_A[3:0]+step_size)<=15)? image[{cur_A[7:4], cur_A[3:0]+step_size}] : 0;
			filter[6] = ((cur_A[7:4]+step_size)<=15 && cur_A[3:0]>=step_size)? image[{cur_A[7:4]+step_size, cur_A[3:0]-step_size}] : 0;
			filter[7] = ((cur_A[7:4]+step_size)<=15)? image[{cur_A[7:4]+step_size, cur_A[3:0]}] : 0;
			filter[8] = ((cur_A[7:4]+step_size)<=15 && (cur_A[3:0]+step_size)<=15)? image[{cur_A[7:4]+step_size, cur_A[3:0]+step_size}] : 0;

			// filter[4] -> out
			count_out_next = count_out + 1;
			if(count_out == 0 && ((cur_A[7:4]-origin[7:4])>3 || cur_A[7:4]<origin[7:4])) begin// finished
				out[15] = filter[4];
			end else if(count_out != 0) begin
				out[count_out-1] = filter[4];
			end else out[0] = 0;
			// next_state
			if((cur_A[7:4]-origin[7:4])>3 || cur_A[7:4]<origin[7:4]) begin
				count_out_next = 0;
				next_state = S_OUT_DATA; // finished
			end	else begin
				median_step_next	= 0;
				next_state	= S_FIND_MEDIAN;
			end
		end

		S_OUTPUT_YCBCR: begin
			// Y  = rouding(0.25R + 0.625G)
			// Cb = rouding(-0.125R - 0.25G + 0.5B + 128)
			// Cr = rouding(0.5R - 0.375G - 0.125B + 128)
			// value = 255 if > 255
			col_OF	= ((cur_A[3:0]+step_size)-origin[3:0] > 3) || ((cur_A[3:0]+step_size) < origin[3:0]);
			row_OF	= ((cur_A[7:4] + step_size)-origin[7:4] > 3) || ((cur_A[7:4]+step_size) < origin[7:4]);
			cur_A_next	= {cur_A[7:4], cur_A[3:0] + step_size}; // shift right
			if(col_OF) begin
				cur_A_next	= {cur_A[7:4] + step_size, origin[3:0]}; // shift down
			end

			count_out_next	= count_out + 1;
			// set out[i]
			out[count_out] = {((image[cur_A][23:16] >> 2) + (image[cur_A][15:8] >> 1) + (image[cur_A][15:8] >> 3)),
							(-(image[cur_A][23:16] >> 3) - (image[cur_A][15:8] >> 2) + (image[cur_A][7:0] >> 1) + 128),
							((image[cur_A][23:16] >> 1) - (image[cur_A][15:8] >> 2) - (image[cur_A][15:8] >> 1) - (image[cur_A][7:0] >> 3) + 128)};

			if(col_OF && row_OF) begin
				count_out_next = 0;
				next_state = S_OUT_DATA; // finished
			end else
				next_state = S_OUTPUT_YCBCR;
		end
		S_OUTPUT_CENSUS: begin
			col_OF	= ((cur_A[3:0]+step_size)-origin[3:0] > 3) || ((cur_A[3:0]+step_size) < origin[3:0]);
			row_OF	= ((cur_A[7:4] + step_size)-origin[7:4] > 3) || ((cur_A[7:4]+step_size) < origin[7:4]);
			cur_A_next	= {cur_A[7:4], cur_A[3:0] + step_size}; // shift right
			if(right_mv > 3) begin
				cur_A_next	= {cur_A[7:4] + step_size, origin[3:0]}; // shift down
			end

			// fill the filter
			filter[0] = (cur_A[7:4]>=step_size && cur_A[3:0]>=step_size)? image[{cur_A[7:4]-step_size, cur_A[3:0]-step_size}] : 0;
			filter[1] = (cur_A[7:4]>=step_size)? image[{cur_A[7:4]-step_size, cur_A[3:0]}] : 0;
			filter[2] = (cur_A[7:4]>=step_size && (cur_A[3:0]+step_size)<=15)? image[{cur_A[7:4]-step_size, cur_A[3:0]+step_size}] : 0;
			filter[3] = (cur_A[3:0]>=step_size)? image[{cur_A[7:4], cur_A[3:0]-step_size}] : 0;
			filter[4] = image[cur_A];
			filter[5] = ((cur_A[3:0]+step_size)<=15)? image[{cur_A[7:4], cur_A[3:0]+step_size}] : 0;
			filter[6] = ((cur_A[7:4]+step_size)<=15 && cur_A[3:0]>=step_size)? image[{cur_A[7:4]+step_size, cur_A[3:0]-step_size}] : 0;
			filter[7] = ((cur_A[7:4]+step_size)<=15)? image[{cur_A[7:4]+step_size, cur_A[3:0]}] : 0;
			filter[8] = ((cur_A[7:4]+step_size)<=15 && (cur_A[3:0]+step_size)<=15)? image[{cur_A[7:4]+step_size, cur_A[3:0]+step_size}] : 0;
 
			count_out_next	= count_out + 1;
			// set out[i]
			//R0 = (filter[0][23:16] > filter[4][23:16]);
			//G0 = (filter[0][15:8] > filter[4][15:8]);
			//B0 = (filter[0][7:0] > filter[4][7:0]);
			out[count_out] = {(filter[0][23:16]>filter[4][23:16]),(filter[1][23:16]>filter[4][23:16]),(filter[2][23:16]>filter[4][23:16]),(filter[3][23:16]>filter[4][23:16]),(filter[5][23:16]>filter[4][23:16]),(filter[6][23:16]>filter[4][23:16]),(filter[7][23:16]>filter[4][23:16]),(filter[8][23:16]>filter[4][23:16]),
							(filter[0][15:8]>filter[4][15:8]),(filter[1][15:8]>filter[4][15:8]),(filter[2][15:8]>filter[4][15:8]),(filter[3][15:8]>filter[4][15:8]),(filter[5][15:8]>filter[4][15:8]),(filter[6][15:8]>filter[4][15:8]),(filter[7][15:8]>filter[4][15:8]),(filter[8][15:8]>filter[4][15:8]),
							(filter[0][7:0]>filter[4][7:0]),(filter[1][7:0]>filter[4][7:0]),(filter[2][7:0]>filter[4][7:0]),(filter[3][7:0]>filter[4][7:0]),(filter[5][7:0]>filter[4][7:0]),(filter[6][7:0]>filter[4][7:0]),(filter[7][7:0]>filter[4][7:0]),(filter[8][7:0]>filter[4][7:0])};
			// next_state
			if(col_OF && row_OF) begin
				count_out_next = 0;
				next_state = S_OUT_DATA; // finished
			end else 
				next_state = S_OUTPUT_YCBCR;
		end
		S_LOAD_IMAGE: begin
			if(!i_in_valid) begin // wait until i_in_valid
				next_state = S_LOAD_IMAGE;
			end else begin // load image
				//------- set image to sram -------//
				sram_i_CEN_w	= 0; 
				sram_i_WEN_w	= 0; // Write
				sram_i_A_w	= count_load; // Adress
				//---------------------------------//
				image[count_load] = i_in_data;
				if(count_load == 255) begin
					next_state = S_IF;
				end else begin
					count_load_next = count_load + 1;
					next_state = S_LOAD_IMAGE;
				end
			end
		end
		S_FIND_MEDIAN: begin
			case(median_step)
			2'b00: begin // sort each col
				// col1
				col1_max=0; col1_mid=0; col1_min=0;
				if(filter[6] > filter[3] && filter[6] > filter[0]) begin
					col1_max = filter[6];
					if(filter[3] > filter[0]) begin
						col1_mid = filter[3]; col1_min = filter[0];
					end else begin
						col1_mid = filter[0]; col1_min = filter[3];
					end
				end else if(filter[3] > filter[6] && filter[3] > filter[0]) begin
					col1_max = filter[3];
					if(filter[6] > filter[0]) begin
						col1_mid = filter[6]; col1_min = filter[0];
					end else begin
						col1_mid = filter[0]; col1_min = filter[6];
					end
				end else if(filter[0] > filter[6] && filter[0] > filter[3]) begin
					col1_max = filter[0];
					if(filter[6] > filter[3]) begin
						col1_mid = filter[6]; col1_min = filter[3];
					end else begin
						col1_mid = filter[3]; col1_min = filter[6];
					end
				end
				// col2
				col2_max=0; col2_mid=0; col2_min=0;
				if(filter[7] > filter[4] && filter[7] > filter[1]) begin
					col2_max = filter[7];
					if(filter[4] > filter[1]) begin
						col2_mid = filter[4]; col2_min = filter[1];
					end else begin
						col2_mid = filter[1]; col2_min = filter[4];
					end
				end else if(filter[4] > filter[7] && filter[4] > filter[1]) begin
					col2_max = filter[4];
					if(filter[7] > filter[1]) begin
						col2_mid = filter[7]; col2_min = filter[1];
					end else begin
						col2_mid = filter[1]; col2_min = filter[7];
					end
				end else if(filter[1] > filter[7] && filter[1] > filter[4]) begin
					col2_max = filter[1];
					if(filter[7] > filter[4]) begin
						col2_mid = filter[7]; col2_min = filter[4];
					end else begin
						col2_mid = filter[4]; col2_min = filter[7];
					end
				end
				// col3
				col3_max=0; col3_mid=0; col3_min=0;
				if(filter[8] > filter[5] && filter[8] > filter[2]) begin
					col3_max = filter[8];
					if(filter[5] > filter[2]) begin
						col3_mid = filter[5]; col3_min = filter[2];
					end else begin
						col3_mid = filter[2]; col3_min = filter[5];
					end
				end else if(filter[5] > filter[8] && filter[5] > filter[2]) begin
					col3_max = filter[5];
					if(filter[8] > filter[2]) begin
						col3_mid = filter[8]; col3_min = filter[2];
					end else begin
						col3_mid = filter[2]; col3_min = filter[8];
					end
				end else if(filter[2] > filter[8] && filter[2] > filter[5]) begin
					col3_max = filter[2];
					if(filter[8] > filter[5]) begin
						col3_mid = filter[8]; col3_min = filter[5];
					end else begin
						col3_mid = filter[5]; col3_min = filter[8];
					end
				end
			end
			2'b01: begin // sort each row
				filter[0] = col1_max; filter[3] = col1_mid; filter[6] = col1_min;
				filter[1] = col2_max; filter[4] = col2_mid; filter[7] = col2_min;
				filter[2] = col3_max; filter[5] = col3_mid; filter[8] = col3_min;
				// row1
				row1_max=0; row1_mid=0; row1_min=0;
				if(filter[2] > filter[1] && filter[2] > filter[0]) begin
					row1_max = filter[2];
					if(filter[1] > filter[0]) begin
						row1_mid = filter[1]; row1_min = filter[0];
					end else begin
						row1_mid = filter[0]; row1_min = filter[1];
					end
				end else if(filter[1] > filter[2] && filter[1] > filter[0]) begin
					row1_max = filter[1];
					if(filter[2] > filter[0]) begin
						row1_mid = filter[2]; row1_min = filter[0];
					end else begin
						row1_mid = filter[0]; row1_min = filter[2];
					end
				end else if(filter[0] > filter[2] && filter[0] > filter[1]) begin
					row1_max = filter[0];
					if(filter[2] > filter[1]) begin
						row1_mid = filter[2]; row1_min = filter[1];
					end else begin
						row1_mid = filter[1]; row1_min = filter[2];
					end
				end
				// row2
				row2_max=0; row2_mid=0; row2_min=0;
				if(filter[5] > filter[4] && filter[5] > filter[3]) begin
					row2_max = filter[5];
					if(filter[4] > filter[3]) begin
						row2_mid = filter[4]; row2_min = filter[3];
					end else begin
						row2_mid = filter[3]; row2_min = filter[4];
					end
				end else if(filter[4] > filter[5] && filter[4] > filter[3]) begin
					row2_max = filter[4];
					if(filter[5] > filter[3]) begin
						row2_mid = filter[5]; row2_min = filter[3];
					end else begin
						row2_mid = filter[3]; row2_min = filter[5];
					end
				end else if(filter[3] > filter[5] && filter[3] > filter[4]) begin
					row2_max = filter[3];
					if(filter[5] > filter[4]) begin
						row2_mid = filter[5]; row2_min = filter[4];
					end else begin
						row2_mid = filter[4]; row2_min = filter[5];
					end
				end
				// row3
				row3_max=0; row3_mid=0; row3_min=0;
				if(filter[8] > filter[7] && filter[8] > filter[6]) begin
					row3_max = filter[8];
					if(filter[7] > filter[6]) begin
						row3_mid = filter[7]; row3_min = filter[6];
					end else begin
						row3_mid = filter[6]; row3_min = filter[7];
					end
				end else if(filter[7] > filter[8] && filter[7] > filter[6]) begin
					row3_max = filter[7];
					if(filter[8] > filter[6]) begin
						row3_mid = filter[8]; row3_min = filter[6];
					end else begin
						row3_mid = filter[6]; row3_min = filter[8];
					end
				end else if(filter[6] > filter[8] && filter[6] > filter[7]) begin
					row3_max = filter[6];
					if(filter[8] > filter[7]) begin
						row3_mid = filter[8]; row3_min = filter[7];
					end else begin
						row3_mid = filter[7]; row3_min = filter[8];
					end
				end
			end
			2'b10: begin // sort diagonal
				filter[0] = row1_min; filter[1] = row1_mid; filter[2] = row1_max;
				filter[3] = row2_min; filter[4] = row2_mid; filter[5] = row2_max;
				filter[6] = row3_min; filter[7] = row3_mid; filter[8] = row3_max;
				col1_max=0; col2_mid=0; col3_min=0;
				if(filter[8] > filter[4] && filter[8] > filter[0]) begin
					col1_max = filter[8];
					if(filter[4] > filter[0]) begin
						col2_mid = filter[4]; col3_min = filter[0];
					end else begin
						col2_mid = filter[0]; col3_min = filter[4];
					end
				end else if(filter[4] > filter[8] && filter[4] > filter[0]) begin
					col1_max = filter[4];
					if(filter[8] > filter[0]) begin
						col2_mid = filter[8]; col3_min = filter[0];
					end else begin
						col2_mid = filter[0]; col3_min = filter[8];
					end
				end else if(filter[0] > filter[8] && filter[0] > filter[4]) begin
					col1_max = filter[0];
					if(filter[8] > filter[4]) begin
						col2_mid = filter[8]; col3_min = filter[4];
					end else begin
						col2_mid = filter[4]; col3_min = filter[8];
					end
				end
			end
			2'b11: begin // end
				filter[0] = col1_max; filter[4] = col2_mid; filter[8] = col3_min;
			end
			endcase
			median_step_next = median_step + 1;
			if(median_step == 3) begin
				next_state	= S_FIND_MEDIAN;
			end else begin
				next_state	= S_OUTPUT_MEDIAN;
			end
		end
		S_END: begin
			next_state = S_END;
		end
	endcase
end


// ---------------------------------------------------------------------------
// Sequential Block
// ---------------------------------------------------------------------------
always@(posedge i_clk or negedge i_rst_n) begin
	if(!i_rst_n) begin
		sram_i_A_r		<= 0;
		sram_i_CEN_r	<= 0;
		sram_i_WEN_r	<= 0;
		//sram_i_A_w		<= 0;
		//sram_i_CEN_w	<= 0;
		//sram_i_WEN_w	<= 0;
		o_op_ready_r	<= 0;
		//o_op_ready_w	<= 0;
		o_in_ready_r	<= 0;
		//o_in_ready_w	<= 0;
		o_out_valid_r	<= 0;
		//o_out_valid_w	<= 0;

		step_size		<= 1;
		origin			<= 0;
		count_load		<= 0;
		//count_load_next	<= 0;
		count_out		<= 0;
		//count_out_next	<= 0;
		median_step		<= 0;
		median_step_next	<= 0;
		count_filter	<= 0;
		count_filter_next	<= 0;
		col_OF			<= 0;
		row_OF			<= 0;
		cur_A			<= 0;
		cur_state		<= 0;
	end else begin
		o_op_ready_r	<= o_op_ready_w;
		o_in_ready_r	<= o_in_ready_w;
		o_out_valid_r	<= o_out_valid_w;

		sram_i_A_r		<= sram_i_A_w;
		sram_i_CEN_r	<= sram_i_CEN_w;
		sram_i_WEN_r	<= sram_i_WEN_w;
		
		cur_state		<= next_state;
		count_load		<= count_load_next;
		count_filter	<= count_filter_next;
		count_out		<= count_out_next;
		origin			<= origin_next;
		cur_A			<= cur_A_next;
		step_size		<= step_size_next;
	end
end
endmodule