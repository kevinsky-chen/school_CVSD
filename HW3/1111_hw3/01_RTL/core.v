
module core (                       //Don't modify interface
	input         i_clk,
	input         i_rst_n,
	input         i_op_valid,
	input  [ 3:0] i_op_mode,
    output        o_op_ready,
	input         i_in_valid,
	input  [ 7:0] i_in_data,
	output        o_in_ready,
	output        o_out_valid,
	output [12:0] o_out_data
);

// ---------------------------------------------------------------------------
// Wires and Registers
// ---------------------------------------------------------------------------
// ---- Add your own wires and registers here if needed ---- //
// outputs
reg	[23:0]	o_out_data_w, o_out_data_r;
reg			o_op_ready_w, o_op_ready_r, 
			o_in_ready_w, o_in_ready_r, 
			o_out_valid_w, o_out_valid_r;

reg  [7:0] sram_addr, sram_addr_0, sram_addr_1, sram_addr_2, sram_addr_3;    //第0,1,2,3塊ram的addr (for 512的ram)
reg  [7:0] sram_addr_w, sram_addr_r, sram_addr_0_w, sram_addr_1_w, sram_addr_2_w, sram_addr_3_w, sram_addr_0_r, sram_addr_1_r, sram_addr_2_r, sram_addr_3_r;    //第0,1,2,3塊ram的addr (for 512的ram)
reg   sram_CEN_0, sram_CEN_1, sram_CEN_2, sram_CEN_3;    //第0,1,2,3塊ram的addr (for 512的ram)
reg   sram_CEN_0_w, sram_CEN_1_w, sram_CEN_2_w, sram_CEN_3_w, sram_CEN_0_r, sram_CEN_1_r, sram_CEN_2_r, sram_CEN_3_r;   
reg   sram_WEN_0, sram_WEN_1, sram_WEN_2, sram_WEN_3;    //第0,1,2,3塊ram的addr (for 512的ram)
reg   sram_WEN_0_w, sram_WEN_1_w, sram_WEN_2_w, sram_WEN_3_w, sram_WEN_0_r, sram_WEN_1_r, sram_WEN_2_r, sram_WEN_3_r;   
//reg  [5:0] cnt_64;       //position on a layer during S_LOAD (0~63)
//reg	 [2:0] cnt_8;        //index of layer in a RAM block(3 RAM blocks in total)
reg  [1:0] depth, depth_nxt;    // DEPTH_8, DEPTH_16, DEPTH_32
wire [7:0] sram_q_0, sram_q_1, sram_q_2, sram_q_3, sram_d_0, sram_d_1, sram_d_2, sram_d_3;    //Q=>data output for SRAM; D=>data input for SRAM
reg		flag_OF, col_OF, row_OF;


reg  [2:0] origin_row, origin_col;    //the row index of the origin; col index of the origin
reg  [3:0] op_mode;
reg  [12:0] accu_0, accu_1, accu_2, accu_3;
reg  [3:0] display_col, display_row;
reg  [7:0] filter_0 [0:15]; // 4*4 filter(sliding window; 可以同時運算)
reg  [7:0] filter_1 [0:15]; // 4*4 filter(sliding window; 可以同時運算)
reg  [7:0] filter_2 [0:15]; // 4*4 filter(sliding window; 可以同時運算)
reg  [7:0] filter_3 [0:15]; // 4*4 filter(sliding window; 可以同時運算)

wire  [7:0] kernal_0 [0:15]; // 4*4 kernal (分成四個, 為了不要重複讀值; 可以同時運算)
wire  [7:0] kernal_1 [0:15]; // 4*4 kernal (分成四個, 為了不要重複讀值)
wire  [7:0] kernal_2 [0:15]; // 4*4 kernal (分成四個, 為了不要重複讀值)
wire  [7:0] kernal_3 [0:15]; // 4*4 kernal (分成四個, 為了不要重複讀值)
// FSM related

integer i, j;

reg [12:0]  out [0:127];    // display output
reg [3:0]	count_out, count_out_next; // 0~3 for output
reg	[10:0]	count_load, count_load_next; // 0~2047 for output
reg [1:0]   count_display;  //0~3
reg [2:0]   count_conv, count_conv_next;  //0~8
reg [3:0]   count_load_conv, count_load_conv_next;  //0~15
reg [1:0]   count_out_conv, count_out_conv_next;  //0~3

reg	[3:0]	cur_state, next_state;
reg	[5:0]	origin, origin_next; // 0~64
reg [2:0]	layer_index, layer_index_next;     //0~7 (4個ram一起共用)
//reg [8:0]   cur_addr, cur_addr_next; //0~511


// paras
parameter S_IDLE			= 4'b0000; // Idle
parameter S_IF				= 4'b0001; // op fetch (o_op_ready=1)
parameter S_EX				= 4'b0010; // run opation
parameter S_LOAD_IMAGE		= 4'b0011; // load image for 256 cycle
parameter S_LOAD_CONV_TOP_LEFT	= 4'b0100; // get image into 4*4 filter(padding both on top and left)
parameter S_LOAD_CONV_TOP	= 4'b0101; // get image into 4*4 filter(padding on top)
parameter S_LOAD_CONV_TOP_RIGHT	= 4'b0110; // get image into 4*4 filter(padding both on top and right)
parameter S_LOAD_CONV_LEFT	= 4'b0111; // get image into 4*4 filter(padding on left)
parameter S_LOAD_CONV_RIGHT	= 4'b1000; // get image into 4*4 filter(padding on right)
parameter S_LOAD_CONV_DOWN_LEFT	= 4'b1001; // get image into 4*4 filter(padding both on down and left)
parameter S_LOAD_CONV_DOWN	= 4'b1010; // get image into 4*4 filter(padding on down)
parameter S_LOAD_CONV_DOWN_RIGHT	= 4'b1011; // get image into 4*4 filter(padding both on down and right)
parameter S_LOAD_CONV_MID  = 4'b1111;  //no padding
parameter S_OUTPUT_REGION	= 4'b1100; // get the outputs of the region(DISPLAY)
parameter S_CAL_CONV		= 4'b1101; // get the outputs of the CONV
parameter S_END				= 4'b1110; // Process End


parameter OP_LOAD = 4'b0000;
parameter OP_RIGHT = 4'b0001;
parameter OP_LEFT = 4'b0010;
parameter OP_UP = 4'b0011;
parameter OP_DOWN = 4'b0100;
parameter OP_REDUCE = 4'b0101;
parameter OP_INCREASE = 4'b0110;
parameter OP_CONV = 4'b0111;
parameter OP_OUTPUT = 4'b1000;

parameter DEPTH_8 = 2'b00;
parameter DEPTH_16 = 2'b01;
parameter DEPTH_24 = 2'b10;
parameter DEPTH_32 = 2'b11;


// ---------------------------------------------------------------------------
// Continuous Assignment
// ---------------------------------------------------------------------------
// ---- Add your own wire data assignments here if needed ---- //
assign origin_row = origin >> 3;   // /8
assign origin_col = origin & 4'b1111;   // %8

assign o_op_ready	= o_op_ready_r;
assign o_in_ready	= o_in_ready_r;
assign o_out_valid	= o_out_valid_r;

assign sram_i_A		= sram_i_A_r;
assign sram_i_CEN	= sram_i_CEN_r;
assign sram_i_WEN	= sram_i_WEN_r;

//kernal initialization(可以換位置)
assign  kernal_0[0] = 1>>4;
assign	kernal_0[1] = 1>>3;
assign	kernal_0[2] = 1>>4;
assign	kernal_0[3] = 0;
assign	kernal_0[4] = 1>>3;
assign	kernal_0[5] = 1>>2;
assign	kernal_0[6] = 1>>3;
assign	kernal_0[7] = 0;
assign	kernal_0[8] = 1>>4;
assign	kernal_0[9] = 1>>3;
assign	kernal_0[10] = 1>>4;
assign	kernal_0[11] = 0;
assign	kernal_0[12] = 0;
assign	kernal_0[13] = 0;
assign	kernal_0[14] = 0;
assign	kernal_0[15] = 0;

//相比kernal_0，用3rd col換1st col
assign  kernal_1[0] = 0;
assign	kernal_1[1] = 1>>4;
assign	kernal_1[2] = 1>>3;
assign	kernal_1[3] = 1>>4;
assign	kernal_1[4] = 0;
assign	kernal_1[5] = 1>>3;
assign	kernal_1[6] = 1>>2;
assign	kernal_1[7] = 1>>3;
assign	kernal_1[8] = 0;
assign	kernal_1[9] = 1>>4;
assign	kernal_1[10] = 1>>3;
assign	kernal_1[11] = 1>>4;
assign	kernal_1[12] = 0;
assign	kernal_1[13] = 0;
assign	kernal_1[14] = 0;
assign	kernal_1[15] = 0;

//從kernal_1變過來，需要取代一些值
assign  kernal_2[0] = 0;
assign	kernal_2[1] = 0;
assign	kernal_2[2] = 0;
assign	kernal_2[3] = 0;
assign	kernal_2[4] = 1>>4;
assign	kernal_2[5] = 1>>3;
assign	kernal_2[6] = 1>>4;
assign	kernal_2[7] = 0;
assign	kernal_2[8] = 1>>3;
assign	kernal_2[9] = 1>>2;
assign	kernal_2[10] = 1>>3;
assign	kernal_2[11] = 0;
assign	kernal_2[12] = 1>>4;
assign	kernal_2[13] = 1>>3;
assign	kernal_2[14] = 1>>4;
assign	kernal_2[15] = 0;

//從kernal_2變過來，需要取代一些值
assign  kernal_3[0] = 0;
assign	kernal_3[1] = 0;
assign	kernal_3[2] = 0;
assign	kernal_3[3] = 0;
assign	kernal_3[4] = 0;
assign	kernal_3[5] = 1>>4;
assign	kernal_3[6] = 1>>3;
assign	kernal_3[7] = 1>>4;
assign	kernal_3[8] = 0;
assign	kernal_3[9] = 1>>3;
assign	kernal_3[10] = 1>>2;
assign	kernal_3[11] = 1>>3;
assign	kernal_3[12] = 0;
assign	kernal_3[13] = 1>>4;
assign	kernal_3[14] = 1>>3;
assign	kernal_3[15] = 1>>4;

sram_512x8 sram_0(
	.Q(sram_q_0),
	.CLK(i_clk),
	.CEN(sram_CEN_0),
	.WEN(sram_WEN_0),
	.A(sram_addr),
	.D(sram_d_0)
);
sram_512x8 sram_1(
	.Q(sram_q_1),
	.CLK(i_clk),
	.CEN(sram_CEN_1),
	.WEN(sram_WEN_1),
	.A(sram_addr),
	.D(sram_d_1)
);
sram_512x8 sram_2(
	.Q(sram_q_2),
	.CLK(i_clk),
	.CEN(sram_CEN_2),
	.WEN(sram_WEN_2),
	.A(sram_addr),
	.D(sram_d_2)
);
sram_512x8 sram_3(
	.Q(sram_q_3),
	.CLK(i_clk),
	.CEN(sram_CEN_3),
	.WEN(sram_WEN_3),
	.A(sram_addr),
	.D(sram_d_3)
)

// ---------------------------------------------------------------------------
// Combinational Blocks
// ---------------------------------------------------------------------------
// ---- Write your conbinational block design here ---- //

// current state logic
always@(*) begin
	// init
	o_out_valid_w	= 0;
	sram_addr_w	= 0;
	sram_CEN_0_w	= 0;
	sram_WEN_0_w	= 1;
	//sram_addr_1_w	= 0;
	sram_CEN_1_w	= 0;
	sram_WEN_1_w	= 1;
	//sram_addr_2_w	= 0;
	sram_CEN_2_w	= 0;
	sram_WEN_2_w	= 1;
	//sram_addr_3_w	= 0;
	sram_CEN_3_w	= 0;
	sram_WEN_3_w	= 1;
	o_in_ready_w	= 1;
	o_op_ready_w	= 0;
	
	count_load_next	= 0;
	count_out_next	= 0;
	count_display_next = 0;
	o_out_valid_w	= 0;
	
	next_state		= 0;
	depth_nxt		= depth;
	origin_next		= origin;
	//cur_addr_next	= cur_addr;

	

	case(state)
		S_IDLE: begin
			o_out_valid_w	= 0;

			//或許可以簡化address, CEN, WEN同個位置
			sram_addr_w	= 0;
			sram_CEN_0_w	= 1;
			sram_WEN_0_w	= 0;
			//sram_addr_1_w	= 0;
			sram_CEN_1_w	= 1;
			sram_WEN_1_w	= 0;
			//sram_addr_2_w	= 0;
			sram_CEN_2_w	= 1;
			sram_WEN_2_w	= 0;
			//sram_addr_3_w	= 0;
			sram_CEN_3_w	= 1;
			sram_WEN_3_w	= 0;
			
			o_in_ready_w	= 1;
			next_state		= S_IF;
		end
		S_IF: begin
			o_out_valid_w	= 0;
			o_op_ready_w	= 1;

			sram_addr_w	= 0;
			sram_CEN_0_w	= 1;
			sram_WEN_0_w	= 0;
			//sram_addr_1_w	= 0;
			sram_CEN_1_w	= 1;
			sram_WEN_1_w	= 0;
			//sram_addr_2_w	= 0;
			sram_CEN_2_w	= 1;
			sram_WEN_2_w	= 0;
			//sram_addr_3_w	= 0;
			sram_CEN_3_w	= 1;
			sram_WEN_3_w	= 0;

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
			end 
			else begin
				case(i_op_mode)
					OP_LOAD: begin
						count_load_next	= 0;
						next_state	= S_LOAD_IMAGE;
					end
					OP_RIGHT: begin
						{col_OF, origin_next[2:0]} = origin[2:0] + 1;    //最大不超過8

						if(col_OF) 
							origin_next = origin;
						
						next_state	= S_IDLE;
					end
					OP_LEFT: begin
						{col_OF, origin_next[2:0]} = origin[2:0] - 1;

						if(col_OF) begin
							origin_next[2:0] = origin[2:0];
						end
						next_state	= S_IDLE;
					end
					OP_UP: begin
						{row_OF, origin_next[5:3]} = origin[5:3] - 8;
						if(row_OF) begin
							origin_next[5:3] = origin[5:3];
						end
						next_state	= S_IDLE;
					end
					OP_DOWN: begin
						{row_OF, origin_next[5:3]} = origin[5:3] + 8;
						if(row_OF) begin
							origin_next[5:3] = origin[5:3];
						end
						next_state	= S_IDLE;
					end
					OP_REDUCE: begin
						if(depth > 8) depth_next = depth << 1;
						next_state	= S_IDLE;
					end
					OP_INCREASE: begin
						if(depth < 32) depth_next = depth >> 1;
						next_state	= S_IDLE;
					end
					OP_CONV: begin
						//next_state		= S_LOAD_FILTER;
						count_out_next	= 0;
						//cur_A	= origin;
						next_state	= S_CONV;
					end
					OP_OUTPUT: begin
						count_out_next	= 0;
						//cur_A	= origin;
						next_state	= S_OUTPUT_REGION;
					end
					
				endcase
			end
		end

		S_OUTPUT_REGION: begin   //Not DONE
			o_out_valid_w	= 1;
			
			for(i=0; i<32; i+=1) begin
				if(i<8) begin
					for(count_display=0; count_display<3; count_display+=1) begin
						if(count_display < 2)
							sram_addr_w  = origin + count_display + i<<6;
						else
							sram_addr_w  = origin + (count_display-2) + 8 + i<<6;
						o_out_data_w = sram_q_0;
					end
				end
				else if(i<16) begin
					for(count_display=0; count_display<3; count_display+=1) begin
						if(count_display < 2)
							sram_addr_w  = origin + count_display + (i-8)<<6;
						else
							sram_addr_w  = origin + (count_display-2) + 8 + (i-8)<<6;
						o_out_data_w = sram_q_1;
					end
				end
				else if(i<24) begin
					for(count_display=0; count_display<3; count_display+=1) begin
						if(count_display < 2)
							sram_addr_w  = origin + count_display + (i-16)<<6;
						else
							sram_addr_w  = origin + (count_display-2) + 8 + (i-16)<<6;
						o_out_data_w = sram_q_2;
					end
				end
				else begin
					for(count_display=0; count_display<3; count_display+=1) begin
						if(count_display < 2)
							sram_addr_w  = origin + count_display + (i-24)<<6;
						else
							sram_addr_w  = origin + (count_display-2) + 8 + (i-24)<<6;
						o_out_data_w = sram_q_3;
					end
				end
			end

			next_state = S_IF;
		end

		S_LOAD_CONV_TOP_LEFT: begin
			//此時的origina相對於左上角的座標為: (1,1)
			//此處總共需要 16 個 cycles
			if(count_load_conv < 16) begin
				if(depth == DEPTH_8 || depth == DEPTH_16 || depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						0,1,2,3,4,8,12: filter_0[count_load_conv] = 0;
						
						5,6,7: begin
							sram_addr_w  = origin + (count_load_conv-5) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						9, 10, 11: begin
							sram_addr_w  = origin + (count_load_conv-1) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						13, 14, 15: begin
							sram_addr_w  = origin + (count_load_conv+3) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end
					endcase
				end

				if(depth == DEPTH_16 || depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						0,1,2,3,4,8,12: filter_1[count_load_conv] = 0;
						
						5,6,7: begin
							sram_addr_w  = origin + (count_load_conv-5) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end

						9, 10, 11: begin
							sram_addr_w  = origin + (count_load_conv-1) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end

						13, 14, 15: begin
							sram_addr_w  = origin + (count_load_conv+3) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end
					endcase
				end

				if(depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						0,1,2,3,4,8,12: filter_2[count_load_conv] = 0;
						
						5,6,7: begin
							sram_addr_w  = origin + (count_load_conv-5) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end

						9, 10, 11: begin
							sram_addr_w  = origin + (count_load_conv-1) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end

						13, 14, 15: begin
							sram_addr_w  = origin + (count_load_conv+3) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end
					endcase
				end

				if(depth == DEPTH_32) begin
					case(count_load_conv)
						0,1,2,3,4,8,12: filter_3[count_load_conv] = 0;
						
						5,6,7: begin
							sram_addr_w  = origin + (count_load_conv-5) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end

						9, 10, 11: begin
							sram_addr_w  = origin + (count_load_conv-1) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end

						13, 14, 15: begin
							sram_addr_w  = origin + (count_load_conv+3) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end
					endcase
				end

				next_state = S_LOAD_CONV_TOP_LEFT;
			end
			else begin
				next_state = S_CAL_CONV;
			end
			count_load_conv_next = count_load_conv+1;
			
		end

		S_LOAD_CONV_TOP: begin
			//此時的origina相對於左上角的座標為: (0,1)
			//此處總共需要 16 個 cycles
			if(count_load_conv < 16) begin
				if(depth == DEPTH_8 || depth == DEPTH_16 || depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						0,1,2,3: filter_0[count_load_conv] = 0;
						
						4,5,6,7: begin
							sram_addr_w  = origin + (count_load_conv-4) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						8, 9, 10, 11: begin
							sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						12, 13, 14, 15: begin
							sram_addr_w  = origin + (count_load_conv+4) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end
					endcase
				end

				if(depth == DEPTH_16 || depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						0,1,2,3: filter_1[count_load_conv] = 0;
						
						4,5,6,7: begin
							sram_addr_w  = origin + (count_load_conv-4) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end

						8, 9, 10, 11: begin
							sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end

						12, 13, 14, 15: begin
							sram_addr_w  = origin + (count_load_conv+4) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end
					endcase
				end

				if(depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						0,1,2,3: filter_2[count_load_conv] = 0;
						
						4,5,6,7: begin
							sram_addr_w  = origin + (count_load_conv-4) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end

						8, 9, 10, 11: begin
							sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end

						12, 13, 14, 15: begin
							sram_addr_w  = origin + (count_load_conv+4) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end
					endcase
				end

				if(depth == DEPTH_32) begin
					case(count_load_conv)
						0,1,2,3: filter_3[count_load_conv] = 0;
						
						4,5,6,7: begin
							sram_addr_w  = origin + (count_load_conv-4) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end

						8, 9, 10, 11: begin
							sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end

						12, 13, 14, 15: begin
							sram_addr_w  = origin + (count_load_conv+4) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end
					endcase
				end
				
				next_state = S_LOAD_CONV_TOP;
			end
			else begin
				next_state = S_CAL_CONV;
			end
			count_load_conv_next = count_load_conv+1;
		end

		S_LOAD_CONV_TOP_RIGHT: begin
			//此時的origina相對於左上角的座標為: (0,1)
			//此處總共需要 16 個 cycles
			if(count_load_conv < 16) begin   //待驗證 <= ?
				if(depth == DEPTH_8 || depth == DEPTH_16 || depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						0,1,2,3,7,11,15: filter_0[count_load_conv] = 0;
						
						4,5,6: begin
							sram_addr_w  = origin + (count_load_conv-4) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						8, 9, 10: begin
							sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						12, 13, 14: begin
							sram_addr_w  = origin + (count_load_conv+4) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end
					endcase
				end

				if(depth == DEPTH_16 || depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						0,1,2,3,7,11,15: filter_0[count_load_conv] = 0;
						
						4,5,6: begin
							sram_addr_w  = origin + (count_load_conv-4) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						8, 9, 10: begin
							sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						12, 13, 14: begin
							sram_addr_w  = origin + (count_load_conv+4) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end
					endcase
				end

				if(depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						0,1,2,3,7,11,15: filter_0[count_load_conv] = 0;
						
						4,5,6: begin
							sram_addr_w  = origin + (count_load_conv-4) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						8, 9, 10: begin
							sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						12, 13, 14: begin
							sram_addr_w  = origin + (count_load_conv+4) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end
					endcase
				end

				if(depth == DEPTH_32) begin
					case(count_load_conv)
						0,1,2,3: filter_3[count_load_conv] = 0;
						
						4,5,6,7: begin
							sram_addr_w  = origin + (count_load_conv-4) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end

						8, 9, 10, 11: begin
							sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end

						12, 13, 14, 15: begin
							sram_addr_w  = origin + (count_load_conv+4) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end
					endcase
				end
				
				next_state = S_LOAD_CONV_TOP_RIGHT;
			end
			else begin
				next_state = S_CAL_CONV;
			end
			count_load_conv_next = count_load_conv+1;
			
		end

		S_LOAD_CONV_LEFT: begin
			//此時的origina相對於左上角的座標為: (1,0)
			//此處總共需要 16 個 cycles
			if(count_load_conv < 16) begin
				if(depth == DEPTH_8 || depth == DEPTH_16 || depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						0,4,8,12: filter_0[count_load_conv] = 0;
						
						1,2,3: begin
							sram_addr_w  = origin + (count_load_conv-1) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						5,6,7: begin
							sram_addr_w  = origin + (count_load_conv+3) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						9,10,11: begin
							sram_addr_w  = origin + (count_load_conv+7) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						13,14,15: begin
							sram_addr_w  = origin + (count_load_conv+11) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end
					endcase
				end

				if(depth == DEPTH_16 || depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						0,4,8,12: filter_1[count_load_conv] = 0;
						
						1,2,3: begin
							sram_addr_w  = origin + (count_load_conv-1) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end

						5,6,7: begin
							sram_addr_w  = origin + (count_load_conv+3) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end

						9,10,11: begin
							sram_addr_w  = origin + (count_load_conv+7) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end

						13,14,15: begin
							sram_addr_w  = origin + (count_load_conv+11) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end
					endcase
				end

				if(depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						0,4,8,12: filter_2[count_load_conv] = 0;
						
						1,2,3: begin
							sram_addr_w  = origin + (count_load_conv-1) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end

						5,6,7: begin
							sram_addr_w  = origin + (count_load_conv+3) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end

						9,10,11: begin
							sram_addr_w  = origin + (count_load_conv+7) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end

						13,14,15: begin
							sram_addr_w  = origin + (count_load_conv+11) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end
					endcase
				end

				if(depth == DEPTH_32) begin
					case(count_load_conv)
						0,4,8,12: filter_3[count_load_conv] = 0;
						
						1,2,3: begin
							sram_addr_w  = origin + (count_load_conv-1) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end

						5,6,7: begin
							sram_addr_w  = origin + (count_load_conv+3) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end

						9,10,11: begin
							sram_addr_w  = origin + (count_load_conv+7) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end

						13,14,15: begin
							sram_addr_w  = origin + (count_load_conv+11) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end
					endcase
				end

				next_state = S_LOAD_CONV_LEFT;
			end
			else begin
				next_state = S_CAL_CONV;
			end
			count_load_conv_next = count_load_conv+1;
			
		end

		S_LOAD_CONV_MID: begin
			//此時的origina相對於左上角的座標為: (0,0)
			//此處總共需要 16 個 cycles
			if(count_load_conv < 16) begin
				if(depth == DEPTH_8 || depth == DEPTH_16 || depth == DEPTH_24 || depth == DEPTH_32) begin
					if(count_load_conv < 4) begin
						sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
						filter_0[count_load_conv] = sram_q_0;
					end
					else if(count_load_conv < 8) begin
						sram_addr_w  = origin + (count_load_conv+8) + layer_index<<6;
						filter_0[count_load_conv] = sram_q_0;
					end
					else if(count_load_conv < 12) begin
						sram_addr_w  = origin + (count_load_conv+16) + layer_index<<6;
						filter_0[count_load_conv] = sram_q_0;
					end
					else begin
						sram_addr_w  = origin + (count_load_conv+24) + layer_index<<6;
						filter_0[count_load_conv] = sram_q_0;
					end
				end

				if(depth == DEPTH_16 || depth == DEPTH_24 || depth == DEPTH_32) begin
					if(count_load_conv < 4) begin
						sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
						filter_1[count_load_conv] = sram_q_1;
					end
					else if(count_load_conv < 8) begin
						sram_addr_w  = origin + (count_load_conv+8) + layer_index<<6;
						filter_1[count_load_conv] = sram_q_1;
					end
					else if(count_load_conv < 12) begin
						sram_addr_w  = origin + (count_load_conv+16) + layer_index<<6;
						filter_1[count_load_conv] = sram_q_1;
					end
					else begin
						sram_addr_w  = origin + (count_load_conv+24) + layer_index<<6;
						filter_1[count_load_conv] = sram_q_1;
					end
				end

				if(depth == DEPTH_24 || depth == DEPTH_32) begin
					if(count_load_conv < 4) begin
						sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
						filter_2[count_load_conv] = sram_q_2;
					end
					else if(count_load_conv < 8) begin
						sram_addr_w  = origin + (count_load_conv+8) + layer_index<<6;
						filter_2[count_load_conv] = sram_q_2;
					end
					else if(count_load_conv < 12) begin
						sram_addr_w  = origin + (count_load_conv+16) + layer_index<<6;
						filter_2[count_load_conv] = sram_q_2;
					end
					else begin
						sram_addr_w  = origin + (count_load_conv+24) + layer_index<<6;
						filter_2[count_load_conv] = sram_q_2;
					end
				end

				if(depth == DEPTH_32) begin
					if(count_load_conv < 4) begin
						sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
						filter_3[count_load_conv] = sram_q_3;
					end
					else if(count_load_conv < 8) begin
						sram_addr_w  = origin + (count_load_conv+8) + layer_index<<6;
						filter_3[count_load_conv] = sram_q_3;
					end
					else if(count_load_conv < 12) begin
						sram_addr_w  = origin + (count_load_conv+16) + layer_index<<6;
						filter_3[count_load_conv] = sram_q_3;
					end
					else begin
						sram_addr_w  = origin + (count_load_conv+24) + layer_index<<6;
						filter_3[count_load_conv] = sram_q_3;
					end
				end

				next_state = S_LOAD_CONV_MID;
			end
			else begin
				next_state = S_CAL_CONV;
			end
			count_load_conv_next = count_load_conv+1;
			
		end

		S_LOAD_CONV_RIGHT: begin
			//此時的origina相對於左上角的座標為: (0,0)
			//此處總共需要 16 個 cycles
			if(count_load_conv < 16) begin
				if(depth == DEPTH_8 || depth == DEPTH_16 || depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						3,7,11,15: filter_0[count_load_conv] = 0;
						
						0,1,2: begin
							sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						4,5,6: begin
							sram_addr_w  = origin + (count_load_conv+4) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						8,9,10: begin
							sram_addr_w  = origin + (count_load_conv+8) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						12,13,14: begin
							sram_addr_w  = origin + (count_load_conv+12) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end
					endcase
				end

				if(depth == DEPTH_16 || depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						3,7,11,15: filter_1[count_load_conv] = 0;
						
						0,1,2: begin
							sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end

						4,5,6: begin
							sram_addr_w  = origin + (count_load_conv+4) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end

						8,9,10: begin
							sram_addr_w  = origin + (count_load_conv+8) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end

						12,13,14: begin
							sram_addr_w  = origin + (count_load_conv+12) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end
					endcase
				end

				if(depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						3,7,11,15: filter_2[count_load_conv] = 0;
						
						0,1,2: begin
							sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end

						4,5,6: begin
							sram_addr_w  = origin + (count_load_conv+4) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end

						8,9,10: begin
							sram_addr_w  = origin + (count_load_conv+8) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end

						12,13,14: begin
							sram_addr_w  = origin + (count_load_conv+12) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end
					endcase
				end

				if(depth == DEPTH_32) begin
					case(count_load_conv)
						3,7,11,15: filter_3[count_load_conv] = 0;
						
						0,1,2: begin
							sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end

						4,5,6: begin
							sram_addr_w  = origin + (count_load_conv+4) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end

						8,9,10: begin
							sram_addr_w  = origin + (count_load_conv+8) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end

						12,13,14: begin
							sram_addr_w  = origin + (count_load_conv+12) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end
					endcase
				end

				next_state = S_LOAD_CONV_RIGHT;
			end
			else begin
				next_state = S_CAL_CONV;
			end
			count_load_conv_next = count_load_conv+1;
			
		end

		S_LOAD_CONV_DOWN: begin
			//此時的origina相對於左上角的座標為: (0,0)
			//此處總共需要 16 個 cycles
			if(count_load_conv < 16) begin
				if(depth == DEPTH_8 || depth == DEPTH_16 || depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						12,13,14,15: filter_0[count_load_conv] = 0;
						
						0,1,2,3: begin
							sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						4,5,6,7: begin
							sram_addr_w  = origin + (count_load_conv+4) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						8,9,10,11: begin
							sram_addr_w  = origin + (count_load_conv+8) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end
					endcase
				end

				if(depth == DEPTH_16 || depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						12,13,14,15: filter_1[count_load_conv] = 0;
						
						0,1,2,3: begin
							sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end

						4,5,6,7: begin
							sram_addr_w  = origin + (count_load_conv+4) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end

						8,9,10,11: begin
							sram_addr_w  = origin + (count_load_conv+8) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end
					endcase
				end

				if(depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						12,13,14,15: filter_2[count_load_conv] = 0;
						
						0,1,2,3: begin
							sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end

						4,5,6,7: begin
							sram_addr_w  = origin + (count_load_conv+4) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end

						8,9,10,11: begin
							sram_addr_w  = origin + (count_load_conv+8) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end
					endcase
				end

				if(depth == DEPTH_32) begin
					case(count_load_conv)
						12,13,14,15: filter_3[count_load_conv] = 0;
						
						0,1,2,3: begin
							sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end

						4,5,6,7: begin
							sram_addr_w  = origin + (count_load_conv+4) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end

						8,9,10,11: begin
							sram_addr_w  = origin + (count_load_conv+8) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end
					endcase
				end

				next_state = S_LOAD_CONV_DOWN;
			end
			else begin
				next_state = S_CAL_CONV;
			end
			count_load_conv_next = count_load_conv+1;
			
		end

		S_LOAD_CONV_DOWN_LEFT: begin
			//此時的origina相對於左上角的座標為: (0,0)
			//此處總共需要 16 個 cycles
			if(count_load_conv < 16) begin
				if(depth == DEPTH_8 || depth == DEPTH_16 || depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						0,4,8,12,13,14,15: filter_0[count_load_conv] = 0;
						
						1,2,3: begin
							sram_addr_w  = origin + (count_load_conv-1) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						5,6,7: begin
							sram_addr_w  = origin + (count_load_conv+3) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						9,10,11: begin
							sram_addr_w  = origin + (count_load_conv+7) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end
					endcase
				end

				if(depth == DEPTH_16 || depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						0,4,8,12,13,14,15: filter_1[count_load_conv] = 0;
						
						1,2,3: begin
							sram_addr_w  = origin + (count_load_conv-1) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end

						5,6,7: begin
							sram_addr_w  = origin + (count_load_conv+3) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end

						9,10,11: begin
							sram_addr_w  = origin + (count_load_conv+7) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end
					endcase
				end

				if(depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						0,4,8,12,13,14,15: filter_2[count_load_conv] = 0;
						
						1,2,3: begin
							sram_addr_w  = origin + (count_load_conv-1) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end

						5,6,7: begin
							sram_addr_w  = origin + (count_load_conv+3) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end

						9,10,11: begin
							sram_addr_w  = origin + (count_load_conv+7) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end
					endcase
				end

				if(depth == DEPTH_32) begin
					case(count_load_conv)
						0,4,8,12,13,14,15: filter_3[count_load_conv] = 0;
						
						1,2,3: begin
							sram_addr_w  = origin + (count_load_conv-1) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end

						5,6,7: begin
							sram_addr_w  = origin + (count_load_conv+3) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end

						9,10,11: begin
							sram_addr_w  = origin + (count_load_conv+7) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end
					endcase
				end

				next_state = S_LOAD_CONV_DOWN_LEFT;
			end
			else begin
				next_state = S_CAL_CONV;
			end
			count_load_conv_next = count_load_conv+1;
			
		end

		S_LOAD_CONV_DOWN_RIGHT: begin
			//此時的origina相對於左上角的座標為: (0,0)
			//此處總共需要 16 個 cycles
			if(count_load_conv < 16) begin
				if(depth == DEPTH_8 || depth == DEPTH_16 || depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						3,7,11,12,13,14,15: filter_0[count_load_conv] = 0;
						
						0,1,2: begin
							sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						4,5,6: begin
							sram_addr_w  = origin + (count_load_conv+4) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end

						8,9,10: begin
							sram_addr_w  = origin + (count_load_conv+8) + layer_index<<6;
							filter_0[count_load_conv] = sram_q_0;
						end
					endcase
				end

				if(depth == DEPTH_16 || depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						3,7,11,12,13,14,15: filter_1[count_load_conv] = 0;
						
						0,1,2: begin
							sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end

						4,5,6: begin
							sram_addr_w  = origin + (count_load_conv+4) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end

						8,9,10: begin
							sram_addr_w  = origin + (count_load_conv+8) + layer_index<<6;
							filter_1[count_load_conv] = sram_q_1;
						end
					endcase
				end

				if(depth == DEPTH_24 || depth == DEPTH_32) begin
					case(count_load_conv)
						3,7,11,12,13,14,15: filter_2[count_load_conv] = 0;
						
						0,1,2: begin
							sram_addr_w  = origin + (count_load_conv) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end

						4,5,6: begin
							sram_addr_w  = origin + (count_load_conv+4) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end

						8,9,10: begin
							sram_addr_w  = origin + (count_load_conv+8) + layer_index<<6;
							filter_2[count_load_conv] = sram_q_2;
						end
					endcase
				end

				if(depth == DEPTH_32) begin
					case(count_load_conv)
						0,4,8,12,13,14,15: filter_3[count_load_conv] = 0;
						
						1,2,3: begin
							sram_addr_w  = origin + (count_load_conv-1) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end

						5,6,7: begin
							sram_addr_w  = origin + (count_load_conv+3) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end

						9,10,11: begin
							sram_addr_w  = origin + (count_load_conv+7) + layer_index<<6;
							filter_3[count_load_conv] = sram_q_3;
						end
					endcase
				end

				next_state = S_LOAD_CONV_DOWN_RIGHT;
			end
			else begin
				next_state = S_CAL_CONV;
			end
			count_load_conv_next = count_load_conv+1;
			
		end

		S_CONV: begin
			//根據原點的位置來判斷要如何load
			if(layer_index < 8) begin
				if(origin_col==0 && origin_row==0)    //左上角
					next_state = S_LOAD_CONV_TOP_LEFT;
				else if(origin_col==7 && origin_row==0)  //右上角
					next_state = S_LOAD_CONV_TOP_RIGHT;
				else if(origin_col==7 && origin_row==7)  //右下角
					next_state = S_LOAD_CONV_DOWN_RIGHT;
				else if(origin_col==0 && origin_row==7)  //左下角
					next_state = S_LOAD_CONV_DOWN_LEFT;
				else if(origin_col==0)                //左邊
					next_state = S_LOAD_CONV_LEFT;
				else if(origin_row==0)                //上邊
					next_state = S_LOAD_CONV_TOP;
				else if(origin_col==7)                //右邊
					next_state = S_LOAD_CONV_RIGHT;
				else if(origin_row==7)                //下邊
					next_state = S_LOAD_CONV_RIGHT;
				else
					next_state = S_LOAD_CONV_MID;
			end
			else
				next_state = S_OUT_CONV;
			layer_index_next = layer_index + 1;
		end

		S_CAL_CONV: begin
			//Done in 1 cycle
			for(j=0; j<16; j+=1) begin
				if(depth == DEPTH_32) begin
					accu_0 = accu_0+(filter_0[j]+filter_1[j]+filter_2[j]+filter_3[j])*kernal_0[j];
					accu_1 = accu_1+(filter_0[j]+filter_1[j]+filter_2[j]+filter_3[j])*kernal_1[j];
					accu_2 = accu_2+(filter_0[j]+filter_1[j]+filter_2[j]+filter_3[j])*kernal_2[j];
					accu_3 = accu_3+(filter_0[j]+filter_1[j]+filter_2[j]+filter_3[j])*kernal_3[j];
				end
				else if(depth == DEPTH_24) begin
					accu_0 = accu_0+(filter_0[j]+filter_1[j]+filter_2[j])*kernal_0[j];
					accu_1 = accu_1+(filter_0[j]+filter_1[j]+filter_2[j])*kernal_1[j];
					accu_2 = accu_2+(filter_0[j]+filter_1[j]+filter_2[j])*kernal_2[j];
					accu_3 = accu_3+(filter_0[j]+filter_1[j]+filter_2[j])*kernal_3[j];
				end
				else if(depth == DEPTH_16) begin
					accu_0 = accu_0+(filter_0[j]+filter_1[j])*kernal_0[j];
					accu_1 = accu_1+(filter_0[j]+filter_1[j])*kernal_1[j];
					accu_2 = accu_2+(filter_0[j]+filter_1[j])*kernal_2[j];
					accu_3 = accu_3+(filter_0[j]+filter_1[j])*kernal_3[j];
				end
				else begin
					accu_0 = accu_0+(filter_0[j])*kernal_0[j];
					accu_1 = accu_1+(filter_0[j])*kernal_1[j];
					accu_2 = accu_2+(filter_0[j])*kernal_2[j];
					accu_3 = accu_3+(filter_0[j])*kernal_3[j];
				end
			end
			next_state = S_CONV;
		end
		

		S_OUT_CONV: begin
			o_out_valid_w	= 1;
			if(count_out_conv <4) begin
				if(count_out_conv==0) o_out_data_w = accu_0;
				else if(count_out_conv==1) o_out_data_w = accu_1;
				else if(count_out_conv==2) o_out_data_w = accu_2;
				else o_out_data_w = accu_3;
				
				next_state = S_OUT_CONV;
			end
			else
				next_state = S_IF;
			count_out_conv_next = count_out_conv +1;
		end

		S_LOAD_IMAGE: begin
			if(!i_in_valid) begin // wait until i_in_valid
				next_state = S_LOAD_IMAGE;
			end 
			else begin // load image
				//------- set image to sram -------//
				sram_CEN_0_w	= 0; 
				sram_WEN_0_w	= 0; 	// Write
				sram_CEN_1_w	= 0; 
				sram_WEN_1_w	= 0; 	// Write
				sram_CEN_2_w	= 0; 
				sram_WEN_2_w	= 0; 	// Write
				sram_CEN_3_w	= 0; 
				sram_WEN_3_w	= 0; 	// Write

				//sram_i_A_w	= count_load; // Adress
				//---------------------------------//

				if(count_load < 512) begin
					sram_addr_w = count_load;
					sram_d_0 = i_in_data;
				end
				else if(count_load < 1024) begin
					sram_addr_w = count_load - 512;
					sram_d_1 = i_in_data;
				end
				else if(count_load < 1536) begin
					sram_addr_w = count_load - 1024;
					sram_d_2 = i_in_data;
				end
				else begin
					sram_addr_w = count_load - 1536;
					sram_d_3 = i_in_data;
				end

				if(count_load == 2047) begin
					next_state = S_IF;
				end 
				else begin
					count_load_next = count_load + 1;
					next_state = S_LOAD_IMAGE;
				end
			end
		end

		S_END: begin
			next_state = S_END;
		end

		default:
	endcase
end

always@(*) begin
	op_ready_nxt = 0;
	case(state)
		S_IDLE : op_ready_nxt = 1;
		
		default: op_ready_nxt = 0;   //其他所有step做完都回到default
		
	endcase
end

// ---------------------------------------------------------------------------
// Sequential Block
// ---------------------------------------------------------------------------
// ---- Write your sequential block design here ---- //
always@(posedge i_clk or negedge i_rst_n) begin
	if(!i_rst_n) begin
		sram_addr_r		<= 0;
		sram_CEN_0_r	<= 0;
		sram_WEN_0_r	<= 0;
		sram_CEN_1_r	<= 0;
		sram_WEN_1_r	<= 0;
		sram_CEN_2_r	<= 0;
		sram_WEN_2_r	<= 0;
		sram_CEN_3_r	<= 0;
		sram_WEN_3_r	<= 0;
		//sram_i_A_w	<= 0;
		//sram_i_CEN_w	<= 0;
		//sram_i_WEN_w	<= 0;
		o_op_ready_r	<= 0;
		//o_op_ready_w	<= 0;
		o_in_ready_r	<= 0;
		//o_in_ready_w	<= 0;
		o_out_valid_r	<= 0;
		//o_out_valid_w	<= 0;

		depth		    <= DEPTH_32;
		origin			<= 0;
		count_load		<= 0;
		//count_load_next	<= 0;
		count_out		<= 0;
		count_out_conv  <= 0;
		count_load_conv <= 0;
		count_conv      <= 0;

		//count_out_next	<= 0;
		
		col_OF			<= 0;
		row_OF			<= 0;
		layer_index     <= 0;
		state		<= 0;
	end else begin
		o_op_ready_r	<= o_op_ready_w;
		o_in_ready_r	<= o_in_ready_w;
		o_out_valid_r	<= o_out_valid_w;

		sram_addr_r		<= sram_addr_w;
		sram_CEN_0_r	<= sram_CEN_0_w;
		sram_WEN_0_r	<= sram_WEN_0_w;
		sram_CEN_1_r	<= sram_CEN_1_w;
		sram_WEN_1_r	<= sram_WEN_1_w;
		sram_CEN_2_r	<= sram_CEN_2_w;
		sram_WEN_2_r	<= sram_WEN_2_w;
		sram_CEN_3_r	<= sram_CEN_3_w;
		sram_WEN_3_r	<= sram_WEN_3_w;
		
		state		    <= next_state;

		count_out		<= count_out_next;
		count_out_conv  <= count_out_conv_next;
		count_load_conv <= count_load_conv_next;
		count_conv      <= count_conv_next;
		count_load      <= count_load_next;
		depth			<= depth_next;
		origin			<= origin_next;
		layer_index     <= layer_index_next;
		
	end
end


endmodule
