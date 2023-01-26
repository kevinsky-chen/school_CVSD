`timescale 1ns/100ps
`define CYCLE       10.0     // CLK period.
`define HCYCLE      (`CYCLE/2)
`define MAX_CYCLE   10000000
`define RST_DELAY   2


`ifdef tb1
    `define INFILE "../00_TESTBED/PATTERN/indata1.dat"
    `define OPFILE "../00_TESTBED/PATTERN/opmode1.dat"
    `define GOLDEN "../00_TESTBED/PATTERN/golden1.dat"
`elsif tb2
    `define INFILE "../00_TESTBED/PATTERN/indata2.dat"
    `define OPFILE "../00_TESTBED/PATTERN/opmode2.dat"
    `define GOLDEN "../00_TESTBED/PATTERN/golden2.dat"
`elsif tb3
    `define INFILE "../00_TESTBED/PATTERN/indata3.dat"
    `define OPFILE "../00_TESTBED/PATTERN/opmode3.dat"
    `define GOLDEN "../00_TESTBED/PATTERN/golden3.dat"
`else
    `define INFILE "../00_TESTBED/PATTERN/indata0.dat"
    `define OPFILE "../00_TESTBED/PATTERN/opmode0.dat"
    `define GOLDEN "../00_TESTBED/PATTERN/golden0.dat"
`endif

`define SDFFILE "../02_SYN/Netlist/core_syn.sdf"  // Modify your sdf file name


module testbed;

reg         clk, rst_n;
wire        op_valid;
wire [ 3:0] op_mode;
wire        op_ready;
wire        in_valid;
wire [ 7:0] in_data;
wire        in_ready;
wire        out_valid;
wire [12:0] out_data;

reg  [ 7:0] indata_mem [0:2047];
reg  [ 3:0] opmode_mem [0:1023];
reg  [12:0] golden_mem [0:4095];


// ==============================================
// TODO: Declare regs and wires you need
// ==============================================

reg op_valid_pre;
reg my_op_valid;
reg [3:0] my_op_mode;
integer op_counter;

reg [7:0] my_in_data;
reg my_in_valid;
integer in_counter;

// For gate-level simulation only
`ifdef SDF
    initial $sdf_annotate(`SDFFILE, u_core);
    initial #1 $display("SDF File %s were used for this simulation.", `SDFFILE);
`endif

// Write out waveform file
initial begin
  $fsdbDumpfile("core.fsdb");
  $fsdbDumpvars(3, "+mda");
end


assign op_valid = my_op_valid;
assign op_mode = my_op_mode;
assign in_data = my_in_data;
assign in_valid = my_in_valid;

core u_core (
	.i_clk       (clk),
	.i_rst_n     (rst_n),
	.i_op_valid  (op_valid),
	.i_op_mode   (op_mode),
    .o_op_ready  (op_ready),
	.i_in_valid  (in_valid),
	.i_in_data   (in_data),
	.o_in_ready  (in_ready),
	.o_out_valid (out_valid),
	.o_out_data  (out_data)
);

// Read in test pattern and golden pattern
initial $readmemb(`INFILE, indata_mem);
initial $readmemb(`OPFILE, opmode_mem);
initial $readmemb(`GOLDEN, golden_mem);

// Clock generation
initial clk = 1'b0;
always begin #(`CYCLE/2) clk = ~clk; end

// Reset generation
initial begin
    rst_n = 1; # (               0.25 * `CYCLE);
    rst_n = 0; # ((`RST_DELAY - 0.25) * `CYCLE);
    rst_n = 1; # (         `MAX_CYCLE * `CYCLE);
    $display("Error! Runtime exceeded!");
    $finish;
end

// opmode generation

initial op_counter = 0 ;

always@(negedge clk or negedge rst_n)begin
    if (!rst_n)begin
        op_valid_pre <= 0;
        my_op_valid <= 0;
        my_op_mode <= 0;
    end else begin
        if (op_ready)begin
            op_valid_pre <= 1;
        end
        if (op_valid_pre)begin
            my_op_valid <= 1;
            my_op_mode <= opmode_mem[op_counter];
            
        end
        if (my_op_valid)begin
            my_op_valid <= 0;
            op_valid_pre <= 0;
            my_op_mode <= 0;
            op_counter <= op_counter + 1;
        end
    end
end


// input given

reg start_give_input;
initial in_counter = 0;

always@(negedge clk or negedge rst_n)begin
    if (!rst_n) begin
        start_give_input <= 0;
        my_in_data <= 0;
        my_in_valid <= 0;
    end else begin
        if (op_valid || start_give_input) begin
            if (in_counter < 2048) begin
                my_in_valid <= 1;
                my_in_data <= indata_mem[in_counter];
                in_counter <= in_counter + 1;
                start_give_input <= 1;
            end else begin
                in_counter <= in_counter;
                my_in_valid <= 0;
            end
        end
    end
end



// ==============================================
// TODO: Check pattern after process finish
// ==============================================
integer output_check_counter;
integer error_counter;
initial begin
    output_check_counter = 0;
    error_counter = 0;
end

always@(negedge clk )begin
    if (golden_mem[output_check_counter] === 13'bx && opmode_mem[op_counter] === 4'bx)begin
        $display("TOTAL ERROR : %d", error_counter);
        $display("SIMULATION FINISH");
        $finish;
    end else begin
        if (out_valid) begin
            if (golden_mem[output_check_counter] !== out_data)begin
                $display("output %d golden output : %d yours : %d FAIL!!!" , output_check_counter, golden_mem[output_check_counter], out_data);
                output_check_counter <= output_check_counter +1;
                error_counter <= error_counter + 1;
            end else begin
                $display("output %d golden output : %d SUCCESS!!!", output_check_counter,golden_mem[output_check_counter]);
                output_check_counter <= output_check_counter +1;
            end 
        end
    end
end


endmodule