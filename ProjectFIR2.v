

module HEXDecoder (input c0, c1, c2, c3, output h0, h1, h2, h3, h4, h5, h6);
	
	assign h0 = !((c0 | c1 | c2 | !c3) & (c0 | !c1 | c2 | c3) & 
						(!c0 | c1 | !c2 | !c3) & (!c0 | !c1 | c2 | !c3));
						
	assign h1 = !((c0 | !c1 | c2 | !c3) & (c0 | !c1 | !c2 | c3) & 
						(!c0 | c1 | !c2 | !c3) & (!c0 | !c1 | c2 | c3) &
						(!c0 | !c1 | !c2 | c3) & (!c0 | !c1 | !c2 | !c3));
						
	assign h2 = !((c0 | c1 | !c2 | c3) & (!c0 | !c1 | c2 | c3) & 
						(!c0 | !c1 | !c2 | c3) & (!c0 | !c1 | !c2 | !c3));
	
	assign h3 = !((c0 | c1 | c2 | !c3) & (c0 | !c1 | c2 | c3) & 
						(c0 | !c1 | !c2 | !c3) & (!c0 | c1 | c2 | !c3) &
						(!c0 | c1 | !c2 | c3) & (!c0 | !c1 | !c2 | !c3));
						
	assign h4 = !((c0 | c1 | c2 | !c3) & (c0 | c1 | !c2 | !c3) & 
						(c0 | !c1 | c2 | c3) & (c0 | !c1 | c2 | !c3) &
						(c0 | !c1 | !c2 | !c3) & (!c0 | c1 | c2 | !c3));
					
	assign h5 = !((c0 | c1 | c2 | !c3) & (c0 | c1 | !c2 | c3) & 
						(c0 | c1 | !c2 | !c3) & (c0 | !c1 | !c2 | !c3) &
						(!c0 | !c1 | c2 | !c3));
						
	assign h6 = !((c0 | c1 | c2 | c3) & (c0 | c1 | c2 | !c3) & 
						(c0 | !c1 | !c2 | !c3) & (!c0 | !c1 | c2 | c3));
						
endmodule

module HEXDisplay(input [3:0] In, output [6:0] Digit);
	
	HEXDecoder U1(
		.c0(In[3]),
		.c1(In[2]),
		.c2(In[1]),
		.c3(In[0]),
		.h0(Digit[0]),
		.h1(Digit[1]),
		.h2(Digit[2]),
		.h3(Digit[3]),
		.h4(Digit[4]),
		.h5(Digit[5]),
		.h6(Digit[6])		
	);
	
endmodule

module toneSynth(Counts, clock, Out, readValid);
	
	input clock;
	output signed [31:0] Out;
	input [18:0] Counts;
	output readValid;

	reg [17:0] Q;
	reg [11:0] enableCount;

	always@(posedge clock) begin
	
		if(enableCount == 12'd0)
			enableCount <= 12'd1042;
		else
			enableCount <= enableCount - 1;
	
		if(Q == 18'd0)
			Q <= Counts;
		else
			Q <= Q - 1;
			
	end

	assign Out = (Q > ((Counts + 1'd1) / 2'd2)? 32'd10000000: -32'd10000000);
	assign readValid = (enableCount == 12'd0)? 1'b1: 1'b0;

endmodule

module toneGenerator(SW, clock, HEX0, HEX1, Tone, readValid);

	input [2:0] SW;
	input clock;
	output [6:0] HEX0;
	output [6:0] HEX1;
	output signed [31:0] Tone;

	reg [3:0] Freq1;
	reg [3:0] Freq2;
	reg [18:0] counts;
	
	output readValid;

	always@(*) begin
		case(SW[2:0])
			3'b000:
				begin
					Freq1 = 4'd2;
					Freq2 = 4'd0;
					counts = 19'd249999;
				end
			3'b001:
				begin
					Freq1 = 4'd4;
					Freq2 = 4'd0;
					counts = 19'd124999;
				end
			3'b010:
				begin
					Freq1 = 4'd8;
					Freq2 = 4'd0;
					counts = 19'd62499;
				end
			3'b011:
				begin
					Freq1 = 4'd6;
					Freq2 = 4'd1;
					counts = 19'd31249;
				end
			3'b100:
				begin
					Freq1 = 4'd0;
					Freq2 = 4'd2;
					counts = 19'd24999;
				end
			3'b101:
				begin
					Freq1 = 4'd0;
					Freq2 = 4'd5;
					counts = 19'd9999;
				end
			3'b110:
				begin
					Freq1 = 4'd0;
					Freq2 = 4'd8;
					counts = 19'd6249;
				end
			default:
				begin
					Freq1 = 4'd2;
					Freq2 = 4'd0;
					counts = 19'd249999;
				end
		endcase 
	end
	
	// Frequency Generation
	toneSynth t0(
		.Counts(counts),
		.clock(clock),
		.Out(Tone),
		.readValid(readValid)
	);
	
	// Frequency HEX
	HEXDisplay h0(
		.In(Freq1),
		.Digit(HEX0)
	);
	
	HEXDisplay h1(
		.In(Freq2),
		.Digit(HEX1)
	);

endmodule
	
module main (CLOCK_50, KEY, SW, HEX0, HEX1, HEX2, HEX3, HEX4,
				HEX5, LEDR, AUD_ADCDAT, AUD_BCLK, AUD_ADCLRCK, 
				AUD_DACLRCK, FPGA_I2C_SDAT, AUD_XCK, AUD_DACDAT, FPGA_I2C_SCLK,
				VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_R, VGA_G, VGA_B);


/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
 
	output VGA_CLK; // VGA Clock
	output VGA_HS; // VGA H_SYNC
	output VGA_VS; // VGA V_SYNC
	output VGA_BLANK_N; // VGA BLANK
	output VGA_SYNC_N; // VGA SYNC
	output [7:0] VGA_R; // VGA Red[7:0] Changed from 10 to 8-bit DAC
	output [7:0] VGA_G; // VGA Green[7:0]
	output [7:0] VGA_B; // VGA Blue[7:0]

	// Inputs
	input	CLOCK_50;
	input	[3:0]	KEY;
	input [9:0] SW;
	output [6:0] HEX0;
	output [6:0] HEX1;
	output [6:0] HEX2;
	output [6:0] HEX3;
	output [6:0] HEX4;
	output [6:0] HEX5;
	output reg [9:0] LEDR;
	input	AUD_ADCDAT;

	// Bidirectionals
	inout	AUD_BCLK;
	inout	AUD_ADCLRCK;
	inout	AUD_DACLRCK;
	inout FPGA_I2C_SDAT;

	// Outputs
	output AUD_XCK;
	output AUD_DACDAT;
	output FPGA_I2C_SCLK;

	// Internal Wires
	wire audio_in_available;
	wire signed [31:0] left_channel_audio_in;
	wire signed [31:0] right_channel_audio_in;
	reg read_audio_in;
	wire audio_out_allowed;
	reg signed [31:0] left_channel_audio_out;
	reg signed [31:0] right_channel_audio_out;
	reg write_audio_out;

	// FIR Output available - Right and Left channels
	wire outAvailableL1;
	wire outAvailableL2;
	wire outAvailableB1;
	wire outAvailableB2;
	wire outAvailableH1;
	wire outAvailableH2;

	// FIR Output - Right and Left channels
	wire signed [31:0] right_channelLow;
	wire signed [31:0] left_channelLow;
	wire signed [31:0] right_channelBand;
	wire signed [31:0] left_channelBand;
	wire signed [31:0] right_channelHigh;
	wire signed [31:0] left_channelHigh;
	wire signed [31:0] tone;

	// FIR Input is valid
	wire readValid;

	// FIR Input - Right and Left channels
	reg signed [31:0] audioInR;
	reg signed [31:0] audioInL;

	// HEX Inputs for Passband display
	reg [3:0] FreqA1;
	reg [3:0] FreqA2;
	reg [3:0] FreqB1;
	reg [3:0] FreqB2;
	
/*****************************************************************************
 *                             Logic Circuit                            *
 *****************************************************************************/
 
	// Frequency Generator
	toneGenerator t0(
		.SW(SW[9:7]),
		.clock(CLOCK_50),
		.HEX0(HEX0),
		.HEX1(HEX1),
		.Tone(tone),
		.readValid(readValid)
	);

	// Controlling which filter is applied
	always@(*) begin
	
		// MicIn input or Tone Generator input
		if(SW[2] == 1'b1)
			begin
				audioInL <= left_channel_audio_in;
				audioInR <= right_channel_audio_in;
				read_audio_in <= audio_in_available & readValid;
				LEDR <= {10{1'b1}};
			end
		else
			begin
				audioInL <= tone;
				audioInR <= tone;
				read_audio_in <= readValid;
				LEDR <= {10{1'b0}};
			end
	
		case(SW[1:0])
			2'b00: // No filter	
				begin
					right_channel_audio_out <= audioInR;
					left_channel_audio_out <= audioInL;
					write_audio_out <= audio_out_allowed & audio_in_available;
					FreqA1 = 4'd0;
					FreqA2 = 4'd0;
					FreqB1 = 4'd0;
					FreqB2 = 4'd0;
				end
			2'b01: // Low Pass
				begin
					right_channel_audio_out <= right_channelLow;
					left_channel_audio_out <= left_channelLow;
					write_audio_out <= audio_out_allowed && outAvailableL1 && outAvailableL2;
					FreqA1 = 4'd0;
					FreqA2 = 4'd0;
					FreqB1 = 4'd1;
					FreqB2 = 4'd0;
				end
			2'b10: // Band Pass
				begin
					right_channel_audio_out <= right_channelBand;
					left_channel_audio_out <= left_channelBand;
					write_audio_out <= audio_out_allowed && outAvailableB1 && outAvailableB2;
					FreqA1 = 4'd1;
					FreqA2 = 4'd0;
					FreqB1 = 4'd3;
					FreqB2 = 4'd0;
				end
			2'b11: // High Pass
				begin
					right_channel_audio_out <= right_channelHigh;
					left_channel_audio_out <= left_channelHigh;
					write_audio_out <= audio_out_allowed && outAvailableH1 && outAvailableH2;
					FreqA1 = 4'd3;
					FreqA2 = 4'd0;
					FreqB1 = 4'd4;
					FreqB2 = 4'd2;
				end
		endcase
	end

	LowFIR500 LP0(
		.clk(CLOCK_50), // clk.clk
		.reset_n(KEY[0]), // rst.reset_n
		.ast_sink_data(audioInR>>>14), // avalon_streaming_sink.data
		.ast_sink_valid(read_audio_in), // .valid
		.ast_sink_error(2'b00), // .error
		.ast_source_data(right_channelLow), // avalon_streaming_source.data
		.ast_source_valid(outAvailableL1), // .valid
		.ast_source_error() // .error
	);

	LowFIR500 LP1(
		.clk(CLOCK_50), // clk.clk
		.reset_n(KEY[0]), // rst.reset_n
		.ast_sink_data(audioInL>>>14), // avalon_streaming_sink.data
		.ast_sink_valid(read_audio_in), // .valid
		.ast_sink_error(2'b00), // .error
		.ast_source_data(left_channelLow), // avalon_streaming_source.data
		.ast_source_valid(outAvailableL2), // .valid
		.ast_source_error() //  .error
	);
	
	BandFIR5503000 BP0(
		.clk(CLOCK_50), // clk.clk
		.reset_n(KEY[0]), // rst.reset_n
		.ast_sink_data(audioInR>>>10), // avalon_streaming_sink.data
		.ast_sink_valid(read_audio_in), // .valid
		.ast_sink_error(2'b00), // .error
		.ast_source_data(right_channelBand), // avalon_streaming_source.data
		.ast_source_valid(outAvailableB1), // .valid
		.ast_source_error() // .error
	);

	BandFIR5503000 BP1(
		.clk(CLOCK_50), // clk.clk
		.reset_n(KEY[0]), // rst.reset_n
		.ast_sink_data(audioInL>>>10), // avalon_streaming_sink.data
		.ast_sink_valid(read_audio_in), // .valid
		.ast_sink_error(2'b00), // .error
		.ast_source_data(left_channelBand), // avalon_streaming_source.data
		.ast_source_valid(outAvailableB2), // .valid
		.ast_source_error() //  .error
	);

	HighFIR HP0(
		.clk(CLOCK_50), // clk.clk
		.reset_n(KEY[0]), // rst.reset_n
		.ast_sink_data(audioInR>>>6), // avalon_streaming_sink.data
		.ast_sink_valid(read_audio_in), // .valid
		.ast_sink_error(2'b00), // .error
		.ast_source_data(right_channelHigh), // avalon_streaming_source.data
		.ast_source_valid(outAvailableH1), // .valid
		.ast_source_error() // .error
	);

	HighFIR HP1(
		.clk(CLOCK_50), // clk.clk
		.reset_n(KEY[0]), // rst.reset_n
		.ast_sink_data(audioInL>>>6), // avalon_streaming_sink.data
		.ast_sink_valid(read_audio_in), // .valid
		.ast_sink_error(2'b00), // .error
		.ast_source_data(left_channelHigh), // avalon_streaming_source.data
		.ast_source_valid(outAvailableH2), // .valid
		.ast_source_error() //  .error
	);
	
	HEXDisplay h2(
		.In(FreqA1),
		.Digit(HEX2)
	);
	
	HEXDisplay h3(
		.In(FreqA2),
		.Digit(HEX3)
	);
	
	HEXDisplay h4(
		.In(FreqB1),
		.Digit(HEX4)
	);
	
	HEXDisplay h5(
		.In(FreqB2),
		.Digit(HEX5)
	);


/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

	Audio_Controller Audio_Controller (
		// Inputs
		.CLOCK_50(CLOCK_50),
		.reset(~KEY[0]),
		.clear_audio_in_memory(),
		.read_audio_in(read_audio_in),
		.clear_audio_out_memory(),
		.left_channel_audio_out(left_channel_audio_out),
		.right_channel_audio_out(right_channel_audio_out),
		.write_audio_out(write_audio_out),
		.AUD_ADCDAT(AUD_ADCDAT),
		// Bidirectionals
		.AUD_BCLK(AUD_BCLK),
		.AUD_ADCLRCK(AUD_ADCLRCK),
		.AUD_DACLRCK(AUD_DACLRCK),
		// Outputs
		.audio_in_available(audio_in_available),
		.left_channel_audio_in(left_channel_audio_in),
		.right_channel_audio_in(right_channel_audio_in),
		.audio_out_allowed(audio_out_allowed),
		.AUD_XCK(AUD_XCK),
		.AUD_DACDAT(AUD_DACDAT)
	);

	avconf #(.USE_MIC_INPUT(0)) avc ( 
		.FPGA_I2C_SCLK(FPGA_I2C_SCLK),
		.FPGA_I2C_SDAT(FPGA_I2C_SDAT),
		.CLOCK_50(CLOCK_50),
		.reset(~KEY[0])
	);
	
	
/*****************************************************************************
 *                              VGA Adapter                            *
 *****************************************************************************/

 	// VGA Monitor to display Audio Equaliser controls

	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;

	vga_adapter VGA(
			.resetn(KEY[0]),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(writeEn),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "320x240";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 8;
		defparam VGA.BACKGROUND_IMAGE = "background.mif";

endmodule