
//// made changes from key 3 to the space bar , kinda scuffed you need to restart using key 0 , weird 
//// when the hex gets to double digits it turns into letters
//// drawing is kinda funky , will be fixed with the background
//module vga_demo(CLOCK_50, SW, KEY, VGA_R, VGA_G, VGA_B,
//				VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK, PS2_CLK, PS2_DAT, HEX0, AUD_ADCDAT,AUD_BCLK,AUD_ADCLRCK,AUD_DACLRCK, AUD_XCK,AUD_DACDAT ,FPGA_I2C_SDAT, FPGA_I2C_SCLK );
//	
//    parameter A = 4'b0000, B = 4'b0001, C = 4'b0010, D = 4'b0011; 
//    parameter E = 4'b0100, F = 4'b0101, G = 4'b0110, H = 4'b0111;
//	 parameter I = 4'b1000, J = 4'b1001, K = 4'b1010, L = 4'b1011;
//	 parameter M = 4'b1100, N = 4'b1101, O = 4'b1110, P = 4'b1111;
//	 //parameter SQUARE_SIZE = 16;
//	 parameter SQUARE_SIZE = 10;
//    parameter XSCREEN = 160, YSCREEN = 120;
//    parameter XDIM = XSCREEN>>1, YDIM = 1;
//	 
//	 parameter PILLARXDIM = 13, PILLARYDIM = 84; // Adjust dimensions for a vertical line
//	 
//	 parameter pillarX0 = 8'd159;  // Start at rightmost position
//	 reg [6:0] pillarY0 = 7'd59;
//	 
//	 wire [6:0] random_pillarY;
//	 LFSR lfsr_inst (
//    .Clock(CLOCK_50),
//    .Resetn(KEY[0]),
//    .random(random_pillarY)
//		);
//	 
//    parameter birdX0 = 8'd19, birdY0 = 7'd59;
//    parameter Kslow = 20;//animation speed: use 20 for hardware, 2 for ModelSim
//
//	input CLOCK_50;	
//	input [9:0] SW;
//	input [3:0] KEY;
//	output [7:0] VGA_R;
//	output [7:0] VGA_G;
//	output [7:0] VGA_B;
//	output VGA_HS;
//	output VGA_VS;
//	output VGA_BLANK_N;
//	output VGA_SYNC_N;
//	output VGA_CLK;	
//
//	reg [7:0] VGA_X; 
//	reg [6:0] VGA_Y;  
//	reg [2:0] VGA_COLOR;
//	
//    reg plot;
//    
//	wire [2:0] birdcolour;
//	wire [7:0] birdX;
//	wire [6:0] birdY;
//    wire [7:0] birdXC;
//    wire [6:0] birdYC;
//	 
//    wire [Kslow-1:0] slow;
//	 
//    wire go, sync;
//	 
//    reg birdLy, birdEy, birdLxc, birdLyc, birdExc, birdEyc;
//    wire birdYdir;
//    reg birdTdir;
//	 
//    reg [3:0] y_Q, Y_D;
//	 
//	 wire toggleY;
//	 assign toggleY = SW[9];
//	
//	assign birdcolour = 3'b110;
//	
//	wire [7:0] pillarX;
//	wire [6:0] pillarY;
//	wire [7:0] pillarXC;
//	wire [6:0] pillarYC;
//	wire freeze;
//	
//	// audio 
//	
//	input				AUD_ADCDAT;
//	// Bidirectionals
//inout				AUD_BCLK;
//inout				AUD_ADCLRCK;
//inout				AUD_DACLRCK;
//
//inout				FPGA_I2C_SDAT;
//
//// Outputs
//output				AUD_XCK;
//output				AUD_DACDAT;
//
//output				FPGA_I2C_SCLK;
//	
//	
//	    //score logic///////
//    output [6:0] HEX0; // Add output for score display
//    
//    reg [3:0] score; // 4-bit register to hold score (0-9)
//	 reg play_sound;
//  //  wire score_point; // Wire to indicate when a point is scored
//    
////    // Score point detection logic
////    // Score when bird's X position is equal to pillar's X position + pillar width
////    assign score_point = (birdX == pillarX + PILLARXDIM) && 
////                        (birdY > (pillarY - SQUARE_SIZE)) && 
////                        (birdY < (pillarY + PILLARYDIM));
////								
////								
////	 if (birdX == (pillarY0 + PILLARYDIM))
//	 
//    
//    // Score counter logic
////    always @(posedge CLOCK_50) begin
////        if (y_Q == A)
////            score <= 4'b0000;
////        else if (birdX == (pillarX + PILLARXDIM)) // Only increment if less than 9
////            score <= score + 1'b1;
////    end
////    
//////Instantiate hex display for score
////  hex7seg score_display(
////      .hex(score),
////      .display(HEX0)
//// );
////
////
//// 
//
//DE1_SoC_Audio_Example speaker(
//	// Inputs
//	CLOCK_50,
//	KEY,
//
//	AUD_ADCDAT,
//
//	// Bidirectionals
//	AUD_BCLK,
//	AUD_ADCLRCK,
//	AUD_DACLRCK,
//
//	FPGA_I2C_SDAT,
//
//	// Outputs
//	AUD_XCK,
//	AUD_DACDAT,
//
//	FPGA_I2C_SCLK,
//	play_sound,
////	SW
//);
//
//
//reg score_updated; // Flag to prevent multiple score increments
//
//always @(posedge CLOCK_50) begin
//    if (!KEY[0]) begin
//        score <= 4'b0000;
//        score_updated <= 1'b0;
//		  play_sound <= 0;
//    end
//    else begin
//        // Check if bird has passed the pillar's x-position and is within a valid vertical range
//        if (birdX == (pillarX + PILLARXDIM) && 
//            !score_updated && 
//            ((birdY > (PILLARYDIM - (YSCREEN - pillarY0))) || 
//             (birdY + SQUARE_SIZE < pillarY0))) begin
//            score <= score + 1'b1;
//				play_sound <= 1;
//            score_updated <= 1'b1;
//        end
//        
//        // Reset the flag when bird moves past the pillar
//        if (birdX > (pillarX + PILLARXDIM)) begin
//            score_updated <= 1'b0;
//				play_sound <= 0;
//        end
//    end
//end
//
//// Uncomment the hex display to show the score
//hex7seg score_display(
//    .hex(score),
//    .display(HEX0)
//);
//
//	// KEYBOARD 
//	inout PS2_CLK;
//   inout PS2_DAT;
//	wire EY;
//	reg Tdir;
//	
//	// KEYBOARD ASSIGN 
//	assign start = (ps2_key_data == 8'h29); // space bar
//	assign toggleYup = (ps2_key_data == 8'hE075);// up A key
//	assign toggleYdown = (ps2_key_data == 8'hE072);// Down Z key 
//
//	assign Ydir = (!toggleYup) ? 1'b1 :
//                  (!toggleYdown) ? 1'b0:  
//                  Ydir;
//	wire [7:0] ps2_key_data;
//	wire ps2_key_pressed;
//
//	reg pillarLx, pillarEx, pillarLy, pillarEy, pillarLxc, pillarLyc, pillarExc, pillarEyc;
//	wire pillarXdir;
//	reg wrap_reset;      
//
//	reg [2:0] x_Q, X_D;
//
//    // Add a game reset signal
//    wire game_reset;
//    assign game_reset = ~KEY[1]; // Active low reset
//
//    // Instantiate modules with game reset consideration
//    UpDn_count U1 (
//        .R(birdY0), 
//        .Clock(CLOCK_50), 
//        .Resetn(KEY[0] & ~game_reset), 
//        .E(birdEy), 
//        .L(game_reset), 
//        .UpDn(Ydir), 
//        .Q(birdY)
//    );
//    defparam U1.n = 7;
//
//    regn U2 (
//        .R(birdX0), 
//        .Resetn(KEY[0] & ~game_reset), 
//        .E(1'b1), 
//        .Clock(CLOCK_50), 
//        .Q(birdX)
//    );
//    defparam U2.n = 8;
//
//    // Reset X and Y counters with game reset
//    UpDn_count U3 (
//        .R(8'd0), 
//        .Clock(CLOCK_50), 
//        .Resetn(KEY[0] & ~game_reset), 
//        .E(birdExc), 
//        .L(birdLxc), 
//        .UpDn(1'b1), 
//        .Q(birdXC)
//    );
//    defparam U3.n = 8;
//    
//    UpDn_count U4 (
//        .R(7'd0), 
//        .Clock(CLOCK_50), 
//        .Resetn(KEY[0] & ~game_reset), 
//        .E(birdEyc), 
//        .L(birdLyc), 
//        .UpDn(1'b1), 
//        .Q(birdYC)
//    );
//    defparam U4.n = 7;
//
//		  
//		  
//    UpDn_count U5 ({Kslow{1'b0}}, CLOCK_50, KEY[0], 1'b1, 1'b0, 1'b1, slow);
//        defparam U5.n = Kslow;
//		  
//		  
//    assign sync = (slow == 0);
//
////    ToggleFFbird U6 (toggleY, KEY[0], CLOCK_50,birdYdir); removed so we can add keyboard 
////ADDED FOR KEYBOARD 
//
//    ToggleFF U6_up(.T(toggleYup), .Resetn(KEY[0] & ~game_reset), .Clock(CLOCK_50), .Q(Ydir_up));
//    ToggleFF U6_down(.T(toggleYdown), .Resetn(KEY[0] & ~game_reset), .Clock(CLOCK_50), .Q(Ydir_down));
//
//    // Pillar counter with game reset
//    UpDn_count U7 (
//        .R(pillarX0), 
//        .Clock(CLOCK_50), 
//        .Resetn(KEY[0] & ~game_reset), 
//        .E(pillarEx), 
//        .L(game_reset | wrap_reset), 
//        .UpDn(1'b0), 
//        .Q(pillarX)
//    );
//    defparam U7.n = 8;
//
//    // Pillar Y position reset
//    regn U8 (
//        .R(pillarY0), 
//        .Resetn(KEY[0] & ~game_reset), 
//        .E(1'b1), 
//        .Clock(CLOCK_50), 
//        .Q(pillarY)
//    );
//    defparam U8.n = 7;
//
//	UpDn_count U9 (8'd0, CLOCK_50, 1'b1, pillarExc, pillarLxc, 1'b1, pillarXC);
//		defparam U9.n = 8;
//	UpDn_count U10 (7'd0, CLOCK_50, 1'b1, pillarEyc, pillarLyc, 1'b1, pillarYC);
//		defparam U10.n = 7;
////
////	UpDn_count U11 ({Kslow{1'b0}}, CLOCK_50, 1'b1, 1'b1, 1'b0, 1'b1, slow); // & ~freeze
////		defparam U11.n = Kslow;
//
//
//// bird from rom 
//
////assign bird_rom_address = birdYC * 16 + birdXC;
//assign bird_rom_address = birdYC * 10 + birdXC;
//
//// Combine coordinates
//// Wires for ROM interaction
//wire [9:0] bird_rom_address;  // ROM address (10 bits)
//wire [2:0] bird_pixel_color; // Output color from ROM
//
//// Instantiate the ROM
//birdy_rom bird_image_rom (
//    .address(bird_rom_address), // Input address
//    .clock(CLOCK_50),           // Input clock
//    .q(bird_pixel_color)        // Output color
//);
//
//
//    // FSM state table
//    always @ (*)
//        case (y_Q)
//            A:  if (!go || !sync) Y_D = A;
//                else Y_D = B;
//            B:  if (birdXC != SQUARE_SIZE-1) Y_D = B;    // draw bird
//                else Y_D = C;
//            C:  if (birdYC != SQUARE_SIZE-1) Y_D = B;
//                else Y_D = D;
//				D:  if (pillarYC != PILLARYDIM-1) Y_D = D; // draw pillar
//					 else Y_D = E;
//				E:  if (pillarXC != PILLARXDIM-1) Y_D = D; // draw pillar
//					 else Y_D = F;
//				F: if (!sync) Y_D = F; //sync waiting
//                else Y_D = G;
//				G:  if (birdXC != SQUARE_SIZE-1) Y_D = G;    // erase bird
//                else Y_D = H;
//            H:  if (birdYC != SQUARE_SIZE-1) Y_D = G; 
//                else Y_D = I;
//				I: if (pillarYC != PILLARYDIM-1) Y_D = I; // erase //E
//					else Y_D = J;
//				J: if (pillarXC != PILLARXDIM-1) Y_D = I; //F
//					else Y_D = K;
//				K: Y_D = L;
//L: if ((birdY == 7'd0) || (birdY == YSCREEN- 9) || ( ((birdX + SQUARE_SIZE) > pillarX && (birdX < (pillarX + PILLARXDIM))) && !( ((birdY) > (PILLARYDIM - (YSCREEN - pillarY0) )) && ((birdY + SQUARE_SIZE) < pillarY0)) )) Y_D = A;		
//		else Y_D = B;
//				
//        endcase
//    // FSM outputs
//    always @ (*)
//    begin
//        // default assignments
//        birdLxc = 1'b0; birdLyc = 1'b0; birdExc = 1'b0; birdEyc = 1'b0; VGA_COLOR = 3'b110; plot = 1'b0;
//        birdEy = 1'b0; birdTdir = 1'b0; 
//		  pillarLxc = 1'b0; pillarLyc = 1'b0; pillarExc = 1'b0; pillarEyc = 1'b0;
//		  pillarEx = 1'b0; wrap_reset = 1'b0;
//        case (y_Q)
//            A:  begin birdLxc = 1'b1; birdLyc = 1'b1; pillarLxc = 1'b1; pillarLyc = 1'b1; end
//            //B:  begin birdExc = 1'b1; plot = 1'b1; VGA_X = birdX + birdXC; VGA_Y = birdY + birdYC;  VGA_COLOR = 3'b110; end   // color a pixel //plot pixel
//				B: begin
//    birdExc = 1'b1;  // Enable X counter for bird
//    plot = 1'b1;     // Signal to plot pixel
//    VGA_X = birdX + birdXC; // X coordinate on VGA screen
//    VGA_Y = birdY + birdYC; // Y coordinate on VGA screen
//    VGA_COLOR = bird_pixel_color; // Color fetched from ROM
//end
//
//            C:  begin birdLxc = 1'b1; birdEyc = 1'b1; end
//				D: begin pillarEyc = 1'b1; plot = 1'b1; VGA_X = pillarX + pillarXC; VGA_Y = pillarY + pillarYC;  VGA_COLOR = 3'b010; end // Draw a pixel
//				E: begin pillarLyc = 1'b1; pillarExc = 1'b1; end
//				F: begin birdLyc = 1'b1; pillarLxc = 1'b1; end
//				G: begin birdExc = 1'b1; plot = 1'b1; VGA_X = birdX + birdXC; VGA_Y = birdY + birdYC; VGA_COLOR = 3'b011; end   // color a pixel ------  VGA_COLOR = ALT;
//            H: begin birdLxc = 1'b1; birdEyc = 1'b1; end
//				I: begin pillarEyc = 1'b1; plot = 1'b1; VGA_X = pillarX + pillarXC; VGA_Y = pillarY + pillarYC; VGA_COLOR = 3'b011; end // Erase a pixel
//				J: begin pillarLyc = 1'b1; pillarExc = 1'b1; end
//				K: begin
//					 birdLyc = 1'b1;
//					 birdTdir = (birdY == 7'd0) || (birdY == YSCREEN-9);
//					 pillarLxc = 1'b1;
//					 if (pillarX == 8'd0 || game_reset) begin
//                      wrap_reset = 1'b1; // Reset X position to rightmost
//                 end
//                 if (wrap_reset || game_reset) begin
//                      pillarY0 <= random_pillarY; // Update pillarY0 dynamically
//                 end
//               end
//
//				L: begin birdEy = 1'b1; pillarEx = 1'b1; end
//        endcase
//    end
//
//    always @(posedge CLOCK_50)
//        if (!KEY[0])
//            y_Q <= 1'b0;
//        else
//            y_Q <= Y_D;
//
//    //assign go = ~KEY[3];
//	 assign go = start;
//	 
//	
//
//    // connect to VGA controller
//    vga_adapter VGA (
//			.resetn(KEY[0]),
//			.clock(CLOCK_50),
//			.colour(VGA_COLOR),
//			.x(VGA_X),
//			.y(VGA_Y),
//			.plot(plot),
//			.VGA_R(VGA_R),
//			.VGA_G(VGA_G),
//			.VGA_B(VGA_B),
//			.VGA_HS(VGA_HS),
//			.VGA_VS(VGA_VS),
//			.VGA_BLANK_N(VGA_BLANK_N),
//			.VGA_SYNC_N(VGA_SYNC_N),
//			.VGA_CLK(VGA_CLK));
//		defparam VGA.RESOLUTION = "160x120";
//		defparam VGA.MONOCHROME = "FALSE";
//		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
//		defparam VGA.BACKGROUND_IMAGE = "newbg.mif"; 
//		
// PS2_Controller PS2(
// .CLOCK_50(CLOCK_50), .reset(~KEY[0]), .PS2_CLK(PS2_CLK),
//                     .PS2_DAT(PS2_DAT), .received_data(ps2_key_data),
//                     .received_data_en(ps2_key_pressed));
//							
//
//		
//endmodule
//
//module regn(R, Resetn, E, Clock, Q);
//    parameter n = 8;
//    input [n-1:0] R;
//    input Resetn, E, Clock;
//    output reg [n-1:0] Q;
//
//    always @(posedge Clock)
//        if (!Resetn)
//            Q <= 0;
//        else if (E)
//            Q <= R;
//endmodule
//
////BIRD TOGGLE
//module ToggleFFbird(T, Resetn, Clock, Q);
//    input T, Resetn, Clock;
//    output reg Q;
//
//    always @(posedge Clock)
//        if (!Resetn)
//            Q <= 0;
//        else if (T)
//            Q <= 0;
//		 else if (~T)
//			Q <= 1;
//			//Q<= 2;
//endmodule
//
//module UpDn_count (R, Clock, Resetn, E, L, UpDn, Q);
//    parameter n = 8;
//    input [n-1:0] R;
//    input Clock, Resetn, E, L, UpDn;
//    output reg [n-1:0] Q;
//
//    always @ (posedge Clock)
//        if (Resetn == 0)
//            Q <= 0;
//        else if (L == 1)
//            Q <= R;
//        else if (E)
//            if (UpDn == 1)
//                Q <= Q + 1;
//            else
//                Q <= Q - 1;
//endmodule
//
//
//module ToggleFF(T, Resetn, Clock, Q);
//    input T, Resetn, Clock;
//    output reg Q;
//
//    always @(posedge Clock)
//        if (!Resetn)
//            Q <= 0;
//        else if (T)
//            Q <= ~Q;
//endmodule
//
//
//module LFSR (
//    input Clock,
//    input Resetn,
//    output reg [6:0] random // Random 7-bit value for pillarY0 (0 to 119)
//);
//    reg [7:0] shift_reg; // 8-bit shift register for LFSR
//
//    always @(posedge Clock  or negedge Resetn) begin
//        if (!Resetn)
//            shift_reg <= 8'b00000001; // Initialize with a non-zero seed
//        else
//            shift_reg <= {shift_reg[6:0], shift_reg[7] ^ shift_reg[5] ^ shift_reg[4] ^ shift_reg[3]}; // Feedback
//    end
//
//    // Limit the random value to the screen's vertical range (0 to 119)
//    always @(*) begin
//        random = 62 + (shift_reg[6:0] % 57); // Modulo to fit the screen's range
//    end
//endmodule
//
//
//
//
//module hex7seg (hex, display);
//    input [3:0] hex;
//    output [6:0] display;
//
//    reg [6:0] display;
//
//    /*
//     *       0  
//     *      ---  
//     *     |   |
//     *    5|   |1
//     *     | 6 |
//     *      ---  
//     *     |   |
//     *    4|   |2
//     *     |   |
//     *      ---  
//     *       3  
//     */
//    always @ (hex)
//        case (hex)
//            4'h0: display = 7'b1000000;
//            4'h1: display = 7'b1111001;
//            4'h2: display = 7'b0100100;
//            4'h3: display = 7'b0110000;
//            4'h4: display = 7'b0011001;
//            4'h5: display = 7'b0010010;
//            4'h6: display = 7'b0000010;
//            4'h7: display = 7'b1111000;
//            4'h8: display = 7'b0000000;
//            4'h9: display = 7'b0011000;
//            4'hA: display = 7'b0001000;
//            4'hB: display = 7'b0000011;
//            4'hC: display = 7'b1000110;
//            4'hD: display = 7'b0100001;
//            4'hE: display = 7'b0000110;
//            4'hF: display = 7'b0001110;
//        endcase
//endmodule
//
//module hexDisplay(X, display); //4 bit
//	input [3:0] X;
//	output [6:0] display;
//	
//	assign display[0] = ~((~X[2] & ~X[0]) | (X[1] & ~X[0]) | (X[3] & ~X[1] & ~X[0]) | (X[3] & ~X[2] & ~X[1]) | (X[2] & X[1]) | (~X[3] & X[2] & X[0]) | (X[1] & ~X[3]));
//	assign display[1] = ~((~X[3] & ~X[2]) | (~X[2] & ~X[0]) | (~X[1] & ~X[0] & ~X[3]) | (X[1] & X[0] & ~X[3]) | (~X[1] & X[0] & X[3]));
//	assign display[2] = ~((~X[3] & ~X[1]) | (~X[3] & X[0]) | (~X[3] & X[2]) | (~X[1] & X[0]) | (X[3] & ~X[2]));
//	assign display[3] = ~((~X[3] & ~X[2] & ~X[1] & ~X[0]) | (X[3] & ~X[1]) | (~X[1] & X[0] & X[2]) | (~X[3] & ~X[2] & X[1]) | (X[1] & ~X[0] & X[2]) | (X[3] & ~X[2] & X[0]));
//	assign display[4] = ~((X[3] & X[2]) | (X[1] & ~X[0]) | (~X[2] & ~X[0]) | (X[3] & ~X[2] & X[1]));
//	assign display[5]= ~((~X[1] & ~X[0]) | (~X[3] & X[2] & ~X[1] & X[0]) | (X[3] & ~X[2]) | (X[3] & X[1]) | (X[2] & X[1] & ~X[0]));
//	assign display[6] = ~((~X[3] & X[2] & ~X[1]) | (X[3] & ~X[2]) | (X[3] & X[0]) | (X[1] & ~X[0]) | (~X[3] & ~X[2] & X[1]));
//	
//	
//endmodule 
//



// made changes from key 3 to the space bar , kinda scuffed you need to restart using key 0 , weird 
// when the hex gets to double digits it turns into letters
// drawing is kinda funky , will be fixed with the background
module vga_demo(CLOCK_50, SW, KEY, VGA_R, VGA_G, VGA_B,
				VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK, PS2_CLK, PS2_DAT, HEX0, AUD_ADCDAT,AUD_BCLK,AUD_ADCLRCK,AUD_DACLRCK, AUD_XCK,AUD_DACDAT ,FPGA_I2C_SDAT, FPGA_I2C_SCLK );
	
    parameter A = 5'b00000, B = 5'b00001, C = 5'b00010, D = 5'b00011; 
    parameter E = 5'b00100, F = 5'b00101, G = 5'b00110, H = 5'b00111;
	 parameter I = 5'b01000, J = 5'b01001, K = 5'b01010, L = 5'b01011;
	 parameter M = 5'b01100, N = 5'b01101, O = 5'b01110, P = 5'b01111;
	 parameter Q = 5'b10000, R = 5'b11000;
	 //parameter SQUARE_SIZE = 16;
	 parameter SQUARE_SIZE = 10;
    parameter XSCREEN = 160, YSCREEN = 120;
    parameter XDIM = XSCREEN>>1, YDIM = 1;
	 
	 parameter PILLARXDIM = 13, PILLARYDIM = 84; // Adjust dimensions for a vertical line
	 
	 parameter pillarX0 = 8'd159;  // Start at rightmost position
	 reg [6:0] pillarY0 = 7'd59;
	 
	 wire [6:0] random_pillarY;
	 LFSR lfsr_inst (
    .Clock(CLOCK_50),
    .Resetn(KEY[0]),
    .random(random_pillarY)
		);
	 
    parameter birdX0 = 8'd19, birdY0 = 7'd59;
    parameter Kslow = 20;//animation speed: use 20 for hardware, 2 for ModelSim

	input CLOCK_50;	
	input [9:0] SW;
	input [3:0] KEY;
	output [7:0] VGA_R;
	output [7:0] VGA_G;
	output [7:0] VGA_B;
	output VGA_HS;
	output VGA_VS;
	output VGA_BLANK_N;
	output VGA_SYNC_N;
	output VGA_CLK;	

	reg [7:0] VGA_X; 
	reg [6:0] VGA_Y;  
	reg [2:0] VGA_COLOR;
	
    reg plot;
    
	wire [2:0] birdcolour;
	wire [7:0] birdX;
	wire [6:0] birdY;
    wire [7:0] birdXC;
    wire [6:0] birdYC;
	 
    wire [Kslow-1:0] slow;
	 
    wire go, sync;
	 
    reg birdLy, birdEy, birdLxc, birdLyc, birdExc, birdEyc;
    wire birdYdir;
    reg birdTdir;
	 
    reg [4:0] y_Q, Y_D;
	 
	 wire toggleY;
	 assign toggleY = SW[9];
	
	assign birdcolour = 3'b110;
	
	wire [7:0] pillarX;
	wire [6:0] pillarY;
	wire [7:0] pillarXC;
	wire [6:0] pillarYC;
	wire freeze;
	
		//---------------endscreen test---------
	wire [7:0] endScreenX;
	wire [6:0] endScreenY;
	wire [7:0] endScreenXC;
	wire [6:0] endScreenYC;
	
	parameter endScreenX0 = 8'd0;  // Start at rightmost position
	reg [6:0] endScreenY0 = 7'd0;
	
	reg endScreenLx, endScreenEx, endScreenLy, endScreenEy, endScreenLxc, endScreenLyc, endScreenExc, endScreenEyc;
	
	
	    UpDn_count E7 (
        .R(endScreenX0), 
        .Clock(CLOCK_50), 
        .Resetn(KEY[0] & ~game_reset), 
        .E(endScreenEx), 
        .L(game_reset | wrap_reset), 
        .UpDn(1'b0), 
        .Q(endScreenX)
    );
    defparam E7.n = 8;

    // Pillar Y position reset
    regn E8 (
        .R(endScreenY0), 
        .Resetn(KEY[0] & ~game_reset), 
        .E(1'b1), 
        .Clock(CLOCK_50), 
        .Q(endScreenY)
    );
    defparam E8.n = 7;

	UpDn_count E9 (8'd0, CLOCK_50, 1'b1, endScreenExc, endScreenLxc, 1'b1, endScreenXC);
		defparam E9.n = 8;
	UpDn_count E10 (7'd0, CLOCK_50, 1'b1, endScreenEyc, endScreenLyc, 1'b1, endScreenYC);
		defparam E10.n = 7;
	
	
	///\--------------------------------------------
	
	// audio 
	
	input				AUD_ADCDAT;
	// Bidirectionals
inout				AUD_BCLK;
inout				AUD_ADCLRCK;
inout				AUD_DACLRCK;

inout				FPGA_I2C_SDAT;

// Outputs
output				AUD_XCK;
output				AUD_DACDAT;

output				FPGA_I2C_SCLK;
	
	
	    //score logic///////
    output [6:0] HEX0; // Add output for score display
    
    reg [3:0] score; // 4-bit register to hold score (0-9)
	 reg play_sound;


DE1_SoC_Audio_Example speaker(
	// Inputs
	CLOCK_50,
	KEY,

	AUD_ADCDAT,

	// Bidirectionals
	AUD_BCLK,
	AUD_ADCLRCK,
	AUD_DACLRCK,

	FPGA_I2C_SDAT,

	// Outputs
	AUD_XCK,
	AUD_DACDAT,

	FPGA_I2C_SCLK,
	play_sound,
//	SW
);


reg score_updated; // Flag to prevent multiple score increments

always @(posedge CLOCK_50) begin
    if (!KEY[0]) begin
        score <= 4'b0000;
        score_updated <= 1'b0;
		  play_sound <= 0;
    end
    else begin
        // Check if bird has passed the pillar's x-position and is within a valid vertical range
        if (birdX == (pillarX + PILLARXDIM) && 
            !score_updated && 
            ((birdY > (PILLARYDIM - (YSCREEN - pillarY0))) || 
             (birdY + SQUARE_SIZE < pillarY0))) begin
            score <= score + 1'b1;
				play_sound <= 1;
            score_updated <= 1'b1;
        end
        
        // Reset the flag when bird moves past the pillar
        if (birdX > (pillarX + PILLARXDIM)) begin
            score_updated <= 1'b0;
				play_sound <= 0;
        end
    end
end

// Uncomment the hex display to show the score
hex7seg score_display(
    .hex(score),
    .display(HEX0)
);

	// KEYBOARD 
	inout PS2_CLK;
   inout PS2_DAT;
	wire EY;
	reg Tdir;
	
	// KEYBOARD ASSIGN 
	assign start = (ps2_key_data == 8'h29); // space bar
	assign toggleYup = (ps2_key_data == 8'hE075);// up A key
	assign toggleYdown = (ps2_key_data == 8'hE072);// Down Z key 

	assign Ydir = (!toggleYup) ? 1'b1 :
                  (!toggleYdown) ? 1'b0:  
                  Ydir;
	wire [7:0] ps2_key_data;
	wire ps2_key_pressed;

	reg pillarLx, pillarEx, pillarLy, pillarEy, pillarLxc, pillarLyc, pillarExc, pillarEyc;
	wire pillarXdir;
	reg wrap_reset;      

	reg [2:0] x_Q, X_D;

    // Add a game reset signal
    wire game_reset;
    assign game_reset = ~KEY[1]; // Active low reset

    // Instantiate modules with game reset consideration
    UpDn_count U1 (
        .R(birdY0), 
        .Clock(CLOCK_50), 
        .Resetn(KEY[0] & ~game_reset), 
        .E(birdEy), 
        .L(game_reset), 
        .UpDn(Ydir), 
        .Q(birdY)
    );
    defparam U1.n = 7;

    regn U2 (
        .R(birdX0), 
        .Resetn(KEY[0] & ~game_reset), 
        .E(1'b1), 
        .Clock(CLOCK_50), 
        .Q(birdX)
    );
    defparam U2.n = 8;

    // Reset X and Y counters with game reset
    UpDn_count U3 (
        .R(8'd0), 
        .Clock(CLOCK_50), 
        .Resetn(KEY[0] & ~game_reset), 
        .E(birdExc), 
        .L(birdLxc), 
        .UpDn(1'b1), 
        .Q(birdXC)
    );
    defparam U3.n = 8;
    
    UpDn_count U4 (
        .R(7'd0), 
        .Clock(CLOCK_50), 
        .Resetn(KEY[0] & ~game_reset), 
        .E(birdEyc), 
        .L(birdLyc), 
        .UpDn(1'b1), 
        .Q(birdYC)
    );
    defparam U4.n = 7;

		  
		  
    UpDn_count U5 ({Kslow{1'b0}}, CLOCK_50, KEY[0], 1'b1, 1'b0, 1'b1, slow);
        defparam U5.n = Kslow;
		  
		  
    assign sync = (slow == 0);

//    ToggleFFbird U6 (toggleY, KEY[0], CLOCK_50,birdYdir); removed so we can add keyboard 
//ADDED FOR KEYBOARD 

    ToggleFF U6_up(.T(toggleYup), .Resetn(KEY[0] & ~game_reset), .Clock(CLOCK_50), .Q(Ydir_up));
    ToggleFF U6_down(.T(toggleYdown), .Resetn(KEY[0] & ~game_reset), .Clock(CLOCK_50), .Q(Ydir_down));

    // Pillar counter with game reset
    UpDn_count U7 (
        .R(pillarX0), 
        .Clock(CLOCK_50), 
        .Resetn(KEY[0] & ~game_reset), 
        .E(pillarEx), 
        .L(game_reset | wrap_reset), 
        .UpDn(1'b0), 
        .Q(pillarX)
    );
    defparam U7.n = 8;

    // Pillar Y position reset
    regn U8 (
        .R(pillarY0), 
        .Resetn(KEY[0] & ~game_reset), 
        .E(1'b1), 
        .Clock(CLOCK_50), 
        .Q(pillarY)
    );
    defparam U8.n = 7;

	UpDn_count U9 (8'd0, CLOCK_50, 1'b1, pillarExc, pillarLxc, 1'b1, pillarXC);
		defparam U9.n = 8;
	UpDn_count U10 (7'd0, CLOCK_50, 1'b1, pillarEyc, pillarLyc, 1'b1, pillarYC);
		defparam U10.n = 7;
//
//	UpDn_count U11 ({Kslow{1'b0}}, CLOCK_50, 1'b1, 1'b1, 1'b0, 1'b1, slow); // & ~freeze
//		defparam U11.n = Kslow;


// bird from rom 


assign bird_rom_address = birdYC * 10 + birdXC;

// Combine coordinates
// Wires for ROM interaction
wire [9:0] bird_rom_address;  // ROM address (10 bits)
wire [2:0] bird_pixel_color; // Output color from ROM

// Instantiate the ROM
birdy_rom bird_image_rom (
    .address(bird_rom_address), // Input address
    .clock(CLOCK_50),           // Input clock
    .q(bird_pixel_color)        // Output color
);


    // FSM state table
    always @ (*)
        case (y_Q)
            A:  if (!go || !sync) Y_D = A;
                else Y_D = O;
            B:  if (birdXC != SQUARE_SIZE-1) Y_D = B;    // draw bird
                else Y_D = C;
            C:  if (birdYC != SQUARE_SIZE-1) Y_D = B;
                else Y_D = D;
				D:  if (pillarYC != PILLARYDIM-1) Y_D = D; // draw pillar
					 else Y_D = E;
				E:  if (pillarXC != PILLARXDIM-1) Y_D = D; // draw pillar
					 else Y_D = F;
				F: if (!sync) Y_D = F; //sync waiting
                else Y_D = G;
				G:  if (birdXC != SQUARE_SIZE-1) Y_D = G;    // erase bird
                else Y_D = H;
            H:  if (birdYC != SQUARE_SIZE-1) Y_D = G; 
                else Y_D = I;
				I: if (pillarYC != PILLARYDIM-1) Y_D = I; // erase //E
					else Y_D = J;
				J: if (pillarXC != PILLARXDIM-1) Y_D = I; //F
					else Y_D = K;
				K: Y_D = L;
		L: if ((birdY == 7'd0) || (birdY == YSCREEN-9) || ( ((birdX + SQUARE_SIZE) > pillarX && (birdX < (pillarX + PILLARXDIM))) && !( ((birdY) > (PILLARYDIM - (YSCREEN - pillarY0) )) && ((birdY + SQUARE_SIZE) < pillarY0)) )) Y_D = M;		
			else Y_D = B;
		M: if (endScreenYC != 119) Y_D = M; // draw screen
				else Y_D = N;
		N: if (endScreenXC != 159) Y_D = M; // draw screen
				else Y_D = A;
		O: if (endScreenYC != 119) Y_D = O; // erase //Endscreen
				else Y_D = P;
		P: if (endScreenXC != 159) Y_D = O; //F
				else Y_D = Q;
		Q: Y_D = B;
				
        endcase
    // FSM outputs
    always @ (*)
    begin
        // default assignments
        birdLxc = 1'b0; birdLyc = 1'b0; birdExc = 1'b0; birdEyc = 1'b0; VGA_COLOR = 3'b110; plot = 1'b0;
        birdEy = 1'b0; birdTdir = 1'b0; 
		  pillarLxc = 1'b0; pillarLyc = 1'b0; pillarExc = 1'b0; pillarEyc = 1'b0;
		  pillarEx = 1'b0; wrap_reset = 1'b0;
		  endScreenLxc = 1'b0; endScreenLyc = 1'b0; endScreenExc = 1'b0; endScreenEyc = 1'b0;
		  endScreenEx = 1'b0;
        case (y_Q)
            A:  begin endScreenLyc = 1'b1; endScreenLxc = 1'b1; end
            //B:  begin birdExc = 1'b1; plot = 1'b1; VGA_X = birdX + birdXC; VGA_Y = birdY + birdYC;  VGA_COLOR = 3'b110; end   // color a pixel //plot pixel
				B: begin
    birdExc = 1'b1;  // Enable X counter for bird
    plot = 1'b1;     // Signal to plot pixel
    VGA_X = birdX + birdXC; // X coordinate on VGA screen
    VGA_Y = birdY + birdYC; // Y coordinate on VGA screen
   VGA_COLOR = bird_pixel_color; // Color fetched from ROM ---------------------------
end

            C:  begin birdLxc = 1'b1; birdEyc = 1'b1; end
				D: begin pillarEyc = 1'b1; plot = 1'b1; VGA_X = pillarX + pillarXC; VGA_Y = pillarY + pillarYC;  VGA_COLOR = 3'b010; end // Draw a pixel
				E: begin pillarLyc = 1'b1; pillarExc = 1'b1; end
				F: begin birdLyc = 1'b1; pillarLxc = 1'b1; end
				G: begin birdExc = 1'b1; plot = 1'b1; VGA_X = birdX + birdXC; VGA_Y = birdY + birdYC; VGA_COLOR = 3'b011; end   // color a pixel ------  VGA_COLOR = ALT;
            H: begin birdLxc = 1'b1; birdEyc = 1'b1; end
				I: begin pillarEyc = 1'b1; plot = 1'b1; VGA_X = pillarX + pillarXC; VGA_Y = pillarY + pillarYC; VGA_COLOR = 3'b011; end // Erase a pixel
				J: begin pillarLyc = 1'b1; pillarExc = 1'b1; end
				K: begin
					 birdLyc = 1'b1;
					 birdTdir = (birdY == 7'd0) || (birdY == YSCREEN-9);
					 pillarLxc = 1'b1;
					 if (pillarX == 8'd0 || game_reset) begin
                      wrap_reset = 1'b1; // Reset X position to rightmost
                 end
                 if (wrap_reset || game_reset) begin
                      pillarY0 <= random_pillarY; // Update pillarY0 dynamically
                 end
               end

				L: begin endScreenLyc = 1'b1; endScreenLxc = 1'b1; birdEy = 1'b1; pillarEx = 1'b1; end
				
				M: begin endScreenEyc = 1'b1; plot = 1'b1; VGA_X = endScreenX + endScreenXC; VGA_Y = endScreenY + endScreenYC;  VGA_COLOR = 3'b100; end // Draw a pixel
				N: begin endScreenLyc = 1'b1; endScreenExc = 1'b1; end
				
				//A goes to erase the backscreen (endgame screen) 
				O: begin endScreenEyc = 1'b1; plot = 1'b1; VGA_X = endScreenX + endScreenXC; VGA_Y = endScreenY + endScreenYC; VGA_COLOR = 3'b011; end // Erase a pixel
				P: begin endScreenLyc = 1'b1; endScreenExc = 1'b1; end
				
				Q: begin birdLxc = 1'b1; birdLyc = 1'b1; pillarLxc = 1'b1; pillarLyc = 1'b1; end

        endcase
    end

    always @(posedge CLOCK_50)
        if (!KEY[0])
            y_Q <= 1'b0;
        else
            y_Q <= Y_D;

    //assign go = ~KEY[3];
	 assign go = start;
	 
	

    // connect to VGA controller
    vga_adapter VGA (
			.resetn(KEY[0]),
			.clock(CLOCK_50),
			.colour(VGA_COLOR),
			.x(VGA_X),
			.y(VGA_Y),
			.plot(plot),
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK_N(VGA_BLANK_N),
			.VGA_SYNC_N(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "newbg.mif"; 
		
 PS2_Controller PS2(
 .CLOCK_50(CLOCK_50), .reset(~KEY[0]), .PS2_CLK(PS2_CLK),
                     .PS2_DAT(PS2_DAT), .received_data(ps2_key_data),
                     .received_data_en(ps2_key_pressed));
							

		
endmodule

module regn(R, Resetn, E, Clock, Q);
    parameter n = 8;
    input [n-1:0] R;
    input Resetn, E, Clock;
    output reg [n-1:0] Q;

    always @(posedge Clock)
        if (!Resetn)
            Q <= 0;
        else if (E)
            Q <= R;
endmodule

//BIRD TOGGLE
module ToggleFFbird(T, Resetn, Clock, Q);
    input T, Resetn, Clock;
    output reg Q;

    always @(posedge Clock)
        if (!Resetn)
            Q <= 0;
        else if (T)
            Q <= 0;
		 else if (~T)
			Q <= 1;
			//Q<= 2;
endmodule

module UpDn_count (R, Clock, Resetn, E, L, UpDn, Q);
    parameter n = 8;
    input [n-1:0] R;
    input Clock, Resetn, E, L, UpDn;
    output reg [n-1:0] Q;

    always @ (posedge Clock)
        if (Resetn == 0)
            Q <= 0;
        else if (L == 1)
            Q <= R;
        else if (E)
            if (UpDn == 1)
                Q <= Q + 1;
            else
                Q <= Q - 1;
endmodule


module ToggleFF(T, Resetn, Clock, Q);
    input T, Resetn, Clock;
    output reg Q;

    always @(posedge Clock)
        if (!Resetn)
            Q <= 0;
        else if (T)
            Q <= ~Q;
endmodule


module LFSR (
    input Clock,
    input Resetn,
    output reg [6:0] random // Random 7-bit value for pillarY0 (0 to 119)
);
    reg [7:0] shift_reg; // 8-bit shift register for LFSR

    always @(posedge Clock  or negedge Resetn) begin
        if (!Resetn)
            shift_reg <= 8'b00000001; // Initialize with a non-zero seed
        else
            shift_reg <= {shift_reg[6:0], shift_reg[7] ^ shift_reg[5] ^ shift_reg[4] ^ shift_reg[3]}; // Feedback
    end

    // Limit the random value to the screen's vertical range (0 to 119)
    always @(*) begin
        random = 62 + (shift_reg[6:0] % 57); // Modulo to fit the screen's range
    end
endmodule




module hex7seg (hex, display);
    input [3:0] hex;
    output [6:0] display;

    reg [6:0] display;

    /*
     *       0  
     *      ---  
     *     |   |
     *    5|   |1
     *     | 6 |
     *      ---  
     *     |   |
     *    4|   |2
     *     |   |
     *      ---  
     *       3  
     */
    always @ (hex)
        case (hex)
            4'h0: display = 7'b1000000;
            4'h1: display = 7'b1111001;
            4'h2: display = 7'b0100100;
            4'h3: display = 7'b0110000;
            4'h4: display = 7'b0011001;
            4'h5: display = 7'b0010010;
            4'h6: display = 7'b0000010;
            4'h7: display = 7'b1111000;
            4'h8: display = 7'b0000000;
            4'h9: display = 7'b0011000;
            4'hA: display = 7'b0001000;
            4'hB: display = 7'b0000011;
            4'hC: display = 7'b1000110;
            4'hD: display = 7'b0100001;
            4'hE: display = 7'b0000110;
            4'hF: display = 7'b0001110;
        endcase
endmodule

module hexDisplay(X, display); //4 bit
	input [3:0] X;
	output [6:0] display;
	
	assign display[0] = ~((~X[2] & ~X[0]) | (X[1] & ~X[0]) | (X[3] & ~X[1] & ~X[0]) | (X[3] & ~X[2] & ~X[1]) | (X[2] & X[1]) | (~X[3] & X[2] & X[0]) | (X[1] & ~X[3]));
	assign display[1] = ~((~X[3] & ~X[2]) | (~X[2] & ~X[0]) | (~X[1] & ~X[0] & ~X[3]) | (X[1] & X[0] & ~X[3]) | (~X[1] & X[0] & X[3]));
	assign display[2] = ~((~X[3] & ~X[1]) | (~X[3] & X[0]) | (~X[3] & X[2]) | (~X[1] & X[0]) | (X[3] & ~X[2]));
	assign display[3] = ~((~X[3] & ~X[2] & ~X[1] & ~X[0]) | (X[3] & ~X[1]) | (~X[1] & X[0] & X[2]) | (~X[3] & ~X[2] & X[1]) | (X[1] & ~X[0] & X[2]) | (X[3] & ~X[2] & X[0]));
	assign display[4] = ~((X[3] & X[2]) | (X[1] & ~X[0]) | (~X[2] & ~X[0]) | (X[3] & ~X[2] & X[1]));
	assign display[5]= ~((~X[1] & ~X[0]) | (~X[3] & X[2] & ~X[1] & X[0]) | (X[3] & ~X[2]) | (X[3] & X[1]) | (X[2] & X[1] & ~X[0]));
	assign display[6] = ~((~X[3] & X[2] & ~X[1]) | (X[3] & ~X[2]) | (X[3] & X[0]) | (X[1] & ~X[0]) | (~X[3] & ~X[2] & X[1]));
	
	
endmodule 




