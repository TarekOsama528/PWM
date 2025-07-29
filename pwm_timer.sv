//=========================================================================================
// Two flip-flop synchronizer for clock domain crossing
//=========================================================================================
//=========================================================================================
// Glitch free Clock Mux
//=========================================================================================
module clock_mux (
    input  wire clk1,
    input wire clk2,
    input  wire select,
    output wire out_clk
);
    wire i_and1, i_and2;
    wire o_and1, o_and2;
    reg flop1_o, flop2_o;

    assign i_and1 = ~select & ~flop2_o;
    assign i_and2 = select & ~flop1_o;
    assign o_and1 = clk1 & flop1_o;
    assign o_and2 = clk2 & flop2_o;
    assign out_clk = o_and1 | o_and2;

    always @(posedge clk1) begin
        flop1_o <= i_and1;
    end

    always @(posedge clk2) begin
        flop2_o <= i_and2;
    end
endmodule
//=========================================================================================
//=========================================================================================

module pwm_timer #(
    parameter NUM_CHANNELS = 4
)(
    input  wire        i_clk,
    input  wire        i_rst,
    input  wire        i_wb_cyc,
    input  wire        i_wb_stb,
    input  wire        i_wb_we,
    input  wire [15:0]  i_wb_adr,
    input  wire [15:0] i_wb_data,
    output reg         o_wb_ack,
    output reg  [15:0] o_wb_data,
    input  wire        i_extclk,
    input  wire [15:0] i_DC [NUM_CHANNELS-1:0],
    input  wire        i_DC_valid [NUM_CHANNELS-1:0],
    output reg         o_pwm [NUM_CHANNELS-1:0]
);

   
    
    // Per-channel control and data registers
    reg [7:0] ctrl_reg [NUM_CHANNELS-1:0];
    reg [15:0] period_reg [NUM_CHANNELS-1:0];
    reg [15:0] dc_reg [NUM_CHANNELS-1:0];
    reg [15:0] divisor_reg [NUM_CHANNELS-1:0];  // Now per-channel
    
    // Internal signals for each channel
    wire actual_clk [NUM_CHANNELS-1:0];
    
    // Per-channel clock division
    reg [15:0] div_counter [NUM_CHANNELS-1:0];
    reg divided_clk_pulse [NUM_CHANNELS-1:0];
    
    // Per-channel synchronized control signals
    wire clk_sel_sync [NUM_CHANNELS-1:0];
    wire mode_sel_sync [NUM_CHANNELS-1:0];
    wire counter_en_sync [NUM_CHANNELS-1:0];
    wire continuous_sync [NUM_CHANNELS-1:0];
    wire pwm_out_en_sync [NUM_CHANNELS-1:0];
    wire ext_dc_sel_sync [NUM_CHANNELS-1:0];
    wire counter_rst_sync [NUM_CHANNELS-1:0];
    
    // Per-channel main counters and control
    reg [15:0] main_counter [NUM_CHANNELS-1:0];
    wire [15:0] used_dc [NUM_CHANNELS-1:0];
    
    // Error signals
    reg [NUM_CHANNELS-1:0] error_dc_too_big;
    reg [NUM_CHANNELS-1:0] error_div_invalid;
    
    // Interrupt handling
    
    wire irq_to_wb_sync [NUM_CHANNELS-1:0];
   
    

    // Address decoding - Updated for per-channel divisors
    wire [2:0] channel_sel = i_wb_adr[5:3];  // Bits [5:3] select channel (0-7, but we use 0-3)
    wire [2:0] reg_sel = i_wb_adr[2:0];      // Bits [2:0] select register within channel
    

    genvar i;
    generate
        for (i = 0; i < NUM_CHANNELS; i = i + 1) begin : channel_gen
            
            assign used_dc[i] = (ext_dc_sel_sync[i] && i_DC_valid[i]) ? i_DC[i] : dc_reg[i];
            
               clock_mux clk_mux_inst (.clk1(i_clk),.clk2(i_extclk),.select(ctrl_reg[i][0]),.out_clk(actual_clk[i]));
            // synchronizer sync_mode_sel   (.clk(actual_clk[i]), .rst(i_rst), .async_in(ctrl_reg[i][1]), .sync_out(mode_sel_sync[i]));
            // synchronizer sync_counter_en (.clk(actual_clk[i]), .rst(i_rst), .async_in(ctrl_reg[i][2]), .sync_out(counter_en_sync[i]));
            // synchronizer sync_continuous (.clk(actual_clk[i]), .rst(i_rst), .async_in(ctrl_reg[i][3]), .sync_out(continuous_sync[i]));
            // synchronizer sync_pwm_out_en (.clk(actual_clk[i]), .rst(i_rst), .async_in(ctrl_reg[i][4]), .sync_out(pwm_out_en_sync[i]));
            // synchronizer sync_ext_dc_sel (.clk(actual_clk[i]), .rst(i_rst), .async_in(ctrl_reg[i][6]), .sync_out(ext_dc_sel_sync[i]));
            // synchronizer sync_counter_rst(.clk(actual_clk[i]), .rst(i_rst), .async_in(ctrl_reg[i][7]), .sync_out(counter_rst_sync[i]));
            assign mode_sel_sync[i]      = ctrl_reg[i][1];
            assign counter_en_sync[i]    = ctrl_reg[i][2];
            assign continuous_sync[i]    = ctrl_reg[i][3];
            assign pwm_out_en_sync[i]    = ctrl_reg[i][4];
            assign ext_dc_sel_sync[i]    = ctrl_reg[i][6];
            assign counter_rst_sync[i]   = ctrl_reg[i][7];

            assign irq_to_wb_sync[i]= ctrl_reg[i][5]; // Interrupt flag to Wishbone
            
            // Interrupt synchronizer
        
            

            
            // Per-channel clock divider
            always @(posedge actual_clk[i] or posedge i_rst) begin
                if(i_rst) begin
                    div_counter[i] <= 0;
                    divided_clk_pulse[i] <= 1'b0;
                    error_div_invalid[i] <= 1'b0;
                end else begin
                    error_div_invalid[i] <= 1'b0; // Reset error flag
                    
                    if(divisor_reg[i] <= 1) begin
                        div_counter[i] <= 0;
                        divided_clk_pulse[i] <= 1'b1;
                        if(divisor_reg[i] == 0) begin
                            error_div_invalid[i] <= 1'b1; 
                        end
                        
                    end else begin
                        if(div_counter[i] < (divisor_reg[i] - 1)) begin
                            div_counter[i] <= div_counter[i] + 1;
                            divided_clk_pulse[i] <= 1'b0;
                        end else begin
                            div_counter[i] <= 0;
                            divided_clk_pulse[i] <= 1'b1;
                        end
                    end
                end
            end
            
            // Main counter for each channel
            always @(posedge actual_clk[i] or posedge i_rst) begin
                if(i_rst || counter_rst_sync[i]) begin
                    main_counter[i] <= 1;
                end else if((counter_en_sync[i] && divided_clk_pulse[i] && !irq_to_wb_sync[i]) || 
                           (counter_en_sync[i] && divided_clk_pulse[i] && continuous_sync[i] && !mode_sel_sync[i])) begin
                    if(main_counter[i] >= period_reg[i]) begin
                        main_counter[i] <= 1;
                    end else begin
                        main_counter[i] <= main_counter[i] + 1;
                    end
                end
            end
            
            // PWM output logic for each channel
            always @(posedge actual_clk[i] or posedge i_rst) begin
                if(i_rst) begin
                    o_pwm[i] <= 1'b0;
                    
                    error_dc_too_big[i] <= 1'b0;
                end else begin
                    error_dc_too_big[i] <= 1'b0; // Reset error flag
                    ctrl_reg[i][7] <= 1'b0;
                    if(mode_sel_sync[i]) begin // PWM mode
                        if(counter_en_sync[i] && pwm_out_en_sync[i]) begin
                            if(period_reg[i] < used_dc[i]) begin
                                o_pwm[i] <= 1'b1; // Error case: DC > Period
                                error_dc_too_big[i] <= 1'b1;
                            end else if (main_counter[i] <= used_dc[i]) begin
                                o_pwm[i] <= 1'b1;
                            end else begin
                                o_pwm[i] <= 1'b0;
                            end
                        end else begin
                            o_pwm[i] <= 1'b0;
                        end
                    end else begin // Timer mode
                        if(period_reg[i] >= used_dc[i])begin
                            o_pwm[i] <=irq_to_wb_sync[i]; // Timer mode, output low
                            error_dc_too_big[i] <= 1'b0; // No error in timer mode
                        end
                        else if(period_reg[i] < used_dc[i]) begin
                            o_pwm[i] <= 1'b1; // Error case: DC > Period
                            error_dc_too_big[i] <= 1'b1;
                        end 
                        if(main_counter[i] >= period_reg[i]) begin
                             
                             ctrl_reg[i][5] <=1; // Set interrupt flag
                             ctrl_reg[i][7] <= 1'b1; // Set reset 
                        end
                        else
                        ctrl_reg[i][5] <=((i_wb_data[5]==0)&&(channel_sel==i)&&(reg_sel==3'b000))? 1'b0:ctrl_reg[i][5]; // Set interrupt flag

                    end
                end
            end
        end
    endgenerate

    // Wishbone interface handling
    integer j;
    always @(posedge i_clk or posedge i_rst) begin
        if(i_rst) begin

            o_wb_ack <= 1'b0;
            
            // Initialize per-channel registers
            for(j = 0; j < NUM_CHANNELS; j = j + 1) begin
                ctrl_reg[j] <= 8'h00;
                period_reg[j] <= 16'h03E8; // Default period (1000)
                dc_reg[j] <= 16'h01F4;     // Default duty cycle (500)
                divisor_reg[j] <= 16'h0001; // Default divisor (no division)
            end
        end else begin
            o_wb_ack <= i_wb_cyc && i_wb_stb;
            
            if (i_wb_cyc && i_wb_stb) begin
                if (i_wb_we) begin // Write operation
                    if (channel_sel < NUM_CHANNELS) begin // Channel registers
                        case (reg_sel)
                            3'b000: begin // Control register
                                ctrl_reg[channel_sel[1:0]] <= i_wb_data[7:0];
                                // Clear interrupt flag when bit 5 is written with 0
                             
                            end
                            3'b001: divisor_reg[channel_sel[1:0]] <= i_wb_data;  // Divisor register
                            3'b010: period_reg[channel_sel[1:0]] <= i_wb_data;   // Period register
                            3'b011: dc_reg[channel_sel[1:0]] <= i_wb_data;       // Duty cycle register
                            default: ;
                        endcase
                    end
                end else begin // Read operation
                   if (channel_sel < NUM_CHANNELS) begin // Channel registers
                        case (reg_sel)
                            3'b000: o_wb_data <= {8'h00, ctrl_reg[channel_sel[1:0]]};           // Control
                            3'b001: o_wb_data <= divisor_reg[channel_sel[1:0]];                 // Divisor
                            3'b010: o_wb_data <= period_reg[channel_sel[1:0]];                  // Period
                            3'b011: o_wb_data <= dc_reg[channel_sel[1:0]];                      // Duty cycle
                            3'b100: o_wb_data <= main_counter[channel_sel[1:0]];                // Counter value
     
                            default: o_wb_data <= 16'h0000;
                        endcase
                    end else begin
                        o_wb_data <= 16'h0000;
                    end
                end
            end else begin
                o_wb_ack <= 1'b0;
            end
            
            // Handle interrupt flags from PWM domain
            for(j = 0; j < NUM_CHANNELS; j = j + 1) begin
                if(irq_to_wb_sync[j]) begin
                    ctrl_reg[j][5] <= 1'b1;
                end
            end
        end
    end

endmodule