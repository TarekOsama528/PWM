module pwm_timer_tb;
    reg         i_clk = 0;
    reg         i_rst = 1;
    reg         i_wb_cyc = 0;
    reg         i_wb_stb = 0;
    reg         i_wb_we  = 0;
    reg  [3:0]  i_wb_adr = 0;
    reg  [15:0] i_wb_data = 0;
    reg         i_extclk = 0;
    reg  [15:0] i_DC = 0;
    reg         i_DC_valid = 0;

    wire        o_wb_ack;
    wire [15:0] o_wb_data;
    wire        o_pwm;

    // Clock generation
    always #5 i_clk = ~i_clk;       // 100MHz system clock (10ns period)
    always #2.5 i_extclk = ~i_extclk;// 200MHz external clock (5ns period)

    // DUT instantiation
    pwm_timer uut (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_wb_cyc(i_wb_cyc),
        .i_wb_stb(i_wb_stb),
        .i_wb_we(i_wb_we),
        .i_wb_adr(i_wb_adr),
        .i_wb_data(i_wb_data),
        .o_wb_ack(o_wb_ack),
        .o_wb_data(o_wb_data),
        .i_extclk(i_extclk),
        .i_DC(i_DC),
        .i_DC_valid(i_DC_valid),
        .o_pwm(o_pwm)
    );

    initial begin
        $display("Starting simulation...");

        // Apply reset
          i_rst=0;
        repeat(2) @(negedge i_clk);
        i_rst = 1;
        @(negedge i_clk)
        i_rst=0;


        // ----------------------------
        // Write period_reg = 100
        @(negedge i_clk);
        i_wb_cyc  = 1;
        i_wb_stb  = 1;
        i_wb_we   = 1;
        i_wb_adr  = 4'h2; // address for period_reg
        i_wb_data = 16'd100;


    // wait for acknowledgment and then reset the signals
        @(negedge i_clk);
        while (!o_wb_ack) @(negedge i_clk);

        i_wb_cyc  = 0;
        i_wb_stb  = 0;
        i_wb_we   = 0;
        i_wb_adr  = 0;
        i_wb_data = 0;

        // ----------------------------
        // Write dc_reg = 40
        @(negedge i_clk);
        i_wb_cyc  = 1;
        i_wb_stb  = 1;
        i_wb_we   = 1;
        i_wb_adr  = 4'h3; // address for dc_reg
        i_wb_data = 16'd40;

        @(negedge i_clk);
        while (!o_wb_ack) @(negedge i_clk);

        i_wb_cyc  = 0;
        i_wb_stb  = 0;
        i_wb_we   = 0;
        i_wb_adr  = 0;
        i_wb_data = 0;

        // ----------------------------
        // Write divisor_reg = 4
        @(negedge i_clk);
        i_wb_cyc  = 1;
        i_wb_stb  = 1;
        i_wb_we   = 1;
        i_wb_adr  = 4'h1; // address for divisor_reg
        i_wb_data = 16'd4;

        @(negedge i_clk);
        while (!o_wb_ack) @(negedge i_clk);

        i_wb_cyc  = 0;
        i_wb_stb  = 0;
        i_wb_we   = 0;
        i_wb_adr  = 0;
        i_wb_data = 0;

        // ----------------------------
        // Write ctrl_reg = 0x16 => 0001_0110 (PWM mode, enable, out enable)
        @(negedge i_clk);
        i_wb_cyc  = 1;
        i_wb_stb  = 1;
        i_wb_we   = 1;
        i_wb_adr  = 4'h0; // address for ctrl_reg
        i_wb_data = 16'h0016;

        @(negedge i_clk);
        while (!o_wb_ack) @(negedge i_clk);

        i_wb_cyc  = 0;
        i_wb_stb  = 0;
        i_wb_we   = 0;
        i_wb_adr  = 0;
        i_wb_data = 0;

        // ----------------------------
        $display("Observing PWM output...");
        repeat (300) @(negedge i_clk);

        // ----------------------------
        // Set external duty cycle = 50 and enable it
        i_DC = 16'd50;
        i_DC_valid = 1;

        @(negedge i_clk);
        i_wb_cyc  = 1;
        i_wb_stb  = 1;
        i_wb_we   = 1;
        i_wb_adr  = 4'h0;
        i_wb_data = 16'h0056; // ext_dc_sel=1, mode_sel=1, counter_en=1, pwm_out_en=1

        @(negedge i_clk);
        while (!o_wb_ack) @(negedge i_clk);

        i_wb_cyc  = 0;
        i_wb_stb  = 0;
        i_wb_we   = 0;
        i_wb_adr  = 0;
        i_wb_data = 0;

        // Observe PWM with external DC
        $display("Observing PWM output with external duty cycle...");
        repeat (10) @(negedge i_clk);

        $display("Simulation finished.");
        $stop;
    end
    initial begin
    $monitor("time=%0t clk=%b rst=%b ack=%b pwm=%b wb_data_out=0x%04h i_DC=%0d i_DC_valid=%b ",
             $time, i_clk, i_rst, o_wb_ack, o_pwm, o_wb_data, i_DC, i_DC_valid);
    end

endmodule
