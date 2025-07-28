module multi_pwm_timer_tb;
    parameter NUM_CHANNELS = 4;
    
    reg         i_clk = 0;
    reg         i_rst = 1;
    reg         i_wb_cyc = 0;
    reg         i_wb_stb = 0;
    reg         i_wb_we  = 0;
    reg  [15:0]  i_wb_adr = 0;
    reg  [15:0] i_wb_data = 0;
    reg         i_extclk = 0;
    reg  [15:0] i_DC [NUM_CHANNELS-1:0];
    reg         i_DC_valid [NUM_CHANNELS-1:0];

    wire        o_wb_ack;
    wire [15:0] o_wb_data;
    wire        o_pwm [NUM_CHANNELS-1:0];

    // Clock generation
    always #500 i_clk = ~i_clk;       // 100MHz system clock (10ns period)
    always #20 i_extclk = ~i_extclk; // 500MHz external clock (2ns period)

    // Initialize arrays
    integer k;
    initial begin
        for(k = 0; k < NUM_CHANNELS; k = k + 1) begin
            i_DC[k] = 0;
            i_DC_valid[k] = 0;
        end
    end

    // DUT instantiation
    multi_pwm_timer #(.NUM_CHANNELS(NUM_CHANNELS)) uut (
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

    // Task to write to a register via Wishbone
    task wb_write(input [7:0] addr, input [15:0] data);
        begin
            @(negedge i_clk);
            i_wb_cyc  = 1;
            i_wb_stb  = 1;
            i_wb_we   = 1;
            i_wb_adr  = addr;
            i_wb_data = data;
            
            @(negedge i_clk);
            while (!o_wb_ack) @(negedge i_clk);
            
            i_wb_cyc  = 0;
            i_wb_stb  = 0;
            i_wb_we   = 0;
            i_wb_adr  = 0;
            i_wb_data = 0;
        end
    endtask

    // Task to read from a register via Wishbone
    task wb_read(input [7:0] addr, output [15:0] data);
        begin
            @(negedge i_clk);
            i_wb_cyc  = 1;
            i_wb_stb  = 1;
            i_wb_we   = 0;
            i_wb_adr  = addr;
            
            @(negedge i_clk);
            while (!o_wb_ack) @(negedge i_clk);
            data = o_wb_data;
            
            i_wb_cyc  = 0;
            i_wb_stb  = 0;
            i_wb_we   = 0;
            i_wb_adr  = 0;
        end
    endtask

    // Address calculation macros
    function [7:0] channel_addr(input [2:0] channel, input [1:0] reg_addr);
        channel_addr = {1'b0, channel, reg_addr};
    endfunction
    
    function [7:0] global_addr(input [1:0] reg_addr);
        global_addr = {1'b1, 5'b0, reg_addr};
    endfunction

    reg [15:0] read_data;
    
    initial begin
        $display("Starting Multi-Channel PWM Timer simulation...");

        // Apply reset
        i_rst = 0;
        repeat(2) @(negedge i_clk);
        i_rst = 1;
        @(negedge i_clk);
        i_rst = 0;

        $display("=== Configuring Global Settings ===");
        // Set global clock divider to 4
        wb_write(global_addr(2'b01), 16'd4);
        $display("Set global divisor to 4");

        $display("=== Configuring Channel 0 (PWM Mode) ===");
        // Channel 0: PWM mode with 50% duty cycle
        wb_write(channel_addr(3'd0, 2'b01), 16'd100); // Period = 100
        wb_write(channel_addr(3'd0, 2'b10), 16'd50);  // DC = 50
        wb_write(channel_addr(3'd0, 2'b00), 8'h16);   // PWM mode, enable, output enable
        $display("Channel 0: Period=100, DC=50, PWM mode enabled");

        $display("=== Configuring Channel 1 (PWM Mode, Different Settings) ===");
        // Channel 1: PWM mode with 25% duty cycle and different period
        wb_write(channel_addr(3'd1, 2'b01), 16'd80);  // Period = 80
        wb_write(channel_addr(3'd1, 2'b10), 16'd20);  // DC = 20
        wb_write(channel_addr(3'd1, 2'b00), 8'h16);   // PWM mode, enable, output enable
        $display("Channel 1: Period=80, DC=20, PWM mode enabled");

        $display("=== Configuring Channel 2 (Timer Mode) ===");
        // Channel 2: Timer mode
        wb_write(channel_addr(3'd2, 2'b01), 16'd60);  // Period = 60
        wb_write(channel_addr(3'd2, 2'b00), 8'h0C);   // Timer mode, enable, continuous
        $display("Channel 2: Period=60, Timer mode enabled (continuous)");

        $display("=== Configuring Channel 3 (PWM Mode with External DC) ===");
        // Channel 3: PWM mode with external duty cycle
        wb_write(channel_addr(3'd3, 2'b01), 16'd120); // Period = 120
        i_DC[3] = 16'd30;                             // External DC = 30
        i_DC_valid[3] = 1;
        wb_write(channel_addr(3'd3, 2'b00), 8'h54);   // PWM mode, enable, output enable, external DC
        $display("Channel 3: Period=120, External DC=30, PWM mode enabled");

        $display("=== Observing Multi-Channel PWM Output ===");
        repeat (500) @(negedge i_clk);

        $display("=== Testing Dynamic Duty Cycle Change on Channel 3 ===");
        // Change external duty cycle for channel 3
        i_DC[3] = 16'd90;
        $display("Changed Channel 3 external DC to 90");
        repeat (300) @(negedge i_clk);

        $display("=== Reading Counter Values ===");
        // Read current counter values for all channels
        for(integer i = 0; i < NUM_CHANNELS; i = i + 1) begin
            wb_read(channel_addr(i, 2'b11), read_data);
            $display("Channel %0d counter value: %0d", i, read_data);
        end

        $display("=== Testing Channel Disable/Enable ===");
        // Disable channel 1
        wb_write(channel_addr(3'd1, 2'b00), 8'h12); // Disable counter
        $display("Disabled Channel 1");
        repeat (200) @(negedge i_clk);

        // Re-enable channel 1
        wb_write(channel_addr(3'd1, 2'b00), 8'h16); // Re-enable
        $display("Re-enabled Channel 1");
        repeat (200) @(negedge i_clk);

        $display("=== Testing Error Conditions ===");
        // Test DC > Period error on channel 0
        wb_write(channel_addr(3'd0, 2'b10), 16'd150); // DC > Period
        @(negedge i_clk);
        $display("Set Channel 0 DC=150 (> Period=100) Error=");
          $stop();
        repeat (100) @(negedge i_clk);
        // Fix the error
        wb_write(channel_addr(3'd0, 2'b10), 16'd25);  // DC < Period
        $display("Fixed Channel 0 DC=25");
        repeat (100) @(negedge i_clk);

        $display("=== Testing Timer Interrupt Handling ===");
        // Check if timer interrupt was generated for channel 2
        wb_read(channel_addr(3'd2, 2'b00), read_data);
        if(read_data[5]) begin
            $display("Timer interrupt detected on Channel 2, clearing...");
            wb_write(channel_addr(3'd2, 2'b00), 8'h0C & ~8'h20); // Clear interrupt
        end

        $display("=== Final Observation ===");
        repeat (300) @(negedge i_clk);

        $display("Simulation completed successfully!");
        $stop;
    end

    // Monitor all channels
    initial begin
        $monitor("Time=%0t | CH0=%b CH1=%b CH2=%b CH3=%b | Ack=%b | Addr=0x%02h Data=0x%04h",
                 $time, o_pwm[0], o_pwm[1], o_pwm[2], o_pwm[3], o_wb_ack, i_wb_adr, o_wb_data);
    end

    // Detailed channel monitoring
    initial begin
        $display("Time\tCH0\tCH1\tCH2\tCH3");
        forever begin
            #100;
            $display("%0t\t%b\t%b\t%b\t%b", $time, o_pwm[0], o_pwm[1], o_pwm[2], o_pwm[3]);
        end
    end

endmodule