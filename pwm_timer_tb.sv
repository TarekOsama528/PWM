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

    // Clock generation - slower clocks for better visibility
    always #5000 i_clk = ~i_clk;      // 100kHz system clock (10us period)
    always #2000 i_extclk = ~i_extclk; // 250kHz external clock (4us period)

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
    task wb_write(input [15:0] addr, input [15:0] data);
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
    task wb_read(input [15:0] addr, output [15:0] data);
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

    // Updated address calculation functions for new mapping
    // Channel registers: {1'b0, channel[2:0], reg[2:0]}
    function [15:0] channel_addr(input [2:0] channel, input [2:0] reg_addr);
        channel_addr = {8'b0, 1'b0, channel[2:0], reg_addr[2:0]};
    endfunction
    
    // Global registers: {1'b1, 7'b0}
    function [15:0] global_addr(input [2:0] reg_addr);
        global_addr = {8'b0, 1'b1, 4'b0, reg_addr[2:0]};
    endfunction

    reg [15:0] read_data;
    
    initial begin
        $display("Starting Multi-Channel PWM Timer simulation with Per-Channel Divisors...");

        // Apply reset
        i_rst = 0;
        repeat(2) @(negedge i_clk);
        i_rst = 1;
        @(negedge i_clk);
        i_rst = 0;
        repeat(2) @(negedge i_clk);

        $display("=== Configuring Global Settings ===");
        // Use internal clock (bit 0 = 0)
        wb_write(global_addr(3'b000), 16'h00);
        $display("Using internal clock");

        $display("=== Configuring Channel 0 (Fast PWM - Divisor=1) ===");
        wb_write(channel_addr(3'd0, 3'b000), 8'h00);      // Reset control first
        wb_write(channel_addr(3'd0, 3'b001), 16'd1);      // Divisor = 1 (fastest)
        wb_write(channel_addr(3'd0, 3'b010), 16'd20);     // Period = 20
        wb_write(channel_addr(3'd0, 3'b011), 16'd10);     // DC = 10 (50% duty)
        wb_write(channel_addr(3'd0, 3'b000), 8'h54);      // PWM mode, enable, output enable
        $display("Channel 0: Divisor=1, Period=20, DC=10, PWM mode enabled");

        $display("=== Configuring Channel 1 (Medium PWM - Divisor=2) ===");
        wb_write(channel_addr(3'd1, 3'b000), 8'h00);      // Reset control first
        wb_write(channel_addr(3'd1, 3'b001), 16'd0);      // Divisor = 2 (half speed)
        wb_write(channel_addr(3'd1, 3'b010), 16'd15);     // Period = 15
        wb_write(channel_addr(3'd1, 3'b011), 16'd111);      // DC = 5 (33% duty)
        wb_write(channel_addr(3'd1, 3'b000), 8'h16);      // PWM mode, enable, output enable
        $display("Channel 1: Divisor=2, Period=15, DC=5, PWM mode enabled");

        $display("=== Configuring Channel 2 (Slow PWM - Divisor=4) ===");
        wb_write(channel_addr(3'd2, 3'b000), 8'h00);      // Reset control first
        wb_write(channel_addr(3'd2, 3'b001), 16'd4);      // Divisor = 4 (quarter speed)
        wb_write(channel_addr(3'd2, 3'b010), 16'd12);     // Period = 12
        wb_write(channel_addr(3'd2, 3'b011), 16'd8);      // DC = 8 (67% duty)
        wb_write(channel_addr(3'd2, 3'b000), 8'h16);      // PWM mode, enable, output enable
        $display("Channel 2: Divisor=4, Period=12, DC=8, PWM mode enabled");

        $display("=== Configuring Channel 3 (Very Slow PWM - Divisor=8) ===");
        wb_write(channel_addr(3'd3, 3'b000), 8'h00);      // Reset control first
        wb_write(channel_addr(3'd3, 3'b001), 16'd8);      // Divisor = 8 (eighth speed)
        wb_write(channel_addr(3'd3, 3'b010), 16'd10);     // Period = 10
        wb_write(channel_addr(3'd3, 3'b011), 16'd3);      // DC = 3 (30% duty)
        wb_write(channel_addr(3'd3, 3'b000), 8'h16);      // PWM mode, enable, output enable
        $display("Channel 3: Divisor=8, Period=10, DC=3, PWM mode enabled");

        $display("=== Observing Multi-Channel PWM Output with Different Periods ===");
        $display("Expected periods (in clock cycles after division): CH0=20, CH1=30, CH2=48, CH3=80");
        
        // Let the PWM run for enough time to see multiple periods
        repeat (2000) @(negedge i_clk);

        $display("=== Reading Counter Values and Status ===");
        for(integer i = 0; i < NUM_CHANNELS; i = i + 1) begin
            wb_read(channel_addr(i, 3'b100), read_data);
            $display("Channel %0d counter value: %0d", i, read_data);
            wb_read(channel_addr(i, 3'b101), read_data);
            $display("Channel %0d error status: 0x%04h", i, read_data);
        end

        $display("=== Testing Dynamic Divisor Changes ===");
        // Change channel 0 to slower divisor
        wb_write(channel_addr(3'd0, 3'b001), 16'd16);     // Change divisor to 16
        $display("Changed Channel 0 divisor to 16 (much slower)");
        repeat (1000) @(negedge i_clk);

        // Change channel 3 to faster divisor
        wb_write(channel_addr(3'd3, 3'b001), 16'd2);      // Change divisor to 2
        $display("Changed Channel 3 divisor to 2 (much faster)");
        repeat (1000) @(negedge i_clk);

        $display("=== Testing External Clock Mode ===");
        // Switch to external clock
        wb_write(global_addr(3'b000), 16'h01);
        $display("Switched to external clock (faster)");
        repeat (500) @(negedge i_clk);

        // Switch back to internal clock
        wb_write(global_addr(3'b000), 16'h00);
        $display("Switched back to internal clock");
        repeat (500) @(negedge i_clk);

        $display("=== Testing Timer Mode on Channel 2 ===");
        // Switch channel 2 to timer mode
        wb_write(channel_addr(3'd2, 3'b000), 8'h0C);      // Timer mode, enable, continuous
        $display("Channel 2 switched to timer mode");
        repeat (300) @(negedge i_clk);

        // Check for timer interrupt
        wb_read(channel_addr(3'd2, 3'b000), read_data);
        if(read_data[5]) begin
            $display("Timer interrupt detected on Channel 2, clearing...");
            wb_write(channel_addr(3'd2, 3'b000), 8'h0C & ~8'h20); // Clear interrupt
        end

        $display("=== Final Extended Observation ===");
        repeat (1500) @(negedge i_clk);

        $display("Simulation completed successfully!");
        $stop;
    end

    // Monitor all channels with better formatting
    initial begin
        $display("=== PWM Output Monitor ===");
        $display("Time(us) | CH0(D=1) | CH1(D=2) | CH2(D=4) | CH3(D=8) | Notes");
        $display("---------|----------|----------|----------|----------|----------");
    end

    // Detailed timing analysis
    integer cycle_count = 0;
    always @(negedge i_clk) begin
        cycle_count = cycle_count + 1;
        if (cycle_count % 50 == 0) begin  // Print every 50 cycles
            $display("%8.1f | %8b | %8b | %8b | %8b | Cycle %0d", 
                     $time/1000.0, o_pwm[0], o_pwm[1], o_pwm[2], o_pwm[3], cycle_count);
        end
    end

    // Track PWM transitions for period measurement
    reg [NUM_CHANNELS-1:0] pwm_prev;
    integer transition_time [NUM_CHANNELS-1:0];
    integer last_transition_time [NUM_CHANNELS-1:0];
    
    initial begin
        for(integer j = 0; j < NUM_CHANNELS; j = j + 1) begin
            transition_time[j] = 0;
            last_transition_time[j] = 0;
        end
    end
    

endmodule