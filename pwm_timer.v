module pwm_timer
(   input  wire        i_clk,
    input  wire        i_rst,//active high
    input  wire        i_wb_cyc,
    input  wire        i_wb_stb,
    input  wire        i_wb_we,
    input  wire [3:0]  i_wb_adr,
    input  wire [15:0] i_wb_data,
    output reg         o_wb_ack,
    output reg  [15:0] o_wb_data,
    input  wire        i_extclk,
    input  wire [15:0] i_DC,
    input  wire        i_DC_valid,
    output reg         o_pwm
);
    //Control registers
    reg [7:0] ctrl_reg;
    reg [15:0] divisor_reg;
    reg [15:0] period_reg;
    reg [15:0] dc_reg;
    
    reg clk_sel;           // 0 = wb_clk, 1 = ext_clk
    reg mode_sel;          // 0 = timer, 1 = PWM
    reg counter_en;        // 0 = stop, 1 = start
    reg continuous;        // 0 = one-shot, 1 = continuous
    reg pwm_out_en;        // PWM output enable
    reg irq_flag;          // Timer interrupt flag
    reg ext_dc_sel;        // 0 = DC_reg, 1 = i_DC
    reg counter_rst;       // Counter reset


    //DC register
    wire [15:0] used_dc;
    assign used_dc = (ext_dc_sel)? i_DC : dc_reg;

    // internal clks
    wire   actual_clk;
    assign actual_clk = (clk_sel) ? i_extclk : i_clk;

    // counter registers
    reg [15:0] div_counter;
    reg divided_clk_pulse;
    reg [15:0] main_counter;
    //extra feautures ideas
    //error signals
    reg error_dc_too_big=0;
    reg error_div_inavlid=0;
    //internal control signals

   //Control signals_assignment_logic
    always @(*) begin
    clk_sel      = ctrl_reg[0];
    mode_sel     = ctrl_reg[1];
    counter_en   = ctrl_reg[2];
    continuous   = ctrl_reg[3];
    pwm_out_en   = ctrl_reg[4];
    irq_flag     = ctrl_reg[5];
    ext_dc_sel   = ctrl_reg[6];
    // if ((main_counter>=used_dc && mode_sel==0))
    //     counter_rst =1;
    // else 
    //     counter_rst  = ctrl_reg[7];
    end

    //Wishbone interface   
    always @(posedge i_clk or posedge i_rst) begin
    if(i_rst) begin
     ctrl_reg    <= 8'h00;
     divisor_reg <= 16'h0001;
     period_reg  <= 16'h03E8; // Default period (1000 in decimal)   
     dc_reg      <= 16'h01F4; // Default duty cycle (500 in decimal)
     o_wb_ack    <= 1'b0;     
    end
    else begin
     o_wb_ack <= i_wb_cyc && i_wb_stb;
        if (i_wb_cyc & i_wb_stb) begin
            if (i_wb_we) begin
                case (i_wb_adr[2:0])
                    3'b000: ctrl_reg    <= i_wb_data[7:0];
                    3'b001: divisor_reg <= i_wb_data;
                    3'b010: period_reg  <= i_wb_data;
                    3'b011: dc_reg      <= i_wb_data;
                    default: ;
                endcase
            end else begin
                case (i_wb_adr[2:0])
                    3'b000: o_wb_data <= {8'h00, ctrl_reg};
                    3'b001: o_wb_data <= divisor_reg;
                    3'b010: o_wb_data <= period_reg;
                    3'b011: o_wb_data <= dc_reg;
                    default: o_wb_data <= 16'h0000; // Default case
                endcase
            end
        end else begin
            o_wb_ack <= 1'b0; 
        end  
    if(main_counter>=used_dc && mode_sel==0) begin
      ctrl_reg[5]<=1;   
    end
    else begin
      ctrl_reg[5]<=ctrl_reg[5];  
    end
    end
end
  //pwm_divider
  always @(posedge actual_clk or posedge i_rst) begin
  if(i_rst)begin
   div_counter <=0;
   divided_clk_pulse <= 1'b0;     
   error_div_inavlid<=0;  
  end
  else begin
    error_div_inavlid<=0;
    if(divisor_reg <=1)begin
        div_counter <= 0;
        divided_clk_pulse <=1; //main counter always enabled like usual 
        error_div_inavlid<=1;
     end
    else begin
        if(div_counter <(divisor_reg))begin
            div_counter <=div_counter+1;
            divided_clk_pulse<=0;    
        end
        else begin
            div_counter<=0;    
            divided_clk_pulse<=1;
        end
    end
    
  end  
  end  

  //main_counter 
  always @(posedge actual_clk or posedge i_rst) begin
  if(i_rst == 1 || counter_rst == 1) begin
    main_counter <= 1;        
    end
    else if((counter_en && divided_clk_pulse && !irq_flag) || 
            (counter_en && divided_clk_pulse && continuous && mode_sel == 0)) begin
        
        if(main_counter >= period_reg) begin
            main_counter <= 1;
        end
        else begin
            main_counter <= main_counter + 1;    
        end
    end
  end

  //pwm out logic
 always @(posedge actual_clk or posedge i_rst) begin
    if(i_rst)begin
       o_pwm<=0;
       //ctrl_reg[5] <=0; //clear interrupt flag 
       counter_rst<=0;
       error_dc_too_big<=0;
      
    end
    else begin
      counter_rst<=ctrl_reg[7]; 
      if(mode_sel)begin//pwm mode
        if(counter_en && pwm_out_en)begin
            
            error_dc_too_big<=0;    
            if(period_reg<used_dc)  begin
                o_pwm<=1;
                error_dc_too_big<=1;  
            end else if (main_counter < used_dc)begin
                o_pwm <= 1'b1;
            end else begin
                o_pwm <= 1'b0;
            end

      

      end
      else begin //timer
         o_pwm<=o_pwm; 
      end       
      end else begin
          
            if(period_reg<used_dc)  begin
                o_pwm<=1;
                error_dc_too_big<=1; 
            end    
            else if (main_counter >= period_reg) begin
                o_pwm <= 1'b1;
                counter_rst<=1; //reset counter
                //set interrupt bit and reset bits in their always block same condition
            end else begin
                o_pwm <= 1'b0;
            end
            
                        
      end   
  end
 end  
endmodule 