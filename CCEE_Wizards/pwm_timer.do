vlib work
vlog pwm_timer.sv
vlog pwm_timer_tb.sv
vsim -voptargs=+acc work.pwm_timer_tb
add wave *
run -all
#quit -sim
