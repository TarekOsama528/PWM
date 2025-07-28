vlib work
vlog pwm_timer.v
vlog pwm_timer_tb.v
vsim -voptargs=+acc work.pwm_timer_tb
add wave *
run -all
#quit -sim