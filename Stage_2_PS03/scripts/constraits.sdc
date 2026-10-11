# Primary Clock Definition (100 MHz target, 10ns period)
create_clock -name pclk -period 10.00 [get_ports pclk]
set_clock_uncertainty 0.20 [get_clocks pclk]
set_clock_transition  0.15 [get_clocks pclk]

# Reset and Asynchronous Inputs
set_false_path -from [get_ports presetn]
set_input_delay -max 2.0 -clock pclk [get_ports feedback_in]
set_input_delay -max 2.0 -clock pclk [get_ports fault_oc_in]

# APB Bus Delays
set_input_delay  -max 3.0 -clock pclk [get_ports {psel penable pwrite paddr[*] pwdata[*]}]
set_output_delay -max 3.0 -clock pclk [get_ports {prdata[*] pready}]

# PWM Outputs
set_output_delay -max 2.0 -clock pclk [get_ports {pwm_h_out pwm_l_out}]
