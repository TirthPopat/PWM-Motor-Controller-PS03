# ============================================================================
# Cadence Tempus Timing Signoff Script (PS-03 Motor Controller)
# ============================================================================

# 1. Load Timing Libraries & Post-Route Verilog Netlist
read_lib /home/install/FOUNDRY/digital/45nm/dig/lib/slow.lib
read_verilog ../outputs_innovus/pwm_motor_controller_top_pr.v
set_top_module pwm_motor_controller_top

# 2. Load SDC Constraints & Post-Route SPEF Parasitics
read_sdc ../outputs/pwm_motor_controller_top_syn.sdc
set_driving_cell -lib_cell BUFX2 [remove_from_collection [all_inputs] [get_ports {pclk presetn}]]
set_propagated_clock [all_clocks]
read_spef ../outputs_innovus/pwm_motor_controller_top.spef

# 3. Perform Signoff Analysis & Write Reports
file mkdir ../reports_tempus

report_timing -max_paths 10 > ../reports_tempus/setup_signoff.rpt
report_timing -early -max_paths 10 > ../reports_tempus/hold_signoff.rpt
report_analysis_summary > ../reports_tempus/summary.rpt

report_constraint -all_violators > ../reports_tempus/violators.rpt
check_timing > ../reports_tempus/check_timing.rpt
check_timing -verbose > ../reports_tempus/check_timing_verbose.rpt

exit
