# ============================================================================
# Cadence Genus Synthesis Script (45nm PDK) - Integrated Flight Control Top
# ============================================================================

# 1. Set HDL Search Path and Target 45nm Standard Cell Library
set_db init_hdl_search_path {../rtl}
set_db library {/home/install/FOUNDRY/digital/45nm/dig/lib/slow.lib}

# 2. Read All Integrated Design RTL Files (PS-02 + PS-03)
read_hdl -sv {
    pid_engine_ps02.v
    pwm_core.v
    feedback_path.v
    pi_controller.v
    protection_stage.v
    pwm_motor_controller_top.v
    integrated_flight_motor_top.v
}

# 3. Elaborate Top-Level Module
elaborate integrated_flight_motor_top
check_design -unresolved

# 4. Apply Timing Constraints
read_sdc constraits.sdc

# 5. Run Synthesis Optimization
syn_generic
syn_map
syn_opt

# 6. Export Reports and Gate-Level Netlist
file mkdir ../reports_genus
file mkdir ../outputs

report_area   > ../reports_genus/area.rpt
report_timing > ../reports_genus/timing.rpt
report_power  > ../reports_genus/power.rpt
report_qor    > ../reports_genus/qor.rpt

write_hdl > ../outputs/integrated_flight_motor_top_syn.v
write_sdc > ../outputs/integrated_flight_motor_top_syn.sdc
