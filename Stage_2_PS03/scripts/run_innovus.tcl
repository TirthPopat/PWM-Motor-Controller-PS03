# ============================================================================
# Cadence Innovus Place & Route Script (45nm GSCL)
# ============================================================================

# 1. Define Design Initialization Variables
set init_lef_file       "/home/install/FOUNDRY/digital/45nm/LIBS/lef/gsclib045.fixed.lef"
set init_verilog        "../outputs/pwm_motor_controller_top_syn.v"
set init_top_cell       "pwm_motor_controller_top"
set init_mmmc_file      "viewDefinition.tcl"
set init_pwr_net        "VDD"
set init_gnd_net        "VSS"

# 2. Initialize Design into Memory
setDesignMode -process 45
init_design

# 3. Floorplanning (Aspect ratio 1.0, 70% target cell density, 10um margins)
floorPlan -r 1.0 0.70 10 10 10 10

# 4. Global Power Connections & Power Rings
globalNetConnect VDD -type pgpin -pin VDD -inst *
globalNetConnect VSS -type pgpin -pin VSS -inst *
addRing -type core_rings -nets {VDD VSS} -width 2 -spacing 1 -layer {top Metal6 bottom Metal6 left Metal5 right Metal5}

# 5. Standard Cell Placement (1-Site Cell Padding to prevent M1 pin abutment)
specifyCellPad * 1
setPlaceMode -fp false
place_design

# 6. Clock Tree Synthesis (CTS via CCOpt)
create_ccopt_clock_tree_spec
ccopt_design

# 7. Detailed Routing (NanoRoute)
setNanoRouteMode -quiet -routeWithEco false
routeDesign

# 8. Post-Route Hold Timing Optimization
setOptMode -holdFixingCells {BUFX1 BUFX2 BUFX4 BUFX8 DLY1X1 DLY2X1 DLY4X1}
setAnalysisMode -analysisType onChipVariation -cppr both
optDesign -postRoute -drv
setOptMode -holdTargetSlack 0.25
optDesign -postRoute -hold

# 9. Physical Verification (DRC & LVS)
verifyGeometry -report ../reports_innovus/drc.rpt
verifyConnectivity -type all -report ../reports_innovus/lvs.rpt

# 10. Parasitic Extraction & Deliverables Export
setExtractRCMode -engine postRoute
extractRC

file mkdir ../reports_innovus
file mkdir ../outputs_innovus

report_analysis_coverage > ../reports_innovus/coverage.rpt
report_timing            > ../reports_innovus/timing_postroute.rpt
report_power             > ../reports_innovus/power_postroute.rpt

rcOut -spef ../outputs_innovus/pwm_motor_controller_top.spef
defOut -floorplan -netlist -routing ../outputs_innovus/pwm_motor_controller_top.def
saveNetlist ../outputs_innovus/pwm_motor_controller_top_pr.v
