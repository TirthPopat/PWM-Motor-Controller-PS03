# ============================================================================
# Innovus MMMC View Definition File
# ============================================================================

# 1. Define Library Set (45nm slow process corner)
create_library_set -name libs_slow \
  -timing {/home/install/FOUNDRY/digital/45nm/dig/lib/slow.lib}

# 2. Define RC Corner
create_rc_corner -name rc_typical \
  -preRoute_res 1 -preRoute_cap 1 \
  -postRoute_res 1 -postRoute_cap 1

# 3. Combine into Delay Corner
create_delay_corner -name delay_slow \
  -library_set libs_slow \
  -rc_corner rc_typical

# 4. Load SDC Timing Constraints
create_constraint_mode -name constraints_default \
  -sdc_files {../outputs/pwm_motor_controller_top_syn.sdc}

# 5. Create and Set Analysis View for Setup & Hold
create_analysis_view -name view_slow \
  -constraint_mode constraints_default \
  -delay_corner delay_slow

set_analysis_view -setup {view_slow} -hold {view_slow}
