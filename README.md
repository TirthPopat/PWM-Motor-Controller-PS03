# PS-03: PWM Motor Controller with Live Throttle Feedback

VLSI Hackathon 2026, Sudarshana Semiconductors problem statement PS-03.

A configurable PWM motor controller for particular instances with closed-loop speed control. An APB-configurable
PWM generator drives an H-bridge (complementary outputs with dead-time). A tachometer or
encoder input is synchronized, filtered and turned into a speed measurement, and a PI
controller adjusts the duty cycle to hold the speed setpoint. A protection stage provides
soft-start, slew-rate limiting, duty clamps, over-current and feedback-loss shutdown, and a
defined safe state (both outputs low).

The design is taken from RTL to a routed layout and signed off with the Cadence flow
**Genus (synthesis) → Innovus (place and route) → Tempus (timing signoff)** in a 45 nm
GSCL standard-cell process at 100 MHz.

| Item | Value |
|---|---|
| Top module | `pwm_motor_controller_top` |
| Clock | `pclk`, 10 ns period (100 MHz), single clock domain |
| Process / corner | 45 nm GSCL (`gsclib045`), slow corner (`slow.lib`), VDD = 1.08 V |
| Configuration interface | APB slave, single-cycle access, 32-bit |
| Tool versions | Genus 21.14-s082_1, Innovus 21.15-s110_1, Tempus 22.11-s001_1 |

---

## 1. Features

| Requirement (PS-03 §6.2) | Implementation |
|---|---|
| PWM core, 12 to 16 bit, programmable frequency | 16-bit counter with 16-bit prescaler. `f_pwm = f_clk / ((PRESCALE+1) × (PERIOD+1))` |
| Glitch-free update | Duty is loaded into the PWM only at the period boundary |
| Dead-time | Programmable (`DEADTIME` register), complementary `pwm_h_out` / `pwm_l_out` |
| Feedback path | 2-flop synchronizer, 4-sample glitch filter, rising-edge pulse counter over a programmable window, watchdog |
| Closed loop | Fixed-point PI controller (Q8.8 gains) with integrator clamp and output clamp |
| Protection | Soft-start ramp, slew-rate limiter, duty min/max clamps, latched over-current and feedback-loss faults |
| Safe state | Both outputs low at reset, when disabled, and on any fault |

Timing behaviour (measured in simulation of this RTL):

| Behaviour | Value |
|---|---|
| Dead-time (both outputs low between transitions) | exactly `DEADTIME` clock cycles; no shoot-through observed |
| Over-current shutdown (`fault_oc_in`) | outputs low 3 clock edges after the first edge that samples the input high (2-flop synchronizer + output register) |
| Feedback-loss shutdown | flag raised `WD_LIMIT + 2` clocks after the last debounced tach edge; outputs low 1 clock later |
| Duty resolution | `log2(PERIOD+1)` bits: 12 bits at the default `PERIOD = 4095`, up to 16 bits |

---

## 2. Repository layout

The three flow scripts use relative paths and **must be run from the `scripts/` directory**.

```
Stage_2_PS03/
├── rtl/                          synthesizable RTL (5 files)
│   ├── pwm_motor_controller_top.v   top level, APB register file, integration
│   ├── pwm_core.v                   PWM counter, double-buffered duty, dead-time
│   ├── feedback_path.v              synchronizer, filter, speed measurement, watchdog
│   ├── pi_controller.v              fixed-point PI controller
│   └── protection_stage.v           faults, soft-start, slew limiter, output gating
├── scripts/                      all flows are run from here
│   ├── run_genus.tcl                synthesis
│   ├── run_innovus.tcl              place and route
│   ├── run_tempus.tcl               timing signoff
│   ├── constraits.sdc               timing constraints read by Genus
│   └── viewDefinition.tcl           Innovus MMMC view (view_slow: slow.lib + constraints)
├── outputs/                      Genus: gate-level netlist and SDC (generated)
├── reports_genus/                Genus reports (generated)
├── outputs_innovus/              post-route netlist, DEF, SPEF (generated)
├── reports_innovus/              Innovus reports (generated)
└── reports_tempus/               Tempus signoff reports (generated)
```

### Design hierarchy

```
pwm_motor_controller_top        APB register file + integration
├── u_fb    feedback_path       feedback_in → measured_speed, feedback_loss
├── u_pi    pi_controller       setpoint − measured_speed → pi_duty
├── u_prot  protection_stage    pi_duty → limited_duty; fault latching; output gating
└── u_pwm   pwm_core            limited_duty → raw complementary PWM
```

Data path: `feedback_in → u_fb → measured_speed → u_pi (with SETPOINT) → pi_duty →
u_prot (soft-start, slew) → limited_duty → u_pwm (loaded at period boundary) → u_prot
(fault gating) → pwm_h_out / pwm_l_out`.

---

## 3. Interface

| Port | Dir | Width | Description |
|---|---|---|---|
| `pclk` | in | 1 | Clock, 100 MHz |
| `presetn` | in | 1 | Active-low reset, asynchronous assert (see §9 item 4) |
| `psel`, `penable`, `pwrite` | in | 1 | APB control |
| `paddr` | in | 8 | APB address (word-aligned byte offsets, §4) |
| `pwdata` | in | 32 | APB write data |
| `prdata` | out | 32 | APB read data (combinational on `paddr`) |
| `pready` | out | 1 | Tied high (no wait states); there is no `pslverr` |
| `feedback_in` | in | 1 | Tachometer / encoder pulse input (asynchronous) |
| `fault_oc_in` | in | 1 | Over-current input, active high (asynchronous) |
| `pwm_h_out` | out | 1 | H-bridge high-side drive, active high |
| `pwm_l_out` | out | 1 | H-bridge low-side drive, active high |

Clocking and reset: one clock domain (`pclk`). The two asynchronous inputs (`feedback_in`,
`fault_oc_in`) each cross through a 2-flop synchronizer. `presetn` resets every flop
asynchronously.

---

## 4. Register map

32-bit registers at word-aligned offsets. Writes to read-only or unmapped addresses are
ignored; reads of unmapped addresses return 0.

| Offset | Name | Access | Reset | Description |
|---|---|---|---|---|
| 0x00 | `CTRL` | RW | 0x0 | `[0]` EN (enable), `[1]` SOFT_START_EN, `[2]` FAULT_CLEAR (self-clearing pulse) |
| 0x04 | `STATUS` | RO | - | `[0]` over-current fault latched, `[1]` feedback-loss fault latched |
| 0x08 | `PRESCALE` | RW | 0 | Counter tick = `PRESCALE + 1` clocks |
| 0x0C | `PERIOD` | RW | 4095 | PWM period = `PERIOD + 1` ticks |
| 0x10 | `DEADTIME` | RW | 20 | Dead-time in clock cycles |
| 0x14 | `SETPOINT` | RW | 0 | Speed target, in tach pulses per measurement window |
| 0x18 | `KP` | RW | 0x0100 | Proportional gain, Q8.8 (0x0100 = 1.0) |
| 0x1C | `KI` | RW | 0x0010 | Integral gain, see note below |
| 0x20 | `DUTY_MIN` | RW | 0 | Lower duty clamp, in PWM ticks |
| 0x24 | `DUTY_MAX` | RW | 4095 | Upper duty clamp, in PWM ticks |
| 0x28 | `SLEW_RATE` | RW | 100 | Maximum duty change per PWM period, in ticks. 0 freezes the duty |
| 0x2C | `WD_LIMIT` | RW | 50000 | Feedback watchdog limit, clock cycles without a tach edge |
| 0x30 | `WIN_PERIOD` | RW | 10000 | Speed-measurement window = `WIN_PERIOD + 1` clocks |
| 0x34 | `MEAS_SPEED` | RO | - | Pulses counted in the last window |
| 0x38 | `ACTIVE_DUTY` | RO | - | Duty currently loaded in the PWM (ticks) |

Units and behaviour:

- **Duty values** (`DUTY_MIN`, `DUTY_MAX`, `SLEW_RATE`, `ACTIVE_DUTY`) are in PWM counter ticks (0 to `PERIOD`).
- **Speed values** (`SETPOINT`, `MEAS_SPEED`) are tach pulses per window, so the setpoint and the measurement are directly comparable.
- **PI law** (evaluated once per PWM period): `duty = error × KP / 256 + I / 512`, where `error = SETPOINT − MEAS_SPEED` and `I` accumulates `error × KI` each period, clamped to ±(2²⁴ − 1). The output is clamped to `[DUTY_MIN, DUTY_MAX]`. The effective integral gain per period is therefore `KI / 512`. The controller is always closed loop when EN = 1; there is no open-loop mode bit.
- **Soft-start** (`CTRL[1]` = 1): the allowed duty ceiling rises by 10 ticks per PWM period (fixed in RTL) until it reaches the PI output.
- **Faults**: over-current and feedback loss are latched in `STATUS`. Writing `CTRL[2]` = 1 clears them (an asserted `fault_oc_in` keeps the outputs off until it deasserts). While faulted, the soft-start ramp and limited duty are reset to 0.
- **PWM frequency examples at 100 MHz**: defaults give 24.4 kHz (`PRESCALE = 0`, `PERIOD = 4095`); 400 Hz ESC-style signalling uses `PRESCALE = 60`, `PERIOD = 4095`.

---

## 5. Prerequisites

- Cadence **Genus**, **Innovus** and **Tempus** (versions above) with a valid license.
- 45 nm GSCL kit files used by the scripts:
  - Timing library: `/home/install/FOUNDRY/digital/45nm/dig/lib/slow.lib`
  - Technology LEF: `/home/install/FOUNDRY/digital/45nm/LIBS/lef/gsclib045.fixed.lef`
- `scripts/constraits.sdc` and `scripts/viewDefinition.tcl` present (see §2).

If the kit is installed elsewhere, edit the three paths at the top of the scripts
(`set_db library` in Genus, `init_lef_file` in Innovus, `read_lib` in Tempus).

### Timing constraints

The constraints applied (as written to `outputs/pwm_motor_controller_top_syn.sdc`):

| Constraint | Value |
|---|---|
| Clock `pclk` | period 10.0 ns, 50% duty, transition 0.15 ns |
| Clock uncertainty | 0.2 ns setup, 0.2 ns hold |
| Input delay (max) | 2.0 ns on `feedback_in`, `fault_oc_in`; 3.0 ns on `psel`, `penable`, `pwrite`, `paddr`, `pwdata` |
| Output delay (max) | 3.0 ns on `prdata`, `pready`; 2.0 ns on `pwm_h_out`, `pwm_l_out` |
| Minimum I/O delay (hold) | 0.5 ns on all inputs and outputs |
| Reset | `presetn` is a false path |

---

## 6. Build instructions

All commands are run from the `scripts/` directory, in this order. Each stage reads what
the previous stage wrote.

```
RTL ──Genus──► gate-level netlist + SDC ──Innovus──► routed netlist + SPEF + DEF ──Tempus──► signoff reports
```

First create the output directories (some scripts write reports before they create the
folders):

```bash
cd Stage_2_PS03/scripts
mkdir -p ../outputs ../reports_genus ../outputs_innovus ../reports_innovus ../reports_tempus
```

### Step 0 (optional): RTL sanity check

Open-source tools, not part of the signoff flow. Compiles and elaborates the RTL and
confirms there are no latches:

```bash
cd ../rtl
iverilog -g2005-sv -Wall -o /tmp/lint.vvp pwm_core.v feedback_path.v pi_controller.v \
         protection_stage.v pwm_motor_controller_top.v
yosys -p "read_verilog -sv pwm_core.v feedback_path.v pi_controller.v protection_stage.v \
          pwm_motor_controller_top.v; hierarchy -top pwm_motor_controller_top -check; \
          proc; check -assert"
cd ../scripts
```

### Step 1: Synthesis (Genus)

```bash
genus -f run_genus.tcl          # or start genus, then: source run_genus.tcl
```

What `run_genus.tcl` does: reads the five RTL files from `../rtl`, elaborates
`pwm_motor_controller_top`, checks for unresolved references, reads `constraits.sdc`, and
runs `syn_generic`, `syn_map`, `syn_opt` against `slow.lib`.

| Output | File |
|---|---|
| Gate-level netlist | `../outputs/pwm_motor_controller_top_syn.v` |
| Constraints for the next stages | `../outputs/pwm_motor_controller_top_syn.sdc` |
| Area, timing, power, QoR | `../reports_genus/{area,timing,power,qor}.rpt` |

Check before continuing: no unresolved modules in the log, and non-negative slack in
`reports_genus/timing.rpt`.

### Step 2: Place and route (Innovus)

```bash
innovus -no_gui -init run_innovus.tcl     # or start innovus, then: source run_innovus.tcl
```

What `run_innovus.tcl` does, in order:

1. `init_design` with the LEF, the Genus netlist and `viewDefinition.tcl`.
2. Floorplan: aspect ratio 1.0, 70% target density, 10 µm margins.
3. Connect VDD/VSS and add core power rings (Metal6 top/bottom, Metal5 left/right, width 2, spacing 1).
4. Place with a 1-site cell pad (keeps cell pins from abutting on Metal1).
5. Clock tree synthesis with CCOpt (`create_ccopt_clock_tree_spec`, `ccopt_design`).
6. Detailed routing with NanoRoute (`routeDesign`).
7. Post-route optimization: design-rule repair (`optDesign -postRoute -drv`), then hold fixing (`-holdTargetSlack 0.25`, OCV analysis with CPPR). The 0.25 ns target covers the 0.2 ns hold uncertainty plus margin.
8. Verification: `verifyGeometry` (DRC) and `verifyConnectivity`.
9. Post-route RC extraction and export.

| Output | File |
|---|---|
| Routed netlist | `../outputs_innovus/pwm_motor_controller_top_pr.v` |
| Parasitics | `../outputs_innovus/pwm_motor_controller_top.spef` |
| Layout (floorplan, netlist, routing) | `../outputs_innovus/pwm_motor_controller_top.def` |
| DRC / connectivity | `../reports_innovus/drc.rpt`, `lvs.rpt` |
| Timing, power, coverage | `../reports_innovus/{timing_postroute,power_postroute,coverage}.rpt` |

Check before continuing: `drc.rpt` shows no violations and `lvs.rpt` shows no problems
(see §9 item 1).

### Step 3: Timing signoff (Tempus)

```bash
tempus -no_gui -init run_tempus.tcl       # or start tempus, then: source run_tempus.tcl
```

What `run_tempus.tcl` does: loads `slow.lib`, the post-route netlist, the Genus SDC and
the post-route SPEF; applies a BUFX2 driving cell to all inputs except `pclk` and
`presetn`; sets propagated clocks; then reports timing and constraint violations.

The reset is excluded from the driving-cell command on purpose: a BUFX2 cannot drive the
reset net's fan-out and would create a false max-capacitance violation.

| Output | File |
|---|---|
| Setup / hold paths | `../reports_tempus/setup_signoff.rpt`, `hold_signoff.rpt` |
| All constraint violations | `../reports_tempus/violators.rpt` |
| Constraint-completeness check | `../reports_tempus/check_timing.rpt`, `check_timing_verbose.rpt` |
| Summary | `../reports_tempus/summary.rpt` |

The batch command lines above are the standard Cadence invocations. Starting the tool
interactively and using `source <script>` works equally well if your lab wraps the tools
differently.

### Pass criteria

| Check | Pass condition |
|---|---|
| Setup | worst slack ≥ 0 in `setup_signoff.rpt` |
| Hold | worst slack ≥ 0 in `hold_signoff.rpt` |
| Violations | `violators.rpt` reports "No paths found" / "No Violations found" for every check type |
| DRC | `drc.rpt`: "No DRC violations were found" |
| Connectivity | `lvs.rpt`: no problems reported |
| `check_timing` | only the expected warnings: `no_drive` for `pclk` and `presetn`, `no_input_delay` for `presetn`, and unconstrained recovery/removal endpoints, all of them the asynchronous reset/set pins (`/RN`, `/SN`) of the flops |

---

## 7. Latest results

From the run completed on 11 Oct 2026 (Innovus 00:46, Tempus 00:50). To be refreshed
after the open items in §9 are closed.

| Metric | Result |
|---|---|
| Setup WNS (Tempus, propagated clock, 0.2 ns uncertainty) | **+0.034 ns** |
| Hold WNS (Tempus) | **+0.150 ns** |
| Constraint violations (`violators.rpt`) | none |
| DRC (`verifyGeometry`) | 0 violations |
| Connectivity (`verifyConnectivity`) | **open**, see §9 item 1 |
| Total power (Innovus, slow corner, 100 MHz, default 0.2 switching activity) | 2.083 mW (internal 1.374, switching 0.709, leakage 0.0006) |
| Instances in design (after CTS and hold fixing) | 4449 |
| Critical path | PI datapath: `u_fb_measured_speed_reg` → multiply/accumulate → `u_pi_integrator_reg` / `u_pi_pi_duty_out_reg` |

Area and cell-type breakdown: see `reports_genus/area.rpt`.

---

## 8. Declarations

- **Third-party IP**: none. All RTL in `rtl/` is the team's own work.
- **Tools**: Cadence Genus, Innovus and Tempus under the institution's licence; 45 nm GSCL kit (`gsclib045`). Icarus Verilog and Yosys are used only for the optional sanity check in Step 0.

---
