// ============================================================================
// File: integrated_flight_motor_top.v
// Module: integrated_flight_motor_top
// Description: Integrated Track Top-Level Wrapper combining PS-02 PID Engine
//              and PS-03 PWM Motor Controller into a unified sensor-to-motor chain.
// ============================================================================
`timescale 1ns / 1ps

module integrated_flight_motor_top (
    input  wire        pclk,
    input  wire        presetn,
    // APB Bus Interface
    input  wire [31:0] paddr,
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [31:0] pwdata,
    output wire [31:0] prdata,
    output wire        pready,
    output wire        pslverr,

    // PS-02 IMU Sensor Inputs
    input  wire [15:0] imu_sample_in,
    input  wire        imu_valid_in,
    input  wire [15:0] flight_target_setpoint,
    input  wire [1:0]  flight_mode_select,

    // PS-03 Hardware Motor & Gate Driver Interfaces
    input  wire        ext_oc_fault,      // External over-current pin
    input  wire        tacho_feedback,    // Tachometer speed pulse feedback
    output wire        pwm_h,             // High-side complementary PWM pin
    output wire        pwm_l,             // Low-side complementary PWM pin
    output wire        system_failsafe    // Failsafe indicator flag
);

    // Interconnect Wires between PS-02 and PS-03
    wire [15:0] pid_throttle_cmd;
    wire        pid_watchdog_fault;
    wire        unified_fault_line;

    // Cross-Block Unified Failsafe Interlock
    assign unified_fault_line = ext_oc_fault | pid_watchdog_fault;
    assign system_failsafe    = unified_fault_line;

    // ------------------------------------------------------------------------
    // Instance 1: PS-02 Mode-Switching PID Engine
    // ------------------------------------------------------------------------
    pid_engine_ps02 #(
        .WATCHDOG_LIMIT(1000)
    ) u_ps02_pid (
        .clk(pclk),
        .rst_n(presetn),
        .imu_sample(imu_sample_in),
        .imu_sample_valid(imu_valid_in),
        .target_setpoint(flight_target_setpoint),
        .flight_mode(flight_mode_select),
        .kp_in(16'd120),
        .ki_in(16'd15),
        .kd_in(16'd45),
        .pid_out(pid_throttle_cmd),
        .watchdog_fault(pid_watchdog_fault)
    );

// ------------------------------------------------------------------------
    // Instance 2: PS-03 PWM Motor Controller Top
    // ------------------------------------------------------------------------
pwm_motor_controller_top u_ps03_pwm (
        .pclk(pclk),
        .presetn(presetn),
        .psel(psel),
        .penable(penable),
        .pwrite(pwrite),
        .paddr(paddr[7:0]),  
        .pwdata(pwdata),
        .prdata(prdata),
        .pready(pready),

        // Hardware Pins:
        .ext_throttle(pid_throttle_cmd),   // <--- NOW CONNECTED
        .fault_oc_in(unified_fault_line), 
        .feedback_in(tacho_feedback),
        .pwm_h_out(pwm_h),
        .pwm_l_out(pwm_l)
    );
endmodule
