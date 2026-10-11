// ============================================================================
// Module: pwm_motor_controller_top
// Description: Top-level integration of APB slave regfile, PWM engine, 
//              speed feedback path, PI controller, and safety protections.
// ============================================================================

module pwm_motor_controller_top (
    input  wire        pclk,
    input  wire        presetn,
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [7:0]  paddr,
    input  wire [31:0] pwdata,
    output reg  [31:0] prdata,
    output wire        pready,

    // Hardware Pins
    input  wire [15:0] ext_throttle,    // Hardware throttle command from PID
    input  wire        feedback_in,     // Tachometer/Encoder input
    input  wire        fault_oc_in,     // Hardware over-current input
    output wire        pwm_h_out,       // H-bridge High-Side Output
    output wire        pwm_l_out        // H-bridge Low-Side Output
);

    assign pready = 1'b1; // Single-cycle APB access

    // Register Map Addresses
    localparam REG_CTRL        = 8'h00; // [0]: En, [1]: SoftStart, [2]: FaultClr, [3]: HW_Throttle_En
    localparam REG_STATUS      = 8'h04; // [1:0]: Faults
    localparam REG_PRESCALE    = 8'h08;
    localparam REG_PERIOD      = 8'h0C;
    localparam REG_DEADTIME    = 8'h10;
    localparam REG_SETPOINT    = 8'h14; // Target speed / software throttle command
    localparam REG_KP          = 8'h18;
    localparam REG_KI          = 8'h1C;
    localparam REG_DUTY_MIN    = 8'h20;
    localparam REG_DUTY_MAX    = 8'h24;
    localparam REG_SLEW_RATE   = 8'h28;
    localparam REG_WD_LIMIT    = 8'h2C;
    localparam REG_WIN_PERIOD  = 8'h30;
    localparam REG_MEAS_SPEED  = 8'h34; // Read-only
    localparam REG_ACTIVE_DUTY = 8'h38; // Read-only

    // Register Files
    reg [31:0] reg_ctrl;
    reg [15:0] reg_prescale;
    reg [15:0] reg_period;
    reg [15:0] reg_deadtime;
    reg [15:0] reg_setpoint;
    reg [15:0] reg_kp;
    reg [15:0] reg_ki;
    reg [15:0] reg_duty_min;
    reg [15:0] reg_duty_max;
    reg [15:0] reg_slew_rate;
    reg [15:0] reg_wd_limit;
    reg [15:0] reg_win_period;

    // Wire interconnects
    wire [15:0] measured_speed;
    wire [15:0] active_duty;
    wire [15:0] pi_duty;
    wire [15:0] limited_duty;
    wire [1:0]  fault_status;
    wire        period_tick;
    wire        feedback_loss;
    wire        pwm_h_raw, pwm_l_raw;

    // Hardware Setpoint Routing
    wire [15:0] active_setpoint;
    assign active_setpoint = reg_ctrl[3] ? ext_throttle : reg_setpoint;

    // APB Write Register Logic
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            reg_ctrl       <= 32'd0;
            reg_prescale   <= 16'd0;
            reg_period     <= 16'd4095; // Default 12-bit period
            reg_deadtime   <= 16'd20;
            reg_setpoint   <= 16'd0;
            reg_kp         <= 16'h0100; // 1.0 in Q8.8
            reg_ki         <= 16'h0010; // 0.0625 in Q8.8
            reg_duty_min   <= 16'd0;
            reg_duty_max   <= 16'd4095;
            reg_slew_rate  <= 16'd100;
            reg_wd_limit   <= 16'd50000;
            reg_win_period <= 16'd10000;
        end else if (psel && penable && pwrite) begin
            case (paddr)
                REG_CTRL:        reg_ctrl       <= pwdata;
                REG_PRESCALE:    reg_prescale   <= pwdata[15:0];
                REG_PERIOD:      reg_period     <= pwdata[15:0];
                REG_DEADTIME:    reg_deadtime   <= pwdata[15:0];
                REG_SETPOINT:    reg_setpoint   <= pwdata[15:0];
                REG_KP:          reg_kp         <= pwdata[15:0];
                REG_KI:          reg_ki         <= pwdata[15:0];
                REG_DUTY_MIN:    reg_duty_min   <= pwdata[15:0];
                REG_DUTY_MAX:    reg_duty_max   <= pwdata[15:0];
                REG_SLEW_RATE:   reg_slew_rate  <= pwdata[15:0];
                REG_WD_LIMIT:    reg_wd_limit   <= pwdata[15:0];
                REG_WIN_PERIOD:  reg_win_period <= pwdata[15:0];
                default: ;
            endcase
        end else begin
            // Auto-clear fault_clear bit after write
            if (reg_ctrl[2]) reg_ctrl[2] <= 1'b0;
        end
    end

    // APB Read Register Logic
    always @(*) begin
        case (paddr)
            REG_CTRL:        prdata = reg_ctrl;
            REG_STATUS:      prdata = {30'd0, fault_status};
            REG_PRESCALE:    prdata = {16'd0, reg_prescale};
            REG_PERIOD:      prdata = {16'd0, reg_period};
            REG_DEADTIME:    prdata = {16'd0, reg_deadtime};
            REG_SETPOINT:    prdata = {16'd0, reg_setpoint};
            REG_KP:          prdata = {16'd0, reg_kp};
            REG_KI:          prdata = {16'd0, reg_ki};
            REG_DUTY_MIN:    prdata = {16'd0, reg_duty_min};
            REG_DUTY_MAX:    prdata = {16'd0, reg_duty_max};
            REG_SLEW_RATE:   prdata = {16'd0, reg_slew_rate};
            REG_WD_LIMIT:    prdata = {16'd0, reg_wd_limit};
            REG_WIN_PERIOD:  prdata = {16'd0, reg_win_period};
            REG_MEAS_SPEED:  prdata = {16'd0, measured_speed};
            REG_ACTIVE_DUTY: prdata = {16'd0, active_duty};
            default:         prdata = 32'd0;
        endcase
    end

    // Sub-module Instantiations
    feedback_path u_fb (
        .clk            (pclk),
        .rst_n          (presetn),
        .enable         (reg_ctrl[0]),
        .feedback_raw   (feedback_in),
        .window_period  (reg_win_period),
        .watchdog_limit (reg_wd_limit),
        .measured_speed (measured_speed),
        .feedback_loss  (feedback_loss)
    );

    pi_controller u_pi (
        .clk            (pclk),
        .rst_n          (presetn),
        .enable         (reg_ctrl[0]),
        .calc_trigger   (period_tick),  
        .setpoint       (active_setpoint), 
        .feedback       (measured_speed),
        .kp             (reg_kp),
        .ki             (reg_ki),
        .duty_min       (reg_duty_min),
        .duty_max       (reg_duty_max),
        .pi_duty_out    (pi_duty)
    );

    protection_stage u_prot (
        .clk             (pclk),
        .rst_n           (presetn),
        .soft_start_en   (reg_ctrl[1]),
        .fault_oc_raw    (fault_oc_in),
        .fault_fbloss    (feedback_loss),
        .fault_clear     (reg_ctrl[2]),
        .target_duty     (pi_duty),
        .slew_rate       (reg_slew_rate),
        .soft_start_step (16'd10),
        .period_tick     (period_tick),
        .pwm_h_in        (pwm_h_raw),
        .pwm_l_in        (pwm_l_raw),
        .pwm_h_safe      (pwm_h_out),
        .pwm_l_safe      (pwm_l_out),
        .limited_duty    (limited_duty),
        .fault_status    (fault_status)
    );

    pwm_core u_pwm (
        .clk         (pclk),
        .rst_n       (presetn),
        .enable      (reg_ctrl[0]),
        .prescaler   (reg_prescale),
        .period      (reg_period),
        .duty_in     (limited_duty),
        .dead_time   (reg_deadtime),
        .active_duty (active_duty),
        .period_tick (period_tick),
        .pwm_h       (pwm_h_raw),
        .pwm_l       (pwm_l_raw)
    );

endmodule
