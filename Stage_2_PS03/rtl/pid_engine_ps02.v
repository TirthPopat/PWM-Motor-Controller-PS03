// ============================================================================
// File: pid_engine_ps02.v
// Module: pid_engine_ps02
// Description: Mode-Switching Hardware PID Engine (PS-02) featuring Acro,
//              Stabilize, Altitude Hold, and Failsafe modes with anti-windup,
//              derivative low-pass filter, and sensor watchdog timeout.
// ============================================================================
`timescale 1ns / 1ps

module pid_engine_ps02 #(
    parameter WATCHDOG_LIMIT = 1000  // Cycles before forcing Failsafe on stale IMU
)(
    input  wire        clk,
    input  wire        rst_n,
    // Sensor & Control Inputs
    input  wire [15:0] imu_sample,       // Q4.12 IMU sensor measurement
    input  wire        imu_sample_valid, // Pulse when new sample arrives
    input  wire [15:0] target_setpoint,  // Desired angle / rate setpoint
    input  wire [1:0]  flight_mode,      // 00: Acro, 01: Stabilize, 10: AltHold, 11: Failsafe
    // Configurable Gain Parameters
    input  wire [15:0] kp_in,
    input  wire [15:0] ki_in,
    input  wire [15:0] kd_in,
    // Outputs
    output reg  [15:0] pid_out,          // 16-bit actuator command output
    output reg         watchdog_fault    // High if sensor drops out
);

    // Mode Definitions
    localparam MODE_ACRO      = 2'b00;
    localparam MODE_STABILIZE = 2'b01;
    localparam MODE_ALTHOLD   = 2'b10;
    localparam MODE_FAILSAFE  = 2'b11;

    // Internal Registers
    reg [15:0] wd_counter;
    reg [31:0] integrator;
    reg [15:0] prev_sample;
    reg [15:0] d_filter;
    reg [1:0]  prev_mode;

    // 1. Watchdog Timer Logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wd_counter     <= 16'd0;
            watchdog_fault <= 1'b0;
        end else if (imu_sample_valid) begin
            wd_counter     <= 16'd0;
            watchdog_fault <= 1'b0;
        end else begin
            if (wd_counter >= WATCHDOG_LIMIT) begin
                watchdog_fault <= 1'b1;
            end else begin
                wd_counter <= wd_counter + 1'b1;
            end
        end
    end

    // 2. PID Control Loop Execution
    wire [1:0] active_mode = (watchdog_fault || flight_mode == MODE_FAILSAFE) ? MODE_FAILSAFE : flight_mode;
    
    // Error Calculation
    wire signed [16:0] error = $signed({1'b0, target_setpoint}) - $signed({1'b0, imu_sample});
    
    // Derivative on Measurement (Avoids Setpoint Kick) + Low Pass Filter
    wire signed [16:0] d_raw = $signed({1'b0, imu_sample}) - $signed({1'b0, prev_sample});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            integrator  <= 32'd0;
            prev_sample <= 16'd0;
            d_filter    <= 16'd0;
            pid_out     <= 16'd0;
            prev_mode   <= MODE_FAILSAFE;
        end else if (imu_sample_valid) begin
            // Detect Mode Switch for Bumpless Transfer
            if (active_mode != prev_mode) begin
                // Back-calculate integrator on mode change to prevent jump
                integrator <= (pid_out > 16'd100) ? ({16'd0, pid_out} << 8) : 32'd0;
            end else begin
                // Mode-Specific Gain Scaling
                case (active_mode)
                    MODE_ACRO: begin
                        // Rate control: Fast integral accumulation
                        if (integrator < 32'h00FF_FFFF)
                            integrator <= integrator + (error * ki_in);
                    end
                    MODE_STABILIZE: begin
                        // Angle + Rate control: Anti-windup clamped integral
                        if (error > -100 && error < 100)
                            integrator <= integrator + (error * ki_in);
                    end
                    MODE_ALTHOLD: begin
                        // Vertical Channel integration
                        integrator <= integrator + ((error * ki_in) >>> 1);
                    end
                    MODE_FAILSAFE: begin
                        integrator <= 32'd0; // Reset integrator in Failsafe
                    end
                endcase
            end

            // Low-pass Filter Derivative Path
            d_filter <= (d_filter >>> 1) + ($signed({1'b0, d_raw}) >>> 1);
            prev_sample <= imu_sample;
            prev_mode <= active_mode;

            // Output Calculation & Saturation
            if (active_mode == MODE_FAILSAFE) begin
                pid_out <= 16'd0; // Safe State
            end else begin
                // Proportional + Integral + Derivative Calculation
                reg signed [31:0] raw_calc;
                raw_calc = (error * kp_in) + (integrator >>> 8) - (d_filter * kd_in);
                
                // Saturating Arithmetic Clamp (0 to 65535)
                if (raw_calc < 0)
                    pid_out <= 16'd0;
                else if (raw_calc > 32'd65535)
                    pid_out <= 16'd65535;
                else
                    pid_out <= raw_calc[15:0];
            end
        end
    end

endmodule
